/**
 * @file driver_myoware.c
 * @brief MyoWare 2.0 EMG sensor driver for Back on Track (ESP-IDF v5.x ADC)
 *
 * Implements EMG sampling via ESP-IDF's ADC oneshot API.
 * Conceptually adapted from SparkFun's MyoWare 2.0 Arduino analog examples:
 *   https://github.com/sparkfun/SparkFun_MyoWare_Code_Examples
 * The original examples use Arduino analogRead(). Here we use the ESP-IDF
 * adc_oneshot_unit_handle_t pattern instead, which is the correct approach
 * for ESP-IDF v5.x projects.
 *
 * Signal processing pipeline:
 *   RAW ADC (12-bit, 0-4095)
 *     -> Voltage conversion (mV, using ADC calibration if available)
 *     -> EMA low-pass filter (smooths noise)
 *     -> Activity = filtered - resting_baseline
 *     -> High activation flag = activity > threshold
 */

#include "driver_myoware.h"
#include "sensor_hal.h"
#include "data_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "myoware";

// ==================== EMA Filter Config ====================
// EMA (Exponential Moving Average): filtered = alpha * new + (1-alpha) * old
// Higher alpha = faster response, more noise
// Lower  alpha = smoother, slower response
// For 100 Hz EMG, alpha = 0.1 gives ~100ms smoothing time constant.
#define EMG_EMA_ALPHA           0.1f

// Baseline tracking EMA (much slower — tracks slow electrode drift)
// At 100 Hz, alpha = 0.001 gives ~10s drift tracking.
#define EMG_BASELINE_EMA_ALPHA  0.001f

// EMG activity threshold: must exceed this many ADC counts above baseline
// to flag high_activation. Tune after electrode placement.
#define EMG_ACTIVITY_THRESHOLD  POSTURE_EMG_HIGH_THRESHOLD

// ==================== Driver State ====================

typedef struct {
    adc_oneshot_unit_handle_t  adc_handle;
    adc_cali_handle_t          cali_handle;
    bool                       cali_available;
    int                        channel;            // ADC channel number
    bool                       initialized;

    float                      ema_filtered;       // EMA output (ADC counts or mV)
    float                      resting_baseline;   // Updated during calibration
    bool                       baseline_set;
} myoware_state_t;

static myoware_state_t s_myoware = {0};

// ==================== Public: Init ====================

bool myoware_init(SensorContext_t *ctx) {
    if (!ctx || !ctx->hw_config) {
        ESP_LOGE(TAG, "myoware_init: ctx or hw_config is NULL");
        return false;
    }

    myoware_adc_config_t *cfg = (myoware_adc_config_t *)ctx->hw_config;

    // --- ADC oneshot unit init ---
    // We use ADC_UNIT_1 (maps to GPIO1-GPIO10 on ESP32-S3).
    // ADC_UNIT_2 conflicts with WiFi/BLE in some configurations — avoid it.
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = (adc_unit_t)cfg->adc_unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_myoware.adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(err));
        return false;
    }

    // --- Configure the specific ADC channel ---
    // ADC_ATTEN_DB_11: input range 0 to ~3100 mV (covers full 3.3V MyoWare output)
    // ADC_BITWIDTH_DEFAULT: 12-bit on ESP32-S3 (0–4095)
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = (adc_atten_t)cfg->atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    err = adc_oneshot_config_channel(s_myoware.adc_handle,
                                      (adc_channel_t)cfg->adc_channel,
                                      &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed (ch=%d): %s",
                 cfg->adc_channel, esp_err_to_name(err));
        adc_oneshot_del_unit(s_myoware.adc_handle);
        return false;
    }

    s_myoware.channel = cfg->adc_channel;

    // --- ADC calibration (optional but improves voltage accuracy) ---
    // Tries curve fitting (ESP32-S3 supports this) first, then line fitting.
    // If neither is available, raw ADC counts are used without voltage conversion.
    s_myoware.cali_available = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = (adc_unit_t)cfg->adc_unit,
        .chan     = (adc_channel_t)cfg->adc_channel,
        .atten   = (adc_atten_t)cfg->atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_myoware.cali_handle);
    if (err == ESP_OK) {
        s_myoware.cali_available = true;
        ESP_LOGI(TAG, "ADC calibration: curve fitting available");
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!s_myoware.cali_available) {
        adc_cali_line_fitting_config_t lf_cfg = {
            .unit_id  = (adc_unit_t)cfg->adc_unit,
            .atten   = (adc_atten_t)cfg->atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        err = adc_cali_create_scheme_line_fitting(&lf_cfg, &s_myoware.cali_handle);
        if (err == ESP_OK) {
            s_myoware.cali_available = true;
            ESP_LOGI(TAG, "ADC calibration: line fitting available");
        }
    }
#endif

    if (!s_myoware.cali_available) {
        ESP_LOGW(TAG, "No ADC calibration available — using raw counts (voltage_mv unreliable)");
    }

    // Initialize EMA filter state to midrange to avoid step on first read
    s_myoware.ema_filtered    = 2048.0f;
    s_myoware.resting_baseline = 2048.0f;
    s_myoware.baseline_set    = false;
    s_myoware.initialized     = true;

    ESP_LOGI(TAG, "MyoWare init OK (GPIO%d, ADC%d_CH%d, cali=%s)",
             cfg->gpio_pin,
             cfg->adc_unit + 1,
             cfg->adc_channel,
             s_myoware.cali_available ? "yes" : "no");
    return true;
}

// ==================== Public: Read Sample ====================

bool myoware_read_sample(SensorContext_t *ctx, emg_sample_t *out) {
    if (!ctx || !out || !s_myoware.initialized) {
        if (out) { out->valid = false; }
        return false;
    }

    // --- Read raw ADC value ---
    int raw = 0;
    esp_err_t err = adc_oneshot_read(s_myoware.adc_handle,
                                      (adc_channel_t)s_myoware.channel,
                                      &raw);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC read error: %s", esp_err_to_name(err));
        out->valid = false;
        return false;
    }

    // Sanity check: ADC value should be in 12-bit range (0–4095)
    if (raw < 0 || raw > 4095) {
        ESP_LOGW(TAG, "ADC out of range: %d", raw);
        out->valid = false;
        return false;
    }

    out->raw_adc      = raw;
    out->timestamp_ms = get_timestamp_ms();
    out->valid        = true;

    // --- Convert to voltage (mV) if calibration is available ---
    if (s_myoware.cali_available) {
        int mv = 0;
        err = adc_cali_raw_to_voltage(s_myoware.cali_handle, raw, &mv);
        out->voltage_mv = (err == ESP_OK) ? (float)mv : (float)raw;
    } else {
        // Approximate: (raw / 4095) * vref_mv
        // This is rough but gives ballpark voltage for display purposes
        myoware_adc_config_t *cfg = (myoware_adc_config_t *)ctx->hw_config;
        out->voltage_mv = ((float)raw / 4095.0f) * cfg->vref_mv;
    }

    // --- EMA filter ---
    // We filter in ADC count space (simpler, consistent with baseline tracking)
    s_myoware.ema_filtered = EMG_EMA_ALPHA * (float)raw
                            + (1.0f - EMG_EMA_ALPHA) * s_myoware.ema_filtered;
    out->filtered = s_myoware.ema_filtered;

    // --- Activity above baseline ---
    out->activity = s_myoware.ema_filtered - s_myoware.resting_baseline;
    if (out->activity < 0.0f) out->activity = 0.0f;  // Clamp to non-negative

    out->high_activation = (out->activity > EMG_ACTIVITY_THRESHOLD);

    return true;
}

// Vtable-compatible wrapper
bool myoware_read_sample_vtable(SensorContext_t *ctx, void *data_out) {
    return myoware_read_sample(ctx, (emg_sample_t *)data_out);
}

// ==================== Public: Baseline Management ====================

/**
 * @brief Update resting baseline with a slow EMA.
 * Call this during the calibration phase when the user is relaxed.
 * After calibration, do NOT call this during normal operation (baseline drifts).
 */
void myoware_update_baseline(emg_sample_t *sample) {
    if (!sample || !sample->valid) return;

    if (!s_myoware.baseline_set) {
        // First baseline sample: snap directly to current value
        s_myoware.resting_baseline = (float)sample->raw_adc;
        s_myoware.baseline_set     = true;
    } else {
        // Slow EMA for gentle drift tracking
        s_myoware.resting_baseline =
            EMG_BASELINE_EMA_ALPHA  * (float)sample->raw_adc
            + (1.0f - EMG_BASELINE_EMA_ALPHA) * s_myoware.resting_baseline;
    }
}

void myoware_reset_baseline(void) {
    s_myoware.resting_baseline = 2048.0f;
    s_myoware.baseline_set     = false;
    s_myoware.ema_filtered     = 2048.0f;
    ESP_LOGI(TAG, "EMG baseline reset");
}
