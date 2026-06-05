/**
 * @file posture_features.c
 * @brief Posture feature extraction implementation for Back on Track
 */

#include "posture_features.h"
#include "driver_bno085.h"
#include "driver_myoware.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "posture_feat";

bool posture_features_compute(const imu_sample_t  *imu,
                               const emg_sample_t  *emg,
                               const calibration_t *cal,
                               posture_features_t  *out) {
    if (!imu || !emg || !cal || !out) return false;

    memset(out, 0, sizeof(*out));
    out->timestamp_ms = get_timestamp_ms();

    // --- IMU Features ---
    out->imu_valid    = imu->valid;
    out->pitch_raw    = imu->pitch_deg;
    out->roll_raw     = imu->roll_deg;
    out->yaw_raw      = imu->yaw_deg;

    if (imu->valid && cal->calibrated) {
        out->pitch_deviation = imu->pitch_deg - cal->neutral_pitch_deg;
        out->roll_deviation  = imu->roll_deg  - cal->neutral_roll_deg;
        out->yaw_deviation   = imu->yaw_deg   - cal->neutral_yaw_deg;
    } else if (imu->valid && !cal->calibrated) {
        // No calibration yet — use raw angles as deviations (less accurate)
        out->pitch_deviation = imu->pitch_deg;
        out->roll_deviation  = imu->roll_deg;
        out->yaw_deviation   = imu->yaw_deg;
        ESP_LOGW(TAG, "Calibration not done — using raw angles as deviation");
    } else {
        // IMU invalid — zero out deviations
        out->pitch_deviation = 0.0f;
        out->roll_deviation  = 0.0f;
        out->yaw_deviation   = 0.0f;
    }

    // --- EMG Features ---
    out->emg_valid    = emg->valid;
    out->emg_raw_adc  = emg->raw_adc;
    out->emg_filtered = emg->filtered;
    out->emg_activity = emg->activity;
    out->emg_high     = emg->high_activation;

    return true;
}

bool posture_calibrate(SensorContext_t *imu_ctx,
                       SensorContext_t *emg_ctx,
                       calibration_t   *cal,
                       uint32_t         duration_ms) {
    if (!imu_ctx || !emg_ctx || !cal) return false;

    ESP_LOGI(TAG, "=== CALIBRATION START ===");
    ESP_LOGI(TAG, "Sit upright in your NEUTRAL posture. Relax your muscles.");
    ESP_LOGI(TAG, "Sampling for %"PRIu32" ms...", duration_ms);

    // Accumulate samples for averaging
    double sum_pitch = 0, sum_roll = 0, sum_yaw = 0, sum_emg = 0;
    int    imu_count = 0, emg_count = 0;

    uint32_t start_ms = get_timestamp_ms();
    uint32_t elapsed  = 0;

    imu_sample_t imu_sample;
    emg_sample_t emg_sample;

    while ((elapsed = get_timestamp_ms() - start_ms) < duration_ms) {
        // Read IMU
        if (bno085_read_orientation(imu_ctx, &imu_sample) && imu_sample.valid) {
            sum_pitch += imu_sample.pitch_deg;
            sum_roll  += imu_sample.roll_deg;
            sum_yaw   += imu_sample.yaw_deg;
            imu_count++;
        }

        // Read EMG and update baseline tracker
        if (myoware_read_sample(emg_ctx, &emg_sample) && emg_sample.valid) {
            myoware_update_baseline(&emg_sample);
            sum_emg += emg_sample.raw_adc;
            emg_count++;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz sampling during calibration
    }

    if (imu_count == 0 || emg_count == 0) {
        ESP_LOGE(TAG, "Calibration FAILED: no valid samples (IMU=%d, EMG=%d)",
                 imu_count, emg_count);
        cal->calibrated = false;
        return false;
    }

    cal->neutral_pitch_deg    = (float)(sum_pitch / imu_count);
    cal->neutral_roll_deg     = (float)(sum_roll  / imu_count);
    cal->neutral_yaw_deg      = (float)(sum_yaw   / imu_count);
    cal->emg_resting_baseline = (float)(sum_emg   / emg_count);
    cal->calibrated           = true;

    ESP_LOGI(TAG, "=== CALIBRATION COMPLETE ===");
    ESP_LOGI(TAG, "  Neutral pitch : %.2f°", cal->neutral_pitch_deg);
    ESP_LOGI(TAG, "  Neutral roll  : %.2f°", cal->neutral_roll_deg);
    ESP_LOGI(TAG, "  Neutral yaw   : %.2f°", cal->neutral_yaw_deg);
    ESP_LOGI(TAG, "  EMG baseline  : %.1f ADC counts", cal->emg_resting_baseline);
    ESP_LOGI(TAG, "  IMU samples   : %d, EMG samples: %d", imu_count, emg_count);

    return true;
}
