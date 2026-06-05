/**
 * @file posture_features.c
 * @brief Dual-IMU posture feature extraction for Back on Track
 *
 * Two IMUs:
 *   IMU0 (upper back/chest) — captures thoracic posture
 *   IMU1 (lower back/lumbar) — captures lumbar posture
 *
 * Key new metric: spinal_flexion_deg
 *   = (lower_pitch - lower_neutral) - (upper_pitch - upper_neutral)
 *   Positive means lower back is rounding forward relative to upper back.
 *   This fires POSTURE_LUMBAR_COLLAPSE even when upper back looks fine.
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

bool posture_features_compute(const imu_sample_t  *imu_upper,
                               const imu_sample_t  *imu_lower,
                               const emg_sample_t  *emg,
                               const calibration_t *cal,
                               posture_features_t  *out) {
    if (!imu_upper || !imu_lower || !emg || !cal || !out) return false;

    memset(out, 0, sizeof(*out));
    out->timestamp_ms = get_timestamp_ms();

    // ---- Upper IMU (IMU0) features ----
    out->upper_imu_valid  = imu_upper->valid;
    out->upper_pitch_raw  = imu_upper->pitch_deg;
    out->upper_roll_raw   = imu_upper->roll_deg;
    out->upper_yaw_raw    = imu_upper->yaw_deg;

    if (imu_upper->valid && cal->calibrated) {
        out->upper_pitch_deviation = imu_upper->pitch_deg - cal->neutral_upper_pitch_deg;
        out->upper_roll_deviation  = imu_upper->roll_deg  - cal->neutral_upper_roll_deg;
        out->upper_yaw_deviation   = imu_upper->yaw_deg   - cal->neutral_upper_yaw_deg;
    } else if (imu_upper->valid) {
        out->upper_pitch_deviation = imu_upper->pitch_deg;
        out->upper_roll_deviation  = imu_upper->roll_deg;
        out->upper_yaw_deviation   = imu_upper->yaw_deg;
        ESP_LOGW(TAG, "No calibration — using raw upper IMU angles");
    }

    // ---- Lower IMU (IMU1) features ----
    out->lower_imu_valid  = imu_lower->valid;
    out->lower_pitch_raw  = imu_lower->pitch_deg;
    out->lower_roll_raw   = imu_lower->roll_deg;
    out->lower_yaw_raw    = imu_lower->yaw_deg;

    if (imu_lower->valid && cal->calibrated) {
        out->lower_pitch_deviation = imu_lower->pitch_deg - cal->neutral_lower_pitch_deg;
        out->lower_roll_deviation  = imu_lower->roll_deg  - cal->neutral_lower_roll_deg;
        out->lower_yaw_deviation   = imu_lower->yaw_deg   - cal->neutral_lower_yaw_deg;
    } else if (imu_lower->valid) {
        out->lower_pitch_deviation = imu_lower->pitch_deg;
        out->lower_roll_deviation  = imu_lower->roll_deg;
        out->lower_yaw_deviation   = imu_lower->yaw_deg;
    }

    // ---- Spinal flexion ----
    // Computed only if both IMUs are valid.
    // Positive value = lower back rounding forward relative to upper back.
    if (imu_upper->valid && imu_lower->valid) {
        float flexion = out->lower_pitch_deviation - out->upper_pitch_deviation;
        // Subtract the neutral spinal flexion offset captured during calibration
        if (cal->calibrated) {
            flexion -= cal->neutral_spinal_flexion_deg;
        }
        out->spinal_flexion_deg   = flexion;
        out->spinal_flexion_valid = true;
    } else {
        out->spinal_flexion_deg   = 0.0f;
        out->spinal_flexion_valid = false;
    }

    // ---- EMG features ----
    out->emg_valid    = emg->valid;
    out->emg_raw_adc  = emg->raw_adc;
    out->emg_filtered = emg->filtered;
    out->emg_activity = emg->activity;
    out->emg_high     = emg->high_activation;

    return true;
}

bool posture_calibrate(SensorContext_t *imu_upper_ctx,
                       SensorContext_t *imu_lower_ctx,
                       SensorContext_t *emg_ctx,
                       calibration_t   *cal,
                       uint32_t         duration_ms) {
    if (!imu_upper_ctx || !imu_lower_ctx || !emg_ctx || !cal) return false;

    ESP_LOGI(TAG, "=== CALIBRATION START ===");
    ESP_LOGI(TAG, "Sit in NEUTRAL upright posture. Relax muscles.");
    ESP_LOGI(TAG, "Sampling for %"PRIu32" ms...", duration_ms);

    double sum_up_pitch = 0, sum_up_roll = 0, sum_up_yaw = 0;
    double sum_lo_pitch = 0, sum_lo_roll = 0, sum_lo_yaw = 0;
    double sum_emg = 0;
    int    up_count = 0, lo_count = 0, emg_count = 0;

    uint32_t start_ms = get_timestamp_ms();
    imu_sample_t imu_up, imu_lo;
    emg_sample_t emg_s;

    while (get_timestamp_ms() - start_ms < duration_ms) {
        if (bno085_read_orientation(imu_upper_ctx, &imu_up) && imu_up.valid) {
            sum_up_pitch += imu_up.pitch_deg;
            sum_up_roll  += imu_up.roll_deg;
            sum_up_yaw   += imu_up.yaw_deg;
            up_count++;
        }
        if (bno085_read_orientation(imu_lower_ctx, &imu_lo) && imu_lo.valid) {
            sum_lo_pitch += imu_lo.pitch_deg;
            sum_lo_roll  += imu_lo.roll_deg;
            sum_lo_yaw   += imu_lo.yaw_deg;
            lo_count++;
        }
        if (myoware_read_sample(emg_ctx, &emg_s) && emg_s.valid) {
            myoware_update_baseline(&emg_s);
            sum_emg += emg_s.raw_adc;
            emg_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (up_count == 0 || lo_count == 0 || emg_count == 0) {
        ESP_LOGE(TAG, "Calibration FAILED (upper=%d lower=%d emg=%d samples)",
                 up_count, lo_count, emg_count);
        cal->calibrated = false;
        return false;
    }

    cal->neutral_upper_pitch_deg = (float)(sum_up_pitch / up_count);
    cal->neutral_upper_roll_deg  = (float)(sum_up_roll  / up_count);
    cal->neutral_upper_yaw_deg   = (float)(sum_up_yaw   / up_count);

    cal->neutral_lower_pitch_deg = (float)(sum_lo_pitch / lo_count);
    cal->neutral_lower_roll_deg  = (float)(sum_lo_roll  / lo_count);
    cal->neutral_lower_yaw_deg   = (float)(sum_lo_yaw   / lo_count);

    // Neutral spinal flexion: how much the lower back naturally pitches
    // differently from the upper back when sitting correctly.
    // Subtracted from future spinal_flexion_deg readings.
    cal->neutral_spinal_flexion_deg =
        cal->neutral_lower_pitch_deg - cal->neutral_upper_pitch_deg;

    cal->emg_resting_baseline = (float)(sum_emg / emg_count);
    cal->calibrated           = true;

    ESP_LOGI(TAG, "=== CALIBRATION COMPLETE ===");
    ESP_LOGI(TAG, "  Upper pitch/roll/yaw: %.2f / %.2f / %.2f deg",
             cal->neutral_upper_pitch_deg,
             cal->neutral_upper_roll_deg,
             cal->neutral_upper_yaw_deg);
    ESP_LOGI(TAG, "  Lower pitch/roll/yaw: %.2f / %.2f / %.2f deg",
             cal->neutral_lower_pitch_deg,
             cal->neutral_lower_roll_deg,
             cal->neutral_lower_yaw_deg);
    ESP_LOGI(TAG, "  Neutral spinal flexion: %.2f deg",
             cal->neutral_spinal_flexion_deg);
    ESP_LOGI(TAG, "  EMG baseline: %.1f counts", cal->emg_resting_baseline);

    return true;
}
