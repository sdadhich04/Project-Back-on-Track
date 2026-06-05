/**
 * @file posture_features.h
 * @brief Posture feature extraction for Back on Track
 *
 * Combines raw IMU and EMG samples with calibration data to produce
 * a posture_features_t struct that the classifier can consume.
 */

#ifndef POSTURE_FEATURES_H
#define POSTURE_FEATURES_H

#include "data_types.h"
#include "sensor_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Compute posture features from the latest IMU and EMG samples.
 *
 * Subtracts the calibrated neutral orientation from current IMU angles to
 * get deviation values. Passes through EMG activity from the EMG sample.
 *
 * @param imu   Most recent IMU sample (must have valid=true for reliable features)
 * @param emg   Most recent EMG sample
 * @param cal   Calibration baseline (must have calibrated=true)
 * @param out   Output posture feature vector
 * @return true if features were computed (even with partial data); false on null input
 */
bool posture_features_compute(const imu_sample_t       *imu,
                               const emg_sample_t       *emg,
                               const calibration_t      *cal,
                               posture_features_t       *out);

/**
 * @brief Run the calibration sequence.
 *
 * Call this while the user is seated in their ideal neutral posture with
 * muscles relaxed. Averages IMU and EMG readings over `duration_ms` milliseconds
 * to establish neutral_pitch, neutral_roll, neutral_yaw, and emg_resting_baseline.
 *
 * Blocks for duration_ms (uses vTaskDelay internally — call from a FreeRTOS task).
 *
 * @param imu_ctx  BNO085 sensor context
 * @param emg_ctx  MyoWare sensor context
 * @param cal      Output calibration struct to fill
 * @param duration_ms How long to sample (recommend 3000–5000 ms)
 * @return true on success
 */
bool posture_calibrate(SensorContext_t *imu_ctx,
                       SensorContext_t *emg_ctx,
                       calibration_t   *cal,
                       uint32_t         duration_ms);

#ifdef __cplusplus
}
#endif

#endif // POSTURE_FEATURES_H
