/**
 * @file posture_features.h
 * @brief Dual-IMU posture feature extraction for Back on Track
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
 * @brief Compute posture features from two IMU samples + one EMG sample.
 *
 * @param imu_upper  Latest sample from IMU0 (upper back / chest)
 * @param imu_lower  Latest sample from IMU1 (lower back / lumbar)
 * @param emg        Latest MyoWare EMG sample
 * @param cal        Calibration baseline (must be calibrated=true for accurate results)
 * @param out        Output posture_features_t to fill
 * @return true if features were computed; false on null input
 */
bool posture_features_compute(const imu_sample_t   *imu_upper,
                               const imu_sample_t   *imu_lower,
                               const emg_sample_t   *emg,
                               const calibration_t  *cal,
                               posture_features_t   *out);

/**
 * @brief Run the dual-IMU calibration sequence.
 *
 * Blocks for duration_ms while sampling both IMUs and the EMG sensor.
 * Call while sitting in NEUTRAL upright posture with muscles relaxed.
 *
 * @param imu_upper_ctx  SensorContext for IMU0 (ctx->id = IMU_ID_UPPER)
 * @param imu_lower_ctx  SensorContext for IMU1 (ctx->id = IMU_ID_LOWER)
 * @param emg_ctx        SensorContext for MyoWare
 * @param cal            Output calibration struct
 * @param duration_ms    Sampling duration (3000–5000 ms recommended)
 */
bool posture_calibrate(SensorContext_t *imu_upper_ctx,
                       SensorContext_t *imu_lower_ctx,
                       SensorContext_t *emg_ctx,
                       calibration_t   *cal,
                       uint32_t         duration_ms);

#ifdef __cplusplus
}
#endif

#endif // POSTURE_FEATURES_H
