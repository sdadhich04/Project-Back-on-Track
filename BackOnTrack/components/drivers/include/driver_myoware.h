/**
 * @file driver_myoware.h
 * @brief MyoWare 2.0 EMG sensor driver header for Back on Track (ESP-IDF ADC)
 *
 * The MyoWare 2.0 Muscle Sensor outputs a rectified, smoothed analog voltage
 * proportional to muscle activation level. Higher activation = higher voltage.
 *
 * This driver replaces Arduino's analogRead() with ESP-IDF's ADC oneshot driver
 * (adc_oneshot_unit_handle_t), as specified for ESP-IDF v5.x.
 *
 * WIRING:
 *   MyoWare SIG   -> GPIO4 (ADC1_CH3) — or change MYOWARE_GPIO_PIN in data_types.h
 *   MyoWare VCC   -> 3.3V or 5V (check MyoWare 2.0 board specs; can run on 3.3V)
 *   MyoWare GND   -> GND (shared with ESP32-S3 GND)
 *   MyoWare RAW+  -> optional raw EMG signal (not connected in this design)
 *
 * SAFETY NOTE (read this before powering on):
 *   - MyoWare and ESP32-S3 share a common ground. When powered via USB from a laptop,
 *     the user's body is connected to the laptop's USB ground through the electrodes.
 *   - This is generally safe for surface EMG prototyping with dry snap electrodes
 *     BUT: do NOT use wet electrodes, needle electrodes, or place electrodes near
 *     the chest/heart area with this non-isolated setup.
 *   - Maximum signal voltage at SIG pin = 3.3V (stay within ESP32-S3 ADC range).
 *     If MyoWare is powered from 5V, its output CAN exceed 3.3V and will damage
 *     the ADC input. Use a voltage divider or power MyoWare from 3.3V.
 *
 * ELECTRODE PLACEMENT NOTES:
 *   - Clean skin with alcohol wipe before placing electrodes.
 *   - Place the two signal electrodes along (parallel to) the muscle belly.
 *   - Place the reference electrode over a bony area (wrist, elbow) away from muscle.
 *   - For upper trap / shoulder posture: place along the upper trapezius.
 *   - Noisy signal? Check electrode contact, re-wet if dry, reduce lead length.
 *
 * Based conceptually on SparkFun MyoWare 2.0 Arduino analog examples:
 * https://github.com/sparkfun/SparkFun_MyoWare_Code_Examples
 */

#ifndef DRIVER_MYOWARE_H
#define DRIVER_MYOWARE_H

#include "sensor_hal.h"
#include "data_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the MyoWare 2.0 ADC channel.
 *
 * Initializes the ESP-IDF ADC oneshot driver for the configured GPIO/channel.
 * Applies ADC_ATTEN_DB_11 attenuation for full 0–3.1V input range.
 * Calibrates the ADC if factory calibration data is available in eFuse.
 *
 * @param ctx Sensor context. ctx->hw_config must point to a myoware_adc_config_t.
 * @return true on success, false on failure
 */
bool myoware_init(SensorContext_t *ctx);

/**
 * @brief Read one EMG sample from the MyoWare 2.0 sensor.
 *
 * Reads raw ADC value, converts to voltage (if calibration available),
 * applies EMA (exponential moving average) filter, computes activity above
 * resting baseline, and sets the high_activation flag if threshold exceeded.
 *
 * @param ctx    Sensor context (must be initialized)
 * @param out    Pointer to emg_sample_t to fill
 * @return true on success, false on ADC error
 */
bool myoware_read_sample(SensorContext_t *ctx, emg_sample_t *out);

/**
 * @brief Compatibility wrapper for SensorContext_t vtable (data_out = emg_sample_t*)
 */
bool myoware_read_sample_vtable(SensorContext_t *ctx, void *data_out);

/**
 * @brief Update the resting EMG baseline with a new sample.
 *
 * Call this during the calibration phase (user sitting relaxed, not flexing).
 * Uses a slow EMA to track baseline drift over time.
 *
 * @param sample Pointer to a freshly read emg_sample_t
 */
void myoware_update_baseline(emg_sample_t *sample);

/**
 * @brief Reset the EMG baseline to uncalibrated state.
 * Call at session start before running the calibration loop.
 */
void myoware_reset_baseline(void);

#ifdef __cplusplus
}
#endif

#endif // DRIVER_MYOWARE_H
