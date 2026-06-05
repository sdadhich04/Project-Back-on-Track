/**
 * @file telemetry_serial.h
 * @brief Serial telemetry output for Back on Track
 *
 * Provides two output modes selectable at compile time:
 *   TELEMETRY_MODE_CSV   — comma-separated values (Serial Plotter compatible)
 *   TELEMETRY_MODE_JSON  — pretty-printed JSON-like lines (human readable)
 *
 * Set TELEMETRY_OUTPUT_MODE below, or override in CMakeLists.txt.
 *
 * TODO: Add BLE telemetry. When ready, add a telemetry_ble.h/c module,
 *       register a BLE GATT characteristic, and send the same JSON string
 *       as a BLE notification. Keep BLE completely separate from this module.
 */

#ifndef TELEMETRY_SERIAL_H
#define TELEMETRY_SERIAL_H

#include "data_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==================== Output Mode ====================
// Change this to switch between CSV (Serial Plotter) and JSON (readable)
// 0 = CSV, 1 = JSON
#define TELEMETRY_OUTPUT_MODE   0

/**
 * @brief Initialize serial telemetry.
 * Currently a no-op (UART is initialized by the IDF startup code).
 * Reserved for future BLE init hook.
 */
void telemetry_init(void);

/**
 * @brief Print one telemetry frame to Serial (UART0 = USB-CDC on ESP32-S3).
 *
 * In CSV mode, prints a header line on first call, then data lines:
 *   timestamp_ms,state,pitch_dev,roll_dev,yaw_dev,emg_raw,emg_filt,emg_act,
 *   good_ms,bad_ms,corrections,points,streak_ms
 *
 * In JSON mode, prints a JSON object per line:
 *   {"t":1234,"state":"GOOD","pitch":-2.1,"roll":0.5,...}
 *
 * @param features   Latest posture feature vector
 * @param result     Latest classification result
 * @param stats      Current session statistics
 */
void telemetry_print(const posture_features_t      *features,
                     const posture_classification_t *result,
                     const session_stats_t          *stats);

/**
 * @brief Print a startup banner to Serial.
 * Call once after init to confirm the device is alive.
 */
void telemetry_print_banner(void);

#ifdef __cplusplus
}
#endif

#endif // TELEMETRY_SERIAL_H
