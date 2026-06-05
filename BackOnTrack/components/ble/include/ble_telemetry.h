/**
 * @file ble_telemetry.h
 * @brief BLE telemetry for Back on Track
 *
 * Implements a NimBLE GATT peripheral that streams raw synchronized sensor
 * packets to a connected client (phone app, Python script, signal processing
 * system, etc.).
 *
 * Architecture intent
 * -------------------
 * The ESP32-S3 firmware is the DATA LAYER only. It does NOT classify posture
 * or score sessions over BLE. That belongs in the external signal processing
 * system (phone app, PC script, etc.).
 *
 * What IS sent over BLE:
 *   - Synchronized timestamp
 *   - Upper IMU orientation (pitch, roll, yaw + raw quaternion)
 *   - Lower IMU orientation (pitch, roll, yaw + raw quaternion)
 *   - EMG raw ADC value + filtered value
 *   - Sensor validity flags
 *
 * What is NOT sent over BLE:
 *   - Posture classification state
 *   - Correction events
 *   - Gamification points/streaks
 *   (Those remain serial-only for local debug; external system computes them)
 *
 * BLE GATT Layout
 * ---------------
 * Service:          Back on Track Sensor Service
 *   UUID:           0x1801  (custom 128-bit expanded, see BOT_SERVICE_UUID)
 *
 * Characteristic 1: Sensor Packet (notify)
 *   UUID:           BOT_SENSOR_CHAR_UUID
 *   Properties:     NOTIFY + READ
 *   Value:          bot_ble_packet_t (packed struct, little-endian)
 *   Max size:       ~60 bytes — fits in one BLE ATT MTU (23 bytes default,
 *                   up to 512 bytes with MTU exchange)
 *
 * Characteristic 2: Control (write)
 *   UUID:           BOT_CONTROL_CHAR_UUID
 *   Properties:     WRITE
 *   Value:          bot_ble_cmd_t (1 byte command)
 *   Commands:       BOT_CMD_START_CALIBRATE, BOT_CMD_RESET_SESSION
 *
 * Connection parameters
 * ---------------------
 * - Advertises as "BackOnTrack"
 * - Accepts one connection (peripheral role, single client)
 * - Notify rate: configurable via BLE_NOTIFY_RATE_HZ (default 20 Hz)
 *   Lower than IMU sample rate — BLE bandwidth is the bottleneck, not the sensors.
 *
 * Usage
 * -----
 *   ble_telemetry_init();           // call once in app_main before tasks start
 *   ble_telemetry_start();          // begin advertising
 *   ble_notify_sensor_packet(&pkt); // call from processing_task each cycle
 *   bool ok = ble_is_connected();   // check before notify if desired
 */

#ifndef BLE_TELEMETRY_H
#define BLE_TELEMETRY_H

#include "data_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==================== BLE Config ====================

// Device name broadcast in BLE advertisements
#define BOT_BLE_DEVICE_NAME         "BackOnTrack"

// Notify rate — how often the processing task pushes a packet over BLE.
// 20 Hz is comfortable for BLE 4.x with default connection interval.
// Increase to 50 Hz only after confirming connection interval < 20ms.
#define BLE_NOTIFY_RATE_HZ          20

// ==================== GATT UUIDs ====================
// Using 16-bit UUIDs in a custom 128-bit base for simplicity.
// Base: 12345678-1234-1234-1234-1234567890XX
// These are arbitrary — replace with your own registered UUIDs for production.

#define BOT_SERVICE_UUID            0xAB00
#define BOT_SENSOR_CHAR_UUID        0xAB01  // Notify: raw sensor packet
#define BOT_CONTROL_CHAR_UUID       0xAB02  // Write:  control commands

// ==================== BLE Packet (wire format) ====================

/**
 * @brief Flat packed sensor packet sent over BLE.
 *
 * This is the ONLY struct the external signal processing system needs.
 * It contains everything raw — no derived states, no classifications.
 * The external system does all the math.
 *
 * Packed with __attribute__((packed)) so the byte layout is deterministic
 * across compiler versions and platforms. The receiving app must use the
 * same layout (document it in your signal processing system).
 *
 * Total size: 4 + 7*4 + 7*4 + 4 + 4 + 2 + 2 = 60 bytes
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;      // ESP32 uptime in ms (monotonic, resets on reboot)

    // --- Upper IMU (IMU0, upper back/chest) ---
    float upper_pitch;          // degrees, + = forward lean
    float upper_roll;           // degrees, + = right lean
    float upper_yaw;            // degrees
    float upper_q_i;            // raw quaternion i (for external sensor fusion)
    float upper_q_j;
    float upper_q_k;
    float upper_q_r;

    // --- Lower IMU (IMU1, lower back/lumbar) ---
    float lower_pitch;
    float lower_roll;
    float lower_yaw;
    float lower_q_i;
    float lower_q_j;
    float lower_q_k;
    float lower_q_r;

    // --- EMG (MyoWare 2.0) ---
    float emg_filtered;         // EMA-smoothed ADC value
    float emg_voltage_mv;       // Calibrated voltage (0 if calibration unavailable)

    // --- Status flags (packed into 1 byte) ---
    // Bit 0: upper IMU valid
    // Bit 1: lower IMU valid
    // Bit 2: EMG valid
    // Bit 3: calibrated (true once startup calibration completed)
    uint8_t  status_flags;

    // --- Sequence number ---
    // Increments every packet. Receiver can detect dropped packets.
    uint8_t  seq;

    // --- Raw EMG ADC (for signal processing) ---
    uint16_t emg_raw_adc;
} bot_ble_packet_t;

// Status flag bit positions
#define BOT_FLAG_UPPER_VALID    (1 << 0)
#define BOT_FLAG_LOWER_VALID    (1 << 1)
#define BOT_FLAG_EMG_VALID      (1 << 2)
#define BOT_FLAG_CALIBRATED     (1 << 3)

// ==================== Control Commands ====================

typedef enum {
    BOT_CMD_START_CALIBRATE = 0x01, // Trigger a new calibration sequence
    BOT_CMD_RESET_SESSION   = 0x02, // Reset session stats (classification side)
    BOT_CMD_PING            = 0xFF, // Connectivity check — no action, just ACK
} bot_ble_cmd_t;

// ==================== Public API ====================

/**
 * @brief Initialize the NimBLE stack and register the GATT service.
 * Call once from app_main BEFORE starting FreeRTOS tasks.
 * Does NOT start advertising — call ble_telemetry_start() for that.
 *
 * @return true on success
 */
bool ble_telemetry_init(void);

/**
 * @brief Start BLE advertising. Call after ble_telemetry_init().
 * The device will appear as "BackOnTrack" to scanners.
 */
void ble_telemetry_start(void);

/**
 * @brief Build a bot_ble_packet_t from current sensor data and notify
 * any connected client. Safe to call even when not connected (no-op).
 *
 * Call from processing_task at BLE_NOTIFY_RATE_HZ.
 *
 * @param imu_upper  Latest upper IMU sample
 * @param imu_lower  Latest lower IMU sample
 * @param emg        Latest EMG sample
 * @param calibrated True if startup calibration has completed
 */
void ble_notify_sensor_packet(const imu_sample_t *imu_upper,
                               const imu_sample_t *imu_lower,
                               const emg_sample_t *emg,
                               bool                calibrated);

/**
 * @brief Returns true if a client is currently connected and has
 * enabled notifications on the sensor characteristic.
 */
bool ble_is_connected(void);

/**
 * @brief Returns true if a calibration command was received over BLE
 * since the last call to this function (auto-clears on read).
 * Poll this from app_main or a supervisor task to re-trigger calibration.
 */
bool ble_calibration_requested(void);

/**
 * @brief Returns true if a session reset command was received over BLE
 * since the last call (auto-clears on read).
 */
bool ble_session_reset_requested(void);

#ifdef __cplusplus
}
#endif

#endif // BLE_TELEMETRY_H
