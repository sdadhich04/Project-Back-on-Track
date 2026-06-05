/**
 * @file data_types.h
 * @brief Shared data structures for Back on Track
 *
 * All sensor samples, posture features, classification results, and
 * gamification state live here. These structs are passed between FreeRTOS
 * tasks via queues. Keep them small and copyable (no heap pointers).
 *
 * Adapted from Project SHIELD data_types.h, simplified for posture use case.
 */

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==================== Queue Sizes ====================

#define IMU_QUEUE_SIZE          20   // 50 Hz × ~400ms buffer; posture doesn't need huge backlog
#define EMG_QUEUE_SIZE          40   // 100 Hz × ~400ms buffer
#define POSTURE_QUEUE_SIZE      10   // 10 Hz telemetry output
#define TELEMETRY_QUEUE_SIZE    10   // Serial output queue

// ==================== Pin Configuration (change all pins here) ====================

// BNO085 SPI pins
#define BNO085_PIN_CS       37
#define BNO085_PIN_SCLK     38
#define BNO085_PIN_MOSI     40
#define BNO085_PIN_MISO     39
#define BNO085_PIN_INT       5  // MUST have external or GPIO pull-up to 3.3V
#define BNO085_PIN_RST       6
#define BNO085_SPI_HOST      2  // SPI3_HOST = 2

// MyoWare 2.0 ADC pin
// GPIO4 = ADC1_CH3 on ESP32-S3. Verify with your specific board pinout.
// Make sure MyoWare output voltage does not exceed 3.3V (ESP32-S3 ADC max).
#define MYOWARE_GPIO_PIN     4
#define MYOWARE_ADC_UNIT     0  // ADC_UNIT_1
#define MYOWARE_ADC_CHANNEL  3  // ADC1_CH3 = GPIO4

// ==================== Sample Rate Configuration ====================

#define IMU_SAMPLE_RATE_HZ       50   // BNO085 orientation at 50 Hz is plenty for posture
#define EMG_SAMPLE_RATE_HZ      100   // MyoWare at 100 Hz; smoothed before classification
#define TELEMETRY_RATE_HZ        10   // Serial output rate (CSV or JSON)
#define CLASSIFICATION_RATE_HZ   10   // Classifier runs at same rate as telemetry

// ==================== IMU Orientation Sample ====================

/**
 * @brief BNO085 orientation output.
 * 
 * The BNO085 game rotation vector report gives quaternion (i, j, k, real)
 * relative to an arbitrary reference frame (no magnetometer = no absolute heading).
 * We convert to pitch/roll/yaw for rule-based posture classification.
 *
 * Quaternion-to-Euler convention used here:
 *   - Pitch: rotation about X axis (forward/back tilt)
 *   - Roll:  rotation about Y axis (left/right tilt)
 *   - Yaw:   rotation about Z axis (twist, less useful for posture)
 *
 * Units: degrees
 */
typedef struct {
    uint32_t timestamp_ms;  // System uptime in ms when sample was taken

    // Raw quaternion from BNO085 game rotation vector report
    float q_i;   // Quaternion i (x) component
    float q_j;   // Quaternion j (y) component
    float q_k;   // Quaternion k (z) component
    float q_r;   // Quaternion real component

    // Computed Euler angles (derived from quaternion above)
    float pitch_deg;  // Tilt forward/back (positive = leaning forward)
    float roll_deg;   // Tilt left/right   (positive = leaning right)
    float yaw_deg;    // Axial rotation    (less reliable without mag)

    // Fallback: raw accelerometer magnitude (m/s²) if quaternion unavailable
    float accel_magnitude;

    bool valid;      // false if sensor returned garbage or timed out
} imu_sample_t;

// ==================== EMG Sample ====================

/**
 * @brief MyoWare 2.0 EMG sample.
 *
 * MyoWare 2.0 outputs a rectified, smoothed envelope signal. The higher
 * the muscle activation, the higher the voltage. We read it via ADC and
 * apply an additional software EMA filter.
 *
 * SAFETY NOTE: The MyoWare 2.0 is powered from the same supply as the
 * ESP32-S3. When powered via USB from a laptop, the user is NOT electrically
 * isolated. This is acceptable for dry-electrode surface EMG prototyping
 * but is NOT a medically safe or clinically approved configuration.
 * DO NOT use wet/needle electrodes with this setup.
 */
typedef struct {
    uint32_t timestamp_ms;   // System uptime in ms

    int     raw_adc;         // Raw 12-bit ADC reading (0–4095)
    float   voltage_mv;      // Converted voltage in millivolts (if calibration available)
    float   filtered;        // EMA-filtered value (same unit as raw_adc or voltage_mv)
    float   activity;        // Filtered value minus resting baseline (0 = relaxed)
    bool    high_activation; // true if activity exceeds threshold
    bool    valid;           // false if ADC returned out-of-range value
} emg_sample_t;

// ==================== Posture Features ====================

/**
 * @brief Combined IMU + EMG posture feature vector.
 * 
 * This is the input to the rule-based classifier.
 * Computed from the most recent IMU and EMG samples.
 */
typedef struct {
    uint32_t timestamp_ms;

    // IMU-derived features (deviation from calibrated neutral)
    float pitch_deviation;   // pitch_deg - neutral_pitch_deg (positive = forward lean)
    float roll_deviation;    // roll_deg  - neutral_roll_deg  (positive = right lean)
    float yaw_deviation;     // yaw_deg   - neutral_yaw_deg   (for twist detection)

    // EMG features
    float emg_filtered;      // Current filtered EMG
    float emg_activity;      // EMG above resting baseline
    bool  emg_high;          // True if EMG activity exceeds threshold

    // Raw values (for telemetry and debugging)
    float pitch_raw;
    float roll_raw;
    float yaw_raw;
    int   emg_raw_adc;

    // Sensor health flags
    bool imu_valid;
    bool emg_valid;
} posture_features_t;

// ==================== Posture Classification ====================

/**
 * @brief Posture state enum.
 * Rule-based, not ML. Keep this simple and readable.
 * TODO: replace with TinyML model when training data is available.
 */
typedef enum {
    POSTURE_GOOD            = 0,
    POSTURE_SLOUCHING       = 1,   // Forward pitch deviation above threshold
    POSTURE_LEANING_FORWARD = 2,   // Similar to slouching but different pitch range
    POSTURE_LEANING_LEFT    = 3,   // Negative roll deviation
    POSTURE_LEANING_RIGHT   = 4,   // Positive roll deviation
    POSTURE_HIGH_TENSION    = 5,   // High EMG activation with otherwise OK posture
    POSTURE_UNKNOWN         = 6,   // Sensor error or unclassifiable state
} posture_state_t;

/**
 * @brief Classification output (result of one classifier run).
 */
typedef struct {
    uint32_t timestamp_ms;
    posture_state_t state;          // Current posture classification
    posture_state_t prev_state;     // Previous state (for detecting transitions)
    uint32_t state_duration_ms;     // How long we've been in current state
    bool correction_triggered;      // True on first frame of a bad-posture event
    const char *state_label;        // Human-readable string (for logging/telemetry)
} posture_classification_t;

// ==================== Posture Classification Thresholds ====================
// All tunable in one place. Adjust after calibration.

#define POSTURE_PITCH_SLOUCH_DEG        15.0f   // Forward lean = bad posture above this
#define POSTURE_PITCH_LEAN_FWD_DEG      30.0f   // More aggressive forward lean
#define POSTURE_ROLL_LEAN_LEFT_DEG     -10.0f   // Left lean (negative = roll left)
#define POSTURE_ROLL_LEAN_RIGHT_DEG     10.0f   // Right lean
#define POSTURE_EMG_HIGH_THRESHOLD      80.0f   // EMG activity units above baseline
#define POSTURE_CONFIRM_WINDOW_MS      750      // Must hold bad posture for this long
                                                // before triggering a correction event
                                                // (avoids single noisy samples triggering)

// ==================== Calibration Baseline ====================

/**
 * @brief Neutral posture calibration values.
 * Captured once at session start, used to compute deviations.
 * Stored in RAM (not NVS) for simplicity in this prototype.
 */
typedef struct {
    float neutral_pitch_deg;    // Pitch while sitting upright and relaxed
    float neutral_roll_deg;     // Roll while sitting upright
    float neutral_yaw_deg;      // Yaw reference (not critical but stored anyway)
    float emg_resting_baseline; // Average EMG ADC value while muscles are relaxed
    bool  calibrated;           // True once calibration has been run
} calibration_t;

// ==================== Session Statistics & Gamification ====================

/**
 * @brief Session-level aggregated stats and gamification state.
 * Updated in the classifier/telemetry task.
 */
typedef struct {
    uint32_t session_start_ms;      // Timestamp when session started

    // Posture time tracking
    uint32_t good_posture_ms;       // Total ms with POSTURE_GOOD
    uint32_t bad_posture_ms;        // Total ms with any bad posture state
    uint32_t correction_count;      // Number of correction events triggered

    // Gamification
    uint32_t points;                // Points awarded for sustained good posture
    uint32_t streak_ms;             // Current consecutive good-posture streak length
    uint32_t best_streak_ms;        // Best streak this session
} session_stats_t;

// ==================== Queue Message Types ====================

typedef enum {
    QUEUE_MSG_DATA,    // Normal sensor data
    QUEUE_MSG_FLUSH,   // Flush/reset signal
    QUEUE_MSG_STOP,    // Stop acquisition
} queue_msg_type_t;

typedef struct {
    queue_msg_type_t type;
    imu_sample_t     data;
} imu_queue_msg_t;

typedef struct {
    queue_msg_type_t type;
    emg_sample_t     data;
} emg_queue_msg_t;

typedef struct {
    queue_msg_type_t type;
    posture_classification_t data;
} posture_queue_msg_t;

// ==================== Utility Function Declarations ====================
// These are implemented in data_types.c and used across all components.
// Every .c file that calls get_timestamp_ms(), calibration_reset(), or
// session_stats_reset() must include data_types.h — which they all do —
// so declaring them here makes them visible everywhere automatically.

uint32_t get_timestamp_ms(void);
void     data_types_init(void);
void     calibration_reset(calibration_t *cal);
void     session_stats_reset(session_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif // DATA_TYPES_H
