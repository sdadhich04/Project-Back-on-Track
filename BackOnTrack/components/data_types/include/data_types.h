/**
 * @file data_types.h
 * @brief Shared data structures for Back on Track
 *
 * DUAL-IMU VERSION:
 *   IMU0 = upper back / chest  (existing pins: CS=37, INT=5,  RST=6)
 *   IMU1 = lower back / lumbar (new pins:      CS=33, INT=35, RST=36)
 *   Both share the same SPI3_HOST bus (MOSI=40, MISO=39, SCLK=38).
 *
 *   The two sensors together allow detection of spinal flexion — the angle
 *   between upper and lower back — which is more meaningful than a single
 *   IMU alone (a single sensor can't distinguish "whole-body forward lean"
 *   from "lumbar collapse with upright shoulders").
 */

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==================== Queue Sizes ====================

#define IMU_QUEUE_SIZE          20
#define EMG_QUEUE_SIZE          40
#define POSTURE_QUEUE_SIZE      10
#define TELEMETRY_QUEUE_SIZE    10

// ==================== IMU Sensor IDs ====================

#define IMU_ID_UPPER    0   // IMU0: upper back / chest
#define IMU_ID_LOWER    1   // IMU1: lower back / lumbar
#define IMU_COUNT       2   // Total number of IMUs in the system

// ==================== Pin Configuration ====================
// All hardware pins in one place. Change here, propagates everywhere.

// --- IMU0: Upper back / chest ---
// Existing pins from original design.
#define BNO085_0_PIN_CS      37
#define BNO085_0_PIN_INT      5   // MUST have 10KΩ pull-up to 3.3V
#define BNO085_0_PIN_RST      6

// --- IMU1: Lower back / lumbar ---
// New pins. Shares the same SPI bus (MOSI/MISO/SCLK) as IMU0.
// GPIO33, 35, 36 are free on DevKitC-1 and ADC-safe for SPI use.
// Verify against your board's pinout before wiring.
#define BNO085_1_PIN_CS      47
#define BNO085_1_PIN_INT     35   // MUST have 10KΩ pull-up to 3.3V
#define BNO085_1_PIN_RST     36

// --- Shared SPI bus (both IMUs) ---
#define BNO085_PIN_SCLK     38
#define BNO085_PIN_MOSI     40
#define BNO085_PIN_MISO     39
#define BNO085_SPI_HOST      2   // SPI3_HOST = 2

// Legacy aliases so old code still compiles without changes
#define BNO085_PIN_CS       BNO085_0_PIN_CS
#define BNO085_PIN_INT      BNO085_0_PIN_INT
#define BNO085_PIN_RST      BNO085_0_PIN_RST

// --- MyoWare 2.0 ADC ---
#define MYOWARE_GPIO_PIN     4
#define MYOWARE_ADC_UNIT     0
#define MYOWARE_ADC_CHANNEL  3

// ==================== Sample Rate Configuration ====================

#define IMU_SAMPLE_RATE_HZ       50
#define EMG_SAMPLE_RATE_HZ      100
#define TELEMETRY_RATE_HZ        10
#define CLASSIFICATION_RATE_HZ   10

// ==================== IMU Orientation Sample ====================

/**
 * @brief BNO085 orientation sample.
 * sensor_id tells you which physical IMU produced this sample:
 *   IMU_ID_UPPER (0) = upper back / chest
 *   IMU_ID_LOWER (1) = lower back / lumbar
 */
typedef struct {
    uint32_t timestamp_ms;
    uint8_t  sensor_id;     // IMU_ID_UPPER or IMU_ID_LOWER

    // Raw quaternion (Game Rotation Vector report)
    float q_i;
    float q_j;
    float q_k;
    float q_r;

    // Euler angles derived from quaternion
    float pitch_deg;
    float roll_deg;
    float yaw_deg;

    // Fallback: accelerometer magnitude if quaternion unavailable
    float accel_magnitude;

    bool valid;
} imu_sample_t;

// ==================== EMG Sample ====================

typedef struct {
    uint32_t timestamp_ms;
    int     raw_adc;
    float   voltage_mv;
    float   filtered;
    float   activity;
    bool    high_activation;
    bool    valid;
} emg_sample_t;

// ==================== Posture Features ====================

/**
 * @brief Combined dual-IMU + EMG posture feature vector.
 *
 * Upper (IMU0) features capture thoracic/shoulder posture.
 * Lower (IMU1) features capture lumbar posture.
 * spinal_flexion_deg is the computed angle BETWEEN the two sensors —
 * the most clinically meaningful single number for "spine bent" detection.
 */
typedef struct {
    uint32_t timestamp_ms;

    // --- Upper IMU (IMU0) features ---
    float upper_pitch_deviation;  // forward lean of upper back vs calibrated neutral
    float upper_roll_deviation;   // lateral lean of upper back
    float upper_yaw_deviation;    // twist of upper back
    float upper_pitch_raw;
    float upper_roll_raw;
    float upper_yaw_raw;
    bool  upper_imu_valid;

    // --- Lower IMU (IMU1) features ---
    float lower_pitch_deviation;  // forward lean of lower back vs calibrated neutral
    float lower_roll_deviation;   // lateral lean of lower back
    float lower_yaw_deviation;
    float lower_pitch_raw;
    float lower_roll_raw;
    float lower_yaw_raw;
    bool  lower_imu_valid;

    // --- Spinal flexion (computed from both IMUs) ---
    // Angle between IMU0 and IMU1 pitch readings, relative to calibrated neutral.
    // Positive = spinal flexion (bending forward).
    // This is the primary "slouching" signal with dual IMUs.
    float spinal_flexion_deg;
    bool  spinal_flexion_valid;  // true only if BOTH IMUs are valid

    // --- EMG features ---
    float emg_filtered;
    float emg_activity;
    bool  emg_high;
    int   emg_raw_adc;
    bool  emg_valid;
} posture_features_t;

// ==================== Posture Classification ====================

typedef enum {
    POSTURE_GOOD              = 0,
    POSTURE_SLOUCHING         = 1,   // Upper back forward, detected via IMU0
    POSTURE_LEANING_FORWARD   = 2,   // Severe whole-body forward lean
    POSTURE_LEANING_LEFT      = 3,
    POSTURE_LEANING_RIGHT     = 4,
    POSTURE_HIGH_TENSION      = 5,   // High EMG with otherwise OK angles
    POSTURE_LUMBAR_COLLAPSE   = 6,   // Lower back rounding (IMU1 pitch >> IMU0 pitch)
    POSTURE_UNKNOWN           = 7,   // Sensor error
} posture_state_t;

typedef struct {
    uint32_t timestamp_ms;
    posture_state_t state;
    posture_state_t prev_state;
    uint32_t state_duration_ms;
    bool correction_triggered;
    const char *state_label;
} posture_classification_t;

// ==================== Posture Classification Thresholds ====================

#define POSTURE_PITCH_SLOUCH_DEG         15.0f
#define POSTURE_PITCH_LEAN_FWD_DEG       30.0f
#define POSTURE_ROLL_LEAN_LEFT_DEG      -10.0f
#define POSTURE_ROLL_LEAN_RIGHT_DEG      10.0f
#define POSTURE_EMG_HIGH_THRESHOLD       80.0f
#define POSTURE_CONFIRM_WINDOW_MS        750

// Lumbar collapse: lower back pitches forward MORE than upper back by this margin.
// e.g. if IMU1 pitch is 12° more forward than IMU0 after neutral correction,
// the user is rounding their lower back even if shoulders look fine.
#define POSTURE_LUMBAR_COLLAPSE_DEG      12.0f

// ==================== Calibration Baseline ====================

/**
 * @brief Dual-IMU calibration baseline.
 * Stores neutral angles for BOTH IMUs independently.
 */
typedef struct {
    // IMU0 (upper back) neutral
    float neutral_upper_pitch_deg;
    float neutral_upper_roll_deg;
    float neutral_upper_yaw_deg;

    // IMU1 (lower back) neutral
    float neutral_lower_pitch_deg;
    float neutral_lower_roll_deg;
    float neutral_lower_yaw_deg;

    // Derived: spinal flexion at neutral (usually ~0 but stored anyway)
    float neutral_spinal_flexion_deg;

    // EMG
    float emg_resting_baseline;

    bool  calibrated;
} calibration_t;

// ==================== Session Statistics ====================

typedef struct {
    uint32_t session_start_ms;
    uint32_t good_posture_ms;
    uint32_t bad_posture_ms;
    uint32_t correction_count;
    uint32_t points;
    uint32_t streak_ms;
    uint32_t best_streak_ms;
} session_stats_t;

// ==================== Queue Message Types ====================

typedef enum {
    QUEUE_MSG_DATA,
    QUEUE_MSG_FLUSH,
    QUEUE_MSG_STOP,
} queue_msg_type_t;

typedef struct {
    queue_msg_type_t type;
    imu_sample_t     data;   // sensor_id field tells you which IMU sent this
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

uint32_t get_timestamp_ms(void);
void     data_types_init(void);
void     calibration_reset(calibration_t *cal);
void     session_stats_reset(session_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif // DATA_TYPES_H
