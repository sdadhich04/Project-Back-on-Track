/**
 * @file posture_classifier.c
 * @brief Dual-IMU rule-based posture classifier for Back on Track
 *
 * New state vs single-IMU version:
 *   POSTURE_LUMBAR_COLLAPSE — fires when spinal_flexion_deg exceeds
 *   POSTURE_LUMBAR_COLLAPSE_DEG. Detects lower-back rounding even when
 *   the upper back / shoulders look fine.
 *
 * Rule priority order:
 *   1. Sensor invalid → UNKNOWN
 *   2. High EMG tension → HIGH_TENSION
 *   3. Lumbar collapse (IMU1 >> IMU0 pitch) → LUMBAR_COLLAPSE
 *   4. Severe whole-body forward lean → LEANING_FORWARD
 *   5. Mild forward lean (upper back) → SLOUCHING
 *   6. Left lean → LEANING_LEFT
 *   7. Right lean → LEANING_RIGHT
 *   8. All OK → GOOD
 */

#include "posture_classifier.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "posture_cls";

typedef struct {
    posture_state_t  candidate_state;
    uint32_t         candidate_start_ms;
    posture_state_t  confirmed_state;
    uint32_t         confirmed_start_ms;
    bool             correction_fired;
} classifier_state_t;

static classifier_state_t s_cls = {0};

void posture_classifier_init(void) {
    memset(&s_cls, 0, sizeof(s_cls));
    s_cls.candidate_state = POSTURE_GOOD;
    s_cls.confirmed_state = POSTURE_GOOD;
    ESP_LOGI(TAG, "Dual-IMU posture classifier initialized");
}

static posture_state_t apply_rules(const posture_features_t *f) {
    // Rule 0: sensor error
    if (!f->upper_imu_valid && !f->lower_imu_valid) {
        return POSTURE_UNKNOWN;
    }

    // Rule 1: high muscle tension
    if (f->emg_valid && f->emg_high) {
        return POSTURE_HIGH_TENSION;
    }

    // Rule 2: lumbar collapse — lower back rounds forward relative to upper back
    // Only fires when BOTH IMUs are valid and spinal flexion is excessive.
    if (f->spinal_flexion_valid
        && f->spinal_flexion_deg >= POSTURE_LUMBAR_COLLAPSE_DEG) {
        return POSTURE_LUMBAR_COLLAPSE;
    }

    // Rules 3–7 use upper IMU (IMU0) only — whole-body posture
    if (f->upper_imu_valid) {
        if (f->upper_pitch_deviation >= POSTURE_PITCH_LEAN_FWD_DEG) {
            return POSTURE_LEANING_FORWARD;
        }
        if (f->upper_pitch_deviation >= POSTURE_PITCH_SLOUCH_DEG) {
            return POSTURE_SLOUCHING;
        }
        if (f->upper_roll_deviation <= POSTURE_ROLL_LEAN_LEFT_DEG) {
            return POSTURE_LEANING_LEFT;
        }
        if (f->upper_roll_deviation >= POSTURE_ROLL_LEAN_RIGHT_DEG) {
            return POSTURE_LEANING_RIGHT;
        }
        // Combined/diagonal lean: total tilt is large even though neither the
        // forward nor the lateral axis alone crossed its threshold. Flag as a
        // generic bad posture (reported as SLOUCHING).
        if (f->upper_lean_deg >= POSTURE_LEAN_ANY_DEG) {
            return POSTURE_SLOUCHING;
        }
    }

    return POSTURE_GOOD;
}

void posture_classify(const posture_features_t  *features,
                       session_stats_t           *stats,
                       posture_classification_t  *result) {
    if (!features || !stats || !result) return;

    uint32_t now_ms     = features->timestamp_ms;
    posture_state_t raw = apply_rules(features);

    if (raw != s_cls.candidate_state) {
        s_cls.candidate_state    = raw;
        s_cls.candidate_start_ms = now_ms;
        s_cls.correction_fired   = false;
    }

    uint32_t held    = now_ms - s_cls.candidate_start_ms;
    bool confirmed   = (held >= POSTURE_CONFIRM_WINDOW_MS);

    if (confirmed && raw != s_cls.confirmed_state) {
        ESP_LOGD(TAG, "State: %s -> %s (held %"PRIu32"ms)",
                 posture_state_to_string(s_cls.confirmed_state),
                 posture_state_to_string(raw), held);
        s_cls.confirmed_state    = raw;
        s_cls.confirmed_start_ms = now_ms;
    }

    result->timestamp_ms      = now_ms;
    result->prev_state        = result->state;
    result->state             = s_cls.confirmed_state;
    result->state_duration_ms = now_ms - s_cls.confirmed_start_ms;
    result->state_label       = posture_state_to_string(result->state);

    bool is_bad = (result->state != POSTURE_GOOD && result->state != POSTURE_UNKNOWN);
    if (is_bad && confirmed && !s_cls.correction_fired) {
        result->correction_triggered = true;
        s_cls.correction_fired       = true;
        stats->correction_count++;
        ESP_LOGI(TAG, "Correction! State=%s count=%"PRIu32,
                 result->state_label, stats->correction_count);
    } else {
        result->correction_triggered = false;
    }
}

void posture_update_gamification(const posture_classification_t *result,
                                  session_stats_t                *stats,
                                  uint32_t                        dt_ms) {
    if (!result || !stats) return;

    if (result->state == POSTURE_GOOD) {
        stats->good_posture_ms += dt_ms;
        stats->streak_ms       += dt_ms;
        static uint32_t acc = 0;
        acc += dt_ms;
        if (acc >= 1000) { stats->points++; acc -= 1000; }
        if (stats->streak_ms > stats->best_streak_ms) {
            stats->best_streak_ms = stats->streak_ms;
        }
    } else if (result->state != POSTURE_UNKNOWN) {
        stats->bad_posture_ms += dt_ms;
        if (result->correction_triggered) stats->streak_ms = 0;
    }
}

const char *posture_state_to_string(posture_state_t state) {
    switch (state) {
        case POSTURE_GOOD:            return "GOOD";
        case POSTURE_SLOUCHING:       return "SLOUCHING";
        case POSTURE_LEANING_FORWARD: return "LEANING_FORWARD";
        case POSTURE_LEANING_LEFT:    return "LEANING_LEFT";
        case POSTURE_LEANING_RIGHT:   return "LEANING_RIGHT";
        case POSTURE_HIGH_TENSION:    return "HIGH_TENSION";
        case POSTURE_LUMBAR_COLLAPSE: return "LUMBAR_COLLAPSE";
        case POSTURE_UNKNOWN:         return "UNKNOWN";
        default:                      return "INVALID";
    }
}