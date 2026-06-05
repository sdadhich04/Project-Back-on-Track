/**
 * @file posture_classifier.c
 * @brief Rule-based posture classifier for Back on Track
 *
 * Applies threshold rules to posture features and uses a confirmation window
 * to avoid triggering correction events from transient movements.
 */

#include "posture_classifier.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "posture_cls";

// ==================== Classifier State ====================

typedef struct {
    posture_state_t  candidate_state;       // State that is "building up" toward confirmation
    uint32_t         candidate_start_ms;    // When we first saw this candidate state
    posture_state_t  confirmed_state;       // Last confirmed (held) state
    uint32_t         confirmed_start_ms;    // When confirmed state began
    bool             correction_fired;      // True if we already fired correction for this event
} classifier_state_t;

static classifier_state_t s_cls = {0};

// ==================== Init ====================

void posture_classifier_init(void) {
    memset(&s_cls, 0, sizeof(s_cls));
    s_cls.candidate_state  = POSTURE_GOOD;
    s_cls.confirmed_state  = POSTURE_GOOD;
    ESP_LOGI(TAG, "Posture classifier initialized");
}

// ==================== Classification Rules ====================

/**
 * @brief Apply rules to feature vector to determine raw posture state.
 *
 * Priority order (first matching rule wins):
 *   1. Sensor invalid -> UNKNOWN
 *   2. High EMG tension (regardless of posture angle) -> HIGH_TENSION
 *   3. Forward pitch (severe) -> LEANING_FORWARD
 *   4. Forward pitch (mild) -> SLOUCHING
 *   5. Roll left -> LEANING_LEFT
 *   6. Roll right -> LEANING_RIGHT
 *   7. All within thresholds -> GOOD
 *
 * NOTE: These rules use simple thresholds. They are NOT ML.
 * TODO: Replace with TinyML inference when training data is available.
 */
static posture_state_t apply_rules(const posture_features_t *f) {
    // Rule 0: Sensor error — can't classify
    if (!f->imu_valid) {
        return POSTURE_UNKNOWN;
    }

    // Rule 1: High muscle tension (user hunching shoulders or tensing)
    // Fires even if pitch/roll look OK — chronic tension is its own feedback target
    if (f->emg_valid && f->emg_high) {
        return POSTURE_HIGH_TENSION;
    }

    // Rule 2: Severe forward lean
    if (f->pitch_deviation >= POSTURE_PITCH_LEAN_FWD_DEG) {
        return POSTURE_LEANING_FORWARD;
    }

    // Rule 3: Mild forward lean / slouch
    if (f->pitch_deviation >= POSTURE_PITCH_SLOUCH_DEG) {
        return POSTURE_SLOUCHING;
    }

    // Rule 4: Left lean
    if (f->roll_deviation <= POSTURE_ROLL_LEAN_LEFT_DEG) {
        return POSTURE_LEANING_LEFT;
    }

    // Rule 5: Right lean
    if (f->roll_deviation >= POSTURE_ROLL_LEAN_RIGHT_DEG) {
        return POSTURE_LEANING_RIGHT;
    }

    // Rule 6: All within bounds — good posture
    return POSTURE_GOOD;
}

// ==================== Public: Classify ====================

void posture_classify(const posture_features_t  *features,
                       session_stats_t           *stats,
                       posture_classification_t  *result) {
    if (!features || !stats || !result) return;

    uint32_t now_ms      = features->timestamp_ms;
    posture_state_t raw  = apply_rules(features);

    // Update the candidate state (time-based confirmation window)
    if (raw != s_cls.candidate_state) {
        // New candidate — reset the confirmation timer
        s_cls.candidate_state    = raw;
        s_cls.candidate_start_ms = now_ms;
        s_cls.correction_fired   = false;
    }

    uint32_t candidate_held_ms = now_ms - s_cls.candidate_start_ms;
    bool confirmed = (candidate_held_ms >= POSTURE_CONFIRM_WINDOW_MS);

    // Transition confirmed state
    if (confirmed && raw != s_cls.confirmed_state) {
        ESP_LOGD(TAG, "State transition: %s -> %s (held %"PRIu32"ms)",
                 posture_state_to_string(s_cls.confirmed_state),
                 posture_state_to_string(raw),
                 candidate_held_ms);
        s_cls.confirmed_state    = raw;
        s_cls.confirmed_start_ms = now_ms;
    }

    // Fill result struct
    result->timestamp_ms     = now_ms;
    result->prev_state       = result->state;      // Save previous before overwriting
    result->state            = s_cls.confirmed_state;
    result->state_duration_ms = now_ms - s_cls.confirmed_start_ms;
    result->state_label      = posture_state_to_string(result->state);

    // Fire correction event on the FIRST frame of a confirmed bad posture
    // (only once per bad posture episode, not every frame)
    bool is_bad = (result->state != POSTURE_GOOD && result->state != POSTURE_UNKNOWN);
    if (is_bad && confirmed && !s_cls.correction_fired) {
        result->correction_triggered = true;
        s_cls.correction_fired       = true;
        stats->correction_count++;
        ESP_LOGI(TAG, "Correction event! State=%s, corrections=%"PRIu32,
                 result->state_label, stats->correction_count);
    } else {
        result->correction_triggered = false;
    }
}

// ==================== Public: Gamification ====================

void posture_update_gamification(const posture_classification_t *result,
                                  session_stats_t                *stats,
                                  uint32_t                        dt_ms) {
    if (!result || !stats) return;

    if (result->state == POSTURE_GOOD) {
        stats->good_posture_ms += dt_ms;
        stats->streak_ms       += dt_ms;

        // +1 point per second of good posture
        // Using integer math: accumulate fractional seconds
        static uint32_t point_accumulator_ms = 0;
        point_accumulator_ms += dt_ms;
        if (point_accumulator_ms >= 1000) {
            stats->points++;
            point_accumulator_ms -= 1000;
        }

        if (stats->streak_ms > stats->best_streak_ms) {
            stats->best_streak_ms = stats->streak_ms;
        }
    } else if (result->state != POSTURE_UNKNOWN) {
        stats->bad_posture_ms += dt_ms;

        // Break the streak on confirmed bad posture
        if (result->correction_triggered) {
            stats->streak_ms = 0;
        }
    }
}

// ==================== Helper ====================

const char *posture_state_to_string(posture_state_t state) {
    switch (state) {
        case POSTURE_GOOD:            return "GOOD";
        case POSTURE_SLOUCHING:       return "SLOUCHING";
        case POSTURE_LEANING_FORWARD: return "LEANING_FORWARD";
        case POSTURE_LEANING_LEFT:    return "LEANING_LEFT";
        case POSTURE_LEANING_RIGHT:   return "LEANING_RIGHT";
        case POSTURE_HIGH_TENSION:    return "HIGH_TENSION";
        case POSTURE_UNKNOWN:         return "UNKNOWN";
        default:                      return "INVALID";
    }
}
