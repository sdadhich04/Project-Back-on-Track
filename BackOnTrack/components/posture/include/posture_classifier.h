/**
 * @file posture_classifier.h
 * @brief Rule-based posture classifier for Back on Track
 *
 * NOTE: This is NOT machine learning. This is a deterministic rule-based classifier.
 * Each posture state is decided by comparing posture_features_t values against
 * configurable thresholds defined in data_types.h.
 *
 * TODO: When labeled training data is collected (e.g. via CSV telemetry),
 *       this classifier can be replaced with a TinyML model:
 *       - Train a decision tree or small neural net on PC
 *       - Export via Edge Impulse or TFLite Micro
 *       - Deploy to ESP32-S3 as a compiled model
 *       - Swap out classify_posture() with model_infer()
 *
 * Time-based confirmation window (POSTURE_CONFIRM_WINDOW_MS):
 *   A single noisy sample won't trigger a "bad posture" event.
 *   The classifier only changes state and increments correction_count after
 *   the features have indicated the same bad state for POSTURE_CONFIRM_WINDOW_MS ms.
 *   This prevents false positives from transient movements (reaching for a cup, etc.)
 */

#ifndef POSTURE_CLASSIFIER_H
#define POSTURE_CLASSIFIER_H

#include "data_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the posture classifier.
 * Resets state machine and timing.
 */
void posture_classifier_init(void);

/**
 * @brief Run the rule-based posture classifier on a feature vector.
 *
 * Applies the time-based confirmation window. Updates session stats.
 * Sets correction_triggered = true only on the first frame of a confirmed bad event.
 *
 * @param features   Computed posture feature vector
 * @param stats      Session stats to update (good/bad posture time, points, etc.)
 * @param result     Output classification result
 */
void posture_classify(const posture_features_t     *features,
                       session_stats_t              *stats,
                       posture_classification_t     *result);

/**
 * @brief Convert posture_state_t enum to human-readable string.
 * Returns a constant string — do not free.
 */
const char *posture_state_to_string(posture_state_t state);

/**
 * @brief Update gamification points and streaks based on classification result.
 * Call after posture_classify() each cycle.
 *
 * Points scheme:
 *   +1 point per second of good posture
 *   Streak resets to 0 on any confirmed bad posture event
 *   Best streak is tracked session-wide
 *
 * @param result  Most recent classification result
 * @param stats   Session stats struct to update
 * @param dt_ms   Time elapsed since last call (milliseconds)
 */
void posture_update_gamification(const posture_classification_t *result,
                                  session_stats_t                *stats,
                                  uint32_t                        dt_ms);

#ifdef __cplusplus
}
#endif

#endif // POSTURE_CLASSIFIER_H
