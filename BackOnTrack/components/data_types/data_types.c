/**
 * @file data_types.c
 * @brief Data types utility implementations for Back on Track.
 * Provides timestamp helper and module init.
 */

#include "data_types.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "data_types";

/**
 * @brief Return current system uptime in milliseconds.
 * Uses esp_timer_get_time() (microseconds) for precision.
 */
uint32_t get_timestamp_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/**
 * @brief Initialize data types module.
 * Currently a no-op; reserved for future use (e.g., NVS calibration load).
 */
void data_types_init(void) {
    ESP_LOGI(TAG, "data_types_init OK");
}

/**
 * @brief Initialize a calibration struct to "uncalibrated" defaults.
 * Call before the calibration sequence starts.
 */
void calibration_reset(calibration_t *cal) {
    if (cal == NULL) return;
    memset(cal, 0, sizeof(calibration_t));
    cal->calibrated = false;
}

/**
 * @brief Initialize session stats to zero.
 * Call at the start of each new posture-monitoring session.
 */
void session_stats_reset(session_stats_t *stats) {
    if (stats == NULL) return;
    memset(stats, 0, sizeof(session_stats_t));
    stats->session_start_ms = get_timestamp_ms();
}
