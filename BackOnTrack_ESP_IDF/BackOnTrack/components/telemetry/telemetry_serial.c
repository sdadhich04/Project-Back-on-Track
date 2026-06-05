/**
 * @file telemetry_serial.c
 * @brief Serial telemetry output implementation for Back on Track
 */

#include "telemetry_serial.h"
#include "posture_classifier.h"
#include "esp_log.h"
#include <stdio.h>
#include <inttypes.h>

static const char *TAG = "telemetry";
static bool s_header_printed = false;

void telemetry_init(void) {
    s_header_printed = false;
    ESP_LOGI(TAG, "Serial telemetry initialized (mode=%s)",
             TELEMETRY_OUTPUT_MODE == 0 ? "CSV" : "JSON");
}

void telemetry_print_banner(void) {
    printf("\r\n");
    printf("========================================\r\n");
    printf("  Back on Track — Posture Monitor\r\n");
    printf("  MCU: ESP32-S3  |  IMU: BNO085 SPI\r\n");
    printf("  EMG: MyoWare 2.0 | Classifier: Rule-Based\r\n");
    printf("  DISCLAIMER: Not a medical device.\r\n");
    printf("  For posture awareness and habit building only.\r\n");
    printf("========================================\r\n");
    printf("\r\n");
}

void telemetry_print(const posture_features_t      *features,
                     const posture_classification_t *result,
                     const session_stats_t          *stats) {
    if (!features || !result || !stats) return;

#if TELEMETRY_OUTPUT_MODE == 0
    // ==================== CSV Mode ====================
    // Header on first call (Arduino Serial Plotter uses this for axis labels)
    if (!s_header_printed) {
        printf("timestamp_ms,state_id,pitch_dev,roll_dev,yaw_dev,"
               "emg_raw,emg_filt,emg_activity,"
               "good_ms,bad_ms,corrections,points,streak_ms\r\n");
        s_header_printed = true;
    }

    printf("%"PRIu32",%d,%.2f,%.2f,%.2f,%d,%.1f,%.1f,%"PRIu32",%"PRIu32",%"PRIu32",%"PRIu32",%"PRIu32"\r\n",
           features->timestamp_ms,
           (int)result->state,
           features->pitch_deviation,
           features->roll_deviation,
           features->yaw_deviation,
           features->emg_raw_adc,
           features->emg_filtered,
           features->emg_activity,
           stats->good_posture_ms,
           stats->bad_posture_ms,
           stats->correction_count,
           stats->points,
           stats->streak_ms);

#else
    // ==================== JSON Mode ====================
    // One JSON object per line — easy to parse on the receiving end
    printf("{\"t\":%"PRIu32","
           "\"state\":\"%s\","
           "\"pitch_d\":%.2f,"
           "\"roll_d\":%.2f,"
           "\"yaw_d\":%.2f,"
           "\"pitch_r\":%.2f,"
           "\"roll_r\":%.2f,"
           "\"emg_raw\":%d,"
           "\"emg_filt\":%.1f,"
           "\"emg_act\":%.1f,"
           "\"emg_hi\":%d,"
           "\"good_ms\":%"PRIu32","
           "\"bad_ms\":%"PRIu32","
           "\"corrections\":%"PRIu32","
           "\"points\":%"PRIu32","
           "\"streak_ms\":%"PRIu32","
           "\"corr_event\":%d}\r\n",
           features->timestamp_ms,
           result->state_label,
           features->pitch_deviation,
           features->roll_deviation,
           features->yaw_deviation,
           features->pitch_raw,
           features->roll_raw,
           features->emg_raw_adc,
           features->emg_filtered,
           features->emg_activity,
           features->emg_high ? 1 : 0,
           stats->good_posture_ms,
           stats->bad_posture_ms,
           stats->correction_count,
           stats->points,
           stats->streak_ms,
           result->correction_triggered ? 1 : 0);
#endif
}
