/**
 * @file telemetry_serial.c
 * @brief Serial telemetry — dual-IMU version
 *
 * CSV columns (TELEMETRY_OUTPUT_MODE 0):
 *   timestamp_ms, state_id, upper_pitch_dev, upper_roll_dev,
 *   lower_pitch_dev, lower_roll_dev, spinal_flexion,
 *   emg_raw, emg_filt, emg_activity,
 *   good_ms, bad_ms, corrections, points, streak_ms
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
    ESP_LOGI(TAG, "Serial telemetry initialized (dual-IMU, mode=%s)",
             TELEMETRY_OUTPUT_MODE == 0 ? "CSV" : "JSON");
}

void telemetry_print_banner(void) {
    printf("\r\n");
    printf("==============================================\r\n");
    printf("  Back on Track — Dual-IMU Posture Monitor\r\n");
    printf("  IMU0: upper back  |  IMU1: lower back\r\n");
    printf("  EMG: MyoWare 2.0  |  Classifier: Rule-Based\r\n");
    printf("  DISCLAIMER: Not a medical device.\r\n");
    printf("==============================================\r\n\r\n");
}

void telemetry_print(const posture_features_t      *features,
                     const posture_classification_t *result,
                     const session_stats_t          *stats) {
    if (!features || !result || !stats) return;

#if TELEMETRY_OUTPUT_MODE == 0
    // CSV mode
    if (!s_header_printed) {
        printf("timestamp_ms,state_id,"
               "upper_pitch_dev,upper_roll_dev,"
               "lower_pitch_dev,lower_roll_dev,"
               "spinal_flexion,"
               "emg_raw,emg_filt,emg_activity,"
               "good_ms,bad_ms,corrections,points,streak_ms\r\n");
        s_header_printed = true;
    }
    printf("%"PRIu32",%d,"
           "%.2f,%.2f,"
           "%.2f,%.2f,"
           "%.2f,"
           "%d,%.1f,%.1f,"
           "%"PRIu32",%"PRIu32",%"PRIu32",%"PRIu32",%"PRIu32"\r\n",
           features->timestamp_ms,
           (int)result->state,
           features->upper_pitch_deviation, features->upper_roll_deviation,
           features->lower_pitch_deviation, features->lower_roll_deviation,
           features->spinal_flexion_deg,
           features->emg_raw_adc, features->emg_filtered, features->emg_activity,
           stats->good_posture_ms, stats->bad_posture_ms,
           stats->correction_count, stats->points, stats->streak_ms);
#else
    // JSON mode
    printf("{\"t\":%"PRIu32","
           "\"state\":\"%s\","
           "\"up_pitch\":%.2f,\"up_roll\":%.2f,"
           "\"lo_pitch\":%.2f,\"lo_roll\":%.2f,"
           "\"flexion\":%.2f,"
           "\"emg_raw\":%d,\"emg_filt\":%.1f,\"emg_act\":%.1f,"
           "\"good_ms\":%"PRIu32",\"bad_ms\":%"PRIu32","
           "\"corrections\":%"PRIu32",\"points\":%"PRIu32","
           "\"streak\":%"PRIu32",\"corr_event\":%d}\r\n",
           features->timestamp_ms,
           result->state_label,
           features->upper_pitch_deviation, features->upper_roll_deviation,
           features->lower_pitch_deviation, features->lower_roll_deviation,
           features->spinal_flexion_deg,
           features->emg_raw_adc, features->emg_filtered, features->emg_activity,
           stats->good_posture_ms, stats->bad_posture_ms,
           stats->correction_count, stats->points,
           stats->streak_ms, result->correction_triggered ? 1 : 0);
#endif
}
