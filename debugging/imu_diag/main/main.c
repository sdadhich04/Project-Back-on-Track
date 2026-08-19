/**
 * @file main.c
 * @brief Entry point for the IMU hardware diagnostic project.
 *
 * This project is a standalone ESP-IDF application that runs the
 * five-stage IMU0 hardware diagnosis and then halts. It has no BLE,
 * no posture classification, and no sensor HAL beyond what the
 * diagnostic needs.
 *
 * To run:
 *   idf.py set-target esp32s3
 *   idf.py build flash monitor
 *
 * To return to Back on Track:
 *   Flash the original project. This project does not modify NVS or
 *   any persistent chip state.
 */

#include "imu_diag.h"

void app_main(void) {
    imu_diag_run();
}
