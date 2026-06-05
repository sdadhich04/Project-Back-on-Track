/**
 * @file driver_bno085.h
 * @brief BNO085 IMU driver header for Back on Track (SPI, ESP-IDF v5.x)
 *
 * Public API for the BNO085 orientation sensor driver.
 * The BNO085 communicates over SPI3_HOST using SHTP (Sensor Hub Transport Protocol).
 * This driver enables the Game Rotation Vector report to get quaternion-based orientation,
 * which is converted to pitch/roll/yaw for posture classification.
 *
 * WIRING (from data_types.h pin defines):
 *   BNO085 CS   -> GPIO37
 *   BNO085 SCLK -> GPIO38
 *   BNO085 MOSI -> GPIO40  (labeled DI on Adafruit BNO085 board)
 *   BNO085 MISO -> GPIO39  (labeled SDA on Adafruit BNO085 board)
 *   BNO085 INT  -> GPIO5   (MUST have 10K pull-up to 3.3V)
 *   BNO085 RST  -> GPIO6
 *   BNO085 PS0  -> 3.3V    (tie HIGH for SPI mode)
 *   BNO085 PS1  -> 3.3V    (tie HIGH for SPI mode)
 */

#ifndef DRIVER_BNO085_H
#define DRIVER_BNO085_H

#include "sensor_hal.h"
#include "data_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the BNO085 over SPI.
 *
 * This function:
 *   1. Configures RST (output) and INT (input with pull-up) GPIO pins
 *   2. Initializes the SPI bus and adds the BNO085 device at 3 MHz
 *   3. Performs hardware reset + soft reset via SHTP EXE channel
 *   4. Waits for Product ID response (up to 3 retries)
 *   5. Enables the Game Rotation Vector report at IMU_SAMPLE_RATE_HZ
 *   6. Falls back to accelerometer report if Game RV is unsupported
 *
 * @param ctx Sensor context. ctx->hw_config must point to a bno085_spi_config_t.
 * @return true on success, false on failure (logs reason with ESP_LOGE)
 */
bool bno085_init(SensorContext_t *ctx);

/**
 * @brief Read the latest orientation sample from BNO085.
 *
 * Waits up to 20ms for INT pin to assert, then reads one SHTP packet.
 * Parses Game Rotation Vector report and converts quaternion to pitch/roll/yaw.
 * Falls back to accelerometer magnitude if only accel data is available.
 *
 * @param ctx Sensor context (must be initialized with bno085_init first)
 * @param out Pointer to imu_sample_t to fill. Will contain last valid reading on timeout.
 * @return true if a new sample was parsed; false on hard error (driver not initialized)
 */
bool bno085_read_orientation(SensorContext_t *ctx, imu_sample_t *out);

/**
 * @brief Compatibility wrapper matching SensorContext_t read_sample signature.
 * Calls bno085_read_orientation internally. data_out must point to imu_sample_t.
 */
bool bno085_read_sample(SensorContext_t *ctx, void *data_out);

#ifdef __cplusplus
}
#endif

#endif // DRIVER_BNO085_H
