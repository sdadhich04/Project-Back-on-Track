/**
 * @file sensor_hal.h
 * @brief Sensor Hardware Abstraction Layer for Back on Track
 *
 * Defines hardware configuration structs and the polymorphic SensorContext_t
 * used across all drivers. Adapted from the Project SHIELD sensor_hal pattern.
 *
 * Usage pattern:
 *   - Each sensor gets a hardware config struct (spi_config_t, myoware_adc_config_t, etc.)
 *   - Each sensor gets a SensorContext_t wired up with init/read function pointers
 *   - Tasks call ctx->init(ctx) and ctx->read_sample(ctx, data_out)
 *
 * C/C++ boundary: use extern "C" {} when including from .cpp files.
 */

#ifndef SENSOR_HAL_H
#define SENSOR_HAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==================== Sensor Type Enum ====================

typedef enum {
    SENSOR_TYPE_IMU_ORIENTATION,  // BNO085 - rotation vector / orientation
    SENSOR_TYPE_IMU_ACCEL,        // BNO085 - raw accelerometer fallback
    SENSOR_TYPE_EMG,              // MyoWare 2.0 - muscle activation
} sensor_type_t;

// ==================== SensorContext_t ====================
// Polymorphic sensor descriptor. Acts like a base class with vtable pointers.

typedef struct SensorContext {
    int             id;                 // Unique sensor ID
    sensor_type_t   type;               // Sensor type (for logging / routing)
    int             sampling_rate_hz;   // Target sample rate in Hz
    bool            enabled;            // false = skip init and acquisition entirely
    void           *hw_config;          // Pointer to hardware config (spi_config_t or myoware_adc_config_t)

    // "Virtual" function pointers — each driver implements these
    bool (*init)(struct SensorContext *ctx);
    bool (*read_sample)(struct SensorContext *ctx, void *data_out);
} SensorContext_t;

// ==================== BNO085 SPI Hardware Config ====================
// Directly adapted from Project SHIELD sensor_hal.h spi_config_t.
// Pin assignments match the Shield reference design.

typedef struct {
    int spi_host;   // Use SPI3_HOST (=2) for BNO085
    int cs_pin;     // GPIO37 (active low chip select)
    int sclk_pin;   // GPIO38 (SPI clock)
    int mosi_pin;   // GPIO40 (MOSI = DI on BNO085 board)
    int miso_pin;   // GPIO39 (MISO = SDA on BNO085 board)
    int int_pin;    // GPIO5  (active-low data-ready; MUST have pull-up)
    int rst_pin;    // GPIO6  (active-low hardware reset; output from ESP32-S3)
} bno085_spi_config_t;

// ==================== MyoWare 2.0 ADC Hardware Config ====================
// MyoWare 2.0 outputs an analog envelope signal proportional to muscle activation.
// This config selects which ESP32-S3 ADC unit, channel, and GPIO to use.

typedef struct {
    int adc_unit;       // ADC unit: 0 = ADC_UNIT_1, 1 = ADC_UNIT_2
                        // Prefer ADC_UNIT_1 (ADC2 conflicts with WiFi/BLE)
    int adc_channel;    // ADC channel number (e.g. ADC_CHANNEL_3 for GPIO4)
    int gpio_pin;       // GPIO number for documentation/wiring reference
                        // GPIO1 = ADC1_CH0, GPIO2 = ADC1_CH1, ... GPIO10 = ADC1_CH9
                        // Recommended: GPIO4 (ADC1_CH3) — away from BNO085 pins
    float vref_mv;      // Reference voltage in millivolts (typically 3300 for 3.3V)
    int atten;          // ADC attenuation: use ADC_ATTEN_DB_11 for 0–3.1V range
} myoware_adc_config_t;

#ifdef __cplusplus
}
#endif

#endif // SENSOR_HAL_H
