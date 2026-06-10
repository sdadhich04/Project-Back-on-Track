/**
 * @file driver_bno085.cpp
 * @brief BNO085 driver for Back on Track — now backed by the esp32_BNO08x library.
 *
 * This replaces the hand-rolled SHTP-over-SPI driver with a thin wrapper over the
 * myles-parfeniuk `esp32_BNO08x` component (CEVA SH-2 stack, interrupt-driven).
 *
 * The PUBLIC C API is unchanged, so main.cpp / posture / telemetry need no edits:
 *   - bno085_init(ctx)
 *   - bno085_read_orientation(ctx, out)
 *   - bno085_read_sample(ctx, data_out)
 *
 * Dual-IMU shared bus:
 *   Both IMUs share one SPI host (SPI3_HOST), each with its own CS/INT/RST.
 *   The library normally calls spi_bus_initialize() per instance; we patched
 *   init_spi() to tolerate an already-initialized bus (ESP_ERR_INVALID_STATE),
 *   and only the FIRST IMU (id == IMU_ID_UPPER) installs the GPIO ISR service.
 *
 * Orientation:
 *   We read the Game Rotation Vector quaternion via get_quat() and derive
 *   pitch/roll/yaw with the SAME formulas the old driver used, so the posture
 *   calibration thresholds remain valid.
 */

#include "driver_bno085.h"   // declares the extern "C" API + pulls in data_types.h
#include "data_types.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#include "BNO08x.hpp"        // the library (C++)

static const char *TAG = "bno085";

#define BNO085_MAX_IMUS 2

// One library instance per physical IMU, indexed by ctx->id (0 = upper, 1 = lower).
static BNO08x      *s_imu[BNO085_MAX_IMUS]         = { nullptr, nullptr };
static bool         s_initialized[BNO085_MAX_IMUS] = { false, false };
static imu_sample_t s_last[BNO085_MAX_IMUS];

// Quaternion -> Euler (degrees), identical convention to the original driver so
// the posture classifier's calibrated neutral angles still line up.
static inline void quat_to_euler(float qr, float qi, float qj, float qk, imu_sample_t *out)
{
    float sinp = 2.0f * (qr * qj - qk * qi);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    out->pitch_deg = asinf(sinp) * (180.0f / (float)M_PI);

    out->roll_deg = atan2f(2.0f * (qr * qi + qj * qk),
                           1.0f - 2.0f * (qi * qi + qj * qj)) * (180.0f / (float)M_PI);

    out->yaw_deg  = atan2f(2.0f * (qr * qk + qi * qj),
                           1.0f - 2.0f * (qj * qj + qk * qk)) * (180.0f / (float)M_PI);
}

// A freshly-enabled report caches an all-zero quaternion until the first packet
// arrives; use that to distinguish "no data yet" from a real reading.
static inline bool quat_is_valid(const bno08x_quat_t &q)
{
    return (q.real != 0.0f) || (q.i != 0.0f) || (q.j != 0.0f) || (q.k != 0.0f);
}

extern "C" bool bno085_init(SensorContext_t *ctx)
{
    if (!ctx || !ctx->hw_config) {
        ESP_LOGE(TAG, "bno085_init: ctx or hw_config is NULL");
        return false;
    }
    if (ctx->id < 0 || ctx->id >= BNO085_MAX_IMUS) {
        ESP_LOGE(TAG, "bno085_init: ctx->id=%d out of range (0..%d)", ctx->id, BNO085_MAX_IMUS - 1);
        return false;
    }

    const int id = ctx->id;
    bno085_spi_config_t *cfg = (bno085_spi_config_t *)ctx->hw_config;

    ESP_LOGI(TAG, "IMU%d init via esp32_BNO08x (host=%d CS=%d INT=%d RST=%d MOSI=%d MISO=%d SCLK=%d)",
             id, cfg->spi_host, cfg->cs_pin, cfg->int_pin, cfg->rst_pin,
             cfg->mosi_pin, cfg->miso_pin, cfg->sclk_pin);

    // Build the library config from our HAL config.
    //  - 1 MHz SCLK: conservative for breadboard wiring (library max is 3 MHz).
    //  - install_isr_service only on the FIRST IMU; the shared GPIO ISR service
    //    is installed once, the second IMU just registers its own pin handler.
    bno08x_config_t lib_cfg(
        (spi_host_device_t)cfg->spi_host,
        (gpio_num_t)cfg->mosi_pin,
        (gpio_num_t)cfg->miso_pin,
        (gpio_num_t)cfg->sclk_pin,
        (gpio_num_t)cfg->cs_pin,
        (gpio_num_t)cfg->int_pin,
        (gpio_num_t)cfg->rst_pin,
        1000000UL,
        (id == IMU_ID_UPPER) /* install_isr_service */);

    if (s_imu[id] == nullptr) {
        s_imu[id] = new BNO08x(lib_cfg);
        if (s_imu[id] == nullptr) {
            ESP_LOGE(TAG, "IMU%d: allocation failed", id);
            return false;
        }
    }
    BNO08x *imu = s_imu[id];

    if (!imu->initialize()) {
        ESP_LOGE(TAG, "IMU%d: BNO08x initialize() failed (check wiring / PS0,PS1=3V3 for SPI)", id);
        return false;
    }

    // Enable Game Rotation Vector at IMU_SAMPLE_RATE_HZ (period in microseconds).
    if (!imu->rpt.rv_game.enable(1000000UL / IMU_SAMPLE_RATE_HZ)) {
        ESP_LOGE(TAG, "IMU%d: Game Rotation Vector enable failed", id);
        return false;
    }

    // Block (bounded) until the first real report so calibration has live data,
    // matching the old driver's "init blocks until first sample" contract.
    memset(&s_last[id], 0, sizeof(s_last[id]));
    s_last[id].sensor_id = (uint8_t)id;

    bool got_first = false;
    for (int i = 0; i < 200 && !got_first; i++) {           // up to ~2 s
        bno08x_quat_t q = imu->rpt.rv_game.get_quat();
        if (quat_is_valid(q)) {
            s_last[id].q_r = q.real; s_last[id].q_i = q.i;
            s_last[id].q_j = q.j;    s_last[id].q_k = q.k;
            quat_to_euler(q.real, q.i, q.j, q.k, &s_last[id]);
            s_last[id].valid        = true;
            s_last[id].timestamp_ms = get_timestamp_ms();
            got_first = true;
            ESP_LOGI(TAG, "IMU%d ready — pitch=%.2f roll=%.2f", id,
                     s_last[id].pitch_deg, s_last[id].roll_deg);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    if (!got_first) {
        ESP_LOGW(TAG, "IMU%d: no report within 2s after enable (continuing anyway)", id);
    }

    s_initialized[id] = true;
    ESP_LOGI(TAG, "IMU%d init complete (game_rv=ON)", id);
    return true;
}

extern "C" bool bno085_read_orientation(SensorContext_t *ctx, imu_sample_t *out)
{
    if (!ctx || !out) { if (out) out->valid = false; return false; }
    if (ctx->id < 0 || ctx->id >= BNO085_MAX_IMUS) { out->valid = false; return false; }

    const int id = ctx->id;
    if (!s_initialized[id] || s_imu[id] == nullptr) { out->valid = false; return false; }

    // get_quat() returns the latest cached report (non-blocking, mutex-protected;
    // the library's background task updates it on every INT). If no fresh report
    // is available we keep the previous sample, exactly like the old driver.
    bno08x_quat_t q = s_imu[id]->rpt.rv_game.get_quat();
    if (quat_is_valid(q)) {
        imu_sample_t s;
        memset(&s, 0, sizeof(s));
        s.sensor_id = (uint8_t)id;
        s.q_r = q.real; s.q_i = q.i; s.q_j = q.j; s.q_k = q.k;
        quat_to_euler(q.real, q.i, q.j, q.k, &s);
        s.valid        = true;
        s.timestamp_ms = get_timestamp_ms();
        s_last[id] = s;
    }

    *out = s_last[id];
    out->timestamp_ms = get_timestamp_ms();
    out->sensor_id    = (uint8_t)id;
    return true;
}

extern "C" bool bno085_read_sample(SensorContext_t *ctx, void *data_out)
{
    return bno085_read_orientation(ctx, (imu_sample_t *)data_out);
}
