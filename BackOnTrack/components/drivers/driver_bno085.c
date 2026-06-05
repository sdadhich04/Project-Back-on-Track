/**
 * @file driver_bno085.c
 * @brief BNO085 IMU SPI driver for Back on Track — DUAL-IMU version
 *
 * Changes from single-IMU version:
 *   - Singleton `s_bno085` replaced with array `s_bno085[IMU_COUNT]`
 *   - All internal helpers accept a `bno085_state_t *st` pointer
 *   - `ctx->id` (IMU_ID_UPPER=0, IMU_ID_LOWER=1) selects which state to use
 *   - Both sensors share SPI3_HOST; each has its own CS/INT/RST
 *   - `spi_bus_initialize()` is called only for the FIRST sensor (index 0);
 *     the second sensor calls `spi_bus_add_device()` on the already-initialized bus
 *   - Output `imu_sample_t` includes `sensor_id` set from `ctx->id`
 *
 * Wiring for dual-IMU:
 *   Shared bus:  MOSI=GPIO40  MISO=GPIO39  SCLK=GPIO38  (same wires for both)
 *   IMU0 upper:  CS=GPIO37   INT=GPIO5    RST=GPIO6
 *   IMU1 lower:  CS=GPIO33   INT=GPIO35   RST=GPIO36
 */

#include "driver_bno085.h"
#include "sensor_hal.h"
#include "data_types.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>
#include <inttypes.h>

static const char *TAG = "bno085";

// ==================== SHTP Protocol Constants ====================

#define SHTP_HEADER_LEN                 4
#define SHTP_MAX_PACKET                 512
#define SHTP_CHANNEL_EXE                1
#define SHTP_CHANNEL_CONTROL            2
#define SHTP_CHANNEL_REPORTS            3

#define SHTP_REPORT_PRODUCT_ID_REQUEST  0xF9
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SET_FEATURE_COMMAND             0xFD

#define SH2_REPORTID_ACCELEROMETER          0x01
#define SH2_REPORTID_GAME_ROTATION_VECTOR   0x08

#define BNO085_REPORT_INTERVAL_US   (1000000 / IMU_SAMPLE_RATE_HZ)
#define BNO085_ACCEL_Q_SCALE        100.0f
#define BNO085_QUAT_Q_SCALE         16384.0f

// ==================== Driver State Array ====================
// One entry per physical IMU. Indexed by ctx->id (IMU_ID_UPPER / IMU_ID_LOWER).

typedef struct {
    spi_device_handle_t     spi_handle;
    bno085_spi_config_t    *cfg;
    bool                    initialized;
    bool                    game_rv_enabled;
    uint8_t                 seq_control;
    uint8_t                 seq_exe;
    imu_sample_t            last_sample;
} bno085_state_t;

// Array of states — one per IMU
static bno085_state_t s_bno085[IMU_COUNT] = {0};

// Track whether the SPI bus itself has been initialized (shared by all devices)
static bool s_spi_bus_initialized = false;

// ==================== Internal: SHTP Helpers ====================
// All helpers now take a `bno085_state_t *st` instead of using the global singleton.

static void shtp_build_packet(uint8_t channel, uint8_t seq,
                               const uint8_t *payload, uint16_t payload_len,
                               uint8_t *out_buf) {
    uint16_t total_len = payload_len + SHTP_HEADER_LEN;
    out_buf[0] = (uint8_t)(total_len & 0xFF);
    out_buf[1] = (uint8_t)((total_len >> 8) & 0x7F);
    out_buf[2] = channel;
    out_buf[3] = seq;
    if (payload && payload_len > 0) {
        memcpy(out_buf + SHTP_HEADER_LEN, payload, payload_len);
    }
}

static bool spi_transfer(bno085_state_t *st, const uint8_t *tx, uint8_t *rx, size_t len) {
    if (!st->spi_handle || len == 0) return false;
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(st->spi_handle, &t);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SPI transfer error: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

static int bno085_read_packet(bno085_state_t *st, uint8_t *buf, size_t buf_size) {
    if (buf_size < SHTP_HEADER_LEN) return -1;
    size_t read_len = (buf_size < SHTP_MAX_PACKET) ? buf_size : SHTP_MAX_PACKET;
    static uint8_t tx_dummy[SHTP_MAX_PACKET];
    memset(tx_dummy, 0x00, read_len);
    if (!st->initialized) {
        tx_dummy[0] = 0x04;
    }
    if (!spi_transfer(st, tx_dummy, buf, read_len)) return -1;
    uint16_t pkt_len = (uint16_t)buf[0] | ((uint16_t)(buf[1] & 0x7F) << 8);
    if (pkt_len < SHTP_HEADER_LEN || pkt_len > buf_size) return -1;
    return (int)pkt_len;
}

static bool bno085_send_packet(bno085_state_t *st, uint8_t channel, uint8_t *seq,
                                const uint8_t *payload, uint16_t payload_len) {
    uint8_t pkt[64];
    if ((size_t)(payload_len + SHTP_HEADER_LEN) > sizeof(pkt)) {
        ESP_LOGE(TAG, "send_packet: payload too large (%d bytes)", payload_len);
        return false;
    }
    shtp_build_packet(channel, *seq, payload, payload_len, pkt);
    *seq = (*seq + 1) & 0xFF;
    uint8_t rx_dummy[64];
    return spi_transfer(st, pkt, rx_dummy, payload_len + SHTP_HEADER_LEN);
}

static void bno085_hw_reset(bno085_state_t *st) {
    int rst_pin = st->cfg->rst_pin;
    if (rst_pin < 0) {
        vTaskDelay(pdMS_TO_TICKS(350));
        return;
    }
    gpio_set_level(rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(350));
}

static bool bno085_wait_int(bno085_state_t *st, uint32_t timeout_ms) {
    int int_pin = st->cfg->int_pin;
    if (int_pin < 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
        return true;
    }
    uint32_t start = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    while ((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - start < timeout_ms) {
        if (gpio_get_level(int_pin) == 0) return true;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

// ==================== Internal: Report Parsers ====================

static bool parse_game_rotation_vector(const uint8_t *payload, uint16_t len,
                                        imu_sample_t *out) {
    if (len < 14 || payload[0] != SH2_REPORTID_GAME_ROTATION_VECTOR) return false;

    int16_t qi_raw = (int16_t)((uint16_t)payload[4]  | ((uint16_t)payload[5]  << 8));
    int16_t qj_raw = (int16_t)((uint16_t)payload[6]  | ((uint16_t)payload[7]  << 8));
    int16_t qk_raw = (int16_t)((uint16_t)payload[8]  | ((uint16_t)payload[9]  << 8));
    int16_t qr_raw = (int16_t)((uint16_t)payload[10] | ((uint16_t)payload[11] << 8));

    float qi = (float)qi_raw / BNO085_QUAT_Q_SCALE;
    float qj = (float)qj_raw / BNO085_QUAT_Q_SCALE;
    float qk = (float)qk_raw / BNO085_QUAT_Q_SCALE;
    float qr = (float)qr_raw / BNO085_QUAT_Q_SCALE;

    out->q_i = qi; out->q_j = qj; out->q_k = qk; out->q_r = qr;

    float sinp = 2.0f * (qr * qj - qk * qi);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    out->pitch_deg = asinf(sinp) * (180.0f / (float)M_PI);

    float sinr_cosp = 2.0f * (qr * qi + qj * qk);
    float cosr_cosp = 1.0f - 2.0f * (qi * qi + qj * qj);
    out->roll_deg = atan2f(sinr_cosp, cosr_cosp) * (180.0f / (float)M_PI);

    float siny_cosp = 2.0f * (qr * qk + qi * qj);
    float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
    out->yaw_deg = atan2f(siny_cosp, cosy_cosp) * (180.0f / (float)M_PI);

    out->valid = true;
    return true;
}

static bool parse_accelerometer(const uint8_t *payload, uint16_t len, imu_sample_t *out) {
    if (len < 10 || payload[0] != SH2_REPORTID_ACCELEROMETER) return false;
    int16_t x = (int16_t)((uint16_t)payload[4] | ((uint16_t)payload[5] << 8));
    int16_t y = (int16_t)((uint16_t)payload[6] | ((uint16_t)payload[7] << 8));
    int16_t z = (int16_t)((uint16_t)payload[8] | ((uint16_t)payload[9] << 8));
    float fx = (float)x / BNO085_ACCEL_Q_SCALE;
    float fy = (float)y / BNO085_ACCEL_Q_SCALE;
    float fz = (float)z / BNO085_ACCEL_Q_SCALE;
    out->accel_magnitude = sqrtf(fx*fx + fy*fy + fz*fz);
    out->pitch_deg = 0.0f; out->roll_deg = 0.0f; out->yaw_deg = 0.0f;
    out->valid = true;
    return true;
}

static bool bno085_enable_report(bno085_state_t *st, uint8_t report_id, uint32_t interval_us) {
    uint8_t cmd[17] = {0};
    cmd[0] = SET_FEATURE_COMMAND;
    cmd[1] = report_id;
    cmd[5] = (uint8_t)( interval_us        & 0xFF);
    cmd[6] = (uint8_t)((interval_us >> 8)  & 0xFF);
    cmd[7] = (uint8_t)((interval_us >> 16) & 0xFF);
    cmd[8] = (uint8_t)((interval_us >> 24) & 0xFF);
    bool ok = bno085_send_packet(st, SHTP_CHANNEL_CONTROL, &st->seq_control, cmd, 17);
    vTaskDelay(pdMS_TO_TICKS(50));
    return ok;
}

// ==================== Public API ====================

bool bno085_init(SensorContext_t *ctx) {
    if (!ctx || !ctx->hw_config) {
        ESP_LOGE(TAG, "bno085_init: ctx or hw_config is NULL");
        return false;
    }
    if (ctx->id < 0 || ctx->id >= IMU_COUNT) {
        ESP_LOGE(TAG, "bno085_init: ctx->id=%d out of range (0..%d)", ctx->id, IMU_COUNT-1);
        return false;
    }

    bno085_spi_config_t *cfg = (bno085_spi_config_t *)ctx->hw_config;
    bno085_state_t *st = &s_bno085[ctx->id];

    ESP_LOGI(TAG, "Initializing IMU%d (CS=GPIO%d INT=GPIO%d RST=GPIO%d)",
             ctx->id, cfg->cs_pin, cfg->int_pin, cfg->rst_pin);

    // --- Configure RST pin ---
    if (cfg->rst_pin >= 0) {
        gpio_config_t rst_cfg = {
            .pin_bit_mask = (1ULL << cfg->rst_pin),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&rst_cfg);
        gpio_set_level(cfg->rst_pin, 1);
    }

    // --- Configure INT pin ---
    if (cfg->int_pin >= 0) {
        gpio_config_t int_cfg = {
            .pin_bit_mask = (1ULL << cfg->int_pin),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&int_cfg);
    }

    spi_host_device_t host = (spi_host_device_t)cfg->spi_host;

    // --- Initialize SPI bus ONCE for first IMU; subsequent IMUs just add devices ---
    if (!s_spi_bus_initialized) {
        spi_bus_config_t bus_cfg = {
            .mosi_io_num     = cfg->mosi_pin,
            .miso_io_num     = cfg->miso_pin,
            .sclk_io_num     = cfg->sclk_pin,
            .quadwp_io_num   = -1,
            .quadhd_io_num   = -1,
            .max_transfer_sz = SHTP_MAX_PACKET,
        };
        esp_err_t err = spi_bus_initialize(host, &bus_cfg, SPI_DMA_CH_AUTO);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
            return false;
        }
        s_spi_bus_initialized = true;
        ESP_LOGI(TAG, "SPI bus initialized on host %d", (int)host);
    } else {
        ESP_LOGI(TAG, "IMU%d: SPI bus already initialized, adding device only", ctx->id);
    }

    // --- Add this sensor's SPI device (each has its own CS) ---
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 3 * 1000 * 1000,
        .mode           = 0,
        .spics_io_num   = cfg->cs_pin,
        .queue_size     = 1,
    };
    esp_err_t err = spi_bus_add_device(host, &dev_cfg, &st->spi_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IMU%d: SPI add device failed: %s", ctx->id, esp_err_to_name(err));
        return false;
    }

    st->cfg             = cfg;
    st->seq_control     = 0;
    st->seq_exe         = 0;
    st->game_rv_enabled = false;
    memset(&st->last_sample, 0, sizeof(st->last_sample));
    st->last_sample.sensor_id = (uint8_t)ctx->id;

    // ==================== Init Retry Loop ====================

    uint8_t rx_buf[SHTP_MAX_PACKET];
    uint8_t saved_pid_buf[SHTP_MAX_PACKET];
    bool found_product_id = false;
    const int MAX_ATTEMPTS = 3;

    for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
        found_product_id     = false;
        st->seq_control      = 0;
        st->seq_exe          = 0;

        if (attempt > 0) {
            ESP_LOGW(TAG, "IMU%d: Init retry %d/%d...", ctx->id, attempt+1, MAX_ATTEMPTS);
        }

        bno085_hw_reset(st);

        uint8_t reset_cmd = 1;
        bno085_send_packet(st, SHTP_CHANNEL_EXE, &st->seq_exe, &reset_cmd, 1);

        bool int_rdy = bno085_wait_int(st, 800);
        if (!int_rdy) {
            ESP_LOGW(TAG, "IMU%d: INT never asserted after reset", ctx->id);
            continue;
        }

        int drain_count = 0;
        bool pid_sent   = false;
        int  idle_count = 0;
        int  n          = -1;

        for (int i = 0; i < 80; i++) {
            int pkt_n;

            if (!pid_sent && drain_count >= 1) {
                static uint8_t tx_cmd[SHTP_MAX_PACKET];
                memset(tx_cmd, 0x00, SHTP_MAX_PACKET);
                uint8_t pid_payload[2] = { SHTP_REPORT_PRODUCT_ID_REQUEST, 0 };
                shtp_build_packet(SHTP_CHANNEL_CONTROL, st->seq_control,
                                  pid_payload, 2, tx_cmd);
                st->seq_control = (st->seq_control + 1) & 0xFF;
                spi_transfer(st, tx_cmd, rx_buf, SHTP_MAX_PACKET);
                pid_sent = true;
                uint16_t rx_len = (uint16_t)rx_buf[0] | ((uint16_t)(rx_buf[1] & 0x7F) << 8);
                pkt_n = (rx_len >= SHTP_HEADER_LEN && rx_len <= SHTP_MAX_PACKET)
                         ? (int)rx_len : -1;
            } else {
                pkt_n = bno085_read_packet(st, rx_buf, sizeof(rx_buf));
            }

            if (pkt_n > 0) {
                idle_count = 0;
                drain_count++;
                uint8_t pkt_ch = rx_buf[2];
                if (pkt_ch == SHTP_CHANNEL_CONTROL
                    && pkt_n >= (int)(SHTP_HEADER_LEN + 2)
                    && rx_buf[SHTP_HEADER_LEN] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
                    found_product_id = true;
                    n = pkt_n;
                    memcpy(saved_pid_buf, rx_buf, pkt_n);
                }
            } else {
                idle_count++;
            }

            if (found_product_id && idle_count >= 3) break;
            if (pid_sent && idle_count >= 15) break;
            if (!pid_sent && idle_count >= 30) break;

            vTaskDelay(pdMS_TO_TICKS(5));
        }

        if (found_product_id && n >= (int)(SHTP_HEADER_LEN + 2)) break;
    }

    if (!found_product_id) {
        ESP_LOGE(TAG, "IMU%d: No product ID response after %d attempts.", ctx->id, MAX_ATTEMPTS);
        ESP_LOGE(TAG, "  CS=GPIO%d INT=GPIO%d RST=GPIO%d", cfg->cs_pin, cfg->int_pin, cfg->rst_pin);
        ESP_LOGE(TAG, "  PS0 and PS1 must be tied to 3.3V for SPI mode");
        spi_bus_remove_device(st->spi_handle);
        return false;
    }

    // Log part number
    if ((int)(SHTP_HEADER_LEN + 8) <= (int)saved_pid_buf[0]) {
        uint32_t part_no =
            (uint32_t) saved_pid_buf[SHTP_HEADER_LEN + 4]        |
            ((uint32_t)saved_pid_buf[SHTP_HEADER_LEN + 5] << 8)  |
            ((uint32_t)saved_pid_buf[SHTP_HEADER_LEN + 6] << 16) |
            ((uint32_t)saved_pid_buf[SHTP_HEADER_LEN + 7] << 24);
        ESP_LOGI(TAG, "IMU%d Part Number: 0x%08"PRIx32, ctx->id, part_no);
    }

    // Drain FRS records
    int frs_drained = 0;
    for (int i = 0; i < 200; i++) {
        if (cfg->int_pin >= 0 && gpio_get_level(cfg->int_pin) != 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            if (gpio_get_level(cfg->int_pin) != 0) break;
        }
        int pkt_n = bno085_read_packet(st, rx_buf, sizeof(rx_buf));
        if (pkt_n > 0) frs_drained++;
        else vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGD(TAG, "IMU%d FRS drain: %d packets", ctx->id, frs_drained);

    // Enable Game Rotation Vector
    ESP_LOGI(TAG, "IMU%d: Enabling Game Rotation Vector at %d Hz", ctx->id, IMU_SAMPLE_RATE_HZ);
    if (bno085_enable_report(st, SH2_REPORTID_GAME_ROTATION_VECTOR, BNO085_REPORT_INTERVAL_US)) {
        st->game_rv_enabled = true;
    } else {
        ESP_LOGW(TAG, "IMU%d: Game RV failed; falling back to accelerometer", ctx->id);
        bno085_enable_report(st, SH2_REPORTID_ACCELEROMETER, BNO085_REPORT_INTERVAL_US);
        st->game_rv_enabled = false;
    }

    st->initialized = true;
    ESP_LOGI(TAG, "IMU%d init complete (game_rv=%s)", ctx->id, st->game_rv_enabled ? "ON" : "FALLBACK");
    return true;
}

bool bno085_read_orientation(SensorContext_t *ctx, imu_sample_t *out) {
    if (!ctx || !out) { if (out) out->valid = false; return false; }
    if (ctx->id < 0 || ctx->id >= IMU_COUNT) { out->valid = false; return false; }

    bno085_state_t *st = &s_bno085[ctx->id];
    if (!st->initialized) { out->valid = false; return false; }

    bno085_wait_int(st, 20);

    uint8_t rx_buf[SHTP_MAX_PACKET];
    int n = bno085_read_packet(st, rx_buf, sizeof(rx_buf));
    if (n < SHTP_HEADER_LEN) {
        *out = st->last_sample;
        out->timestamp_ms = get_timestamp_ms();
        out->sensor_id    = (uint8_t)ctx->id;
        return true;
    }

    uint8_t channel      = rx_buf[2];
    uint16_t payload_len = (uint16_t)(n - SHTP_HEADER_LEN);
    uint8_t *payload     = rx_buf + SHTP_HEADER_LEN;

    if (channel != SHTP_CHANNEL_REPORTS || payload_len < 10) {
        *out = st->last_sample;
        out->timestamp_ms = get_timestamp_ms();
        out->sensor_id    = (uint8_t)ctx->id;
        return true;
    }

    out->timestamp_ms = get_timestamp_ms();
    out->sensor_id    = (uint8_t)ctx->id;
    out->valid        = false;

    uint16_t offset = 0;
    while (offset < payload_len) {
        uint8_t report_id  = payload[offset];
        uint16_t remaining = payload_len - offset;

        if (report_id == SH2_REPORTID_GAME_ROTATION_VECTOR && remaining >= 14) {
            if (parse_game_rotation_vector(payload + offset, remaining, out)) {
                out->timestamp_ms = get_timestamp_ms();
                out->sensor_id    = (uint8_t)ctx->id;
                st->last_sample   = *out;
                return true;
            }
            offset += 14;
        } else if (report_id == SH2_REPORTID_ACCELEROMETER && remaining >= 10) {
            if (parse_accelerometer(payload + offset, remaining, out)) {
                out->timestamp_ms = get_timestamp_ms();
                out->sensor_id    = (uint8_t)ctx->id;
                st->last_sample   = *out;
                return true;
            }
            offset += 10;
        } else {
            offset++;
        }
    }

    *out = st->last_sample;
    out->timestamp_ms = get_timestamp_ms();
    out->sensor_id    = (uint8_t)ctx->id;
    return true;
}

bool bno085_read_sample(SensorContext_t *ctx, void *data_out) {
    return bno085_read_orientation(ctx, (imu_sample_t *)data_out);
}
