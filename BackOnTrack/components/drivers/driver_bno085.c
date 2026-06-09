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
// BNO085 SHTP packet size limit.
// The BNO085 sends a 1024-byte SHTP advertisement (channel 0) after init.
// With SHTP_MAX_PACKET=512, that packet is rejected (1024 > 512), the
// advertisement never drains, and the BNO085 cannot send FEATURE_RESPONSE
// or sensor data. Must be 1024 to match the BNO085's actual advertisement size.
#define SHTP_MAX_PACKET                 1024
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
    uint8_t                 rx_buf[SHTP_MAX_PACKET]; // per-IMU RX buffer — not on stack
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

        /* No packet available */
    if (st->cfg && st->cfg->int_pin >= 0 &&
    gpio_get_level(st->cfg->int_pin))
    {

        return 0;
    }
    if (buf_size < SHTP_HEADER_LEN) return -1;
    size_t read_len = (buf_size < SHTP_MAX_PACKET) ? buf_size : SHTP_MAX_PACKET;
    static uint8_t tx_dummy[SHTP_MAX_PACKET];
    memset(tx_dummy, 0x00, read_len);
    // Always send all-zeros TX. The imu_diag confirmed that [04 00 00 00]
    // (heartbeat) triggers the BNO085 to re-send the ch=0 advertisement on
    // every read, flooding the output channel and preventing FEATURE_RESPONSE
    // and sensor data from coming through. All-zeros is the correct dummy TX.
    if (!spi_transfer(st, tx_dummy, buf, read_len)) return -1;
    uint16_t pkt_len = (uint16_t)buf[0] | ((uint16_t)(buf[1] & 0x7F) << 8);
    if (pkt_len < SHTP_HEADER_LEN)
    {
        ESP_LOGW(TAG,
                "Invalid packet length %u",
                pkt_len);
        return -1;
    }

    if (pkt_len > SHTP_MAX_PACKET)
    {
        ESP_LOGW(TAG,
                "Packet larger than buffer (%u)",
                pkt_len);

        pkt_len = SHTP_MAX_PACKET;
    }
    ESP_LOGI(TAG,
         "RX len=%u ch=%u seq=%u hdr=[%02X %02X %02X %02X]",
         pkt_len,
         buf[2],
         buf[3],
         buf[0],
         buf[1],
         buf[2],
         buf[3]);
    return (int)pkt_len;
}

static bool bno085_send_packet(bno085_state_t *st, uint8_t channel, uint8_t *seq,
                                const uint8_t *payload, uint16_t payload_len) {
    // imu_diag stage5 confirmed: send EXACTLY payload_len + SHTP_HEADER_LEN bytes.
    // Sending 1024 bytes after a 21-byte SET_FEATURE fills bytes 22-1024 with zeros.
    // The BNO085 receives all 1024 clock cycles and interprets the trailing zeros as
    // additional SHTP packets with length=0 (invalid), confusing its state machine
    // and preventing INT from being asserted after the command.
    static uint8_t tx_buf[SHTP_MAX_PACKET];
    static uint8_t rx_buf[SHTP_MAX_PACKET];

    size_t pkt_size = payload_len + SHTP_HEADER_LEN;
    if (pkt_size > SHTP_MAX_PACKET) {
        ESP_LOGE(TAG, "send_packet: payload too large (%d bytes)", payload_len);
        return false;
    }

    memset(tx_buf, 0x00, pkt_size);
    shtp_build_packet(channel, *seq, payload, payload_len, tx_buf);
    *seq = (*seq + 1) & 0xFF;

    return spi_transfer(st, tx_buf, rx_buf, pkt_size);
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
    if (len < 14)
    {
        ESP_LOGW(TAG,
                "GRV packet too short (%u)",
                len);
        return false;
    }

    if (payload[0] != SH2_REPORTID_GAME_ROTATION_VECTOR)
    {
        ESP_LOGW(TAG,
                "Unexpected report id 0x%02X",
                payload[0]);
        return false;
    }
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
    cmd[1] = report_id;              // report ID to enable (e.g. 0x08 = Game Rotation Vector)
    cmd[2] = 0x00;                  // feature flags

    cmd[5] = (uint8_t)( interval_us        & 0xFF);
    cmd[6] = (uint8_t)((interval_us >> 8)  & 0xFF);
    cmd[7] = (uint8_t)((interval_us >> 16) & 0xFF);
    cmd[8] = (uint8_t)((interval_us >> 24) & 0xFF);

    ESP_LOGI(TAG,
         "SET_FEATURE report=0x%02X interval=%lu",
         report_id,
         (unsigned long)interval_us);

    ESP_LOGI(TAG,
         "Sending SET_FEATURE seq=%u",
         st->seq_control);

    ESP_LOGI(TAG,
         "INT before send = %d",
         gpio_get_level(st->cfg->int_pin));

    ESP_LOGI(TAG,
         "CMD BYTES: "
         "%02X %02X %02X %02X %02X %02X %02X %02X "
         "%02X %02X %02X %02X %02X %02X %02X %02X %02X",
         cmd[0], cmd[1], cmd[2], cmd[3],
         cmd[4], cmd[5], cmd[6], cmd[7],
         cmd[8], cmd[9], cmd[10], cmd[11],
         cmd[12], cmd[13], cmd[14], cmd[15],
         cmd[16]);


    // imu_diag stage5: send SET_FEATURE as a SMALL (21-byte) transaction.
    // Boot drain already emptied the chip's queue, so there's no competing
    // slave data — a small TX works cleanly here.
    bool ok = bno085_send_packet(st, SHTP_CHANNEL_CONTROL, &st->seq_control, cmd, 17);

    ESP_LOGI(TAG,
         "SET_FEATURE send returned %s",
         ok ? "OK" : "FAIL");

    if (!ok) return false;

    // imu_diag stage5: explicitly wait for Get Feature Response (0xFC on ch=2)
    // before returning. This confirms the chip received and processed the command.
    // Without this confirmation, sensor reads start before the BNO085 has armed
    // the report, so the first-report polling loop finds nothing.
    static uint8_t feat_rx[SHTP_MAX_PACKET];
    bool confirmed = false;
    for (int i = 0; i < 30; i++) {
        if (!bno085_wait_int(st, 100)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        memset(feat_rx, 0, sizeof(feat_rx));
        int n = bno085_read_packet(st, feat_rx, sizeof(feat_rx));
        if (n > 0)
        {
            ESP_LOGI(TAG,
                    "0xFC wait: len=%d ch=%d seq=%d rid=0x%02X",
                    n,
                    feat_rx[2],
                    feat_rx[3],
                    (n > SHTP_HEADER_LEN)
                        ? feat_rx[SHTP_HEADER_LEN]
                        : 0xFF);

            ESP_LOGI(TAG,
                    "PKT: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                    feat_rx[0], feat_rx[1], feat_rx[2], feat_rx[3],
                    feat_rx[4], feat_rx[5], feat_rx[6], feat_rx[7],
                    feat_rx[8], feat_rx[9], feat_rx[10], feat_rx[11]);
        }

        if (n <= 0) continue;
        uint8_t ch  = feat_rx[2];
        uint8_t rid = n > SHTP_HEADER_LEN ? feat_rx[SHTP_HEADER_LEN] : 0x00;
        ESP_LOGI(TAG, "  feat confirm [%d]: ch=%d rid=0x%02X n=%d", i, ch, rid, n);
        if (ch == SHTP_CHANNEL_CONTROL && rid == 0xFC) {
            ESP_LOGI(TAG,
                    "FEATURE_RESPONSE confirmed "
                    "(0xFC) ch=%d n=%d",
                    ch,
                    n);            
            confirmed = true;
            break;
        }
        if (ch == SHTP_CHANNEL_REPORTS) {
            ESP_LOGI(TAG, "  Sensor data on ch=3 — feature already active.");
            confirmed = true;
            break;
        }
    }
    if (!confirmed) {
        ESP_LOGW(TAG, "  No 0xFC confirmation received.");
    }
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
    // 1 MHz instead of 3 MHz — breadboard wire parasitics cause signal integrity
    // issues at 3 MHz. The BNO085 supports up to 3 MHz but on a breadboard with
    // ~20cm jumper wires, 1 MHz gives 3x more margin on setup/hold times.
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
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

    // Static: bno085_init is called sequentially (IMU0 then IMU1), never concurrently.
    // Keeps 2KB off the main task stack, preventing stack overflow with 1024-byte SHTP_MAX_PACKET.
    static uint8_t rx_buf[SHTP_MAX_PACKET];
    static uint8_t saved_pid_buf[SHTP_MAX_PACKET];
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

        // NOTE: No software reset sent here. Hardware reset alone is sufficient.
        // Sending an additional soft reset (EXE channel, 0x01) causes a second
        // boot cycle and has been found to prevent SET_FEATURE_COMMAND from
        // being processed correctly. Hardware reset → wait 350ms → INT asserts
        // is the correct and sufficient init sequence.

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

    // Drain ALL boot packets using INT as the termination signal.
    // imu_diag stage5 confirmed: the BNO085 sends its capability advertisement
    // in many fragments on ch=0 after reset. We must read until INT de-asserts
    // naturally BEFORE sending any command — otherwise our TX bytes are ignored
    // because the chip is still in its boot-sequence transmit state.
    // Stop condition: INT stays HIGH for 50ms = chip has nothing more to send.
    // Use all-zeros TX (NOT heartbeat) to avoid triggering more ch=0 re-sends.
    int frs_drained = 0;
    for (int i = 0; i < 200; i++) {
        if (!bno085_wait_int(st, 50)) {
            ESP_LOGI(TAG, "IMU%d FRS drain: %d packets (INT idle)", ctx->id, frs_drained);
            break;
        }
        int pkt_n = bno085_read_packet(st, rx_buf, sizeof(rx_buf));
        if (pkt_n > 0) {
            frs_drained++;
        } else {
            break;
        }
    }
    // Extra guard: wait 20ms after last INT de-assertion before sending commands.
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGW(TAG,
         "IMU%d: Performing diagnostic-style reset before GRV enable",
         ctx->id);

    gpio_set_level(cfg->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_level(cfg->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    /* Drain boot packets exactly like imu_diag */
    int boot_packets = 0;

    ESP_LOGI(TAG,
         "INT level before boot drain = %d",
         gpio_get_level(cfg->int_pin));

    bool int_seen = bno085_wait_int(st, 500);

    ESP_LOGI(TAG,
            "INT asserted after reset = %s",
            int_seen ? "YES" : "NO");

    int n = bno085_read_packet(
        st,
        rx_buf,
        sizeof(rx_buf));

    ESP_LOGI(TAG,
            "BOOT DRAIN: n=%d ch=%u seq=%u",
            n,
            rx_buf[2],
            rx_buf[3]);

    if (n > 0)
    {
        boot_packets = 1;
    }

    ESP_LOGI(TAG,
            "IMU%d: Diagnostic drain after reset = %d packets",
            ctx->id,
            boot_packets);

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

    // Read packets until we get the first real sensor report and cache it in
    // last_sample with valid=true. This means bno085_read_orientation() returns
    // valid data immediately after init — critical for calibration working.
    //
    // Polls unconditionally (does NOT check INT pin) so it works even if:
    //   - The 10K pull-up is fitted but INT is briefly high between reports
    //   - The pull-up is missing (INT floats high)
    // At 50 Hz, a real report arrives every 20ms. We poll for up to 2 seconds.
    ESP_LOGI(TAG, "IMU%d: waiting for first sensor report...", ctx->id);
    ESP_LOGI(TAG, "DEBUG: entering first-report loop");
    bool got_first = false;
    for (int i = 0; i < 300 && !got_first; i++) {
        // Wait for INT to assert before reading — imu_diag stage5 approach.
        if (!bno085_wait_int(st, 100)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        int pkt_n = bno085_read_packet(st, rx_buf, sizeof(rx_buf));
        if (pkt_n >= (int)(SHTP_HEADER_LEN + 1)) {
            uint8_t  ch  = rx_buf[2];
            uint8_t  rid = rx_buf[SHTP_HEADER_LEN];
            uint16_t pay = (uint16_t)(pkt_n - SHTP_HEADER_LEN);
            uint8_t *payload = rx_buf + SHTP_HEADER_LEN;
            ESP_LOGI(TAG, "IMU%d pkt[%d]: len=%d ch=%d seq=%d payload=[%02X %02X %02X %02X]",
                     ctx->id, i, pkt_n, ch, rx_buf[3],
                     pay > 0 ? payload[0] : 0, pay > 1 ? payload[1] : 0,
                     pay > 2 ? payload[2] : 0, pay > 3 ? payload[3] : 0);
            if (ch == SHTP_CHANNEL_REPORTS) {
                // Scan payload, skipping 0xFB/0xFA timestamp records (5 bytes each)
                // imu_diag confirmed: BNO085 prepends timestamps before GRV data
                int offset = 0;
                while (offset < (int)pay) {
                    uint8_t id = payload[offset];
                    if ((id == 0xFB || id == 0xFA) && (offset + 5) <= (int)pay) {
                        offset += 5;
                        continue;
                    }
                    imu_sample_t tmp = {};
                    tmp.sensor_id = (uint8_t)ctx->id;
                    bool parsed = false;
                    int  rem    = (int)pay - offset;
                    if (id == SH2_REPORTID_GAME_ROTATION_VECTOR && rem >= 14)
                        parsed = parse_game_rotation_vector(payload + offset, rem, &tmp);
                    else if (id == SH2_REPORTID_ACCELEROMETER && rem >= 10)
                        parsed = parse_accelerometer(payload + offset, rem, &tmp);
                    if (parsed && tmp.valid) {
                        tmp.timestamp_ms = get_timestamp_ms();
                        st->last_sample  = tmp;
                        got_first        = true;
                        ESP_LOGI(TAG, "IMU%d ready — pitch=%.2f roll=%.2f",
                                 ctx->id, tmp.pitch_deg, tmp.roll_deg);
                    }
                    break;
                }
            }
        } else {
            ESP_LOGI(TAG, "IMU%d pkt[%d]: n=-1 raw=[%02X %02X %02X %02X]",
                     ctx->id, i, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    if (!got_first) {
        ESP_LOGW(TAG, "IMU%d: no sensor report in 2s — wiring may be incomplete", ctx->id);
    }

    return true;
}

bool bno085_read_orientation(SensorContext_t *ctx, imu_sample_t *out) {
    if (!ctx || !out) { if (out) out->valid = false; return false; }
    if (ctx->id < 0 || ctx->id >= IMU_COUNT) { out->valid = false; return false; }

    bno085_state_t *st = &s_bno085[ctx->id];
    if (!st->initialized) { out->valid = false; return false; }

    // Wait up to 50ms for INT to assert.
    // At 50 Hz the report period is 20ms. 50ms gives 2.5x margin —
    // handles the case where calibration calls read_orientation right after
    // init when the BNO085 is still sending SHTP control ACK packets before
    // the first sensor report arrives. 20ms was too short and timed out,
    // returning last_sample with valid=false during calibration priming.
    bno085_wait_int(st, 50);

    // Use the per-IMU rx_buf from state struct — not stack allocated.
    // imu_upper_task and imu_lower_task each have their own bno085_state_t,
    // so this is concurrent-safe (no shared buffer between the two IMUs).
    uint8_t *rx_buf = st->rx_buf;
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

        // imu_diag stage5 confirmed: the BNO085 prepends a Base Timestamp
        // Reference (0xFB, 5 bytes) or Timestamp Rebase (0xFA, 5 bytes) record
        // immediately before sensor reports in the same SHTP packet on ch=3.
        // We must skip these or we'll never find the actual GRV report (0x08).
        if ((report_id == 0xFB || report_id == 0xFA) && remaining >= 5) {
            offset += 5;
            continue;
        }

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