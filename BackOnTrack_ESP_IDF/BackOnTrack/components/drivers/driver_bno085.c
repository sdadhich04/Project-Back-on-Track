/**
 * @file driver_bno085.c
 * @brief BNO085 IMU SPI driver for Back on Track
 *
 * Implements SHTP (Sensor Hub Transport Protocol) for the Adafruit BNO085.
 * Heavily adapted from the Project SHIELD driver_bno085.c (which was verified
 * to work correctly on ESP32-S3 hardware with SPI3_HOST at 3 MHz).
 *
 * Key changes from Shield driver:
 *   - Enables Game Rotation Vector (0x08) instead of raw accelerometer
 *   - Parses quaternion payload and converts to pitch/roll/yaw
 *   - Output is imu_sample_t (not float magnitude)
 *   - Fallback: if Game RV fails to parse, uses accelerometer magnitude
 *
 * SHTP Overview:
 *   BNO085 uses SHTP layered over SPI. Each "packet" has a 4-byte header
 *   [len_lo, len_hi, channel, seq] followed by a payload. CS must stay LOW
 *   for the entire packet — splitting into two SPI transactions breaks things.
 *
 * SPI mode 0 (CPOL=0, CPHA=0), max 3 MHz for reliable operation.
 * INT pin is active-low; assert when BNO085 has data ready.
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
#define SHTP_CHANNEL_EXE                1    // Executable channel (soft reset)
#define SHTP_CHANNEL_CONTROL            2    // Control channel (feature config)
#define SHTP_CHANNEL_REPORTS            3    // Sensor reports

#define SHTP_REPORT_PRODUCT_ID_REQUEST  0xF9
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SET_FEATURE_COMMAND             0xFD

// ==================== BNO085 Report IDs ====================

#define SH2_REPORTID_ACCELEROMETER          0x01  // Raw accelerometer (fallback)
#define SH2_REPORTID_GAME_ROTATION_VECTOR   0x08  // Quaternion without magnetometer

// Report interval for the feature enable command (in microseconds)
// 1,000,000 / IMU_SAMPLE_RATE_HZ = period_us
// At 50 Hz: 20000 us
#define BNO085_REPORT_INTERVAL_US   (1000000 / IMU_SAMPLE_RATE_HZ)

// Accelerometer Q-point scale: Q8.2 -> divide by 100 to get m/s²
#define BNO085_ACCEL_Q_SCALE    100.0f

// Game Rotation Vector Q-point scale: Q14 -> divide by (1 << 14) = 16384
// to get values in range [-1.0, +1.0] for unit quaternion components
#define BNO085_QUAT_Q_SCALE     16384.0f

// ==================== Driver State (module-level singleton) ====================

typedef struct {
    spi_device_handle_t     spi_handle;
    bno085_spi_config_t    *cfg;
    bool                    initialized;
    bool                    game_rv_enabled;   // true if Game Rotation Vector report is active
    uint8_t                 seq_control;       // SHTP sequence counter for channel 2
    uint8_t                 seq_exe;           // SHTP sequence counter for channel 1
    imu_sample_t            last_sample;       // Cache of last valid reading
} bno085_state_t;

static bno085_state_t s_bno085 = {0};

// ==================== Internal: SHTP Helpers ====================

/**
 * @brief Build a 4-byte SHTP header + payload into out_buf.
 * Caller must ensure out_buf is large enough for SHTP_HEADER_LEN + payload_len.
 */
static void shtp_build_packet(uint8_t channel, uint8_t seq,
                               const uint8_t *payload, uint16_t payload_len,
                               uint8_t *out_buf) {
    uint16_t total_len = payload_len + SHTP_HEADER_LEN;
    out_buf[0] = (uint8_t)(total_len & 0xFF);
    out_buf[1] = (uint8_t)((total_len >> 8) & 0x7F);  // Bit 15 = 0 (no continuation)
    out_buf[2] = channel;
    out_buf[3] = seq;
    if (payload && payload_len > 0) {
        memcpy(out_buf + SHTP_HEADER_LEN, payload, payload_len);
    }
}

/**
 * @brief Raw SPI full-duplex transfer.
 * CS is managed by the spi_master driver (spics_io_num was set at add_device).
 */
static bool spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len) {
    if (!s_bno085.spi_handle || len == 0) return false;
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(s_bno085.spi_handle, &t);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SPI transfer error: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

/**
 * @brief Read one SHTP packet into buf.
 *
 * IMPORTANT: Must be a single SPI transaction (CS stays LOW). Splitting into
 * header-then-payload causes CS to toggle mid-packet and confuses the BNO085
 * SHTP state machine (sequence numbers will increment by 2 instead of 1).
 *
 * TX strategy:
 *   - During init (initialized==false): send [04 00 00 00 ...] heartbeat
 *     so BNO085 sees valid SHTP activity and advances its state machine.
 *   - During runtime (initialized==true): send [00 00 00 00 ...] (no host data)
 *     to avoid triggering spurious channel-0 acknowledgment responses from BNO085.
 *
 * @return Number of bytes in valid packet (>= SHTP_HEADER_LEN), or -1 on error.
 */
static int bno085_read_packet(uint8_t *buf, size_t buf_size) {
    if (buf_size < SHTP_HEADER_LEN) return -1;

    size_t read_len = (buf_size < SHTP_MAX_PACKET) ? buf_size : SHTP_MAX_PACKET;
    static uint8_t tx_dummy[SHTP_MAX_PACKET];
    memset(tx_dummy, 0x00, read_len);

    // Init-phase heartbeat: keep BNO085 SHTP state machine ticking
    if (!s_bno085.initialized) {
        tx_dummy[0] = 0x04;
    }

    if (!spi_transfer(tx_dummy, buf, read_len)) {
        return -1;
    }

    uint16_t pkt_len = (uint16_t)buf[0] | ((uint16_t)(buf[1] & 0x7F) << 8);
    if (pkt_len < SHTP_HEADER_LEN || pkt_len > buf_size) {
        return -1;  // Garbage or zero-length packet
    }
    return (int)pkt_len;
}

/**
 * @brief Send one SHTP packet over SPI.
 * Increments the channel sequence counter after sending.
 */
static bool bno085_send_packet(uint8_t channel, uint8_t *seq,
                                const uint8_t *payload, uint16_t payload_len) {
    uint8_t pkt[64];
    if ((size_t)(payload_len + SHTP_HEADER_LEN) > sizeof(pkt)) {
        ESP_LOGE(TAG, "send_packet: payload too large (%d bytes)", payload_len);
        return false;
    }
    shtp_build_packet(channel, *seq, payload, payload_len, pkt);
    *seq = (*seq + 1) & 0xFF;

    uint8_t rx_dummy[64];
    return spi_transfer(pkt, rx_dummy, payload_len + SHTP_HEADER_LEN);
}

/**
 * @brief Assert RST low for 10ms, then release and wait 350ms for BNO085 boot.
 * The 350ms boot delay is per the BNO085 datasheet.
 */
static void bno085_hw_reset(int rst_pin) {
    if (rst_pin < 0) {
        ESP_LOGW(TAG, "RST pin not configured; waiting 350ms passively");
        vTaskDelay(pdMS_TO_TICKS(350));
        return;
    }
    gpio_set_level(rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(350));
}

/**
 * @brief Poll INT pin until it goes LOW (active-low = data ready) or timeout.
 * Returns true if INT went LOW, false if timeout.
 */
static bool bno085_wait_int(int int_pin, uint32_t timeout_ms) {
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

/**
 * @brief Parse a Game Rotation Vector report (report ID 0x08).
 *
 * Payload layout (14 bytes total):
 *   [0]     Report ID = 0x08
 *   [1]     Sequence number
 *   [2]     Status (bits 1:0 = accuracy estimate; 0=unknown, 3=high)
 *   [3]     Delay (report delay in 100us units)
 *   [4:5]   i (x) component, int16_t, Q14 scale
 *   [6:7]   j (y) component, int16_t, Q14 scale
 *   [8:9]   k (z) component, int16_t, Q14 scale
 *   [10:11] real component,  int16_t, Q14 scale
 *   [12:13] accuracy (uint16_t, in radians * 2^12)
 *
 * Q14 scale: float_value = int16_value / 16384.0f
 * Result is a unit quaternion representing orientation.
 */
static bool parse_game_rotation_vector(const uint8_t *payload, uint16_t len,
                                        imu_sample_t *out) {
    if (len < 14 || payload[0] != SH2_REPORTID_GAME_ROTATION_VECTOR) return false;

    int16_t qi_raw = (int16_t)((uint16_t)payload[4]  | ((uint16_t)payload[5]  << 8));
    int16_t qj_raw = (int16_t)((uint16_t)payload[6]  | ((uint16_t)payload[7]  << 8));
    int16_t qk_raw = (int16_t)((uint16_t)payload[8]  | ((uint16_t)payload[9]  << 8));
    int16_t qr_raw = (int16_t)((uint16_t)payload[10] | ((uint16_t)payload[11] << 8));

    // Convert from Q14 integer to float unit quaternion
    float qi = (float)qi_raw / BNO085_QUAT_Q_SCALE;
    float qj = (float)qj_raw / BNO085_QUAT_Q_SCALE;
    float qk = (float)qk_raw / BNO085_QUAT_Q_SCALE;
    float qr = (float)qr_raw / BNO085_QUAT_Q_SCALE;

    out->q_i = qi;
    out->q_j = qj;
    out->q_k = qk;
    out->q_r = qr;

    // ---- Quaternion to Euler (ZYX extrinsic = XYZ intrinsic) ----
    // Convention: q = qr + qi*i + qj*j + qk*k
    // Pitch = rotation about X (nod forward/back)
    // Roll  = rotation about Y (lean left/right)
    // Yaw   = rotation about Z (twist)
    //
    // Standard aerospace ZYX Euler extraction:
    //   pitch = asin(2*(qr*qj - qk*qi))
    //   roll  = atan2(2*(qr*qi + qj*qk), 1 - 2*(qi*qi + qj*qj))
    //   yaw   = atan2(2*(qr*qk + qi*qj), 1 - 2*(qj*qj + qk*qk))
    //
    // NOTE: "Pitch" here aligns with forward/back body tilt when the BNO085
    // is mounted with its X axis pointing forward (towards screen/user front).
    // Adjust axis mapping in posture_features.c if your mounting differs.

    float sinp = 2.0f * (qr * qj - qk * qi);
    // Clamp for numerical safety (avoids NaN at ±90°)
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

/**
 * @brief Parse a raw accelerometer report (report ID 0x01).
 * Used as fallback if Game Rotation Vector is not available.
 * Outputs the vector magnitude (total acceleration in m/s²).
 *
 * Payload layout (10 bytes):
 *   [0]    Report ID = 0x01
 *   [1]    Sequence number
 *   [2]    Status
 *   [3]    Delay
 *   [4:5]  X, int16_t, Q8.2 (divide by 100)
 *   [6:7]  Y
 *   [8:9]  Z
 */
static bool parse_accelerometer(const uint8_t *payload, uint16_t len,
                                  imu_sample_t *out) {
    if (len < 10 || payload[0] != SH2_REPORTID_ACCELEROMETER) return false;

    int16_t x = (int16_t)((uint16_t)payload[4] | ((uint16_t)payload[5] << 8));
    int16_t y = (int16_t)((uint16_t)payload[6] | ((uint16_t)payload[7] << 8));
    int16_t z = (int16_t)((uint16_t)payload[8] | ((uint16_t)payload[9] << 8));

    float fx = (float)x / BNO085_ACCEL_Q_SCALE;
    float fy = (float)y / BNO085_ACCEL_Q_SCALE;
    float fz = (float)z / BNO085_ACCEL_Q_SCALE;

    out->accel_magnitude = sqrtf(fx * fx + fy * fy + fz * fz);
    // Note: without full orientation, we can't compute pitch/roll/yaw here.
    // Posture classifier will use accel_magnitude as a rough "deviation" proxy.
    // This is a FALLBACK path only. TODO: implement tilt from accel if needed.
    out->pitch_deg = 0.0f;
    out->roll_deg  = 0.0f;
    out->yaw_deg   = 0.0f;
    out->valid = true;
    return true;
}

// ==================== Internal: Enable Feature Report ====================

/**
 * @brief Send a SET_FEATURE_COMMAND to enable a specific BNO085 report.
 * 
 * @param report_id   BNO085 report ID (e.g. SH2_REPORTID_GAME_ROTATION_VECTOR)
 * @param interval_us Desired report period in microseconds
 */
static bool bno085_enable_report(uint8_t report_id, uint32_t interval_us) {
    // SET_FEATURE_COMMAND payload: 17 bytes
    // [0]    Command ID = 0xFD
    // [1]    Feature Report ID
    // [2]    Feature flags (0)
    // [3:4]  Change sensitivity (0 = ignore)
    // [5:8]  Report interval (uint32_t, little-endian, in microseconds)
    // [9:12] Batch interval (0 = disabled)
    // [13:16] Sensor-specific config (0)
    uint8_t cmd[17] = {0};
    cmd[0] = SET_FEATURE_COMMAND;
    cmd[1] = report_id;
    // cmd[2..4] = 0 (flags, change sensitivity)
    cmd[5] = (uint8_t)( interval_us        & 0xFF);
    cmd[6] = (uint8_t)((interval_us >> 8)  & 0xFF);
    cmd[7] = (uint8_t)((interval_us >> 16) & 0xFF);
    cmd[8] = (uint8_t)((interval_us >> 24) & 0xFF);
    // cmd[9..16] = 0 (batch interval, sensor config)

    bool ok = bno085_send_packet(SHTP_CHANNEL_CONTROL, &s_bno085.seq_control, cmd, 17);
    vTaskDelay(pdMS_TO_TICKS(50));  // Give BNO085 time to activate the report
    return ok;
}

// ==================== Public API ====================

bool bno085_init(SensorContext_t *ctx) {
    if (!ctx || !ctx->hw_config) {
        ESP_LOGE(TAG, "bno085_init: ctx or hw_config is NULL");
        return false;
    }

    bno085_spi_config_t *cfg = (bno085_spi_config_t *)ctx->hw_config;

    // Validate critical pins
    if (cfg->cs_pin < 0 || cfg->sclk_pin < 0 || cfg->mosi_pin < 0 || cfg->miso_pin < 0) {
        ESP_LOGE(TAG, "bno085_init: incomplete SPI pin config (cs=%d sclk=%d mosi=%d miso=%d)",
                 cfg->cs_pin, cfg->sclk_pin, cfg->mosi_pin, cfg->miso_pin);
        return false;
    }

    // --- Configure RST pin as output ---
    if (cfg->rst_pin >= 0) {
        gpio_config_t rst_cfg = {
            .pin_bit_mask   = (1ULL << cfg->rst_pin),
            .mode           = GPIO_MODE_OUTPUT,
            .pull_up_en     = GPIO_PULLUP_DISABLE,
            .pull_down_en   = GPIO_PULLDOWN_DISABLE,
            .intr_type      = GPIO_INTR_DISABLE,
        };
        gpio_config(&rst_cfg);
        gpio_set_level(cfg->rst_pin, 1);  // Hold RST HIGH (active low)
    }

    // --- Configure INT pin as input with pull-up ---
    // BNO085 asserts INT LOW when data is ready. The pull-up keeps it HIGH otherwise.
    // Hardware pull-up is preferred; this GPIO pull-up is a software backup.
    if (cfg->int_pin >= 0) {
        gpio_config_t int_cfg = {
            .pin_bit_mask   = (1ULL << cfg->int_pin),
            .mode           = GPIO_MODE_INPUT,
            .pull_up_en     = GPIO_PULLUP_ENABLE,   // Internal pull-up as backup
            .pull_down_en   = GPIO_PULLDOWN_DISABLE,
            .intr_type      = GPIO_INTR_DISABLE,
        };
        gpio_config(&int_cfg);
    }

    // --- Initialize SPI bus BEFORE hardware reset ---
    // BNO085's SPI slave interface must see properly driven MOSI/SCLK/CS during boot.
    // If these lines float during reset, the SPI slave may boot into a bad state.
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = cfg->mosi_pin,
        .miso_io_num     = cfg->miso_pin,
        .sclk_io_num     = cfg->sclk_pin,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = SHTP_MAX_PACKET,
    };

    spi_host_device_t host = (spi_host_device_t)cfg->spi_host;
    esp_err_t err = spi_bus_initialize(host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return false;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 3 * 1000 * 1000,  // 3 MHz — reliable for BNO085 over wires
        .mode           = 0,                  // SPI mode 0: CPOL=0, CPHA=0
        .spics_io_num   = cfg->cs_pin,        // SPI master handles CS automatically
        .queue_size     = 1,
    };

    err = spi_bus_add_device(host, &dev_cfg, &s_bno085.spi_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(err));
        spi_bus_free(host);
        return false;
    }

    s_bno085.cfg             = cfg;
    s_bno085.seq_control     = 0;
    s_bno085.seq_exe         = 0;
    s_bno085.game_rv_enabled = false;
    memset(&s_bno085.last_sample, 0, sizeof(s_bno085.last_sample));

    // ==================== Init Retry Loop ====================
    // BNO085 boot timing varies. We retry up to 3 times.
    // Each attempt: hardware reset -> soft reset -> drain init packets -> Product ID check

    uint8_t rx_buf[SHTP_MAX_PACKET];
    uint8_t saved_pid_buf[SHTP_MAX_PACKET];
    bool found_product_id = false;
    const int MAX_ATTEMPTS = 3;

    for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
        found_product_id = false;
        s_bno085.seq_control = 0;
        s_bno085.seq_exe     = 0;

        if (attempt > 0) {
            ESP_LOGW(TAG, "Init retry %d/%d...", attempt + 1, MAX_ATTEMPTS);
        }

        // Step 1: Hardware reset (RST low 10ms, then high, then 350ms boot wait)
        bno085_hw_reset(cfg->rst_pin);

        // Step 2: Soft reset via EXE channel.
        // This produces a clean, predictable init sequence (advert + reset + init response).
        // Without soft reset, hardware-reset-only floods us with 80+ FRS dump packets
        // before the Product ID response, making init unreliable.
        uint8_t reset_cmd = 1;
        bno085_send_packet(SHTP_CHANNEL_EXE, &s_bno085.seq_exe, &reset_cmd, 1);

        // Step 3: Wait for BNO085 to re-assert INT after soft reset
        bool int_rdy = bno085_wait_int(cfg->int_pin, 800);
        if (!int_rdy) {
            ESP_LOGW(TAG, "Attempt %d: INT never asserted after reset", attempt + 1);
            continue;
        }

        // Step 4: Drain init packets and embed Product ID request.
        // BNO085 requires continuous SPI clock activity (heartbeats) to drive its SHTP
        // state machine. The [04 00 00 00] heartbeat in bno085_read_packet handles this.
        //
        // We embed the Product ID request inside a full 512-byte SPI transaction (not a
        // separate 6-byte send) because a separate send fails intermittently when BNO085
        // has pending data buffered — the 512-byte combined tx/rx handles both at once.

        int drain_count = 0;
        bool pid_sent   = false;
        int  idle_count = 0;
        int  n          = -1;

        for (int i = 0; i < 80; i++) {
            int pkt_n;

            if (!pid_sent && drain_count >= 1) {
                // Build a 512-byte TX buffer: Product ID request header + zero padding
                static uint8_t tx_cmd[SHTP_MAX_PACKET];
                memset(tx_cmd, 0x00, SHTP_MAX_PACKET);
                uint8_t pid_payload[2] = { SHTP_REPORT_PRODUCT_ID_REQUEST, 0 };
                shtp_build_packet(SHTP_CHANNEL_CONTROL, s_bno085.seq_control,
                                  pid_payload, 2, tx_cmd);
                s_bno085.seq_control = (s_bno085.seq_control + 1) & 0xFF;

                spi_transfer(tx_cmd, rx_buf, SHTP_MAX_PACKET);
                pid_sent = true;

                uint16_t rx_len = (uint16_t)rx_buf[0] | ((uint16_t)(rx_buf[1] & 0x7F) << 8);
                pkt_n = (rx_len >= SHTP_HEADER_LEN && rx_len <= SHTP_MAX_PACKET)
                         ? (int)rx_len : -1;
            } else {
                pkt_n = bno085_read_packet(rx_buf, sizeof(rx_buf));
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
                    ESP_LOGD(TAG, "Product ID response found (drain_count=%d)", drain_count);
                }
            } else {
                idle_count++;
            }

            if (found_product_id && idle_count >= 3) break;
            if (pid_sent && idle_count >= 15) break;
            if (!pid_sent && idle_count >= 30) break;

            vTaskDelay(pdMS_TO_TICKS(5));
        }

        if (found_product_id && n >= (int)(SHTP_HEADER_LEN + 2)) {
            break;
        }
    }

    if (!found_product_id) {
        ESP_LOGE(TAG, "BNO085: No product ID response after %d attempts. Check wiring:", MAX_ATTEMPTS);
        ESP_LOGE(TAG, "  CS=GPIO%d  SCLK=GPIO%d  MOSI=GPIO%d  MISO=GPIO%d",
                 cfg->cs_pin, cfg->sclk_pin, cfg->mosi_pin, cfg->miso_pin);
        ESP_LOGE(TAG, "  INT=GPIO%d  RST=GPIO%d", cfg->int_pin, cfg->rst_pin);
        ESP_LOGE(TAG, "  PS0 and PS1 must be tied to 3.3V for SPI mode");
        spi_bus_remove_device(s_bno085.spi_handle);
        spi_bus_free(host);
        return false;
    }

    // Log product part number if present in response
    // Note: saved_pid_buf is a stack array — checking it as a pointer always
    // evaluates true and triggers -Werror=address. Check the length byte instead.
    if ((int)(SHTP_HEADER_LEN + 8) <= (int)saved_pid_buf[0]) {
        uint32_t part_no =
            (uint32_t) saved_pid_buf[SHTP_HEADER_LEN + 4]        |
            ((uint32_t)saved_pid_buf[SHTP_HEADER_LEN + 5] << 8)  |
            ((uint32_t)saved_pid_buf[SHTP_HEADER_LEN + 6] << 16) |
            ((uint32_t)saved_pid_buf[SHTP_HEADER_LEN + 7] << 24);
        ESP_LOGI(TAG, "BNO085 Part Number: 0x%08"PRIx32, part_no);
    }

    // --- Drain remaining FRS records ---
    // After soft reset BNO085 sends FRS (Flash Record System) data on ch0.
    // We must drain these before enabling feature reports or they interfere.
    int frs_drained = 0;
    for (int i = 0; i < 200; i++) {
        if (cfg->int_pin >= 0 && gpio_get_level(cfg->int_pin) != 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            if (gpio_get_level(cfg->int_pin) != 0) break;
        }
        int pkt_n = bno085_read_packet(rx_buf, sizeof(rx_buf));
        if (pkt_n > 0) frs_drained++;
        else vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGD(TAG, "FRS drain: %d packets consumed", frs_drained);

    // --- Enable Game Rotation Vector report ---
    // This gives us orientation as a quaternion (no magnetometer dependency).
    // Best for relative posture tracking — drifts very slowly.
    ESP_LOGI(TAG, "Enabling Game Rotation Vector at %d Hz (%"PRIu32" us interval)",
             IMU_SAMPLE_RATE_HZ, (uint32_t)BNO085_REPORT_INTERVAL_US);

    if (bno085_enable_report(SH2_REPORTID_GAME_ROTATION_VECTOR, BNO085_REPORT_INTERVAL_US)) {
        s_bno085.game_rv_enabled = true;
        ESP_LOGI(TAG, "Game Rotation Vector enabled");
    } else {
        // Fallback: enable raw accelerometer report
        ESP_LOGW(TAG, "Game RV enable failed; falling back to accelerometer report");
        bno085_enable_report(SH2_REPORTID_ACCELEROMETER, BNO085_REPORT_INTERVAL_US);
        s_bno085.game_rv_enabled = false;
    }

    s_bno085.initialized = true;
    ESP_LOGI(TAG, "BNO085 init complete (game_rv=%s)", s_bno085.game_rv_enabled ? "ON" : "FALLBACK");
    return true;
}

bool bno085_read_orientation(SensorContext_t *ctx, imu_sample_t *out) {
    if (!ctx || !out || !s_bno085.initialized) {
        if (out) { out->valid = false; }
        return false;
    }

    bno085_spi_config_t *cfg = s_bno085.cfg;

    // Wait up to 20ms for INT to assert (data ready)
    bno085_wait_int(cfg->int_pin, 20);

    uint8_t rx_buf[SHTP_MAX_PACKET];
    int n = bno085_read_packet(rx_buf, sizeof(rx_buf));
    if (n < SHTP_HEADER_LEN) {
        // No new packet — return cached last sample (non-fatal; just no new data)
        *out = s_bno085.last_sample;
        out->timestamp_ms = get_timestamp_ms();
        return true;
    }

    uint8_t channel     = rx_buf[2];
    uint16_t payload_len = (uint16_t)(n - SHTP_HEADER_LEN);
    uint8_t *payload    = rx_buf + SHTP_HEADER_LEN;

    // Sensor reports arrive on channel 3
    if (channel != SHTP_CHANNEL_REPORTS || payload_len < 10) {
        // Non-report packet (e.g. advertisement, control response) — ignore
        *out = s_bno085.last_sample;
        out->timestamp_ms = get_timestamp_ms();
        return true;
    }

    out->timestamp_ms = get_timestamp_ms();
    out->valid = false;  // Will be set true by the parser on success

    // Walk the payload — multiple reports can be packed into one SHTP packet
    uint16_t offset = 0;
    while (offset < payload_len) {
        uint8_t report_id  = payload[offset];
        uint16_t remaining = payload_len - offset;

        if (report_id == SH2_REPORTID_GAME_ROTATION_VECTOR && remaining >= 14) {
            if (parse_game_rotation_vector(payload + offset, remaining, out)) {
                out->timestamp_ms = get_timestamp_ms();
                s_bno085.last_sample = *out;
                return true;
            }
            offset += 14;
        } else if (report_id == SH2_REPORTID_ACCELEROMETER && remaining >= 10) {
            if (parse_accelerometer(payload + offset, remaining, out)) {
                out->timestamp_ms = get_timestamp_ms();
                s_bno085.last_sample = *out;
                return true;
            }
            offset += 10;
        } else {
            // Unknown report ID — skip 1 byte and try to resync
            offset++;
        }
    }

    // No recognizable report found — return last valid
    *out = s_bno085.last_sample;
    out->timestamp_ms = get_timestamp_ms();
    return true;
}

// Compatibility wrapper for SensorContext_t vtable signature
bool bno085_read_sample(SensorContext_t *ctx, void *data_out) {
    return bno085_read_orientation(ctx, (imu_sample_t *)data_out);
}
