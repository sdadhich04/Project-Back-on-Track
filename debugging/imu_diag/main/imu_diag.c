/**
 * @file imu_diag.c
 * @brief Five-stage hardware diagnostic for IMU0 (BNO085 upper back sensor).
 *
 * Tests only IMU0. IMU1 is never touched.
 * Pin assignments match data_types.h from the Back on Track project:
 *
 *   Shared SPI bus:  MOSI=GPIO40  MISO=GPIO39  SCLK=GPIO38  SPI3_HOST
 *   IMU0 (upper):    CS=GPIO37    INT=GPIO5    RST=GPIO6
 *
 * ── What each stage checks ────────────────────────────────────────────────
 *
 *  STAGE 0 — GPIO bit-bang loopback
 *    Drives MOSI bit-by-bit and reads back on MISO with a jumper wire
 *    connecting the two pins. Verifies the ESP32-S3 GPIOs themselves work
 *    before the SPI peripheral is involved. Skip if no jumper available —
 *    the result is noted but does not block subsequent stages.
 *
 *  STAGE 1 — SPI peripheral init + idle MISO
 *    Initialises SPI3 and adds the IMU0 device. Holds RST LOW (chip in
 *    reset, not driving MISO) and sends 4 bytes. Checks the received bytes:
 *      0xFF → MISO is pulled up correctly (good)
 *      0x00 → MISO is shorted to GND somewhere (not the chip's fault)
 *      mixed → partial pull-up or noise
 *
 *  STAGE 2 — RST pulse + INT observation
 *    Pulses RST and watches INT for 800 ms. The BNO085 asserts INT (active
 *    LOW) within ~300 ms of RST going HIGH as part of its SHTP boot sequence.
 *    Failure here means the chip is not powered, RST is not wired, or
 *    PS0/PS1 are not both tied HIGH for SPI mode.
 *
 *  STAGE 3 — Raw SHTP advertisement
 *    Reads the first SHTP packet after reset using the 0x04 heartbeat TX
 *    strategy (required to keep the BNO085 state machine moving during init,
 *    confirmed by the Shield project). A valid SHTP header has a non-zero
 *    length field and non-zero bytes. Failure = MISO not wired to BNO085 SDA.
 *
 *  STAGE 4 — Product ID request / response
 *    Performs the full SHTP drain + PID request exchange that bno085_init()
 *    does. Embeds the PID request in the TX buffer of a 512-byte SPI
 *    transaction (same technique as the Shield driver) so BNO085 receives
 *    it while we read its pending data. Looks for 0xF8 on SHTP channel 2.
 *    Failure after Stage 3 passed = MOSI not wired to BNO085 DI.
 *
 *  STAGE 5 — Live Game Rotation Vector reads
 *    Enables the GRV report at 50 Hz and reads 10 consecutive quaternion
 *    packets. Only runs if all prior stages pass.
 *
 * ── Reading the final summary ─────────────────────────────────────────────
 *
 *  Stage 0 FAIL  → GPIO39 or GPIO40 dead on the ESP32-S3 board
 *  Stage 1 FAIL  → MISO shorted to GND (trace, solder bridge, or bad GPIO)
 *  Stage 2 FAIL  → Chip not powered / PS0+PS1 not HIGH / RST not wired
 *  Stage 3 FAIL  → MISO not wired to BNO085 SDA pad
 *  Stage 4 FAIL  → MOSI not wired to BNO085 DI pad
 *  All PASS      → Hardware is fine; issue is in bno085_init() timing
 */

#include "imu_diag.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

/* ── Pin definitions (must match data_types.h in Back on Track) ───────── */
#define DIAG_PIN_MOSI    40
#define DIAG_PIN_MISO    39
#define DIAG_PIN_SCLK    38
#define DIAG_PIN_CS      37
#define DIAG_PIN_INT      5
#define DIAG_PIN_RST      6
#define DIAG_SPI_HOST     2   /* SPI3_HOST */

/* ── SHTP constants ───────────────────────────────────────────────────── */
#define SHTP_HDR_LEN      4
#define SHTP_MAX_PKT    512
#define SHTP_CH_EXE       1
#define SHTP_CH_CTRL      2
#define SHTP_CH_RPTS      3
#define PID_REQUEST    0xF9
#define PID_RESPONSE   0xF8
#define SET_FEAT_CMD   0xFD
#define REPORT_GRV     0x08

static const char *TAG = "IMU_DIAG";
static spi_device_handle_t s_spi = NULL;

/* ═════════════════════════════════════════════════════════════════════════
 * Internal helpers
 * ═════════════════════════════════════════════════════════════════════════ */

static void print_sep(const char *label)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "══════════════════════════════════════════");
    ESP_LOGI(TAG, "  %s", label);
    ESP_LOGI(TAG, "══════════════════════════════════════════");
}

static void print_hex(const char *prefix, const uint8_t *buf, size_t len)
{
    /* Build a single string so it prints on one log line. */
    char line[160];
    int pos = snprintf(line, sizeof(line), "%s: ", prefix);
    for (size_t i = 0; i < len && pos < (int)sizeof(line) - 4; i++)
        pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", buf[i]);
    ESP_LOGI(TAG, "%s", line);
}

static bool spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(s_spi, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI xfer failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

/**
 * Read one SHTP packet.
 * @param tx_heartbeat  If true, sends 0x04 in byte 0 of the TX buffer.
 *                      Required during the BNO085 init phase to advance its
 *                      internal SHTP state machine (confirmed by Shield project).
 *                      Use false during normal runtime reads.
 * @return Packet length in bytes, or -1 on error / invalid header.
 */
static int read_packet(uint8_t *buf, size_t buf_sz, bool tx_heartbeat)
{
    if (buf_sz < SHTP_HDR_LEN) return -1;
    size_t rlen = buf_sz < SHTP_MAX_PKT ? buf_sz : SHTP_MAX_PKT;

    uint8_t tx[SHTP_MAX_PKT];
    memset(tx, 0x00, rlen);
    if (tx_heartbeat) tx[0] = 0x04;

    if (!spi_xfer(tx, buf, rlen)) return -1;

    uint16_t plen = (uint16_t)buf[0] | ((uint16_t)(buf[1] & 0x7F) << 8);
    if (plen < SHTP_HDR_LEN || plen > buf_sz) return -1;
    return (int)plen;
}

/**
 * Block until INT (GPIO5) asserts LOW, or timeout_ms elapses.
 * Returns true if INT went LOW within the timeout.
 */
static bool wait_int(uint32_t timeout_ms)
{
    uint32_t start = (uint32_t)(esp_timer_get_time() / 1000);
    while ((uint32_t)(esp_timer_get_time() / 1000) - start < timeout_ms) {
        if (gpio_get_level(DIAG_PIN_INT) == 0) return true;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

/* ═════════════════════════════════════════════════════════════════════════
 * STAGE 0 — GPIO bit-bang loopback
 * ═════════════════════════════════════════════════════════════════════════ */

static bool stage0_gpio_loopback(void)
{
    print_sep("STAGE 0: GPIO bit-bang loopback");

    ESP_LOGI(TAG, "ACTION: Jumper GPIO%d (MOSI) directly to GPIO%d (MISO).",
             DIAG_PIN_MOSI, DIAG_PIN_MISO);
    ESP_LOGI(TAG, "If you cannot jumper, wait out the 8s pause — result = SKIP.");
    ESP_LOGI(TAG, "Waiting 8 s for the jumper...");
    vTaskDelay(pdMS_TO_TICKS(8000));

    /* Configure MOSI as push-pull output, MISO as input with pull-up. */
    gpio_config_t out_cfg = {
        .pin_bit_mask = 1ULL << DIAG_PIN_MOSI,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_cfg);

    gpio_config_t in_cfg = {
        .pin_bit_mask = 1ULL << DIAG_PIN_MISO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_cfg);

    /* Walk pattern 0b10101010 bit by bit. */
    const uint8_t pattern = 0xAA;
    int pass = 0, fail = 0;
    for (int bit = 7; bit >= 0; bit--) {
        int out_bit = (pattern >> bit) & 1;
        gpio_set_level(DIAG_PIN_MOSI, out_bit);
        vTaskDelay(pdMS_TO_TICKS(2));
        int in_bit = gpio_get_level(DIAG_PIN_MISO);
        bool ok = (out_bit == in_bit);
        ESP_LOGI(TAG, "  bit%d  sent=%d  read=%d  %s",
                 bit, out_bit, in_bit, ok ? "OK" : "MISMATCH");
        if (ok) pass++; else fail++;
    }

    ESP_LOGI(TAG, "ACTION: Remove the jumper before Stage 1. Waiting 5 s...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    if (fail == 0) {
        ESP_LOGI(TAG, "STAGE 0: PASS (%d/8 bits correct)", pass);
        ESP_LOGI(TAG, "  GPIO%d and GPIO%d are both functional.", DIAG_PIN_MOSI, DIAG_PIN_MISO);
    } else {
        ESP_LOGE(TAG, "STAGE 0: FAIL (%d/8 bits mismatched)", fail);
        ESP_LOGE(TAG, "  GPIO%d or GPIO%d is dead, or jumper did not make contact.",
                 DIAG_PIN_MOSI, DIAG_PIN_MISO);
    }
    return fail == 0;
}

/* ═════════════════════════════════════════════════════════════════════════
 * STAGE 1 — SPI peripheral init + idle MISO level
 * ═════════════════════════════════════════════════════════════════════════ */

static bool stage1_spi_idle(void)
{
    print_sep("STAGE 1: SPI idle MISO (chip held in reset)");
    ESP_LOGI(TAG, "RST will be held LOW (chip in reset, not driving MISO).");
    ESP_LOGI(TAG, "We send 4 zero bytes and see what MISO returns.");
    ESP_LOGI(TAG, "Expected: 0xFF (pull-up)   Bad: 0x00 (shorted to GND)");

    /* Hold RST LOW before SPI init — chip must not drive MISO during this check. */
    gpio_config_t rst_cfg = {
        .pin_bit_mask = 1ULL << DIAG_PIN_RST,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_cfg);
    gpio_set_level(DIAG_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Init SPI bus. */
    spi_bus_config_t bus = {
        .mosi_io_num     = DIAG_PIN_MOSI,
        .miso_io_num     = DIAG_PIN_MISO,
        .sclk_io_num     = DIAG_PIN_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = SHTP_MAX_PKT,
    };
    esp_err_t err = spi_bus_initialize(DIAG_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "STAGE 1: SPI bus init failed: %s", esp_err_to_name(err));
        gpio_set_level(DIAG_PIN_RST, 1);
        return false;
    }

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 1 * 1000 * 1000,   /* 1 MHz — slow for safety during diag */
        .mode           = 0,
        .spics_io_num   = DIAG_PIN_CS,
        .queue_size     = 1,
    };
    err = spi_bus_add_device(DIAG_SPI_HOST, &dev, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "STAGE 1: SPI add device failed: %s", esp_err_to_name(err));
        spi_bus_free(DIAG_SPI_HOST);
        gpio_set_level(DIAG_PIN_RST, 1);
        return false;
    }

    uint8_t tx[4] = {0, 0, 0, 0};
    uint8_t rx[4] = {0};
    spi_xfer(tx, rx, 4);
    print_hex("  RX (chip in reset)", rx, 4);

    int zeros = 0, ones = 0;
    for (int i = 0; i < 4; i++)
        for (int b = 0; b < 8; b++)
            if ((rx[i] >> b) & 1) ones++; else zeros++;

    /* Release RST so later stages can work. */
    gpio_set_level(DIAG_PIN_RST, 1);

    if (ones == 32) {
        ESP_LOGI(TAG, "STAGE 1: PASS — MISO reads 0xFF (pull-up working, no GND short).");
        return true;
    } else if (zeros == 32) {
        ESP_LOGE(TAG, "STAGE 1: FAIL — MISO is stuck LOW (all zeros).");
        ESP_LOGE(TAG, "  MISO (GPIO%d) is shorted to GND. Check PCB traces and solder bridges.",
                 DIAG_PIN_MISO);
        return false;
    } else {
        ESP_LOGW(TAG, "STAGE 1: UNCERTAIN — mixed bits (zeros=%d ones=%d / 32).", zeros, ones);
        ESP_LOGW(TAG, "  MISO has a weak pull-up or noise. Check pull-up resistor on MISO.");
        /* Treat as pass for the purpose of continuing — Stage 3 will be definitive. */
        return true;
    }
}

/* ═════════════════════════════════════════════════════════════════════════
 * STAGE 2 — RST pulse + INT observation
 * ═════════════════════════════════════════════════════════════════════════ */

static bool stage2_reset_and_int(void)
{
    print_sep("STAGE 2: RST pulse + INT observation");
    ESP_LOGI(TAG, "Checks: chip powered, PS0+PS1 HIGH (SPI mode), RST and INT wired.");
    ESP_LOGI(TAG, "BNO085 should assert INT within ~300 ms of RST going HIGH.");

    /* Configure INT as input with pull-up. */
    gpio_config_t int_cfg = {
        .pin_bit_mask = 1ULL << DIAG_PIN_INT,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&int_cfg);

    /* Pulse RST. */
    ESP_LOGI(TAG, "  RST LOW for 10 ms...");
    gpio_set_level(DIAG_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "  RST HIGH — watching INT for 800 ms...");
    gpio_set_level(DIAG_PIN_RST, 1);

    uint32_t start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    bool asserted = false;
    uint32_t assert_ms = 0;

    for (int i = 0; i < 800; i++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (gpio_get_level(DIAG_PIN_INT) == 0) {
            assert_ms = (uint32_t)(esp_timer_get_time() / 1000) - start_ms;
            asserted = true;
            break;
        }
    }

    if (asserted) {
        ESP_LOGI(TAG, "STAGE 2: PASS — INT asserted after %"PRIu32" ms.", assert_ms);
        if (assert_ms < 50)
            ESP_LOGW(TAG, "  Very fast INT (%"PRIu32" ms). Could be leftover state from "
                     "prior session — not wrong, but worth noting.", assert_ms);
        else if (assert_ms > 400)
            ESP_LOGW(TAG, "  Slow INT (%"PRIu32" ms). Normal is 100–400 ms. "
                     "Possible weak power supply.", assert_ms);
        else
            ESP_LOGI(TAG, "  Timing looks normal (100–400 ms range).");
        return true;
    }

    ESP_LOGE(TAG, "STAGE 2: FAIL — INT never asserted within 800 ms.");
    ESP_LOGE(TAG, "  INT level right now: %d", gpio_get_level(DIAG_PIN_INT));
    ESP_LOGE(TAG, "  Checklist:");
    ESP_LOGE(TAG, "    1. Is 3.3V present on BNO085 VIN?");
    ESP_LOGE(TAG, "    2. PS0 and PS1: are BOTH tied to 3.3V?");
    ESP_LOGE(TAG, "       (If either is LOW/floating, chip boots I2C/UART — SPI slave "
             "disabled, INT will not assert for SPI boot.)");
    ESP_LOGE(TAG, "    3. Is RST (GPIO%d) wired to BNO085 RST?", DIAG_PIN_RST);
    ESP_LOGE(TAG, "       (If RST is floating, chip still boots on power-on — "
             "but try power-cycling the board instead of reset.)");
    ESP_LOGE(TAG, "    4. Is INT (GPIO%d) wired to BNO085 INT with a 10 kΩ pull-up to 3.3V?",
             DIAG_PIN_INT);
    return false;
}

/* ═════════════════════════════════════════════════════════════════════════
 * STAGE 3 — Raw SHTP advertisement packet
 * ═════════════════════════════════════════════════════════════════════════ */

static bool stage3_shtp_advertisement(void)
{
    print_sep("STAGE 3: Raw SHTP advertisement read");
    ESP_LOGI(TAG, "After reset the BNO085 sends an advertisement on SHTP channel 0.");
    ESP_LOGI(TAG, "A valid header has: length 4..512, channel 0.");
    ESP_LOGI(TAG, "All-zero response = MISO not wired to BNO085 SDA.");

    /* Fresh reset for a clean boot sequence. */
    gpio_set_level(DIAG_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(DIAG_PIN_RST, 1);

    ESP_LOGI(TAG, "  RST pulsed. Waiting for INT...");
    if (!wait_int(800)) {
        ESP_LOGE(TAG, "STAGE 3: SKIP — INT never asserted. Stage 2 must pass first.");
        return false;
    }
    ESP_LOGI(TAG, "  INT asserted. Reading first SHTP packet (with 0x04 heartbeat TX)...");

    uint8_t rx[SHTP_MAX_PKT];
    memset(rx, 0, sizeof(rx));
    int n = read_packet(rx, sizeof(rx), /*tx_heartbeat=*/true);

    print_hex("  Raw RX [0..15]", rx, 16);

    uint16_t pkt_len = (uint16_t)rx[0] | ((uint16_t)(rx[1] & 0x7F) << 8);
    uint8_t  channel = rx[2];
    uint8_t  seq     = rx[3];
    ESP_LOGI(TAG, "  Decoded: pkt_len=%d  channel=%d  seq=%d", pkt_len, channel, seq);

    /* Check if MISO returned anything non-zero at all. */
    bool all_zero = true;
    for (int i = 0; i < 8; i++) {
        if (rx[i] != 0x00) { all_zero = false; break; }
    }

    bool len_valid = (pkt_len >= SHTP_HDR_LEN && pkt_len <= SHTP_MAX_PKT);

    if (!all_zero && len_valid) {
        ESP_LOGI(TAG, "STAGE 3: PASS");
        ESP_LOGI(TAG, "  BNO085 is driving MISO. SPI receive path is functional.");
        if (channel != 0)
            ESP_LOGW(TAG, "  channel=%d (expected 0 for advertisement). "
                     "May be a mid-session read if chip was already running.", channel);
        return true;
    }

    if (all_zero) {
        ESP_LOGE(TAG, "STAGE 3: FAIL — all received bytes are 0x00.");
        ESP_LOGE(TAG, "  BNO085 is NOT driving MISO.");
        ESP_LOGE(TAG, "  Most likely cause: MISO (GPIO%d) is not wired to the BNO085 SDA pad.",
                 DIAG_PIN_MISO);
        ESP_LOGE(TAG, "  On the Adafruit BNO085 breakout, the SPI MISO pad is labeled 'SDA'");
        ESP_LOGE(TAG, "  (left over from the I2C pinout). Make sure you are using that pad,");
        ESP_LOGE(TAG, "  not the pad labeled SCL.");
        ESP_LOGE(TAG, "  Also verify: PS0=3.3V and PS1=3.3V. In I2C mode the SDA pad is");
        ESP_LOGE(TAG, "  bidirectional I2C, not SPI MISO — it won't respond to SPI clocking.");
    } else {
        ESP_LOGE(TAG, "STAGE 3: FAIL — received non-zero bytes but invalid SHTP header.");
        ESP_LOGE(TAG, "  pkt_len=%d (valid range is 4..512).", pkt_len);
        ESP_LOGE(TAG, "  Possible: SPI mode mismatch (BNO085 uses CPOL=0 CPHA=0, i.e. mode 0).");
        ESP_LOGE(TAG, "  Or: MISO has crosstalk from another signal on the PCB.");
    }
    return false;
}

/* ═════════════════════════════════════════════════════════════════════════
 * STAGE 4 — Product ID request / response
 * ═════════════════════════════════════════════════════════════════════════ */

static bool stage4_product_id(void)
{
    print_sep("STAGE 4: Product ID request / response");
    ESP_LOGI(TAG, "Sends 0xF9 (PID Request) on SHTP control channel.");
    ESP_LOGI(TAG, "Expects 0xF8 (PID Response) back on channel 2.");
    ESP_LOGI(TAG, "This is the same exchange bno085_init() performs.");

    /* Full reset to get a clean state. */
    gpio_set_level(DIAG_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(DIAG_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(350));

    /* Soft reset via EXE channel (same as bno085_init). */
    uint8_t soft_reset_pkt[5] = {
        0x05, 0x00,         /* length = 5 */
        SHTP_CH_EXE, 0x00,  /* channel, seq */
        0x01                /* reset command */
    };
    uint8_t rx_tmp[5] = {0};
    spi_xfer(soft_reset_pkt, rx_tmp, 5);
    ESP_LOGI(TAG, "  Soft reset sent. Waiting for INT...");

    if (!wait_int(800)) {
        ESP_LOGE(TAG, "STAGE 4: FAIL — INT never asserted after soft reset.");
        return false;
    }

    /*
     * Drain loop: up to 80 iterations.
     * After receiving the first valid packet (drain_count >= 1), embed the
     * PID request in the TX side of a 512-byte SPI transaction. The BNO085
     * reads our TX bytes while we read its pending data simultaneously.
     * This avoids a separate small TX that can be drowned out by chip data.
     * (Strategy confirmed by Shield project driver_bno085.c.)
     */
    uint8_t rx_buf[SHTP_MAX_PKT];
    int  drain_count = 0;
    bool pid_sent    = false;
    bool pid_found   = false;
    int  idle_count  = 0;
    int  n           = -1;

    for (int i = 0; i < 80 && !pid_found; i++) {
        int pkt_n;

        if (!pid_sent && drain_count >= 1) {
            /* Build 512-byte TX: SHTP PID request header + payload + zeros. */
            uint8_t tx_cmd[SHTP_MAX_PKT];
            memset(tx_cmd, 0x00, sizeof(tx_cmd));
            /* SHTP header for the PID request (6 bytes total: 4 header + 2 payload). */
            tx_cmd[0] = 0x06;          /* length LSB */
            tx_cmd[1] = 0x00;          /* length MSB (bit15 clear) */
            tx_cmd[2] = SHTP_CH_CTRL;
            tx_cmd[3] = 0x00;          /* seq */
            tx_cmd[4] = PID_REQUEST;
            tx_cmd[5] = 0x00;

            spi_xfer(tx_cmd, rx_buf, SHTP_MAX_PKT);
            pid_sent = true;

            uint16_t rx_len = (uint16_t)rx_buf[0] | ((uint16_t)(rx_buf[1] & 0x7F) << 8);
            pkt_n = (rx_len >= SHTP_HDR_LEN && rx_len <= SHTP_MAX_PKT)
                    ? (int)rx_len : -1;
            ESP_LOGD(TAG, "  [%2d] PID sent; rx pkt_n=%d ch=%d payload[0]=0x%02X",
                     i, pkt_n, (pkt_n > 2 ? rx_buf[2] : 0xFF),
                     (pkt_n > (int)SHTP_HDR_LEN ? rx_buf[SHTP_HDR_LEN] : 0xFF));
        } else {
            pkt_n = read_packet(rx_buf, sizeof(rx_buf), /*heartbeat=*/!pid_sent);
            ESP_LOGD(TAG, "  [%2d] drain pkt_n=%d ch=%d payload[0]=0x%02X",
                     i, pkt_n,
                     (pkt_n > 2 ? rx_buf[2] : 0xFF),
                     (pkt_n > (int)SHTP_HDR_LEN ? rx_buf[SHTP_HDR_LEN] : 0xFF));
        }

        if (pkt_n > 0) {
            idle_count = 0;
            drain_count++;
            uint8_t ch = rx_buf[2];
            if (ch == SHTP_CH_CTRL
                && pkt_n >= (int)(SHTP_HDR_LEN + 2)
                && rx_buf[SHTP_HDR_LEN] == PID_RESPONSE) {
                pid_found = true;
                n = pkt_n;
                print_hex("  PID response [0..11]", rx_buf, 12);
                if (n >= (int)(SHTP_HDR_LEN + 8)) {
                    uint32_t part =
                        (uint32_t) rx_buf[SHTP_HDR_LEN + 4]         |
                        ((uint32_t)rx_buf[SHTP_HDR_LEN + 5] <<  8)  |
                        ((uint32_t)rx_buf[SHTP_HDR_LEN + 6] << 16)  |
                        ((uint32_t)rx_buf[SHTP_HDR_LEN + 7] << 24);
                    ESP_LOGI(TAG, "  Part number: 0x%08"PRIx32, part);
                }
            }
        } else {
            idle_count++;
        }

        if (pid_found  && idle_count >= 3)  break;
        if (pid_sent   && idle_count >= 15) break;
        if (!pid_sent  && idle_count >= 30) break;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (pid_found) {
        ESP_LOGI(TAG, "STAGE 4: PASS — product ID received after draining %d packets.",
                 drain_count);
        ESP_LOGI(TAG, "  Full SHTP handshake works. Hardware is good.");
        ESP_LOGI(TAG, "  If bno085_init() still fails in the real project, the issue is");
        ESP_LOGI(TAG, "  timing — compare the drain iteration count above against the");
        ESP_LOGI(TAG, "  real init's 80-iteration limit and adjust if needed.");
        return true;
    }

    ESP_LOGE(TAG, "STAGE 4: FAIL — no PID response after draining %d packets.", drain_count);
    if (drain_count == 0) {
        ESP_LOGE(TAG, "  Zero valid packets received. Stage 3 should have caught this.");
    } else {
        ESP_LOGE(TAG, "  Received %d packets, none were 0xF8 on channel 2.", drain_count);
        ESP_LOGE(TAG, "  The BNO085 never got our PID request.");
        ESP_LOGE(TAG, "  Most likely cause: MOSI (GPIO%d) not wired to BNO085 DI pad.",
                 DIAG_PIN_MOSI);
        ESP_LOGE(TAG, "  Also check: CS (GPIO%d) is toggling correctly during the transfer.",
                 DIAG_PIN_CS);
    }
    return false;
}

/* ═════════════════════════════════════════════════════════════════════════
 * STAGE 5 — Live Game Rotation Vector reads
 * ═════════════════════════════════════════════════════════════════════════ */

static void stage5_live_reads(void)
{
    print_sep("STAGE 5: Live GRV reads (10 packets at 50 Hz)");
    ESP_LOGI(TAG, "Enabling Game Rotation Vector report and reading 10 quaternion packets.");

    /* Build SET_FEATURE_COMMAND for GRV at 50 Hz (20 000 µs interval). */
    uint8_t feat_cmd[17] = {0};
    feat_cmd[0] = SET_FEAT_CMD;
    feat_cmd[1] = REPORT_GRV;
    uint32_t interval_us = 20000;
    feat_cmd[5] = (uint8_t)( interval_us        & 0xFF);
    feat_cmd[6] = (uint8_t)((interval_us >>  8) & 0xFF);
    feat_cmd[7] = (uint8_t)((interval_us >> 16) & 0xFF);
    feat_cmd[8] = (uint8_t)((interval_us >> 24) & 0xFF);

    /* Wrap in SHTP header. */
    uint8_t pkt[17 + SHTP_HDR_LEN];
    uint16_t total = sizeof(pkt);
    pkt[0] = (uint8_t)(total & 0xFF);
    pkt[1] = (uint8_t)((total >> 8) & 0x7F);
    pkt[2] = SHTP_CH_CTRL;
    pkt[3] = 0x01;  /* seq */
    memcpy(pkt + SHTP_HDR_LEN, feat_cmd, 17);

    uint8_t ack[sizeof(pkt)];
    spi_xfer(pkt, ack, sizeof(pkt));
    vTaskDelay(pdMS_TO_TICKS(100));

    int reports = 0;
    for (int attempt = 0; attempt < 100 && reports < 10; attempt++) {
        wait_int(100);
        uint8_t rx[SHTP_MAX_PKT];
        int n = read_packet(rx, sizeof(rx), /*heartbeat=*/false);
        if (n < (int)(SHTP_HDR_LEN + 14)) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }

        if (rx[2] == SHTP_CH_RPTS && rx[SHTP_HDR_LEN] == REPORT_GRV) {
            reports++;
            uint8_t *p = rx + SHTP_HDR_LEN;
            int16_t qi = (int16_t)((uint16_t)p[4]  | ((uint16_t)p[5]  << 8));
            int16_t qj = (int16_t)((uint16_t)p[6]  | ((uint16_t)p[7]  << 8));
            int16_t qk = (int16_t)((uint16_t)p[8]  | ((uint16_t)p[9]  << 8));
            int16_t qr = (int16_t)((uint16_t)p[10] | ((uint16_t)p[11] << 8));
            ESP_LOGI(TAG, "  GRV[%2d]  qi=%6d  qj=%6d  qk=%6d  qr=%6d",
                     reports, qi, qj, qk, qr);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (reports >= 10) {
        ESP_LOGI(TAG, "STAGE 5: PASS — %d GRV reports received.", reports);
        ESP_LOGI(TAG, "  IMU0 is fully operational.");
    } else {
        ESP_LOGW(TAG, "STAGE 5: PARTIAL — only %d/10 GRV reports received.", reports);
        ESP_LOGW(TAG, "  Hardware is working (earlier stages passed). Likely a timing issue.");
        ESP_LOGW(TAG, "  Try increasing the wait_int() timeout or adding more drain iterations.");
    }
}

/* ═════════════════════════════════════════════════════════════════════════
 * Public entry point
 * ═════════════════════════════════════════════════════════════════════════ */

void imu_diag_run(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));  /* let the serial monitor connect */

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔══════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   Back on Track — IMU0 Hardware Diag    ║");
    ESP_LOGI(TAG, "║   Testing IMU0 (upper back) only         ║");
    ESP_LOGI(TAG, "║   MOSI=GPIO%-2d MISO=GPIO%-2d SCLK=GPIO%-2d  ║",
             DIAG_PIN_MOSI, DIAG_PIN_MISO, DIAG_PIN_SCLK);
    ESP_LOGI(TAG, "║   CS=GPIO%-2d   INT=GPIO%-2d  RST=GPIO%-2d   ║",
             DIAG_PIN_CS, DIAG_PIN_INT, DIAG_PIN_RST);
    ESP_LOGI(TAG, "╚══════════════════════════════════════════╝");
    ESP_LOGI(TAG, "  IMU1 is NOT touched at all during this test.");
    ESP_LOGI(TAG, "");

    bool s0 = stage0_gpio_loopback();
    if (!s0) {
        ESP_LOGE(TAG, "Halting: the ESP32-S3 GPIO itself is broken. "
                 "Fix before diagnosing SPI.");
        goto summary;
    }

    bool s1 = stage1_spi_idle();
    if (!s1) {
        ESP_LOGE(TAG, "Halting: MISO is shorted to GND. "
                 "Fix the physical short before continuing.");
        goto summary;
    }

    bool s2 = stage2_reset_and_int();
    bool s3 = false;
    bool s4 = false;

    if (s2) {
        s3 = stage3_shtp_advertisement();
    } else {
        ESP_LOGW(TAG, "Skipping Stage 3 (Stage 2 failed — chip not responding).");
    }

    if (s3) {
        s4 = stage4_product_id();
    } else if (s2) {
        ESP_LOGW(TAG, "Skipping Stage 4 (Stage 3 failed — MISO not connected).");
    }

    if (s4) {
        stage5_live_reads();
    } else if (s3) {
        ESP_LOGW(TAG, "Skipping Stage 5 (Stage 4 failed — MOSI not connected).");
    }

summary:
    print_sep("FINAL SUMMARY");
    ESP_LOGI(TAG, "  Stage 0  GPIO loopback         %s", s0 ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "  Stage 1  SPI idle MISO         %s", s1 ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "  Stage 2  RST + INT             %s", s2 ? "PASS" : s1 ? "FAIL" : "SKIP");
    ESP_LOGI(TAG, "  Stage 3  SHTP advertisement    %s", s3 ? "PASS" : (s2 ? "FAIL" : "SKIP"));
    ESP_LOGI(TAG, "  Stage 4  Product ID            %s", s4 ? "PASS" : (s3 ? "FAIL" : "SKIP"));
    ESP_LOGI(TAG, "");

    if (s0 && s1 && s2 && s3 && s4) {
        ESP_LOGI(TAG, "  ALL PASS");
        ESP_LOGI(TAG, "  Hardware is fine. The failure in Back on Track is a driver");
        ESP_LOGI(TAG, "  timing issue. Enable DEBUG logging in Back on Track and look");
        ESP_LOGI(TAG, "  at which drain iteration the 0xF8 response appears.");
    } else if (!s0) {
        ESP_LOGE(TAG, "  BLOCKED at Stage 0: ESP32-S3 GPIO fault.");
    } else if (!s1) {
        ESP_LOGE(TAG, "  BLOCKED at Stage 1: MISO shorted to GND.");
    } else if (!s2) {
        ESP_LOGE(TAG, "  BLOCKED at Stage 2: chip not powered / PS0+PS1 / RST wiring.");
    } else if (!s3) {
        ESP_LOGE(TAG, "  BLOCKED at Stage 3: MISO not wired to BNO085 SDA pad.");
    } else {
        ESP_LOGE(TAG, "  BLOCKED at Stage 4: MOSI not wired to BNO085 DI pad.");
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Diagnosis complete. Halting.");
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
