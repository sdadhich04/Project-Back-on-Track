/**
 * @file main.cpp
 * @brief Back on Track — Main application (Dual-IMU + BLE version)
 *
 * System Architecture
 * ───────────────────
 * ┌──────────────────────────────────────────────────────────────────┐
 * │  Core 0 — Sensor Acquisition                                     │
 * │                                                                  │
 * │  imu_upper_task  (50 Hz)  BNO085 CS=GPIO37 → imu_queue          │
 * │  imu_lower_task  (50 Hz)  BNO085 CS=GPIO33 → imu_queue          │
 * │  emg_task       (100 Hz)  MyoWare GPIO4   → emg_queue           │
 * ├──────────────────────────────────────────────────────────────────┤
 * │  Core 1 — Processing + Output                                    │
 * │                                                                  │
 * │  processing_task (20 Hz)                                         │
 * │    1. Drain queues → latest_upper, latest_lower, latest_emg     │
 * │    2. ble_notify_sensor_packet() → raw data over BLE            │
 * │    3. posture_features_compute() → feature vector               │
 * │    4. posture_classify() → local rule-based classification      │
 * │    5. telemetry_print() → Serial CSV/JSON (debug only)          │
 * └──────────────────────────────────────────────────────────────────┘
 *
 * Two-layer output design
 * ───────────────────────
 * BLE (primary):    Raw bot_ble_packet_t at BLE_NOTIFY_RATE_HZ (20 Hz).
 *                   Contains only raw sensor values — no classification.
 *                   Designed for consumption by an external signal
 *                   processing system (phone app, Python, MATLAB, etc.)
 *
 * Serial (debug):   CSV or JSON at CLASSIFICATION_RATE_HZ (10 Hz).
 *                   Contains classification results + gamification.
 *                   For development only — not part of the data layer.
 *
 * BLE remote control
 * ──────────────────
 * The external signal processing system can write to the BLE control
 * characteristic to trigger:
 *   BOT_CMD_START_CALIBRATE (0x01) — re-run calibration
 *   BOT_CMD_RESET_SESSION   (0x02) — reset session stats
 * These are polled each processing cycle via ble_calibration_requested()
 * and ble_session_reset_requested().
 *
 * SPI Wiring (pins in data_types.h)
 * ──────────────────────────────────
 *   Shared bus:  MOSI=GPIO40  MISO=GPIO39  SCLK=GPIO38  SPI3_HOST
 *   IMU0 upper:  CS=GPIO37    INT=GPIO5    RST=GPIO6
 *   IMU1 lower:  CS=GPIO33    INT=GPIO35   RST=GPIO36
 *   MyoWare:     ENV=GPIO4 (ADC1_CH3)
 */

extern "C" {
    #include "sensor_hal.h"
    #include "data_types.h"
    #include "driver_bno085.h"
    #include "driver_myoware.h"
    #include "posture_features.h"
    #include "posture_classifier.h"
    #include "telemetry_serial.h"
    #include "ble_telemetry.h"

    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
    #include "esp_log.h"
    #include "esp_timer.h"

    void app_main(void);
}

static const char *TAG = "main";

// ==================== Queues ====================

static QueueHandle_t g_imu_queue = NULL;
static QueueHandle_t g_emg_queue = NULL;

// ==================== Hardware Configs ====================

static bno085_spi_config_t g_bno085_upper_cfg = {
    .spi_host  = BNO085_SPI_HOST,
    .cs_pin    = BNO085_0_PIN_CS,
    .sclk_pin  = BNO085_PIN_SCLK,
    .mosi_pin  = BNO085_PIN_MOSI,
    .miso_pin  = BNO085_PIN_MISO,
    .int_pin   = BNO085_0_PIN_INT,
    .rst_pin   = BNO085_0_PIN_RST,
};

static bno085_spi_config_t g_bno085_lower_cfg = {
    .spi_host  = BNO085_SPI_HOST,
    .cs_pin    = BNO085_1_PIN_CS,
    .sclk_pin  = BNO085_PIN_SCLK,
    .mosi_pin  = BNO085_PIN_MOSI,
    .miso_pin  = BNO085_PIN_MISO,
    .int_pin   = BNO085_1_PIN_INT,
    .rst_pin   = BNO085_1_PIN_RST,
};

static myoware_adc_config_t g_myoware_cfg = {
    .adc_unit    = MYOWARE_ADC_UNIT,
    .adc_channel = MYOWARE_ADC_CHANNEL,
    .gpio_pin    = MYOWARE_GPIO_PIN,
    .vref_mv     = 3300,
    .atten       = 3,
};

// ==================== Sensor Contexts ====================

static SensorContext_t g_imu_upper_ctx = {
    .id               = IMU_ID_UPPER,
    .type             = SENSOR_TYPE_IMU_ORIENTATION,
    .sampling_rate_hz = IMU_SAMPLE_RATE_HZ,
    .enabled          = true,
    .hw_config        = &g_bno085_upper_cfg,
    .init             = bno085_init,
    .read_sample      = bno085_read_sample,
};

static SensorContext_t g_imu_lower_ctx = {
    .id               = IMU_ID_LOWER,
    .type             = SENSOR_TYPE_IMU_ORIENTATION,
    .sampling_rate_hz = IMU_SAMPLE_RATE_HZ,
    .enabled          = true,
    .hw_config        = &g_bno085_lower_cfg,
    .init             = bno085_init,
    .read_sample      = bno085_read_sample,
};

static SensorContext_t g_emg_ctx = {
    .id               = 2,
    .type             = SENSOR_TYPE_EMG,
    .sampling_rate_hz = EMG_SAMPLE_RATE_HZ,
    .enabled          = true,
    .hw_config        = &g_myoware_cfg,
    .init             = myoware_init,
    .read_sample      = myoware_read_sample_vtable,
};

static calibration_t   g_calibration = {0};
static session_stats_t g_stats       = {0};

// ==================== Acquisition Tasks ====================

static void imu_upper_task(void *pvParameters) {
    ESP_LOGI(TAG, "imu_upper_task started (IMU0, %d Hz)", IMU_SAMPLE_RATE_HZ);
    const TickType_t period = pdMS_TO_TICKS(1000 / IMU_SAMPLE_RATE_HZ);
    TickType_t last_wake    = xTaskGetTickCount();
    imu_queue_msg_t msg     = { .type = QUEUE_MSG_DATA };

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        if (!g_imu_upper_ctx.enabled) continue;
        if (bno085_read_orientation(&g_imu_upper_ctx, &msg.data)) {
            if (xQueueSend(g_imu_queue, &msg, 0) != pdTRUE) {
                ESP_LOGW(TAG, "IMU0 queue full");
            }
        }
    }
}

static void imu_lower_task(void *pvParameters) {
    ESP_LOGI(TAG, "imu_lower_task started (IMU1, %d Hz)", IMU_SAMPLE_RATE_HZ);
    const TickType_t period = pdMS_TO_TICKS(1000 / IMU_SAMPLE_RATE_HZ);
    TickType_t last_wake    = xTaskGetTickCount();
    imu_queue_msg_t msg     = { .type = QUEUE_MSG_DATA };

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        if (!g_imu_lower_ctx.enabled) continue;
        if (bno085_read_orientation(&g_imu_lower_ctx, &msg.data)) {
            if (xQueueSend(g_imu_queue, &msg, 0) != pdTRUE) {
                ESP_LOGW(TAG, "IMU1 queue full");
            }
        }
    }
}

static void emg_task(void *pvParameters) {
    ESP_LOGI(TAG, "emg_task started (%d Hz)", EMG_SAMPLE_RATE_HZ);
    const TickType_t period = pdMS_TO_TICKS(1000 / EMG_SAMPLE_RATE_HZ);
    TickType_t last_wake    = xTaskGetTickCount();
    emg_queue_msg_t msg     = { .type = QUEUE_MSG_DATA };

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        if (!g_emg_ctx.enabled) continue;
        if (myoware_read_sample(&g_emg_ctx, &msg.data)) {
            if (xQueueSend(g_emg_queue, &msg, 0) != pdTRUE) {
                ESP_LOGW(TAG, "EMG queue full");
            }
        }
    }
}

// ==================== Processing Task ====================

static void processing_task(void *pvParameters) {
    ESP_LOGI(TAG, "processing_task started");

    // BLE notifies at 20 Hz; classification/serial at 10 Hz
    // We run the loop at 20 Hz and gate serial output every other cycle
    const TickType_t period   = pdMS_TO_TICKS(1000 / BLE_NOTIFY_RATE_HZ);
    TickType_t last_wake      = xTaskGetTickCount();

    imu_sample_t    latest_upper = {0};
    imu_sample_t    latest_lower = {0};
    emg_sample_t    latest_emg   = {0};
    imu_queue_msg_t imu_msg;
    emg_queue_msg_t emg_msg;

    posture_features_t       features = {0};
    posture_classification_t result   = {0};

    uint32_t last_update_ms   = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t ble_cycle        = 0;  // Counts every BLE notify cycle
    uint32_t serial_divider   = BLE_NOTIFY_RATE_HZ / CLASSIFICATION_RATE_HZ;
    // serial_divider = 20/10 = 2, so serial prints every other BLE cycle

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        // --- Drain IMU queue — route by sensor_id ---
        while (xQueueReceive(g_imu_queue, &imu_msg, 0) == pdTRUE) {
            if (imu_msg.type == QUEUE_MSG_DATA) {
                if (imu_msg.data.sensor_id == IMU_ID_UPPER)
                    latest_upper = imu_msg.data;
                else if (imu_msg.data.sensor_id == IMU_ID_LOWER)
                    latest_lower = imu_msg.data;
            }
        }

        // --- Drain EMG queue ---
        while (xQueueReceive(g_emg_queue, &emg_msg, 0) == pdTRUE) {
            if (emg_msg.type == QUEUE_MSG_DATA)
                latest_emg = emg_msg.data;
        }

        // ====================================================
        // OUTPUT 1: BLE — raw sensor data every cycle (20 Hz)
        // This is the primary output for the signal processing system.
        // ====================================================
        ble_notify_sensor_packet(&latest_upper, &latest_lower,
                                  &latest_emg, g_calibration.calibrated);

        // ====================================================
        // OUTPUT 2: Classification + Serial — every other cycle (10 Hz)
        // Local debug only. External system does its own classification.
        // ====================================================
        if (ble_cycle % serial_divider == 0) {
            posture_features_compute(&latest_upper, &latest_lower,
                                      &latest_emg, &g_calibration, &features);

            posture_classify(&features, &g_stats, &result);

            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            posture_update_gamification(&result, &g_stats, now_ms - last_update_ms);
            last_update_ms = now_ms;

            telemetry_print(&features, &result, &g_stats);
        }

        // ====================================================
        // Check for BLE remote control commands from the client
        // ====================================================
        if (ble_calibration_requested()) {
            ESP_LOGI(TAG, "BLE-triggered re-calibration starting...");
            posture_calibrate(&g_imu_upper_ctx, &g_imu_lower_ctx,
                               &g_emg_ctx, &g_calibration, 5000);
        }

        if (ble_session_reset_requested()) {
            ESP_LOGI(TAG, "BLE-triggered session reset");
            session_stats_reset(&g_stats);
        }

        ble_cycle++;
    }
}

// ==================== app_main ====================

void app_main(void) {
    ESP_LOGI(TAG, "=== Back on Track (Dual-IMU + BLE) — Starting ===");

    data_types_init();
    calibration_reset(&g_calibration);
    session_stats_reset(&g_stats);
    telemetry_init();
    telemetry_print_banner();

    // --- BLE init FIRST (before tasks) ---
    // NimBLE starts its own host task internally.
    // Must be called before any FreeRTOS tasks that use BLE.
    if (!ble_telemetry_init()) {
        ESP_LOGE(TAG, "BLE init FAILED — continuing without BLE");
    }

    // --- IMU0 (upper back) ---
    ESP_LOGI(TAG, "Initializing IMU0 upper (CS=GPIO%d)...", BNO085_0_PIN_CS);
    if (!bno085_init(&g_imu_upper_ctx)) {
        ESP_LOGE(TAG, "IMU0 FAILED — upper back data will be invalid");
        g_imu_upper_ctx.enabled = false;
    } else { ESP_LOGI(TAG, "IMU0 OK"); }

    // --- IMU1 (lower back) — reuses SPI bus from IMU0 init ---
    ESP_LOGI(TAG, "Initializing IMU1 lower (CS=GPIO%d)...", BNO085_1_PIN_CS);
    if (!bno085_init(&g_imu_lower_ctx)) {
        ESP_LOGE(TAG, "IMU1 FAILED — lower back data will be invalid");
        g_imu_lower_ctx.enabled = false;
    } else { ESP_LOGI(TAG, "IMU1 OK"); }

    // --- MyoWare ---
    ESP_LOGI(TAG, "Initializing MyoWare EMG (GPIO%d)...", MYOWARE_GPIO_PIN);
    if (!myoware_init(&g_emg_ctx)) {
        ESP_LOGE(TAG, "MyoWare FAILED");
        g_emg_ctx.enabled = false;
    } else { ESP_LOGI(TAG, "MyoWare OK"); }

    // --- Calibration ---
    ESP_LOGI(TAG, "Calibration in 3 seconds — sit upright, relax muscles...");
    ESP_LOGI(TAG, "BLE: connect now if you want to trigger calibration remotely later");
    vTaskDelay(pdMS_TO_TICKS(3000));
    posture_calibrate(&g_imu_upper_ctx, &g_imu_lower_ctx, &g_emg_ctx,
                       &g_calibration, 5000);

    if (!g_calibration.calibrated) {
        ESP_LOGW(TAG, "Calibration incomplete — deviations will be from raw zero");
    }

    posture_classifier_init();

    // --- Create queues ---
    g_imu_queue = xQueueCreate(IMU_QUEUE_SIZE * 2, sizeof(imu_queue_msg_t));
    g_emg_queue = xQueueCreate(EMG_QUEUE_SIZE,     sizeof(emg_queue_msg_t));
    if (!g_imu_queue || !g_emg_queue) {
        ESP_LOGE(TAG, "Queue creation FAILED — insufficient heap");
        return;
    }

    // --- Launch tasks ---
    // Core 0: acquisition tasks (SPI + ADC)
    // Core 1: processing + BLE notify + serial output
    xTaskCreatePinnedToCore(imu_upper_task, "imu0_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(imu_lower_task, "imu1_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(emg_task,       "emg_task",  4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(processing_task,"proc_task", 8192, NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "All tasks launched.");
    ESP_LOGI(TAG, "BLE: advertising as \"%s\"", BOT_BLE_DEVICE_NAME);
    ESP_LOGI(TAG, "Serial: %d baud, CSV/JSON at %d Hz",
             115200, CLASSIFICATION_RATE_HZ);
}
