/**
 * @file ble_telemetry.c
 * @brief NimBLE GATT peripheral implementation for Back on Track
 *
 * Uses ESP-IDF NimBLE (CONFIG_BT_NIMBLE_ENABLED=y in sdkconfig).
 * Single peripheral role — connects to one client at a time.
 * Streams bot_ble_packet_t via GATT notify on the sensor characteristic.
 *
 * NimBLE component dependencies (set in CMakeLists.txt REQUIRES):
 *   bt  (umbrella component that pulls in NimBLE)
 */

#include "ble_telemetry.h"
#include "data_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// NimBLE headers
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include <string.h>
#include <inttypes.h>

static const char *TAG = "ble_telem";

// ==================== Module State ====================

static uint16_t s_sensor_char_handle  = 0;  // GATT characteristic value handle
static uint16_t s_conn_handle         = BLE_HS_CONN_HANDLE_NONE;
static bool     s_notify_enabled      = false;
static bool     s_calibration_req     = false;
static bool     s_session_reset_req   = false;
static uint8_t  s_seq                 = 0;

// Mutex protecting the packet buffer written by processing_task
// and read by the BLE notify path (which runs in the NimBLE host task)
static SemaphoreHandle_t s_pkt_mutex  = NULL;
static bot_ble_packet_t  s_latest_pkt = {0};
static bool              s_pkt_ready  = false;

// ==================== 128-bit UUID helpers ====================
// NimBLE uses ble_uuid128_t for custom UUIDs.
// We expand our 16-bit shorthand into the base:
//   12345678-1234-1234-1234-123456780000 | 16-bit value in bytes [12:13]

static void make_uuid128(ble_uuid128_t *out, uint16_t short_uuid) {
    // Base UUID bytes (little-endian in NimBLE's layout)
    static const uint8_t base[16] = {
        0x00, 0x00,                         // [0:1]  replaced by short_uuid
        0x56, 0x78,                         // [2:3]
        0x34, 0x12,                         // [4:5]
        0x34, 0x12,                         // [6:7]
        0x34, 0x12,                         // [8:9]
        0x78, 0x56, 0x34, 0x12, 0x34, 0x12 // [10:15]
    };
    out->u.type = BLE_UUID_TYPE_128;
    memcpy(out->value, base, 16);
    out->value[0] = (uint8_t)(short_uuid & 0xFF);
    out->value[1] = (uint8_t)(short_uuid >> 8);
}

// Static UUID storage (NimBLE keeps pointers to these)
static ble_uuid128_t s_svc_uuid;
static ble_uuid128_t s_sensor_char_uuid;
static ble_uuid128_t s_control_char_uuid;

// ==================== GATT callbacks ====================

/**
 * @brief Called by NimBLE when the sensor characteristic is read or
 * when a CCCD write enables/disables notifications.
 */
static int sensor_char_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg) {
    (void)conn_handle; (void)attr_handle; (void)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // Client reading the characteristic — return latest packet
        if (xSemaphoreTake(s_pkt_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            int rc = os_mbuf_append(ctxt->om, &s_latest_pkt, sizeof(bot_ble_packet_t));
            xSemaphoreGive(s_pkt_mutex);
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

/**
 * @brief Called when the control characteristic is written by the client.
 * Parses the single-byte command and sets the appropriate request flag.
 */
static int control_char_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg) {
    (void)conn_handle; (void)attr_handle; (void)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t cmd = 0;
        uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
        if (om_len < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

        ble_hs_mbuf_to_flat(ctxt->om, &cmd, sizeof(cmd), NULL);

        switch ((bot_ble_cmd_t)cmd) {
            case BOT_CMD_START_CALIBRATE:
                s_calibration_req = true;
                ESP_LOGI(TAG, "BLE: calibration requested by client");
                break;
            case BOT_CMD_RESET_SESSION:
                s_session_reset_req = true;
                ESP_LOGI(TAG, "BLE: session reset requested by client");
                break;
            case BOT_CMD_PING:
                ESP_LOGD(TAG, "BLE: ping received");
                break;
            default:
                ESP_LOGW(TAG, "BLE: unknown command 0x%02X", cmd);
                break;
        }
        return 0;
    }
    return 0;
}

// ==================== GATT Service Table ====================
// Defined statically — NimBLE registers these at startup.

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // Sensor packet characteristic — notify + read
                .uuid       = &s_sensor_char_uuid.u,
                .access_cb  = sensor_char_access_cb,
                .val_handle = &s_sensor_char_handle,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                // Control characteristic — write only
                .uuid      = &s_control_char_uuid.u,
                .access_cb = control_char_access_cb,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 }  // Terminator
        },
    },
    { 0 }  // Terminator
};

// ==================== GAP Event Handler ====================

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    (void)arg;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                s_conn_handle = event->connect.conn_handle;
                s_notify_enabled = false;  // Client must enable CCCD separately
                ESP_LOGI(TAG, "BLE client connected (handle=%d)", s_conn_handle);
            } else {
                // Connection failed — restart advertising
                ESP_LOGW(TAG, "BLE connection failed (status=%d), restarting adv",
                         event->connect.status);
                ble_telemetry_start();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE client disconnected (reason=%d)", event->disconnect.reason);
            s_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
            s_notify_enabled = false;
            // Restart advertising so next client can connect
            ble_telemetry_start();
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            // Client wrote to CCCD to enable/disable notifications
            if (event->subscribe.attr_handle == s_sensor_char_handle) {
                s_notify_enabled = (event->subscribe.cur_notify != 0);
                ESP_LOGI(TAG, "BLE notifications %s",
                         s_notify_enabled ? "ENABLED" : "DISABLED");
            }
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "BLE MTU exchanged: conn=%d mtu=%d",
                     event->mtu.conn_handle, event->mtu.value);
            break;

        default:
            break;
    }
    return 0;
}

// ==================== NimBLE Host Sync Callback ====================

static void ble_on_sync(void) {
    // Called by NimBLE host task when the BT controller is ready
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE host synced — starting advertising");
    ble_telemetry_start();
}

static void ble_on_reset(int reason) {
    ESP_LOGW(TAG, "BLE host reset (reason=%d)", reason);
}

// ==================== NimBLE Host Task ====================
// NimBLE requires its own FreeRTOS task for the host stack.

static void nimble_host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();  // Blocks until nimble_port_stop() is called
    nimble_port_freertos_deinit();
}

// ==================== Public API ====================

bool ble_telemetry_init(void) {
    ESP_LOGI(TAG, "Initializing BLE telemetry (NimBLE)");

    // Build UUIDs from short IDs
    make_uuid128(&s_svc_uuid,          BOT_SERVICE_UUID);
    make_uuid128(&s_sensor_char_uuid,  BOT_SENSOR_CHAR_UUID);
    make_uuid128(&s_control_char_uuid, BOT_CONTROL_CHAR_UUID);

    // Create packet mutex
    s_pkt_mutex = xSemaphoreCreateMutex();
    if (!s_pkt_mutex) {
        ESP_LOGE(TAG, "Failed to create BLE packet mutex");
        return false;
    }

    // Initialize NimBLE port (sets up the BT controller)
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return false;
    }

    // Register host callbacks
    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    // Set device name (shown in BLE scan results)
    int rc = ble_svc_gap_device_name_set(BOT_BLE_DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_svc_gap_device_name_set failed: %d", rc);
        return false;
    }

    // Initialize built-in GAP and GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Register our custom GATT service table
    rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return false;
    }
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return false;
    }

    // Start the NimBLE host task
    // Must run on Core 0 — BT controller is on Core 0 on ESP32-S3
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "BLE init complete. Device name: \"%s\"", BOT_BLE_DEVICE_NAME);
    ESP_LOGI(TAG, "Sensor packet size: %d bytes", (int)sizeof(bot_ble_packet_t));
    return true;
}

void ble_telemetry_start(void) {
    // Build advertising data: flags + name
    struct ble_hs_adv_fields fields = {0};
    fields.flags                 = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name                  = (uint8_t *)BOT_BLE_DEVICE_NAME;
    fields.name_len              = strlen(BOT_BLE_DEVICE_NAME);
    fields.name_is_complete      = 1;
    fields.tx_pwr_lvl            = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.tx_pwr_lvl_is_present = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    // Advertising parameters — undirected connectable
    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                            &adv_params, gap_event_handler, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        // BLE_HS_EALREADY = already advertising, not an error
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE advertising started as \"%s\"", BOT_BLE_DEVICE_NAME);
}

void ble_notify_sensor_packet(const imu_sample_t *imu_upper,
                               const imu_sample_t *imu_lower,
                               const emg_sample_t *emg,
                               bool                calibrated) {
    if (!imu_upper || !imu_lower || !emg) return;

    // Build the packet
    bot_ble_packet_t pkt = {0};
    pkt.timestamp_ms  = get_timestamp_ms();
    pkt.seq           = s_seq++;

    // Upper IMU
    pkt.upper_pitch   = imu_upper->pitch_deg;
    pkt.upper_roll    = imu_upper->roll_deg;
    pkt.upper_yaw     = imu_upper->yaw_deg;
    pkt.upper_q_i     = imu_upper->q_i;
    pkt.upper_q_j     = imu_upper->q_j;
    pkt.upper_q_k     = imu_upper->q_k;
    pkt.upper_q_r     = imu_upper->q_r;

    // Lower IMU
    pkt.lower_pitch   = imu_lower->pitch_deg;
    pkt.lower_roll    = imu_lower->roll_deg;
    pkt.lower_yaw     = imu_lower->yaw_deg;
    pkt.lower_q_i     = imu_lower->q_i;
    pkt.lower_q_j     = imu_lower->q_j;
    pkt.lower_q_k     = imu_lower->q_k;
    pkt.lower_q_r     = imu_lower->q_r;

    // EMG
    pkt.emg_filtered  = emg->filtered;
    pkt.emg_voltage_mv = emg->voltage_mv;
    pkt.emg_raw_adc   = (uint16_t)(emg->raw_adc & 0xFFFF);

    // Status flags
    pkt.status_flags = 0;
    if (imu_upper->valid) pkt.status_flags |= BOT_FLAG_UPPER_VALID;
    if (imu_lower->valid) pkt.status_flags |= BOT_FLAG_LOWER_VALID;
    if (emg->valid)       pkt.status_flags |= BOT_FLAG_EMG_VALID;
    if (calibrated)       pkt.status_flags |= BOT_FLAG_CALIBRATED;

    // Update the shared packet buffer
    if (xSemaphoreTake(s_pkt_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        s_latest_pkt = pkt;
        s_pkt_ready  = true;
        xSemaphoreGive(s_pkt_mutex);
    }

    // Send BLE notification if a client is connected and notifications enabled
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_notify_enabled) return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(&pkt, sizeof(bot_ble_packet_t));
    if (!om) {
        ESP_LOGW(TAG, "ble_notify: mbuf alloc failed (stack full?)");
        return;
    }

    int rc = ble_gatts_notify_custom(s_conn_handle, s_sensor_char_handle, om);
    if (rc != 0) {
        // Non-fatal — client may have disconnected mid-notify
        ESP_LOGD(TAG, "ble_gatts_notify_custom rc=%d", rc);
    }
}

bool ble_is_connected(void) {
    return (s_conn_handle != BLE_HS_CONN_HANDLE_NONE);
}

bool ble_calibration_requested(void) {
    bool val = s_calibration_req;
    s_calibration_req = false;  // auto-clear
    return val;
}

bool ble_session_reset_requested(void) {
    bool val = s_session_reset_req;
    s_session_reset_req = false;
    return val;
}
