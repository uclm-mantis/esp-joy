#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include "host/ble_gatt.h"

#define TAG                    "BLE_HID_HOST"
#define TARGET_NAME            "Fire TV Remote"
#define HIDS_SERVICE_UUID16    0x1812
#define INPUT_REPORT_UUID16    0x2A4D
#define SCAN_DURATION_SECONDS  10
#define SCAN_DURATION_MS       (SCAN_DURATION_SECONDS * 1000)

static uint8_t own_addr_type;
static struct {
    uint16_t start;
    uint16_t end;
    uint16_t input_val_handle;
    uint16_t cccd_handle;
} hid_handles;

// Prototipos de callbacks
static void     ble_on_reset(int reason);
static void     ble_on_sync(void);
static void     ble_host_task(void *param);
static int      ble_gap_event(struct ble_gap_event *event, void *arg);
static int      gatt_svc_discover(uint16_t conn_handle,
                                  const struct ble_gatt_error *error,
                                  const struct ble_gatt_svc *service,
                                  void *arg);
static int      gatt_chr_discover(uint16_t conn_handle,
                                  const struct ble_gatt_error *error,
                                  const struct ble_gatt_chr *chr,
                                  void *arg);
static int      gatt_dsc_discover(uint16_t conn_handle,
                                  const struct ble_gatt_error *error,
                                  uint16_t chr_val_handle,
                                  const struct ble_gatt_dsc *dsc,
                                  void *arg);
static int      gatt_notify_cb(uint16_t conn_handle,
                               const struct ble_gatt_error *error,
                               struct ble_gatt_attr *attr,
                               void *arg);

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 1) Inicializa controlador + HCI NimBLE
    nimble_port_init();

    // 2) Configura callbacks del host NimBLE
    ble_hs_cfg.reset_cb        = ble_on_reset;
    ble_hs_cfg.sync_cb         = ble_on_sync;

    // 3) Arranca la tarea de NimBLE
    nimble_port_freertos_init(ble_host_task);
}

// Called when the controller or host resets
static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

// Called when the BLE host and controller are synced and ready
static void ble_on_sync(void)
{
    int rc;

    // Determina type de dirección (public o random)
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed; rc=%d", rc);
        return;
    }

    // Inicializa el cliente GATT
    rc = ble_gattc_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gattc_init failed; rc=%d", rc);
        return;
    }

    // Parámetros de scanning
    struct ble_gap_disc_params disc_params;
    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.passive       = 0;
    disc_params.itvl          = 0x50;
    disc_params.window        = 0x30;
    disc_params.filter_policy = BLE_HCI_SCAN_FILT_NO_WL;
    disc_params.limited       = 0;

    ESP_LOGI(TAG, "Starting BLE scan for %d seconds…", SCAN_DURATION_SECONDS);
    rc = ble_gap_disc(own_addr_type,
                      SCAN_DURATION_MS,
                      &disc_params,
                      ble_gap_event,
                      NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed; rc=%d", rc);
    }
}

// Tarea que ejecuta el loop de NimBLE
static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// GAP event handler: DISC, CONNECT, DISCONNECT, etc.
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;
        int rc = ble_hs_adv_parse_fields(&fields,
                                         event->disc.data,
                                         event->disc.length_data);
        if (rc == 0 && fields.name_len) {
            char name[32];
            memcpy(name, fields.name, fields.name_len);
            name[fields.name_len] = '\0';
            ESP_LOGI(TAG, "DISC %s", name);
            if (strcmp(name, TARGET_NAME) == 0) {
                ESP_LOGI(TAG, "Found %s, connecting…", name);
                ble_gap_disc_cancel();
                rc = ble_gap_connect(own_addr_type,
                                     &event->disc.addr,
                                     30000,         // timeout ms
                                     NULL,          // use default connection params
                                     ble_gap_event,
                                     NULL);
                if (rc != 0) {
                    ESP_LOGE(TAG, "ble_gap_connect failed; rc=%d", rc);
                }
                return 0;
            }
        }
        break;
    }

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "Connection established; discovering HID service…");
            ble_uuid16_t svc_uuid = BLE_UUID16_INIT(HIDS_SERVICE_UUID16);
            int rc = ble_gattc_disc_svc_by_uuid(event->connect.conn_handle,
                                                &svc_uuid.u,
                                                gatt_svc_discover,
                                                NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "ble_gattc_disc_svc_by_uuid failed; rc=%d", rc);
            }
        } else {
            ESP_LOGW(TAG, "Connection failed; status=%d", event->connect.status);
            ESP_LOGI(TAG, "Restarting scan");
            ble_on_sync();
        }
        break;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "Scan complete; restarting…");
        ble_on_sync();
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Disconnected; reason=%d", event->disconnect.reason);
        ESP_LOGI(TAG, "Restarting scan");
        ble_on_sync();
        break;

    default:
        break;
    }
    return 0;
}

// Descubre el servicio HID (UUID 0x1812)
static int gatt_svc_discover(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             const struct ble_gatt_svc *service,
                             void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG, "Service discovery error; status=%d", error->status);
        return 0;
    }

    ble_uuid16_t uuid = BLE_UUID16_INIT(HIDS_SERVICE_UUID16);
    if (ble_uuid_cmp(&service->uuid.u, &uuid.u) == 0) {
        hid_handles.start = service->start_handle;
        hid_handles.end   = service->end_handle;
        ESP_LOGI(TAG, "Found HID service %d…%d",
                 hid_handles.start, hid_handles.end);

        // Ahora descubre la característica Input Report (0x2A4D)
        ble_uuid16_t chr_uuid = BLE_UUID16_INIT(INPUT_REPORT_UUID16);
        int rc = ble_gattc_disc_chrs_by_uuid(conn_handle,
                                             hid_handles.start,
                                             hid_handles.end,
                                             &chr_uuid.u,
                                             gatt_chr_discover,
                                             NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Characteristic discovery failed; rc=%d", rc);
        }
    }
    return 0;
}

// Descubre la característica Input Report
static int gatt_chr_discover(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             const struct ble_gatt_chr *chr,
                             void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG, "Char discover error; status=%d", error->status);
        return 0;
    }

    ble_uuid16_t uuid = BLE_UUID16_INIT(INPUT_REPORT_UUID16);
    if (ble_uuid_cmp(&chr->uuid.u, &uuid.u) == 0) {
        hid_handles.input_val_handle = chr->val_handle;
        ESP_LOGI(TAG, "Found Input Report char; value_handle=%d",
                 chr->val_handle);

        // Descubre todos los descriptores para luego encontrar el CCCD
        int rc = ble_gattc_disc_all_dscs(conn_handle,
                                         hid_handles.input_val_handle,
                                         hid_handles.end,
                                         gatt_dsc_discover,
                                         NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Descriptor discovery failed; rc=%d", rc);
        }
    }
    return 0;
}

// Descubre el descriptor CCCD (0x2902) y habilita notificaciones
static int gatt_dsc_discover(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             uint16_t chr_val_handle,
                             const struct ble_gatt_dsc *dsc,
                             void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG, "Descriptor discover error; status=%d", error->status);
        return 0;
    }

    ble_uuid16_t cccd_uuid = BLE_UUID16_INIT(0x2902);
    if (ble_uuid_cmp(&dsc->uuid.u, &cccd_uuid.u) == 0) {
        hid_handles.cccd_handle = dsc->handle;
        ESP_LOGI(TAG, "Found CCCD handle=%d", dsc->handle);

        // Habilita notificaciones
        uint8_t notify_en[2] = {0x01, 0x00};
        int rc = ble_gattc_write_flat(conn_handle,
                                      dsc->handle,
                                      notify_en,
                                      sizeof(notify_en),
                                      gatt_notify_cb,
                                      NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to write CCCD; rc=%d", rc);
        }
    }
    return 0;
}

// Callback para recibir notificaciones del Input Report
static int gatt_notify_cb(uint16_t conn_handle,
                          const struct ble_gatt_error *error,
                          struct ble_gatt_attr *attr,
                          void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG, "Notify error; status=%d", error->status);
        return 0;
    }
    ESP_LOGI(TAG, "Notification on handle=%d, len=%d",
             attr->handle, attr->om->om_len);
    ESP_LOG_BUFFER_HEX(TAG, attr->om->om_data, attr->om->om_len);
    return 0;
}
