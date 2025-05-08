/* =====================================================================
   BLE HID Host example using NimBLE on ESP32 (ESP-IDF v5.4)
   - BLE (no Classic) to connect to a HID over GATT device (e.g., EasySMX T37)
   - Uses NimBLE stack
   ===================================================================== */

   #include <stdio.h>
   #include <string.h>
   #include "nvs_flash.h"
   #include "esp_nimble_hci.h"
   #include "nimble/nimble_port.h"
   #include "nimble/nimble_port_freertos.h"
   #include "host/ble_hs.h"
   #include "host/ble_uuid.h"
   #include "host/ble_hs_id.h"
   #include "host/ble_hs_cfg.h"
   #include "host/util/util.h"
   #include "host/ble_gatt.h"
   #include "esp_log.h"
   
   static const char *TAG = "BLE_HID_HOST";
   static uint16_t conn_handle;
   
   // HID over GATT service and input report characteristic UUIDs
   static const ble_uuid16_t hid_svc_uuid = BLE_UUID16_INIT(0x1812);
   static const ble_uuid16_t input_rep_uuid = BLE_UUID16_INIT(0x2A4D);
   
   // Scan parameters
   static const struct ble_gap_disc_params scan_params = {
       .itvl = BLE_GAP_SCAN_FAST_INTERVAL_MIN,
       .window = BLE_GAP_SCAN_FAST_WINDOW,
       .filter_policy = BLE_HCI_SCAN_FILT_NO_WL,
       .passive = 0,
   };
   
   // Forward declarations
   static int ble_gap_event(struct ble_gap_event *event, void *arg);
   static int gatt_svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                               const struct ble_gatt_svc *service, void *arg);
   static int gatt_chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                               const struct ble_gatt_chr *chr, void *arg);
   static int gatt_dsc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                               const struct ble_gatt_dsc *dsc, void *arg);
   
   void ble_app_on_sync(void) {
       // Begin scanning
       ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HCI_SCAN_TYPE_ACTI,
                    &scan_params, ble_gap_event, NULL);
       ESP_LOGI(TAG, "BLE scan started");
   }
   
   void ble_app_on_reset(int reason) {
       ESP_LOGE(TAG, "BLE reset; reason=%d", reason);
   }
   
   void host_task(void *param) {
       nimble_port_run();
   }
   
   void app_main(void) {
       // Initialize NVS
       esp_err_t ret = nvs_flash_init();
       ESP_ERROR_CHECK(ret);
   
       // Initialize NimBLE
       esp_nimble_hci_and_controller_init();
       nimble_port_init();
   
       // Configure host
       ble_hs_cfg.reset_cb = ble_app_on_reset;
       ble_hs_cfg.sync_cb = ble_app_on_sync;
       ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
   
       // Start host task
       nimble_port_freertos_init(host_task);
   }
   
   static int ble_gap_event(struct ble_gap_event *event, void *arg) {
       switch (event->type) {
       case BLE_GAP_EVENT_DISC: {
           struct ble_hs_adv_fields fields;
           ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
           // Look for HID service UUID
           for (int i = 0; i < fields.num_uuids16; i++) {
               if (fields.uuids16[i].u16 == hid_svc_uuid.u.value) {
                   ESP_LOGI(TAG, "Found HID device, connecting...");
                   ble_gap_disc_cancel();
                   ble_gap_connect(BLE_OWN_ADDR_PUBLIC,
                                   &event->disc.addr,
                                   BLE_HCI_CONN_LE,
                                   ble_gap_event,
                                   NULL);
                   return 0;
               }
           }
           return 0;
       }
       case BLE_GAP_EVENT_CONNECT:
           if (event->connect.status == 0) {
               conn_handle = event->connect.conn_handle;
               ESP_LOGI(TAG, "Connected, handle=%d", conn_handle);
               // Discover HID service
               ble_gattc_disc_svc_by_uuid(conn_handle,
                                          (ble_uuid_t *)&hid_svc_uuid,
                                          gatt_svc_disc_cb,
                                          NULL);
           } else {
               ESP_LOGE(TAG, "Connect failed; status=%d", event->connect.status);
               ble_gap_disc(BLE_OWN_ADDR_PUBLIC,
                            BLE_HCI_SCAN_TYPE_ACTI,
                            &scan_params,
                            ble_gap_event,
                            NULL);
           }
           return 0;
       case BLE_GAP_EVENT_DISCONNECT:
           ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
           ble_gap_disc(BLE_OWN_ADDR_PUBLIC,
                        BLE_HCI_SCAN_TYPE_ACTI,
                        &scan_params,
                        ble_gap_event,
                        NULL);
           return 0;
       case BLE_GAP_EVENT_NOTIFY_RX:
           // Received HID report
           ESP_LOGI(TAG, "HID report len=%d", event->notify_rx.om->om_len);
           // Process event->notify_rx.om->om_data here
           return 0;
       default:
           return 0;
       }
   }
   
   static int gatt_svc_disc_cb(uint16_t conn_hdl,
                               const struct ble_gatt_error *error,
                               const struct ble_gatt_svc *service,
                               void *arg) {
       if (error->status != 0) {
           ESP_LOGE(TAG, "Service discovery failed; status=%d", error->status);
           return 0;
       }
       ESP_LOGI(TAG, "HID service: start=%d end=%d",
                service->start_handle,
                service->end_handle);
       // Discover characteristics
       ble_gattc_disc_all_chrs(conn_hdl,
                                service->start_handle,
                                service->end_handle,
                                gatt_chr_disc_cb,
                                NULL);
       return 0;
   }
   
   static int gatt_chr_disc_cb(uint16_t conn_hdl,
                               const struct ble_gatt_error *error,
                               const struct ble_gatt_chr *chr,
                               void *arg) {
       if (error->status != 0) {
           ESP_LOGE(TAG, "Chr discovery failed; status=%d", error->status);
           return 0;
       }
       if (ble_uuid_cmp(&chr->uuid.u,
                        (ble_uuid_t *)&input_rep_uuid) == 0) {
           ESP_LOGI(TAG, "Found Input Report chr; handle=%d", chr->val_handle);
           // Subscribe to notifications (CCCD)
           uint16_t desc_uuid = BLE_UUID16_DECLARE(0x2902);
           ble_gattc_disc_all_dscs(conn_hdl,
                                    chr->val_handle,
                                    chr->val_handle + 1,
                                    gatt_dsc_disc_cb,
                                    NULL);
       }
       return 0;
   }
   
   static int gatt_dsc_disc_cb(uint16_t conn_hdl,
                               const struct ble_gatt_error *error,
                               const struct ble_gatt_dsc *dsc,
                               void *arg) {
       if (error->status != 0) {
           ESP_LOGE(TAG, "Desc discovery failed; status=%d", error->status);
           return 0;
       }
       if (ble_uuid_cmp(&dsc->uuid.u,
                        BLE_UUID16_DECLARE(0x2902)) == 0) {
           ESP_LOGI(TAG, "Found CCCD; handle=%d", dsc->handle);
           // Enable notifications
           uint16_t npref = htole16(0x0001);
           ble_gattc_write_flat(conn_hdl,
                                 dsc->handle,
                                 &npref,
                                 sizeof(npref),
                                 NULL,
                                 NULL);
       }
       return 0;
   }
   