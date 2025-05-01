#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_hidh_api.h"
#include "esp_log.h"

#define TAG           "FIRETV_HID"
#define REMOTE_NAME   "Fire TV Remote"
#define INQ_DURATION  10

static void gap_cb(esp_bt_gap_cb_event_t e, esp_bt_gap_cb_param_t *p);
static void hidh_cb(esp_hidh_cb_event_t e, esp_hidh_cb_param_t *param);

void app_main(void) {
    nvs_flash_init();
    esp_bt_controller_init(&BT_CONTROLLER_INIT_CONFIG_DEFAULT());
    esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    // Registro callbacks
    esp_bt_gap_register_callback(gap_cb);
    esp_hidh_profile_app_register(hidh_cb);

    // Iniciar inquiry
    ESP_LOGI(TAG, "Iniciando escaneo BLE…");
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, INQ_DURATION, 0);
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    if (event == ESP_BT_GAP_DISC_RES_EVT) {
        uint8_t *eir = NULL; uint8_t len = 0;
        eir = esp_bt_gap_resolve_eir_data(param->disc_res.eir,
                                          ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);
        if (eir && len && strncmp((char*)eir, REMOTE_NAME, len)==0) {
            ESP_LOGI(TAG, "Encontrado %s, conectando…", REMOTE_NAME);
            esp_bt_gap_cancel_discovery();
            esp_hidh_dev_open(param->disc_res.bda, ESP_HID_TRANSPORT_LE);
        }
    } else if (event == ESP_BT_GAP_DISC_STATE_CHANGED_EVT
            && param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
        ESP_LOGI(TAG, "Inquiry terminada, reiniciando…");
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY,
                                  INQ_DURATION, 0);
    }
}

static void hidh_cb(esp_hidh_cb_event_t event, esp_hidh_cb_param_t *param) {
    switch (event) {
    case ESP_HIDH_OPEN_EVENT:
        ESP_LOGI(TAG, "Remote conectado");
        break;
    case ESP_HIDH_INPUT_EVENT: {
        const uint8_t *r = param->input.data;
        size_t len = param->input.length;
        ESP_LOG_BUFFER_HEXDUMP(TAG, r, len, ESP_LOG_INFO);
        // Tras ver el dump, extrae tus botones
        break;
    }
    case ESP_HIDH_CLOSE_EVENT:
        ESP_LOGI(TAG, "Remote desconectado, reiniciando inquiry…");
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY,
                                  INQ_DURATION, 0);
        break;
    default:
        break;
    }
}
