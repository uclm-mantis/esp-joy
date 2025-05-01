/* =====================================================================
   sdkconfig.defaults (at project root)

   # Enable Classic Bluetooth and HID Host Role for ESP32-WROVER-IB on ESP-IDF v5.4
   CONFIG_BT_ENABLED=y
   CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY=1        # Classic BT only
   CONFIG_BT_HID_HOST_ENABLED=y               # Enable Bluedroid HID Host
   ===================================================================== */

/* main.c */
#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_hidh_api.h"
#include "esp_log.h"

#define TAG "BT_HID_HOST"
#define TARGET_NAME "Pro Controller"
#define INQ_DURATION 10  // seconds

// GAP callback for device discovery
static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
// HIDH callback for HID Host events
static void hidh_cb(esp_hidh_cb_event_t event, esp_hidh_cb_param_t *param);

void app_main(void) {
    // 1) Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2) Initialize BT controller in Classic mode
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));

    // 3) Initialize Bluedroid stack
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // 4) Register GAP callback
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));

    // 5) Register HID Host callback and init Host
    ESP_ERROR_CHECK(esp_bt_hid_host_register_callback(hidh_cb));
    ESP_ERROR_CHECK(esp_bt_hid_host_init());

    // 6) Start device inquiry
    ESP_LOGI(TAG, "Starting inquiry for %d seconds...", INQ_DURATION);
    ESP_ERROR_CHECK(esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, INQ_DURATION, 0));
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    if (event == ESP_BT_GAP_DISC_RES_EVT) {
        for (int i = 0; i < param->disc_res.num_prop; i++) {
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR) {
                uint8_t *eir = (uint8_t *)param->disc_res.prop[i].val;
                uint8_t len = eir[0];
                uint8_t dtype = eir[1];
                if (dtype == ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME && len > 1) {
                    char name[32] = {0};
                    memcpy(name, &eir[2], len - 1);
                    name[len - 1] = '\0';
                    if (strcmp(name, TARGET_NAME) == 0) {
                        ESP_LOGI(TAG, "Found %s, connecting...", name);
                        ESP_ERROR_CHECK(esp_bt_gap_cancel_discovery());
                        ESP_ERROR_CHECK(esp_bt_hid_host_connect(param->disc_res.bda));
                        return;
                    }
                }
            }
        }
    } else if (event == ESP_BT_GAP_DISC_STATE_CHANGED_EVT) {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            ESP_LOGI(TAG, "Inquiry complete");
        }
    }
}

typedef struct __attribute__((packed)) {
    uint8_t id;
    struct {
        uint16_t b:1;
        uint16_t a:1;
        uint16_t y:1;
        uint16_t x:1;
        uint16_t l:1;
        uint16_t r:1;
        uint16_t zl:1;
        uint16_t zr:1;
        uint16_t minus:1;
        uint16_t plus:1;
        uint16_t lstick:1;
        uint16_t rstick:1;
        uint16_t home:1;
        uint16_t turbo:1;
        uint16_t reserved2:2;
    };
    int8_t dpad;
    uint16_t lx;
    uint16_t ly;
    uint16_t rx;
    uint16_t ry;
} joydata_t;

const char* buttons(joydata_t* data) {
    static char buf[15];
    buf[0] = data->a ? 'A' : '.';
    buf[1] = data->b ? 'B' : '.';
    buf[2] = data->x ? 'X' : '.';
    buf[3] = data->y ? 'Y' : '.';
    buf[4] = data->l ? 'l' : '.';
    buf[5] = data->r ? 'r' : '.';
    buf[6] = data->zl ? 'L' : '.';
    buf[7] = data->zr ? 'R' : '.';
    buf[8] = data->minus ? '-' : '.';
    buf[9] = data->plus ? '+' : '.';
    buf[10] = data->lstick ? 's' : '.';
    buf[11] = data->rstick ? 'S' : '.';
    buf[12] = data->home ? 'H' : '.';
    buf[13] = data->turbo ? 'T' : '.';
    buf[14] = '\0';
    return buf;
}

static void hidh_cb(esp_hidh_cb_event_t event, esp_hidh_cb_param_t *param) {
    switch (event) {
        case ESP_HIDH_INIT_EVT:
            ESP_LOGI(TAG, "HID Host initialized, status=%d", param->init.status);
            break;

        case ESP_HIDH_OPEN_EVT:
            ESP_LOGI(TAG, "HID Device connected: %02x:%02x:%02x:%02x:%02x:%02x",
                     param->open.bd_addr[0], param->open.bd_addr[1], param->open.bd_addr[2],
                     param->open.bd_addr[3], param->open.bd_addr[4], param->open.bd_addr[5]);
            break;

        case ESP_HIDH_DATA_IND_EVT: {
            int length = param->data_ind.len;
            if (length >= sizeof(joydata_t)) {
                joydata_t* data = (joydata_t*)param->data_ind.data;
                uint32_t* p = (uint32_t*)data;
                uint16_t b = (*p >> 8) & 0xFFFF;
                ESP_LOGI(TAG, "b=%04x, buttons=%s, dpad=%d, lx=%d, ly=%d, rx=%d, ry=%d",
                         b, buttons(data), (int)data->dpad, data->lx, data->ly, data->rx, data->ry);
            }
            break;
        }

        case ESP_HIDH_CLOSE_EVT:
            ESP_LOGI(TAG, "HID Device disconnected, restarting inquiry...");
            ESP_ERROR_CHECK(esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, INQ_DURATION, 0));
            break;

        default:
            break;
    }
}
