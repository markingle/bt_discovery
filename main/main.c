/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */



/****************************************************************************
*
* This file is for Classic Bluetooth device and service discovery Demo.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "driver/uart.h"
#include "freertos/queue.h"

#define GAP_TAG          "GAP"

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

static const char *TAG = "uart_events";


typedef enum {
    APP_GAP_STATE_IDLE = 0,
    APP_GAP_STATE_DEVICE_DISCOVERING,
    APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE,
    APP_GAP_STATE_SERVICE_DISCOVERING,
    APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE,
} app_gap_state_t;

typedef struct {
    bool dev_found;
    uint8_t bdname_len;
    uint8_t eir_len;
    uint8_t rssi;
    uint32_t cod;
    uint8_t eir[ESP_BT_GAP_EIR_DATA_LEN];
    uint8_t bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
    esp_bd_addr_t bda;
    app_gap_state_t state;
} app_gap_cb_t;

static app_gap_cb_t m_dev_info;

static bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

static void bt_app_gap_init(void)
{
    app_gap_cb_t *p_dev = &m_dev_info;
    memset(p_dev, 0, sizeof(app_gap_cb_t));

    p_dev->state = APP_GAP_STATE_IDLE;
}

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static char *parsevalue(char *data_string)

{
    char str[30];
    char * pch;
    sprintf(str, "%s",data_string);
    ESP_LOGI(TAG, "New String: %s", str);
    pch = strtok(str,"=");
    while (pch != NULL)
        {
            ESP_LOGI(TAG, "PCH!!!: %s", pch);
            pch = strtok (NULL, "=");
            data_string = pch;
        }
    return data_string;
}

static void update_device_info(esp_bt_gap_cb_param_t *param)
{
    char bda_str[18];
    uint32_t cod = 0;
    int32_t rssi = -129; /* invalid value */
    uint8_t *bdname = NULL;
    uint8_t bdname_len = 0;
    uint8_t *eir = NULL;
    uint8_t eir_len = 0;
    esp_bt_gap_dev_prop_t *p;

    ESP_LOGI(GAP_TAG, "Device found: %s", bda2str(param->disc_res.bda, bda_str, 18));
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            ESP_LOGI(GAP_TAG, "--Class of Device: 0x%"PRIx32, cod);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)(p->val);
            ESP_LOGI(GAP_TAG, "--RSSI: %"PRId32, rssi);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
            bdname_len = (p->len > ESP_BT_GAP_MAX_BDNAME_LEN) ? ESP_BT_GAP_MAX_BDNAME_LEN :
                          (uint8_t)p->len;
            bdname = (uint8_t *)(p->val);
            ESP_LOGI(GAP_TAG, "Device name: %s", bdname);
            break;
        case ESP_BT_GAP_DEV_PROP_EIR: {
            eir_len = p->len;
            eir = (uint8_t *)(p->val);
            break;
        }
        default:
            break;
        }
    }

    app_gap_cb_t *p_dev = &m_dev_info;
    if (p_dev->dev_found) {
        return;
    }

    if (!esp_bt_gap_is_valid_cod(cod) ||
   (!(esp_bt_gap_get_cod_major_dev(cod) == ESP_BT_COD_MAJOR_DEV_PHONE) &&
             !(esp_bt_gap_get_cod_major_dev(cod) == ESP_BT_COD_MAJOR_DEV_AV))) {
        return;
    }

    memcpy(p_dev->bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
    p_dev->dev_found = true;

    p_dev->cod = cod;
    p_dev->rssi = rssi;
    if (bdname_len > 0) {
        memcpy(p_dev->bdname, bdname, bdname_len);
        p_dev->bdname[bdname_len] = '\0';
        p_dev->bdname_len = bdname_len;
    }
    if (eir_len > 0) {
        memcpy(p_dev->eir, eir, eir_len);
        p_dev->eir_len = eir_len;
    }

    if (p_dev->bdname_len == 0) {
        get_name_from_eir(p_dev->eir, p_dev->bdname, &p_dev->bdname_len);
    }

    ESP_LOGI(GAP_TAG, "Found a target device, address %s, name %s", bda_str, p_dev->bdname);
    p_dev->state = APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE;
   
    //******  REMOVE THESE STATEMENTS TO FORCE DISCOVERY UNTIL TIMEOUT  *******
    //ESP_LOGI(GAP_TAG, "Cancel device discovery ...");
    //esp_bt_gap_cancel_discovery();
}

static char *bdrmtname2str(uint8_t *rmtname, char *str, size_t size)
{
    if (rmtname == NULL || str == NULL || size < ESP_BT_GAP_READ_REMOTE_NAME_EVT) {
        return NULL;
    }
    
    return str = ((char *)rmtname);
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    //app_gap_cb_t *p_dev = &m_dev_info;
    //char bda_str[18];
    //char uuid_str[37];  // < SHUTUP COMPILER
    char device_name[ESP_BT_GAP_MAX_BDNAME_LEN + 1];


    switch (event) {

    case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
        //ESP_LOGI(GAP_TAG, "Device found: %s", bda2str(param->read_rmt_name.rmt_name, bda_str, 18));

        // TO DO - This line is causing the ESP to crash randomly....
        ESP_LOGI(GAP_TAG, "Remote Device Name: %s", bdrmtname2str(param->read_rmt_name.rmt_name, device_name, ESP_BT_GAP_MAX_BDNAME_LEN+1));
        //printf("Device Name: %s\n", param->read_rmt_name.rmt_name);

    case ESP_BT_GAP_DISC_RES_EVT: {
        update_device_info(param);
        esp_bt_gap_read_remote_name(param->disc_res.bda);  // THIS FINDS THE DEVICE NAME IN THE EIR IF ITS PRESENT!!!!
        break;
    }
    // ******************* NO NEED TO DISCOVER SERVICES AT THIS POINT **********************
    //
    //case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
    //    if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
    //        ESP_LOGI(GAP_TAG, "Device discovery stopped.");
    //        if ( (p_dev->state == APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE ||
    //                p_dev->state == APP_GAP_STATE_DEVICE_DISCOVERING)
    //                && p_dev->dev_found) {
    //            p_dev->state = APP_GAP_STATE_SERVICE_DISCOVERING;
    //            ESP_LOGI(GAP_TAG, "Discover services ...");
    //            esp_bt_gap_get_remote_services(p_dev->bda);
    //        }
    //    } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
    //        ESP_LOGI(GAP_TAG, "Discovery started.");
    //    }
    //    break;
    //}
    //
    //case ESP_BT_GAP_RMT_SRVCS_EVT: {
    //    if (memcmp(param->rmt_srvcs.bda, p_dev->bda, ESP_BD_ADDR_LEN) == 0 &&
    //            p_dev->state == APP_GAP_STATE_SERVICE_DISCOVERING) {
    //        p_dev->state = APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE;
    //        if (param->rmt_srvcs.stat == ESP_BT_STATUS_SUCCESS) {
    //            ESP_LOGI(GAP_TAG, "Services for device %s found",  bda2str(p_dev->bda, bda_str, 18));
    //            for (int i = 0; i < param->rmt_srvcs.num_uuids; i++) {
    //                esp_bt_uuid_t *u = param->rmt_srvcs.uuid_list + i;
    //                ESP_LOGI(GAP_TAG, "--%s", uuid2str(u, uuid_str, 37));
    //            }
    //        } else {
    //            ESP_LOGI(GAP_TAG, "Services for device %s not found",  bda2str(p_dev->bda, bda_str, 18));
    //        }
    //    }
    //    break;
    //}

    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
    default: {
        ESP_LOGI(GAP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

static void bt_app_gap_start_up(void)
{
    /* register GAP callback function */
    esp_bt_gap_register_callback(bt_app_gap_cb);

    char *dev_name = "ESP_GAP_INQRUIY";
    esp_bt_dev_set_device_name(dev_name);

    /* set discoverable and connectable mode, wait to be connected */
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    /* inititialize device information and status */
    bt_app_gap_init();

    /* start to discover nearby Bluetooth devices */
    app_gap_cb_t *p_dev = &m_dev_info;
    p_dev->state = APP_GAP_STATE_DEVICE_DISCOVERING;
    //esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 30, 0);
}



static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    uint8_t * data = (uint8_t *) malloc(BUF_SIZE);
    char str[20];
    char * cmd;
    char * param;

    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    int len = uart_read_bytes(EX_UART_NUM, data, (BUF_SIZE - 1), 100/portTICK_PERIOD_MS);
                    data[len] = '\0';
                        //bt_app_gap_start_up();
                    sprintf(str, "%s", (char *) data);
                    //ESP_LOGI(TAG, "New String: %s", str);
                    cmd = strtok(str,"=");
                    param = strtok(NULL,"=");
                    ESP_LOGI(TAG, "Command: %s", cmd);
                    ESP_LOGI(TAG, "Parameter: %s", param);
                    if (strcmp((char*)cmd, "rssi") == 0)
                        {
                           ESP_LOGI(TAG, "Set RSSI");
                        }
                    if (cmd == 'btaddr')
                        {
                            ESP_LOGI(TAG, "Set BT ADDRESS");
                        }
                    ESP_LOGI(TAG, "YoYO!!!");
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider increasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                /*case UART_PATTERN_DET:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(EX_UART_NUM);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(EX_UART_NUM);
                    } else {
                        uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "YOU ARE NOW IN A MODE!!!  MORE TO COME...");
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;*/
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

// ********************* THIS FUNCTION IS NOT NEEDED YET **********************

/*static char *uuid2str(esp_bt_uuid_t *uuid, char *str, size_t size)
{
    if (uuid == NULL || str == NULL) {
        return NULL;
    }

    if (uuid->len == 2 && size >= 5) {
        sprintf(str, "%04x", uuid->uuid.uuid16);
    } else if (uuid->len == 4 && size >= 9) {
        sprintf(str, "%08"PRIx32, uuid->uuid.uuid32);
    } else if (uuid->len == 16 && size >= 37) {
        uint8_t *p = uuid->uuid.uuid128;
        sprintf(str, "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                p[15], p[14], p[13], p[12], p[11], p[10], p[9], p[8],
                p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0]);
    } else {
        return NULL;
    }

    return str;
}*/

void app_main(void)
{
    /* Initialize NVS — it is used to store PHY calibration data and save key-value pairs in flash memory*/
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(GAP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(GAP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(GAP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(GAP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_dev_set_device_name("PC_DONGLE")) != ESP_OK) {
        ESP_LOGE(GAP_TAG, "%s initialize device name failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bredr_tx_power_set(ESP_PWR_LVL_N0, ESP_PWR_LVL_N0)) != ESP_OK) {
        ESP_LOGE(GAP_TAG, "%s set RSSI power level failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

   // bt_app_gap_start_up();

    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //USE THIS FUNCTION TO MAKE WAY FOR SEPARATE MODES NEEDED ON THE DONGLE
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);

    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}
