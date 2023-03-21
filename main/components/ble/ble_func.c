#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "nvs_flash.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "lsm6dso_reg.h"
#include "lis2mdl_reg.h"
#include "ble_func.h"

extern char *TAG;
extern struct ble_gatt_svc_def gatt_svcs[];

/* EXTERN I2C --------------------------------------------------------*/
extern int16_t data_raw_acceleration[3];
extern int16_t data_raw_angular_rate[3];
extern int16_t data_raw_temperature;
extern float acceleration_mg[3];
extern float angular_rate_mdps[3];
extern float temperature_degC;
extern uint8_t whoamI_LSM6DSO, rst_LSM6DSO;
extern stmdev_ctx_t dev_ctx_LSM6DSO;
extern int16_t data_raw_magnetic[3];
extern int16_t data_raw_temperature;
extern float magnetic_mG[3];
extern float temperature_degC;
extern uint8_t whoamI_LIS2MDL, rst_LIS2MDL;
extern stmdev_ctx_t dev_ctx_LIS2MDL;

extern float heading;
extern TaskHandle_t xHandle;
extern SemaphoreHandle_t xSemaphore_i2c;
/* Private variables - BLE --------------------------------------------------------*/
uint8_t ble_addr_type;
uint8_t ble_connect = 1;

/* BLE functions ---------------------------------------------------------*/
/*
device_read_notes
*/
// Write data to ESP32 defined as server
int device_write_notes(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    printf("Note: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
    return 0;
}

// Read data from ESP32 defined as server
int device_read_heading(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char c[50];
    sprintf(c, "direction : %f", heading);
    //os_mbuf_append(ctxt->om, "Data from the server", strlen("Data from the server"));
    os_mbuf_append(ctxt->om, c, strlen(c));
    return 0;
}
int device_read_acceleration(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char c[50];
    sprintf(c, "acceleration : (%f, %f, %f)", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
    //os_mbuf_append(ctxt->om, "Data from the server", strlen("Data from the server"));
    os_mbuf_append(ctxt->om, c, strlen(c));
    return 0;
}
int device_read_angular(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char c[50];
    sprintf(c, "acceleration : (%f, %f, %f)", angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
    //os_mbuf_append(ctxt->om, "Data from the server", strlen("Data from the server"));
    os_mbuf_append(ctxt->om, c, strlen(c));
    return 0;
}

// BLE event handling
int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ESP_LOGI(TAG, "fail");
            ble_app_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected");
        ble_app_advertise();
        break;

    // Advertise again after completion of the event
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

// Define the BLE connection
void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

void ble_start() {
    // Start BLE stack
    nvs_flash_init(); // 1 - Initialize NVS flash using
    nimble_port_init(); // 3 - Initialize the host stack

    // Set device name and initialize GAP and GATT services
    ble_svc_gap_device_name_set("BLE-Server");  // 4 - Initialize NimBLE configuration - server name
    ble_svc_gap_init();                         // 4 - Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                        // 4 - Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);             // 4 - Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);              // 4 - Initialize NimBLE configuration - queues gatt services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;       // 5 - Initialize application
}

void ble_stop() {
    nimble_port_deinit();
    nvs_flash_deinit();
}

void vTaskFunction(void *pvParameters) {
    uint8_t mini_flag = 0;

    while (1) {
        if (ble_connect == 1) {
            if (mini_flag == 0)
            {
                ESP_LOGE(TAG, "Activating BLE");
                ble_start();
                nimble_port_freertos_init(host_task);
                mini_flag = 1;
            }
            
        } else if (ble_connect == 0) 
        {
            if (mini_flag == 1){
                ESP_LOGE(TAG, "DEACTIVATING BLE");
                nimble_port_freertos_deinit(); // deinitialize the BLE stack
                ble_stop();
                mini_flag = 0;
            }
            
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 milliseconds
    }
}

void vApplicationIdleHook(void) {
    // Do nothing
}
