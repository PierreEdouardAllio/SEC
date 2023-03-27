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
#include "i2c_func.h"
#include "ble_func.h"

char *TAG = "i2c-BLE";

/* EXTERN I2C --------------------------------------------------------*/

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define LSM6DSO_SENSOR_ADDR 0x6b   /*!< slave address for LSM6DSO sensor */
#define LIS2MDL_SENSOR_ADDR 0x1e   /*!< slave address for LIS2MDL sensor */
#define SENSOR_BUS I2C_MASTER_NUM
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

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

/* EXTERN BLE --------------------------------------------------------*/
extern uint8_t ble_addr_type;
extern uint8_t ble_connect;
/* Extra BS ---------------------------------------------------------*/


// Array of pointers to other service definitions
// UUID - Universal Unique Identifier
/* 
819b71d0-c005-11ed-afa1-0242ac120002

819b74b4-c005-11ed-afa1-0242ac120002

819b7810-c005-11ed-afa1-0242ac120002
*/
 struct ble_gatt_svc_def gatt_svcs[] = {
    
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180),                 // Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[])
     {
         {.uuid = BLE_UUID16_DECLARE(0xFEF4),           // Define UUID for reading Compass value
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read_heading},

          {.uuid = BLE_UUID16_DECLARE(0xEFF5),           // Define UUID for reading acceleration values
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read_acceleration},

          {.uuid = BLE_UUID16_DECLARE(0xEFF6),           // Define UUID for reading angular rate
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read_angular},

         {0}
     }
    },

    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x280),                 // Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[])
     {
         // {.uuid = BLE_UUID16_DECLARE(0xEFF4),           // Define UUID for reading
         //  .flags = BLE_GATT_CHR_F_READ,
         //  .access_cb = device_read_notes},

         {.uuid = BLE_UUID16_DECLARE(0xEDAD),           // Define UUID for writing
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write_notes},
         {0}
     }
    },
    {0}
};

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());

    /* This acts as the entry point of ST's LSM6DSO driver */
    dev_ctx_LSM6DSO.write_reg = i2c_master_write_slave_LSM6DSO;
    dev_ctx_LSM6DSO.read_reg = i2c_master_read_slave_LSM6DSO;
    dev_ctx_LSM6DSO.i2c_number = SENSOR_BUS;

    /* This acts as the entry point of ST's LIS2MDL driver */
    dev_ctx_LIS2MDL.write_reg = i2c_master_write_slave_LIS2MDL;
    dev_ctx_LIS2MDL.read_reg = i2c_master_read_slave_LIS2MDL;
    dev_ctx_LIS2MDL.i2c_number = SENSOR_BUS;


    /* Restore default configuration - LSM6DSO */
    lsm6dso_reset_set(&dev_ctx_LSM6DSO, PROPERTY_ENABLE);

    do {
        lsm6dso_reset_get(&dev_ctx_LSM6DSO, &rst_LSM6DSO);
    } while (rst_LSM6DSO);

    /* Restore default configuration - LIS2MDL */
    lis2mdl_reset_set(&dev_ctx_LIS2MDL, PROPERTY_ENABLE);

    do {
        lis2mdl_reset_get(&dev_ctx_LIS2MDL, &rst_LIS2MDL);
    } while (rst_LIS2MDL);

    /* Disable I3C interface */
    lsm6dso_i3c_disable_set(&dev_ctx_LSM6DSO, LSM6DSO_I3C_DISABLE);
    /* Enable Block Data Update */
    lsm6dso_block_data_update_set(&dev_ctx_LSM6DSO, PROPERTY_ENABLE);
    /* Set Output Data Rate */
    lsm6dso_xl_data_rate_set(&dev_ctx_LSM6DSO, LSM6DSO_XL_ODR_12Hz5);
    lsm6dso_gy_data_rate_set(&dev_ctx_LSM6DSO, LSM6DSO_GY_ODR_12Hz5);
    /* Set full scale */
    lsm6dso_xl_full_scale_set(&dev_ctx_LSM6DSO, LSM6DSO_2g);
    lsm6dso_gy_full_scale_set(&dev_ctx_LSM6DSO, LSM6DSO_2000dps);
    /* Configure filtering chain(No aux interface)
     * Accelerometer - LPF1 + LPF2 path
     */
    lsm6dso_xl_hp_path_on_out_set(&dev_ctx_LSM6DSO, LSM6DSO_LP_ODR_DIV_100);
    lsm6dso_xl_filter_lp2_set(&dev_ctx_LSM6DSO, PROPERTY_ENABLE);


    /* Enable Block Data Update */
    lis2mdl_block_data_update_set(&dev_ctx_LIS2MDL, PROPERTY_ENABLE);
    /* Set Output Data Rate */
    lis2mdl_data_rate_set(&dev_ctx_LIS2MDL, LIS2MDL_ODR_10Hz);
    /* Set / Reset sensor mode */
    lis2mdl_set_rst_mode_set(&dev_ctx_LIS2MDL, LIS2MDL_SENS_OFF_CANC_EVERY_ODR);
    /* Enable temperature compensation */
    lis2mdl_offset_temp_comp_set(&dev_ctx_LIS2MDL, PROPERTY_ENABLE);
    /* Set device in continuous mode */
    lis2mdl_operating_mode_set(&dev_ctx_LIS2MDL, LIS2MDL_CONTINUOUS_MODE);



    /* Create a mutex type semaphore. */
    xSemaphore_i2c = xSemaphoreCreateMutex();

    /* start whoami task */
    xTaskCreate(whoami_task, "i2c_whoami_task", 1024 * 2, (void *)0, 10, &xHandle);

    /* start BLE task */
    xTaskCreate(&vTaskFunction, "Task", 2048, NULL, 1, NULL);
}


