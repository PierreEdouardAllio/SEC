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

extern char *TAG;

/* Private variables - LSM6DSO --------------------------------------------------------*/
int16_t data_raw_acceleration[3];
int16_t data_raw_angular_rate[3];
int16_t data_raw_temperature;
float acceleration_mg[3];
float angular_rate_mdps[3];
float temperature_degC;
uint8_t whoamI_LSM6DSO, rst_LSM6DSO;
stmdev_ctx_t dev_ctx_LSM6DSO;

/* Private variables - LIS2MDL --------------------------------------------------------*/
int16_t data_raw_magnetic[3];
int16_t data_raw_temperature;
float magnetic_mG[3];
float temperature_degC;
uint8_t whoamI_LIS2MDL, rst_LIS2MDL;
stmdev_ctx_t dev_ctx_LIS2MDL;

float heading;

/* whoami task handler*/
TaskHandle_t xHandle = NULL;

/* Extra BS ---------------------------------------------------------*/
SemaphoreHandle_t xSemaphore_i2c;


/* I2C functions ---------------------------------------------------------*/

void whoami_task(void *args)
{
    int ret;

    while (1) {
        ESP_LOGI(TAG, "WHOAMI TASK");
        ESP_LOGI(TAG, "CHECKING ALL SENSORS");

        /* LSM6DSO */
        lsm6dso_device_id_get(&dev_ctx_LSM6DSO, &whoamI_LSM6DSO);
        if ( whoamI_LSM6DSO != LSM6DSO_ID )
            ret = ESP_ERR_TIMEOUT; /* manage here device not found */
        else ret = ESP_OK;
       
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("LSM6DSO connected. Product ID= %d \n", whoamI_LSM6DSO); 
        } else {
            ESP_LOGW(TAG, "%s: LSM6DSO sensor not connected, No ack...skip...", esp_err_to_name(ret));
        }

        /* LIS2MDL */
        lis2mdl_device_id_get(&dev_ctx_LIS2MDL, &whoamI_LIS2MDL);
        if ( whoamI_LIS2MDL != LIS2MDL_ID )
            ret = ESP_ERR_TIMEOUT; /* manage here device not found */
        else ret = ESP_OK;
       
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("LIS2MDL connected. Product ID= %d \n", whoamI_LIS2MDL); 
        } else {
            ESP_LOGW(TAG, "%s: LIS2MDL sensor not connected, No ack...skip...", esp_err_to_name(ret));
        }

       printf("End of Task WHOAMI!\n\n");

       /* whoami task is run only once. At the end we start get_temp_pressure_task n*/
       xTaskCreate(get_lsm6dso_data_task, "get_lsm6dso_data_task", 1024 * 2, (void *)0, 5, NULL);
       xTaskCreate(get_lis2mdl_data_task, "get_lis2mdl_data_task", 1024 * 2, (void *)0, 5, NULL);
       vTaskDelete(xHandle);
  }
}

int32_t i2c_master_read_slave_LSM6DSO(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_rd, uint16_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSO_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSO_SENSOR_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int32_t i2c_master_write_slave_LSM6DSO(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_wr, uint16_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSO_SENSOR_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int32_t i2c_master_read_slave_LIS2MDL(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_rd, uint16_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS2MDL_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS2MDL_SENSOR_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int32_t i2c_master_write_slave_LIS2MDL(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_wr, uint16_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS2MDL_SENSOR_ADDR<< 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void get_lsm6dso_data_task(void *args)
{
    uint8_t reg;
    while(1)
    {
        /* See if we can obtain the semaphore.  If the semaphore is not
        available wait 10 ticks to see if it becomes free. */
        if( xSemaphoreTake( xSemaphore_i2c, ( TickType_t ) 10 ) == pdTRUE )
        {
            ESP_LOGI(TAG, "GYRO got the mutex");
            /* Read output only if new xl value is available */
            lsm6dso_xl_flag_data_ready_get(&dev_ctx_LSM6DSO, &reg);

            if (reg) {
              /* Read acceleration field data */
              memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
              lsm6dso_acceleration_raw_get(&dev_ctx_LSM6DSO, data_raw_acceleration);
              acceleration_mg[0] =
                lsm6dso_from_fs2_to_mg(data_raw_acceleration[0]);
              acceleration_mg[1] =
                lsm6dso_from_fs2_to_mg(data_raw_acceleration[1]);
              acceleration_mg[2] =
                lsm6dso_from_fs2_to_mg(data_raw_acceleration[2]);
              printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
            }

            lsm6dso_gy_flag_data_ready_get(&dev_ctx_LSM6DSO, &reg);

            if (reg) {
              /* Read angular rate field data */
              memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
              lsm6dso_angular_rate_raw_get(&dev_ctx_LSM6DSO, data_raw_angular_rate);
              angular_rate_mdps[0] =
                lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[0]);
              angular_rate_mdps[1] =
                lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[1]);
              angular_rate_mdps[2] =
                lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[2]);
              printf("Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n", angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
            }

            lsm6dso_temp_flag_data_ready_get(&dev_ctx_LSM6DSO, &reg);vTaskDelay(2000 / portTICK_PERIOD_MS);

            if (reg) {
              /* Read temperature data */
              memset(&data_raw_temperature, 0x00, sizeof(int16_t));
              lsm6dso_temperature_raw_get(&dev_ctx_LSM6DSO, &data_raw_temperature);
              temperature_degC =
                lsm6dso_from_lsb_to_celsius(data_raw_temperature);
              printf("Temperature [degC]:%6.2f\r\n", temperature_degC);
            }
            //vTaskDelay(2000 / portTICK_PERIOD_MS);

            /* We have finished accessing the shared resource.  Release the
            semaphore. */
            xSemaphoreGive( xSemaphore_i2c );
            taskYIELD();
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        
    }
}

void get_lis2mdl_data_task(void *args)
{
    uint8_t reg;

    while (1) 
    {
        if( xSemaphoreTake( xSemaphore_i2c, ( TickType_t ) 10 ) == pdTRUE )
        {
            ESP_LOGI(TAG, "MAGNETO got the mutex");
            /* Read output only if new value is available */
            lis2mdl_mag_data_ready_get(&dev_ctx_LIS2MDL, &reg);

            if (reg) {
              /* Read magnetic field data */
              memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
              lis2mdl_magnetic_raw_get(&dev_ctx_LIS2MDL, data_raw_magnetic);
              magnetic_mG[0] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic[0]);
              magnetic_mG[1] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic[1]);
              magnetic_mG[2] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic[2]);
              //printf("Magnetic field [mG]:%4.2f\t%4.2f\t%4.2f\r\n", magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);

              heading = atan2(magnetic_mG[1], magnetic_mG[0]) * 180 / M_PI;
                if (heading < 0) {
                    heading += 360;
                }
                printf("Heading: %f degrees\n", heading);

              /* Read temperature data */
              memset(&data_raw_temperature, 0x00, sizeof(int16_t));
              lis2mdl_temperature_raw_get(&dev_ctx_LIS2MDL, &data_raw_temperature);
              temperature_degC = lis2mdl_from_lsb_to_celsius(data_raw_temperature);
              //printf("Temperature [degC]: %6.2f\r\n", temperature_degC);
            }
            //vTaskDelay(2000 / portTICK_PERIOD_MS);   

            /* We have finished accessing the shared resource.  Release the
            semaphore. */
            xSemaphoreGive( xSemaphore_i2c );
            taskYIELD();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
        } 
    
    }
}
