/* Private functions I2C ---------------------------------------------------------*/
int32_t i2c_master_read_slave_LSM6DSO(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_rd, uint16_t size);
int32_t i2c_master_write_slave_LSM6DSO(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_wr, uint16_t size);
int32_t i2c_master_read_slave_LIS2MDL(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_rd, uint16_t size);
int32_t i2c_master_write_slave_LIS2MDL(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_wr, uint16_t size);
void get_lis2mdl_data_task(void *args);
void get_lsm6dso_data_task(void *args);

esp_err_t i2c_master_init(void);
void whoami_task(void *args);