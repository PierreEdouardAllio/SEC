/* Private functions BLE ---------------------------------------------------------*/
int ble_gap_event(struct ble_gap_event *event, void *arg);
void ble_app_advertise(void);
void ble_app_on_sync(void);
void host_task(void *param);
void ble_start();
void ble_stop();
void vTaskFunction(void *pvParameters);
void vApplicationIdleHook(void);

int device_write_notes(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
int device_read_heading(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
int device_read_acceleration(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
int device_read_angular(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
