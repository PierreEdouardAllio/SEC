
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

//FreeRTOS variables//
extern TaskHandle_t xWiFi_Connect_SNTP;
extern TaskHandle_t xGet_RTC_Time;
extern TaskHandle_t xSleep_Mode;
//Tasks included in TimeManagement.c"
void WiFi_Connect_SNTP(void);
void Get_RTC_Time(void);
void Sleep_Mode(void);

void app_main(void)
{
	//Create Task//
	xTaskCreatePinnedToCore(&WiFi_Connect_SNTP, "WiFi_Connect_SNTP", 10000, NULL,1,&xWiFi_Connect_SNTP,0);
	vTaskDelay(5000/portTICK_PERIOD_MS);
	xTaskCreatePinnedToCore(&Get_RTC_Time,"Get_RTC_Time",1000,NULL,3,&xGet_RTC_Time,0);
	vTaskDelay(5000/portTICK_PERIOD_MS);
	xTaskCreatePinnedToCore(&Sleep_Mode,"Sleep_Mode",1000,NULL,5,&xSleep_Mode,0);
	vTaskDelay(1000/portTICK_PERIOD_MS);
}
