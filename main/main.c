#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "time.h"
#include "TimeManagement.h"

//FreeRTOS variables//
extern TaskHandle_t xWiFi_Connect_SNTP;
TaskHandle_t xState_Machine;
SemaphoreHandle_t RTC_timer_semaphore = NULL;
SemaphoreHandle_t NTP_timer_semaphore = NULL;
QueueSetHandle_t semaphore_queueset = NULL;
static const char *TAG = "State_Machine";

bool IRAM_ATTR RTC_timer_IRQ_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(RTC_timer_semaphore, &xHigherPriorityTaskWoken);
	return true;
}

bool IRAM_ATTR NTP_timer_IRQ_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(NTP_timer_semaphore, &xHigherPriorityTaskWoken);
	return true;
}
void init_RTC_timer(uint64_t NR_delay) {
	gptimer_handle_t NR_timer_handle = NULL;
	gptimer_config_t NR_timer_config = {
			.clk_src = GPTIMER_CLK_SRC_DEFAULT,
			.direction = GPTIMER_COUNT_UP,
			.resolution_hz = 1000 * 1000,
	};
	ESP_ERROR_CHECK(gptimer_new_timer(&NR_timer_config, &NR_timer_handle));

	gptimer_alarm_config_t NR_alarm_config = {
			.alarm_count = NR_delay,
			.reload_count = 0,
			.flags.auto_reload_on_alarm = true,
	};
	ESP_ERROR_CHECK(gptimer_set_alarm_action(NR_timer_handle, &NR_alarm_config));

	gptimer_event_callbacks_t NR_cbs_config = {
			.on_alarm = RTC_timer_IRQ_handler,
	};
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(NR_timer_handle, &NR_cbs_config, NULL));

	ESP_ERROR_CHECK(gptimer_enable(NR_timer_handle));
	ESP_ERROR_CHECK(gptimer_start(NR_timer_handle));
}

void init_NTP_timer(uint64_t NR_delay) {
	gptimer_handle_t NR_timer_handle = NULL;
	gptimer_config_t NR_timer_config = {
			.clk_src = GPTIMER_CLK_SRC_DEFAULT,
			.direction = GPTIMER_COUNT_UP,
			.resolution_hz = 1000 * 1000,
	};
	ESP_ERROR_CHECK(gptimer_new_timer(&NR_timer_config, &NR_timer_handle));

	gptimer_alarm_config_t NR_alarm_config = {
			.alarm_count = NR_delay,
			.reload_count = 0,
			.flags.auto_reload_on_alarm = true,
	};
	ESP_ERROR_CHECK(gptimer_set_alarm_action(NR_timer_handle, &NR_alarm_config));

	gptimer_event_callbacks_t NR_cbs_config = {
			.on_alarm = NTP_timer_IRQ_handler,
	};
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(NR_timer_handle, &NR_cbs_config, NULL));

	ESP_ERROR_CHECK(gptimer_enable(NR_timer_handle));
	ESP_ERROR_CHECK(gptimer_start(NR_timer_handle));
}

void state_machine(void)
{
	QueueSetMemberHandle_t received_semaphore;
	while(1)
	{
		received_semaphore = xQueueSelectFromSet(semaphore_queueset, 5000/portTICK_PERIOD_MS);
		if(received_semaphore == RTC_timer_semaphore)
		{
			ESP_LOGW(TAG, "ME : Timer semaphore");
			xSemaphoreTake(RTC_timer_semaphore, 0);
			Get_RTC_Time();
		}
		else if(received_semaphore == NTP_timer_semaphore)
		{
			ESP_LOGW(TAG,"ME : NTP semaphore");
			xSemaphoreTake(NTP_timer_semaphore, 0);
			vTaskResume(xWiFi_Connect_SNTP);
		}
		else
		{
			ESP_LOGW(TAG,"ME : Sleep");
			Sleep_Mode();
		}
	}
}
void app_main(void)
{
	ESP_LOGI(TAG, "Start app_main");
	RTC_timer_semaphore = xSemaphoreCreateBinary();
	NTP_timer_semaphore = xSemaphoreCreateBinary();
	semaphore_queueset = xQueueCreateSet(2);
	xQueueAddToSet(RTC_timer_semaphore, semaphore_queueset);
	xQueueAddToSet(NTP_timer_semaphore,semaphore_queueset);
	//Initialisation périphériques//
	WiFi_Init();
	RTC_Init();
	//Create Task//
	xTaskCreatePinnedToCore(&WiFi_Connect_SNTP, "WiFi_Connect_SNTP", 10000, NULL,1,&xWiFi_Connect_SNTP,0);
	vTaskSuspend(xWiFi_Connect_SNTP);
	xTaskCreatePinnedToCore(&state_machine, "state_machine", 10000, NULL, 6, &xState_Machine,0);
	init_RTC_timer(1*1000 * 1000);
	init_NTP_timer(60*1000*1000);
	ESP_LOGI(TAG, "app_main is done");
}
