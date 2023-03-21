//******************************************************************************
//                                 INCLUDES
//******************************************************************************
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "lvgl.h"
#include "lvgl_helpers.h"
#include "esp_timer.h"
#include "driver/gptimer.h"

#include "GUI.h"

//******************************************************************************
//                                  DEFINES
//******************************************************************************
#define TAG "lvgl_first"
#define red_led 0
#define green_led 0
#define blue_led 4

#define right_btn 34
#define left_btn 35
#define ok_btn 36

#define GPIO_INPUT_PIN_SEL  ((1ULL<<right_btn) | (1ULL<<left_btn) | (1ULL<<ok_btn))

extern uint8_t screen_state;

//******************************************************************************
//                              GLOBAL VARIABLES
//******************************************************************************
TaskHandle_t blink_task_handle = NULL;
TaskHandle_t state_machine_handle = NULL;

SemaphoreHandle_t right_btn_semaphore = NULL;
SemaphoreHandle_t left_btn_semaphore = NULL;
SemaphoreHandle_t ok_btn_semaphore = NULL;
SemaphoreHandle_t acq_timer_semaphore = NULL;
SemaphoreHandle_t aff_update_semaphore = NULL;

QueueSetHandle_t semaphore_queueset = NULL;

//******************************************************************************
//                                IRQ HANDLERS
//******************************************************************************
static void IRAM_ATTR gpio_right_IRQ_handler(void *args) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(right_btn_semaphore, &xHigherPriorityTaskWoken);
}

static void IRAM_ATTR gpio_left_IRQ_handler(void *args) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(left_btn_semaphore, &xHigherPriorityTaskWoken);
}

static void IRAM_ATTR gpio_ok_IRQ_handler(void *args) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(ok_btn_semaphore, &xHigherPriorityTaskWoken);
}

static bool IRAM_ATTR acq_timer_IRQ_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(acq_timer_semaphore, &xHigherPriorityTaskWoken);
	return true;
}

static bool IRAM_ATTR display_timer_IRQ_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(aff_update_semaphore, &xHigherPriorityTaskWoken);
	return true;
}

//******************************************************************************
//                            TIMERS CONFIGURATION
//******************************************************************************
void init_acq_timer(uint64_t NR_delay) {
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
			.on_alarm = acq_timer_IRQ_handler,
	};
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(NR_timer_handle, &NR_cbs_config, NULL));

	ESP_ERROR_CHECK(gptimer_enable(NR_timer_handle));
	ESP_ERROR_CHECK(gptimer_start(NR_timer_handle));
}

void init_display_timer(uint64_t NR_delay) {
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
			.on_alarm = display_timer_IRQ_handler,
	};
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(NR_timer_handle, &NR_cbs_config, NULL));

	ESP_ERROR_CHECK(gptimer_enable(NR_timer_handle));
	ESP_ERROR_CHECK(gptimer_start(NR_timer_handle));
}

//******************************************************************************
//                               LVGL INIT
//******************************************************************************

//******************************************************************************
//                               FREERTOS TASKS
//******************************************************************************
void blink_task(void *arg)
{
	while(1)
	{
		gpio_set_level(blue_led, 0);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		gpio_set_level(blue_led, 1);
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

//******************************************************************************
//                               STATE MACHINE
//******************************************************************************
static void state_machine(void *arg)
{
	uint8_t state_aff = 0;
	QueueSetMemberHandle_t received_semaphore;

	vTaskResume(blink_task_handle);

	while(1) {
		received_semaphore = xQueueSelectFromSet(semaphore_queueset, portMAX_DELAY);

		if (received_semaphore == right_btn_semaphore) {
			ESP_LOGE(TAG, "Right button pressed");
			xSemaphoreTake(right_btn_semaphore, 0);
			if (screen_state++ >= 5) screen_state = 0;
			change_screen();
		}
		else if (received_semaphore == left_btn_semaphore) {
			ESP_LOGE(TAG, "Left button pressed");
			xSemaphoreTake(left_btn_semaphore, 0);
			if (screen_state-- <= 0) screen_state = 5;
			change_screen();
		}
		else if (received_semaphore == ok_btn_semaphore) {
			ESP_LOGE(TAG, "OK button pressed");
			xSemaphoreTake(ok_btn_semaphore, 0);
			//gpio_set_direction(ok_btn, GPIO_MODE_INPUT);
			//screen_state = !screen_state;
			//change_screen();
			//gpio_set_direction(ok_btn, GPIO_MODE_OUTPUT);
		}
		else if (received_semaphore == aff_update_semaphore) {
			//ESP_LOGI(TAG, "ME : Display_update");
			xSemaphoreTake(aff_update_semaphore, 0);

			lv_task_handler();
		}
		else if (received_semaphore == acq_timer_semaphore) {
			ESP_LOGW(TAG, "ME : Timer semaphore");
			xSemaphoreTake(acq_timer_semaphore, 0);

			ui_update_time();
		}
	}
}
//******************************************************************************
//                                APP MAIN
//******************************************************************************
void app_main(void)
{
	ESP_LOGI(TAG, "Start app_main");

	gpio_reset_pin(right_btn);
	gpio_set_direction(right_btn, GPIO_MODE_INPUT);
	gpio_set_intr_type(right_btn, GPIO_INTR_POSEDGE);

	gpio_reset_pin(left_btn);
	gpio_set_direction(left_btn, GPIO_MODE_INPUT);
	gpio_set_intr_type(left_btn, GPIO_INTR_POSEDGE);

	gpio_reset_pin(ok_btn);
	gpio_set_direction(ok_btn, GPIO_MODE_INPUT);
	gpio_set_intr_type(ok_btn, GPIO_INTR_POSEDGE);

	gpio_reset_pin(blue_led);
	gpio_set_direction(blue_led, GPIO_MODE_OUTPUT);
	gpio_set_level(blue_led, 0);

	right_btn_semaphore = xSemaphoreCreateBinary();
	left_btn_semaphore = xSemaphoreCreateBinary();
	ok_btn_semaphore = xSemaphoreCreateBinary();
	acq_timer_semaphore = xSemaphoreCreateBinary();
	aff_update_semaphore = xSemaphoreCreateBinary();

	semaphore_queueset = xQueueCreateSet(2);
	xQueueAddToSet(right_btn_semaphore, semaphore_queueset);
	xQueueAddToSet(left_btn_semaphore, semaphore_queueset);
	xQueueAddToSet(ok_btn_semaphore, semaphore_queueset);
	xQueueAddToSet(acq_timer_semaphore, semaphore_queueset);
	xQueueAddToSet(aff_update_semaphore, semaphore_queueset);

	init_lvgl();
	create_GUI_widgets();

	xTaskCreate(blink_task, "blink_task", 10000, NULL, 3, &blink_task_handle);
	vTaskSuspend(blink_task_handle);

	xTaskCreate(state_machine, "state_machine", 10000, NULL, 6, &state_machine_handle);

	gpio_install_isr_service(0);
	//gpio_isr_handler_add(boot_btn, gpio_IRQ_handler, (void*)boot_btn);
	gpio_isr_handler_add(right_btn, gpio_right_IRQ_handler, (void*)right_btn);
	gpio_isr_handler_add(left_btn, gpio_left_IRQ_handler, (void*)left_btn);
	gpio_isr_handler_add(ok_btn, gpio_ok_IRQ_handler, (void*)ok_btn);

	init_acq_timer(1000 * 1000);
	init_display_timer(20 * 1000);

	ESP_LOGI(TAG, "app_main is done");
}
