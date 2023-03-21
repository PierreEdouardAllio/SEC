
#include "lvgl.h"
#include "lvgl_helpers.h"
#include "esp_timer.h"
#include "GUI.h"

#define LV_TICK_PERIOD_MS 1

static void lv_tick_task(void *arg) {
	(void) arg;
	lv_tick_inc(LV_TICK_PERIOD_MS);
}
// Declare the images
LV_IMG_DECLARE(watchface_background);
LV_IMG_DECLARE(hours_arm);
LV_IMG_DECLARE(minutes_arm);
LV_IMG_DECLARE(seconds_arm);

uint8_t hour = 2;
uint8_t minute = 48;
double second = 29.;
int16_t angle_hour;
int16_t angle_minute;
double angle_second;

// Create the images pointers
lv_obj_t * ui_watch_face, * ui_hours, * ui_minutes, * ui_seconds;
lv_obj_t * calendar_background, * health_background, * compass_background, * ble_background, * raw_data_background;
lv_obj_t * main_screen, * calendar_screen, * health_screen, * compass_screen, * ble_screen, *raw_data_screen;
lv_obj_t * label_health_title;
lv_obj_t * label_compass_title;
lv_obj_t * label_ble_title;
lv_obj_t * label_raw_data_title;
uint8_t screen_state = 0;

void create_GUI_widgets(void)
{
	// Create the 6 screens objects
	main_screen = lv_obj_create(NULL, NULL);
	calendar_screen = lv_obj_create(NULL, NULL);
	health_screen = lv_obj_create(NULL, NULL);
	compass_screen = lv_obj_create(NULL, NULL);
	ble_screen = lv_obj_create(NULL, NULL);
	raw_data_screen = lv_obj_create(NULL, NULL);

	// *********************************************
	// **     Create the main screen objects      **
	// *********************************************
	ui_watch_face = lv_img_create(main_screen, NULL);
	ui_hours = lv_img_create(main_screen, NULL);
	ui_minutes = lv_img_create(main_screen, NULL);
	ui_seconds = lv_img_create(main_screen, NULL);
	// Set the source for every image and change their pivoting points
	lv_img_set_src(ui_watch_face, &watchface_background);
	lv_obj_align(ui_watch_face, NULL, LV_ALIGN_CENTER, 0, 0);

	lv_img_set_src(ui_hours, &hours_arm);
	lv_obj_align(ui_hours, NULL, LV_ALIGN_CENTER, 0, -20);
	lv_img_set_pivot(ui_hours, 32, 72);

	lv_img_set_src(ui_minutes, &minutes_arm);
	lv_obj_align(ui_minutes, NULL, LV_ALIGN_CENTER, 0, -45);
	lv_img_set_pivot(ui_minutes, 4, 100);

	lv_img_set_src(ui_seconds, &seconds_arm);
	lv_obj_align(ui_seconds, NULL, LV_ALIGN_CENTER, 0, -46);
	lv_img_set_pivot(ui_seconds, 4, 105);


	angle_hour = 10 * (hour % 12) * 360 / 12;
	angle_minute = 10 * minute * 360 / 60;
	angle_second = 10 * second * 360 / 60;
	lv_img_set_angle(ui_seconds, angle_second);
	lv_img_set_angle(ui_minutes, angle_minute);
	lv_img_set_angle(ui_hours, angle_hour);

	// *********************************************
	// **   Create the calendar screen objects    **
	// *********************************************
	calendar_background = lv_img_create(calendar_screen, NULL);
	lv_img_set_src(calendar_background, &watchface_background);
	lv_obj_align(calendar_background, NULL, LV_ALIGN_CENTER, 0, 0);

	lv_obj_t  * calendar = lv_calendar_create(calendar_screen, NULL);
	lv_obj_set_size(calendar, 169, 169);
	lv_obj_align(calendar, NULL, LV_ALIGN_CENTER, 0, 0);
	// Make the date number smaller to be sure they fit into their area
	lv_obj_set_style_local_text_font(calendar, LV_CALENDAR_PART_DATE, LV_STATE_DEFAULT, lv_theme_get_font_small());
	/*Set today's date*/
	lv_calendar_date_t today;
	today.year = 2023;
	today.month = 03;
	today.day = 8;
	lv_calendar_set_today_date(calendar, &today);
	lv_calendar_set_showed_date(calendar, &today);

	// *********************************************
	// **    Create the health screen objects     **
	// *********************************************
	health_background = lv_img_create(health_screen, NULL);
	lv_img_set_src(health_background, &watchface_background);
	lv_obj_align(health_background, NULL, LV_ALIGN_CENTER, 0, 0);

	label_health_title = lv_label_create(health_screen, NULL);
	lv_label_set_recolor(label_health_title, 1);
	lv_label_set_text(label_health_title, "#ffffff Health#");
	lv_obj_align(label_health_title, NULL, LV_ALIGN_CENTER, 0, -100);


	// *********************************************
	// **   Create the compass screen objects     **
	// *********************************************
	compass_background = lv_img_create(compass_screen, NULL);
	lv_img_set_src(compass_background, &watchface_background);
	lv_obj_align(compass_background, NULL, LV_ALIGN_CENTER, 0, 0);

	label_compass_title = lv_label_create(compass_screen, NULL);
	lv_label_set_recolor(label_compass_title, 1);
	lv_label_set_text(label_compass_title, "#ffffff Compass#");
	lv_obj_align(label_compass_title, NULL, LV_ALIGN_CENTER, 0, -100);


	// *********************************************
	// **     Create the ble screen objects       **
	// *********************************************
	ble_background = lv_img_create(ble_screen, NULL);
	lv_img_set_src(ble_background, &watchface_background);
	lv_obj_align(ble_background, NULL, LV_ALIGN_CENTER, 0, 0);

	label_ble_title = lv_label_create(ble_screen, NULL);
	lv_label_set_recolor(label_ble_title, 1);
	lv_label_set_text(label_ble_title, "#ffffff BLE settings#");
	lv_obj_align(label_ble_title, NULL, LV_ALIGN_CENTER, 0, -100);


	// *********************************************
	// **     Create the ble screen objects       **
	// *********************************************
	raw_data_background = lv_img_create(raw_data_screen, NULL);
	lv_img_set_src(raw_data_background, &watchface_background);
	lv_obj_align(raw_data_background, NULL, LV_ALIGN_CENTER, 0, 0);

	label_raw_data_title = lv_label_create(raw_data_screen, NULL);
	lv_label_set_recolor(label_raw_data_title, 1);
	lv_label_set_text(label_raw_data_title, "#ffffff Raw data#");
	lv_obj_align(label_raw_data_title, NULL, LV_ALIGN_CENTER, 0, -100);



	lv_scr_load(main_screen);
	lv_task_handler();
}

void change_screen()
{
	switch (screen_state) {
	    case 0 :
	    	lv_scr_load(main_screen);
	        break;
	    case 1 :
	    	lv_scr_load(calendar_screen);
	    	break;
	    case 2 :
	    	lv_scr_load(health_screen);
	    	break;
	    case 3 :
	    	lv_scr_load(compass_screen);
	    	break;
	    case 4 :
	    	lv_scr_load(ble_screen);
	    	break;
	    case 5 :
	    	lv_scr_load(raw_data_screen);
	    	break;
	    default:
	    	lv_scr_load(main_screen);
	}
}

void ui_update_time()
{
	//uint8_t hour = 2;
	//uint8_t minute = 45;
	//uint8_t second = 20;
	//int16_t angle_hour = 10 * (hour % 12) * 360 / 12;
	//int16_t angle_minute = 10 * minute * 360 / 60;
	//int16_t angle_second = 10 * second * 360 / 60;
	//lv_img_set_src(ui_seconds, &seconds_arm);
	//lv_obj_align(ui_seconds, NULL, LV_ALIGN_CENTER, 0, -46);
	// Change the pivoting point
	//lv_img_set_pivot(ui_seconds, 4, 105);
	//while(1)
	//{
		//angle_hour = 10 * (hour % 12) * 360 / 12;
		//angle_minute = 10 * minute * 360 / 60;
		angle_second = 10 * second * 360 / 60;
		lv_img_set_angle(ui_seconds, angle_second);
		//lv_img_set_angle(ui_minutes, angle_minute);
		//lv_img_set_angle(ui_hours, angle_hour);
		//if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
			//create_lvgl_first_application();
		if (second >= 60) second = 0;
		else second++;
			//ESP_LOGI(TAG2, "supposed to do my job");
			//xSemaphoreGive(xGuiSemaphore);
		//vTaskDelay(10 / portTICK_PERIOD_MS);
		//taskYIELD();
		//}
	//}
	//vTaskDelete(NULL); //Delete this task if it exits from the loop above
}

void init_lvgl(void) {

	lv_init();

	/* Initialize SPI or I2C bus used by the drivers */
	lvgl_driver_init();
	static lv_color_t buf1[DISP_BUF_SIZE];
	/* Use double buffered when not working with monochrome displays */
	static lv_color_t *buf2 = NULL;
	static lv_disp_buf_t disp_buf;
	uint32_t size_in_px = DISP_BUF_SIZE;

	/* Initialize the working buffer depending on the selected display.
	 * NOTE: buf2 == NULL when using monochrome displays. */
	lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

	lv_disp_drv_t disp_drv;

	lv_disp_drv_init(&disp_drv);
	disp_drv.flush_cb = disp_driver_flush;
	disp_drv.buffer = &disp_buf;
	lv_disp_drv_register(&disp_drv);
	/* Create and start a periodic timer interrupt to call lv_tick_inc */
	const esp_timer_create_args_t periodic_timer_args = {
		.callback = &lv_tick_task,
		.name = "periodic_gui"
	};
	esp_timer_handle_t periodic_timer;
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));
}


