/*
 * author : Pierre-Edouard ALLIO
 * Project : Average IQ Watch
 * Description : Smart Watch prototype using ESP32
 */
/*
 * In this file you will find all the necessary functions
 * needed to manage all the project's time functionalities
 */

//###################################################################################################
//#######################  					INCLUDES 					 ############################
//###################################################################################################
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "apps/sntp/sntp.h"
#include "driver/rtc_io.h"
#include "rtc.h"

//###################################################################################################
//#######################  					VARIABLES 					 ############################
//###################################################################################################
//WiFi Configuration //
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  3
//SNTP Configuration//
#define CURRENT_YEAR 2023
//All types of security protocols used to connect to a WiFi network//
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif
//Logging variables//
static const char *TAG = "wifi station";
//WiFi connection related variables//
int Number_Tries = 0;
//Time global variables//
struct tm timedata;
time_t currenttime;
//FreeRTOS variables//
TaskHandle_t xWiFi_Connect_SNTP=NULL;
TaskHandle_t xGet_RTC_Time=NULL;
//###################################################################################################
//#######################  					FONCTIONS 					 ############################
//###################################################################################################
/*
 * This function connects to a NTP server using the WiFi network previously
 * configured in the configuration menu. Function is supposed to be used with FreeRTOS.
 * This task connects to the WiFi network, retrieves the time from the NTP server,
 * synchronizes the RTC to the time retrieved and disconnects from WiFi network.
 */
void WiFi_Connect_SNTP(void)
{
    //1-Initialize NVS//
    nvs_flash_init();
    //2-Initialize TCP-IP//
    esp_netif_init();
    esp_event_loop_create_default();
    ESP_LOGI(TAG, "WiFi prerequisites activated");
    //3-Configure WiFi Station//
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    //4-Initialize WiFi//
    esp_wifi_init(&cfg);
    //5-Configure WiFi//
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
			.password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    ESP_LOGI(TAG, "WiFi configuration done");
    //6-Start WiFi Connection//
    esp_wifi_start();
    ESP_LOGI(TAG, "WiFi turned on");
    //6-8-10-Try to Connect to WiFi//
	while ((esp_wifi_connect()!=ESP_OK)&&(Number_Tries<EXAMPLE_ESP_MAXIMUM_RETRY))
	{
		Number_Tries++;
		ESP_LOGI(TAG, "Connecting to WiFi network please wait...");
	}
	//7-Suspend Task Unable to Connect to WiFi//
	if(Number_Tries==EXAMPLE_ESP_MAXIMUM_RETRY)
	{
		ESP_LOGI(TAG, "Failed to connect to WiFi network -> suspending WiFi task");
		vTaskSuspend(xWiFi_Connect_SNTP);
	}
	ESP_LOGI(TAG, "Successfully connected to WiFi network");
    //11-Initialize the SNTP service//
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
	sntp_setservername(0, "time.google.com");
	//12-Connect to the SNTP service//
	sntp_init();
	ESP_LOGI(TAG, "Connected to NTP server");
	// wait for the service to set the time
	time_t now;
	struct tm timeinfo;
	//System clock time as time_t//
	time(&now);
	//Converted system time as tm//
	localtime_r(&now, &timeinfo);
	//As long as RTC doesnt have good value//
	while(timeinfo.tm_year < (CURRENT_YEAR - 1900))
	{
		ESP_LOGI(TAG, "Waiting for SNTP to respond please wait...");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		//Getting RTC time//
		time(&now);
		localtime_r(&now, &timeinfo);
	}
	ESP_LOGI(TAG, "Time has been acquired from SNTP service");
	//14-Stop WiFi Connection//
    esp_wifi_disconnect();
    ESP_LOGI(TAG, "Successfully disconnected from WiFi network");
    //15-Turn OFF WiFi//
    esp_wifi_stop();
    ESP_LOGI(TAG, "WiFi turned off");
    //16-Task Finished//
    ESP_LOGI(TAG, "Suspending WiFi task");
    vTaskSuspend(xWiFi_Connect_SNTP);
}
/*
 * This function updates the time storage variable "timedata" with the RTC time.
 * This function is supposed to be used with FreeRTOS.
 */
void Get_RTC_Time(void)
{
	//Calling time() is necessary in order to get the current RTC time//
	while(1)
	{
		//Get current RTC time in time_t//
		time(&currenttime);
		//Transform RTC time into tm struct//
		localtime_r(&currenttime,&timedata);
		//Display (https://www.tutorialspoint.com/c_standard_library/time_h.htm)//
		ESP_LOGI(TAG,"Time : %d : %d : %d",timedata.tm_hour,timedata.tm_min,timedata.tm_sec);
		ESP_LOGI(TAG,"Date : %d : %d : %d",timedata.tm_mday,1+timedata.tm_mon,1900+timedata.tm_year);
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}
