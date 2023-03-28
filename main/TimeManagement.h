/*
 * author : Pierre-Edouard ALLIO
 * Project : Average IQ Watch
 * Description : Smart Watch prototype using ESP32
 */
/*
 * In this file you will find all the necessary functions
 * needed to manage all the project's time functionalities
 */

#ifndef MAIN_TIMEMANAGEMENT_H_
#define MAIN_TIMEMANAGEMENT_H_

void RTC_Init(void);
void WiFi_Init(void);
void WiFi_Connect_SNTP(void);
void Get_RTC_Time(void);
void Sleep_Mode(void);



#endif /* MAIN_TIMEMANAGEMENT_H_ */
