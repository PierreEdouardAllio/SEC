idf_component_register(SRCS "project_GUI_copy_main.c" "GUI.h" "GUI.c" "TimeManagement.c" "hours_hand.c" "minutes_hand.c" "seconds_hand.c" "watch_background.c" "compass_background.c" "compass_cross.c" "health_background.c" "plain_background.c"        
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_timer lvgl_esp32_drivers lvgl esp_wifi esp_event nvs_flash)
