idf_component_register(SRCS "project_GUI_main.c" "hours_arm.c" "minutes_arm.c" "seconds_arm.c" "watchface_background.c" "GUI.h" "GUI.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_timer lvgl_esp32_drivers lvgl)
