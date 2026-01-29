idf_component_register(
    SRCS "app_gyro_maze.cpp" "gyro_maze_icon.c" "menu_system.cpp"
    INCLUDE_DIRS "."
    REQUIRES brookesia_core lvgl esp_lcd_touch
    WHOLE_ARCHIVE
)
