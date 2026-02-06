idf_component_register(
    SRCS "app_gyro_maze.cpp" "gyro_maze_icon.c" "menu_system.cpp" "adventure_mode.cpp"
    INCLUDE_DIRS "."
    REQUIRES brookesia_core lvgl esp_lcd_touch app_gyro_game
    WHOLE_ARCHIVE
)
