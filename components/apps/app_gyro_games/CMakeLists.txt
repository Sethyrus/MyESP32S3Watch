idf_component_register(
    SRCS "app_gyro_games.cpp"
         "modes/simple_maze/gyro_maze_icon.c"
         "shared/menu_system.cpp"
         "modes/big_maze/adventure_mode.cpp"
         "modes/test_gyro/gyro_game_icon.c"
    INCLUDE_DIRS "." "shared" "modes/big_maze"
    REQUIRES brookesia_core lvgl esp_lcd_touch
    WHOLE_ARCHIVE
)
