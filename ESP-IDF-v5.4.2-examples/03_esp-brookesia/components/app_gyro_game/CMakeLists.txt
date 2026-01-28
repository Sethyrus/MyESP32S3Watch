idf_component_register(
    SRCS "app_gyro_game.cpp"
    INCLUDE_DIRS "."
    REQUIRES brookesia_core lvgl esp_lcd_touch
    WHOLE_ARCHIVE
)
