
#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif

const uint8_t gyro_maze_icon_map[] = {
    // 1x1 transparent pixel for placeholder
    0x00, 0x00, 0x00, 0x00, 
};

const lv_image_dsc_t gyro_maze_icon = {
  .header.cf = LV_COLOR_FORMAT_ARGB8888,
  .header.magic = LV_IMAGE_HEADER_MAGIC,
  .header.w = 1,
  .header.h = 1,
  .data_size = 4,
  .data = gyro_maze_icon_map,
};
