#pragma once

#include "lvgl.h"
#include <string>
#include <vector>
#include <functional>

namespace esp_brookesia::apps::gyro_maze {

struct MenuItem {
    std::string label;
    std::function<void(void)> callback;
    bool disabled = false;
};

class MenuSystem {
public:
    /**
     * @brief Create a menu screen with a list of buttons
     * 
     * @param parent The parent object (usually the screen or a container)
     * @param title The menu title (optional, empty to skip)
     * @param items Vector of menu items
     * @return lv_obj_t* Pointer to the created menu container
     */
    static lv_obj_t* create(lv_obj_t *parent, const char *title, const std::vector<MenuItem>& items);
};

} // namespace esp_brookesia::apps::gyro_maze
