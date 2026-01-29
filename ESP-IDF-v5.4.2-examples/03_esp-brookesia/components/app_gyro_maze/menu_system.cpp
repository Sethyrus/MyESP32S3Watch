#include "menu_system.hpp"

namespace esp_brookesia::apps::gyro_maze {

// Wrapper to bridge C-style LVGL callback to C++ std::function
static void btn_event_handler(lv_event_t *e) {
    auto *callback = static_cast<std::function<void(void)>*>(lv_event_get_user_data(e));
    if (callback && *callback) {
        (*callback)();
    }
}

// Cleanup function to delete the callback from the heap when the button is deleted
static void btn_delete_handler(lv_event_t *e) {
    auto *callback = static_cast<std::function<void(void)>*>(lv_event_get_user_data(e));
    if (callback) {
        delete callback;
    }
}

lv_obj_t* MenuSystem::create(lv_obj_t *parent, const char *title, const std::vector<MenuItem>& items) {
    // 1. Main Container
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, lv_pct(100), lv_pct(100)); // Full size
    lv_obj_set_style_bg_color(container, lv_color_black(), 0);
    lv_obj_set_style_border_width(container, 0, 0);
    lv_obj_set_style_radius(container, 0, 0);
    lv_obj_center(container);
    
    // Use flex layout for vertical list
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(container, 20, 0);
    lv_obj_set_style_pad_row(container, 15, 0); // Gap between items

    // 2. Title (Optional)
    if (title && title[0] != '\0') {
        lv_obj_t *lbl_title = lv_label_create(container);
        lv_label_set_text(lbl_title, title);
        lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(lbl_title, lv_color_white(), 0);
        // Add some extra space after title
        lv_obj_set_style_margin_bottom(lbl_title, 20, 0);
    }

    // 3. Buttons
    for (const auto& item : items) {
        lv_obj_t *btn = lv_btn_create(container);
        lv_obj_set_width(btn, lv_pct(80)); // 80% width
        lv_obj_set_height(btn, 50);
        
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, item.label.c_str());
        lv_obj_center(lbl);

        if (item.disabled) {
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x222222), 0);
            lv_obj_set_style_text_color(lbl, lv_color_hex(0x888888), 0);
        } else {
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x444444), 0);
            
            if (item.callback) {
                // Allocate callback on heap to pass to C-handler
                auto *cb_ptr = new std::function<void(void)>(item.callback);
                lv_obj_add_event_cb(btn, btn_event_handler, LV_EVENT_CLICKED, cb_ptr);
                // Important: Clean up heap memory when button is destroyed
                lv_obj_add_event_cb(btn, btn_delete_handler, LV_EVENT_DELETE, cb_ptr);
            }
        }
    }

    return container;
}

} // namespace esp_brookesia::apps::gyro_maze
