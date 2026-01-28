/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cmath>
#include "app_gyro_game.hpp"
#include "esp_brookesia.hpp"
#include "bsp/esp32_s3_touch_amoled_2_06.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

// --- QMI8658 sensor --
#ifdef ESP_UTILS_LOG_TAG
#   undef ESP_UTILS_LOG_TAG
#endif
#define ESP_UTILS_LOG_TAG "GyroGame"
#include "esp_lib_utils.h"

#define APP_NAME "Gyro Game"
#define LOG_TAG "GyroGame"

// Physics constants
// Physics constants
#define PHYSICS_FRICTION 0.90f       // Less drag (was 0.88) to keep momentum
#define PHYSICS_ACCEL_FACTOR 3.5f    // Higher sensitivity (was 2.5) for instant reaction
#define PHYSICS_BOUNCE 0.5f          // Bounce
#define PHYSICS_MAX_VEL 30.0f        // Higher terminal velocity
#define INPUT_SMOOTHING 0.3f         // Faster response (was 0.1)
#define CALIBRATION_DEADZONE 0.015f  // Tiny deadzone (was 0.08) to detect subtle diagonal tilts

using namespace std;
using namespace esp_brookesia::gui;
using namespace esp_brookesia::systems;

namespace esp_brookesia::apps {

GyroGame *GyroGame::_instance = nullptr;

GyroGame *GyroGame::requestInstance(bool use_status_bar, bool use_navigation_bar)
{
    if (_instance == nullptr) {
        _instance = new GyroGame(use_status_bar, use_navigation_bar);
    }
    return _instance;
}

GyroGame::GyroGame(bool use_status_bar, bool use_navigation_bar):
    App(APP_NAME, nullptr, false, use_status_bar, use_navigation_bar),
    container(nullptr), box(nullptr), label_debug(nullptr),
    pos_x(0), pos_y(0), vel_x(0), vel_y(0),
    screen_width(0), screen_height(0), box_size(50),
    imu_initialized(false), calibration_done(false),
    accel_bias_x(0), accel_bias_y(0), 
    smooth_ax(0), smooth_ay(0),
    qmi_dev(nullptr)
{
}

GyroGame::~GyroGame()
{
    if (qmi_dev) {
        free(qmi_dev);
    }
}

void GyroGame::perform_calibration() {
    if (!qmi_dev) return;

    ESP_LOGI(LOG_TAG, "Starting calibration...");
    if (label_debug) lv_label_set_text(label_debug, "Calibrating...");
    
    // Simple UI feedback, force render
    lv_refr_now(NULL);

    const int samples = 200;
    float sum_x = 0;
    float sum_y = 0;
    qmi8658_data_t data;

    for (int i = 0; i < samples; i++) {
        if (qmi8658_read_sensor_data(qmi_dev, &data) == ESP_OK) {
            // Normalize to g
            sum_x += (data.accelX / 1000.0f);
            sum_y += (data.accelY / 1000.0f);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    accel_bias_x = sum_x / samples;
    accel_bias_y = sum_y / samples;
    
    // Reset smoothing
    smooth_ax = 0;
    smooth_ay = 0;
    
    calibration_done = true;

    ESP_LOGI(LOG_TAG, "Calibration done. Bias X: %.3f, Y: %.3f", accel_bias_x, accel_bias_y);
    if (label_debug) lv_label_set_text_fmt(label_debug, "Calibrated!\nBias X:%.2f Y:%.2f", accel_bias_x, accel_bias_y);
}

void GyroGame::init_imu() {
    if (imu_initialized) return;

    ESP_LOGI(LOG_TAG, "Initializing QMI8658 Component...");
    esp_log_level_set(LOG_TAG, ESP_LOG_INFO); // Force INFO logging
    
    i2c_master_bus_handle_t bus_handle = bsp_i2c_get_handle();
    if (!bus_handle) {
         ESP_LOGE(LOG_TAG, "Failed to get I2C bus handle");
         return;
    }

    qmi_dev = (qmi8658_dev_t*)malloc(sizeof(qmi8658_dev_t));
    if (!qmi_dev) return;

    // Use High Address (0x6B) as in reference example
    if (qmi8658_init(qmi_dev, bus_handle, QMI8658_ADDRESS_HIGH) != ESP_OK) {
        ESP_LOGE(LOG_TAG, "QMI8658 Init Failed!");
        free(qmi_dev);
        qmi_dev = nullptr;
        return;
    }

    // Switch to 2G range for better tilt resolution
    qmi8658_set_accel_range(qmi_dev, QMI8658_ACCEL_RANGE_2G); 
    qmi8658_set_accel_odr(qmi_dev, QMI8658_ACCEL_ODR_500HZ);
    qmi8658_set_accel_unit_mps2(qmi_dev, false); // Return in 'g'

    qmi8658_write_register(qmi_dev, QMI8658_CTRL5, 0x03); 

    imu_initialized = true;
    
    // Auto calibrate on start
    perform_calibration();
}

void GyroGame::read_imu(float &acc_x, float &acc_y) {
    if (!imu_initialized) init_imu();
    if (!qmi_dev) {
        acc_x = 0; acc_y = 0;
        return;
    }

    qmi8658_data_t data;
    if (qmi8658_read_sensor_data(qmi_dev, &data) == ESP_OK) {
        
        // Normalize milli-g to g
        float raw_x = data.accelX / 1000.0f;
        float raw_y = data.accelY / 1000.0f;

        // Subtract bias
        if (calibration_done) {
            raw_x -= accel_bias_x;
            raw_y -= accel_bias_y;
        }

        // Apply Low Pass Filter (Smoothing)
        smooth_ax = smooth_ax + INPUT_SMOOTHING * (raw_x - smooth_ax);
        smooth_ay = smooth_ay + INPUT_SMOOTHING * (raw_y - smooth_ay);

        // Deadzone on SMOOTHED data
        if (fabsf(smooth_ax) < CALIBRATION_DEADZONE) smooth_ax = 0;
        if (fabsf(smooth_ay) < CALIBRATION_DEADZONE) smooth_ay = 0;

        acc_x = smooth_ax;
        acc_y = smooth_ay;
        
    } else {
        // ESP_LOGE(LOG_TAG, "IMU Read Failed");
        acc_x = 0;
        acc_y = 0;
    }
}

void GyroGame::event_handler(lv_event_t *e) {
    GyroGame *app = (GyroGame *)lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        app->perform_calibration();
    }
}

void GyroGame::update_physics(lv_timer_t *timer) {
    GyroGame *app = (GyroGame *)timer->user_data;
    static int log_counter = 0;
    
    float ax, ay;
    app->read_imu(ax, ay);

    // Invert/Swap based on reference
    float force_x = -ay;
    float force_y = ax;

    app->vel_x += force_x * PHYSICS_ACCEL_FACTOR;
    app->vel_y += force_y * PHYSICS_ACCEL_FACTOR;
    
    // Apply friction
    app->vel_x *= PHYSICS_FRICTION;
    app->vel_y *= PHYSICS_FRICTION;

    // Terminal velocity clamp
    if (app->vel_x > PHYSICS_MAX_VEL) app->vel_x = PHYSICS_MAX_VEL;
    if (app->vel_x < -PHYSICS_MAX_VEL) app->vel_x = -PHYSICS_MAX_VEL;
    if (app->vel_y > PHYSICS_MAX_VEL) app->vel_y = PHYSICS_MAX_VEL;
    if (app->vel_y < -PHYSICS_MAX_VEL) app->vel_y = -PHYSICS_MAX_VEL;

    // Update position
    app->pos_x += app->vel_x;
    app->pos_y += app->vel_y;

    // Wall collisions
    if (app->pos_x < 0) {
        app->pos_x = 0;
        app->vel_x = -app->vel_x * PHYSICS_BOUNCE;
    }
    if (app->pos_x > app->screen_width - app->box_size) {
        app->pos_x = app->screen_width - app->box_size;
        app->vel_x = -app->vel_x * PHYSICS_BOUNCE;
    }
    
    if (app->pos_y < 0) {
        app->pos_y = 0;
        app->vel_y = -app->vel_y * PHYSICS_BOUNCE;
    }
    if (app->pos_y > app->screen_height - app->box_size) {
        app->pos_y = app->screen_height - app->box_size;
        app->vel_y = -app->vel_y * PHYSICS_BOUNCE;
    }

    // Update UI
    lv_obj_set_pos(app->box, (lv_coord_t)app->pos_x, (lv_coord_t)app->pos_y);

    // Debug logging (every 10 frames = ~0.2 sec)
    log_counter++;
    if (log_counter >= 10) {
        log_counter = 0;
        ESP_LOGI(LOG_TAG, "In(%.2f, %.2f) -> Vel(%.2f, %.2f) -> Pos(%d, %d)", 
                 ax, ay, app->vel_x, app->vel_y, (int)app->pos_x, (int)app->pos_y);
    }
}

void GyroGame::timer_cb(lv_timer_t *timer) {
    GyroGame *app = (GyroGame *)timer->user_data;
    app->update_physics(timer);
}

bool GyroGame::run(void)
{
    // Get screen dimensions
    lv_obj_t *scr = lv_scr_act();
    screen_width = lv_obj_get_width(scr);
    screen_height = lv_obj_get_height(scr);

    // Create container
    container = lv_obj_create(scr);
    lv_obj_set_size(container, screen_width, screen_height);
    lv_obj_set_style_bg_color(container, lv_color_black(), 0);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_center(container);

    label_debug = lv_label_create(container);
    lv_label_set_text(label_debug, "Gyro Game Demo");
    lv_obj_align(label_debug, LV_ALIGN_TOP_MID, 0, 10);

    // Create a Calibration Button
    lv_obj_t *btn_calib = lv_btn_create(container);
    lv_obj_t *lb = lv_label_create(btn_calib);
    lv_label_set_text(lb, "Calibrate");
    lv_obj_align(btn_calib, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn_calib, event_handler, LV_EVENT_CLICKED, this);

    // Create Blue Box
    box = lv_obj_create(container);
    lv_obj_set_size(box, box_size, box_size);
    lv_obj_set_style_bg_color(box, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_set_style_radius(box, 10, 0); // Rounded corners
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

    // Initial position
    pos_x = (screen_width - box_size) / 2.0f;
    pos_y = (screen_height - box_size) / 2.0f;
    lv_obj_set_pos(box, (lv_coord_t)pos_x, (lv_coord_t)pos_y);

    // Create Physics Timer (50Hz = 20ms)
    lv_timer_create(timer_cb, 20, this);
    
    ESP_UTILS_CHECK_FALSE_RETURN(startRecordResource(), false, "Start record failed");
    return true;
}

bool GyroGame::back(void)
{
    return notifyCoreClosed();
}

bool GyroGame::close(void)
{
    return true;
}

ESP_UTILS_REGISTER_PLUGIN_WITH_CONSTRUCTOR(systems::base::App, GyroGame, APP_NAME, []() {
    return std::shared_ptr<GyroGame>(GyroGame::requestInstance(), [](GyroGame *p) {});
});

} // namespace esp_brookesia::apps
