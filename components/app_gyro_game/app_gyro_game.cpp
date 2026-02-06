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

// --- Logging ---
#ifdef ESP_UTILS_LOG_TAG
#   undef ESP_UTILS_LOG_TAG
#endif
#define ESP_UTILS_LOG_TAG "GyroGame"
#include "esp_lib_utils.h"

#define GYRO_GAME_APP_NAME "Gyro Game"
#define GYRO_GAME_LOG_TAG "GyroGame"

// Physics constants
#define GYRO_GAME_PHYSICS_FRICTION 0.90f
#define GYRO_GAME_PHYSICS_ACCEL_FACTOR 3.5f
#define GYRO_GAME_PHYSICS_BOUNCE 0.5f
#define GYRO_GAME_PHYSICS_MAX_VEL 30.0f
#define GYRO_GAME_INPUT_SMOOTHING 0.3f
#define GYRO_GAME_CALIBRATION_DEADZONE 0.015f

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
    App(GYRO_GAME_APP_NAME, &gyro_game_icon, false, use_status_bar, use_navigation_bar),
    _container(nullptr), _box(nullptr), _physics_timer(nullptr),
    pos_x(0), pos_y(0), vel_x(0), vel_y(0),
    screen_width(0), screen_height(0), box_size(50),
    imu_initialized(false), calibration_done(false),
    accel_bias_x(0), accel_bias_y(0), 
    _smooth_ax(0), _smooth_ay(0),
    _qmi_dev(nullptr)
{
}

GyroGame::~GyroGame()
{
    if (_qmi_dev) {
        free(_qmi_dev);
    }
}

void GyroGame::perform_calibration() {
    if (!_qmi_dev) return;

    ESP_LOGI(GYRO_GAME_LOG_TAG, "Starting calibration...");
    
    // Simple UI feedback, force render
    lv_refr_now(NULL);

    const int samples = 200;
    float sum_x = 0;
    float sum_y = 0;
    qmi8658_data_t data;

    for (int i = 0; i < samples; i++) {
        if (qmi8658_read_sensor_data(_qmi_dev, &data) == ESP_OK) {
            // Normalize to g
            sum_x += (data.accelX / 1000.0f);
            sum_y += (data.accelY / 1000.0f);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    accel_bias_x = sum_x / samples;
    accel_bias_y = sum_y / samples;
    
    // Reset smoothing
    _smooth_ax = 0;
    _smooth_ay = 0;
    
    calibration_done = true;

    ESP_LOGI(GYRO_GAME_LOG_TAG, "Calibration done. Bias X: %.3f, Y: %.3f", accel_bias_x, accel_bias_y);
}

void GyroGame::init_imu() {
    if (imu_initialized) return;

    ESP_LOGI(GYRO_GAME_LOG_TAG, "Initializing QMI8658 Component...");
    esp_log_level_set(GYRO_GAME_LOG_TAG, ESP_LOG_INFO);
    
    i2c_master_bus_handle_t bus_handle = bsp_i2c_get_handle();
    if (!bus_handle) {
         ESP_LOGE(GYRO_GAME_LOG_TAG, "Failed to get I2C bus handle");
         return;
    }

    _qmi_dev = (qmi8658_dev_t*)malloc(sizeof(qmi8658_dev_t));
    if (!_qmi_dev) return;

    // Use High Address (0x6B) as in reference example
    if (qmi8658_init(_qmi_dev, bus_handle, QMI8658_ADDRESS_HIGH) != ESP_OK) {
        ESP_LOGE(GYRO_GAME_LOG_TAG, "QMI8658 Init Failed!");
        free(_qmi_dev);
        _qmi_dev = nullptr;
        return;
    }

    // Switch to 2G range for better tilt resolution
    qmi8658_set_accel_range(_qmi_dev, QMI8658_ACCEL_RANGE_2G); 
    qmi8658_set_accel_odr(_qmi_dev, QMI8658_ACCEL_ODR_500HZ);
    qmi8658_set_accel_unit_mps2(_qmi_dev, false); // Return in 'g'

    qmi8658_write_register(_qmi_dev, QMI8658_CTRL5, 0x03); 

    imu_initialized = true;
    
    // Auto calibrate on start
    perform_calibration();
}

void GyroGame::read_imu(float &acc_x, float &acc_y) {
    if (!imu_initialized) init_imu();
    if (!_qmi_dev) {
        acc_x = 0; acc_y = 0;
        return;
    }

    qmi8658_data_t data;
    if (qmi8658_read_sensor_data(_qmi_dev, &data) == ESP_OK) {
        
        // Normalize milli-g to g
        float raw_x = data.accelX / 1000.0f;
        float raw_y = data.accelY / 1000.0f;

        // Subtract bias
        if (calibration_done) {
            raw_x -= accel_bias_x;
            raw_y -= accel_bias_y;
        }

        // Apply Low Pass Filter (Smoothing)
        _smooth_ax = _smooth_ax + GYRO_GAME_INPUT_SMOOTHING * (raw_x - _smooth_ax);
        _smooth_ay = _smooth_ay + GYRO_GAME_INPUT_SMOOTHING * (raw_y - _smooth_ay);

        // Deadzone on SMOOTHED data
        if (fabsf(_smooth_ax) < GYRO_GAME_CALIBRATION_DEADZONE) _smooth_ax = 0;
        if (fabsf(_smooth_ay) < GYRO_GAME_CALIBRATION_DEADZONE) _smooth_ay = 0;

        acc_x = _smooth_ax;
        acc_y = _smooth_ay;
        
    } else {
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

    app->vel_x += force_x * GYRO_GAME_PHYSICS_ACCEL_FACTOR;
    app->vel_y += force_y * GYRO_GAME_PHYSICS_ACCEL_FACTOR;
    
    // Apply friction
    app->vel_x *= GYRO_GAME_PHYSICS_FRICTION;
    app->vel_y *= GYRO_GAME_PHYSICS_FRICTION;

    // Terminal velocity clamp
    if (app->vel_x > GYRO_GAME_PHYSICS_MAX_VEL) app->vel_x = GYRO_GAME_PHYSICS_MAX_VEL;
    if (app->vel_x < -GYRO_GAME_PHYSICS_MAX_VEL) app->vel_x = -GYRO_GAME_PHYSICS_MAX_VEL;
    if (app->vel_y > GYRO_GAME_PHYSICS_MAX_VEL) app->vel_y = GYRO_GAME_PHYSICS_MAX_VEL;
    if (app->vel_y < -GYRO_GAME_PHYSICS_MAX_VEL) app->vel_y = -GYRO_GAME_PHYSICS_MAX_VEL;

    // Update position
    app->pos_x += app->vel_x;
    app->pos_y += app->vel_y;

    // Wall collisions
    if (app->pos_x < 0) {
        app->pos_x = 0;
        app->vel_x = -app->vel_x * GYRO_GAME_PHYSICS_BOUNCE;
    }
    if (app->pos_x > app->screen_width - app->box_size) {
        app->pos_x = app->screen_width - app->box_size;
        app->vel_x = -app->vel_x * GYRO_GAME_PHYSICS_BOUNCE;
    }
    
    if (app->pos_y < 0) {
        app->pos_y = 0;
        app->vel_y = -app->vel_y * GYRO_GAME_PHYSICS_BOUNCE;
    }
    if (app->pos_y > app->screen_height - app->box_size) {
        app->pos_y = app->screen_height - app->box_size;
        app->vel_y = -app->vel_y * GYRO_GAME_PHYSICS_BOUNCE;
    }

    // Update UI
    lv_obj_set_pos(app->_box, (lv_coord_t)app->pos_x, (lv_coord_t)app->pos_y);

    // Debug logging (every 50 frames = ~1 sec)
    log_counter++;
    if (log_counter >= 50) {
        log_counter = 0;
        ESP_LOGI(GYRO_GAME_LOG_TAG, "In(%.2f, %.2f) -> Vel(%.2f, %.2f) -> Pos(%d, %d)", 
                 ax, ay, app->vel_x, app->vel_y, (int)app->pos_x, (int)app->pos_y);
    }
}

void GyroGame::timer_cb(lv_timer_t *timer) {
    GyroGame *app = (GyroGame *)timer->user_data;
    app->update_physics(timer);
}

bool GyroGame::run(void)
{
    // Start recording resources for recents screen snapshots
    ESP_UTILS_CHECK_FALSE_RETURN(startRecordResource(), false, "Start record failed");

    // Create a new screen for the app
    lv_obj_t *new_screen = lv_obj_create(NULL);
    lv_scr_load(new_screen);

    // Get screen dimensions from the new screen
    screen_width = lv_obj_get_width(new_screen);
    screen_height = lv_obj_get_height(new_screen);

    // Create container on the NEW screen
    _container = lv_obj_create(new_screen);
    lv_obj_set_size(_container, screen_width, screen_height);
    lv_obj_set_style_bg_color(_container, lv_color_black(), 0);
    lv_obj_set_style_border_width(_container, 0, 0);
    lv_obj_set_style_radius(_container, 0, 0);
    lv_obj_clear_flag(_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(_container, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(_container, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_center(_container);

    // Create a Calibration Button
    lv_obj_t *btn_calib = lv_btn_create(_container);
    lv_obj_t *lb = lv_label_create(btn_calib);
    lv_label_set_text(lb, "Calibrar");
    lv_obj_align(btn_calib, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn_calib, event_handler, LV_EVENT_CLICKED, this);

    // Create Blue Box
    _box = lv_obj_create(_container);
    lv_obj_set_size(_box, box_size, box_size);
    lv_obj_set_style_bg_color(_box, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_set_style_radius(_box, 10, 0);
    lv_obj_clear_flag(_box, LV_OBJ_FLAG_SCROLLABLE);

    // Initial position
    pos_x = (screen_width - box_size) / 2.0f;
    pos_y = (screen_height - box_size) / 2.0f;
    lv_obj_set_pos(_box, (lv_coord_t)pos_x, (lv_coord_t)pos_y);

    // Create Physics Timer (50Hz = 20ms) and store handle
    _physics_timer = lv_timer_create(timer_cb, 20, this);
    
    // End recording
    ESP_UTILS_CHECK_FALSE_RETURN(endRecordResource(), false, "End record failed");

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

bool GyroGame::pause(void)
{
    ESP_LOGI(GYRO_GAME_LOG_TAG, "App paused, stopping physics timer");
    if (_physics_timer) {
        lv_timer_pause(_physics_timer);
    }
    return true;
}

bool GyroGame::resume(void)
{
    ESP_LOGI(GYRO_GAME_LOG_TAG, "App resumed, restarting physics timer");
    if (_physics_timer) {
        lv_timer_resume(_physics_timer);
    }
    return true;
}

ESP_UTILS_REGISTER_PLUGIN_WITH_CONSTRUCTOR(systems::base::App, GyroGame, GYRO_GAME_APP_NAME, []() {
    return std::shared_ptr<GyroGame>(GyroGame::requestInstance(), [](GyroGame *p) {});
});

} // namespace esp_brookesia::apps
