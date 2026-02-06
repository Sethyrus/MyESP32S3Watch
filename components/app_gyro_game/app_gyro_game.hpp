/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "systems/phone/esp_brookesia_phone_app.hpp"
#include "lvgl.h"

// Launcher icon declaration
LV_IMG_DECLARE(gyro_game_icon);
#include "qmi8658.h"
#include "driver/i2c_master.h"

namespace esp_brookesia::apps {

/**
 * @brief Gyro Game App: A blue square controlled by the accelerometer (QMA7981).
 */
class GyroGame: public systems::phone::App {
public:
    /**
     * @brief Get the singleton instance of GyroGame
     */
    static GyroGame *requestInstance(bool use_status_bar = false, bool use_navigation_bar = false);

    /**
     * @brief Destructor
     */
    virtual ~GyroGame();

protected:
    /**
     * @brief Private constructor to enforce singleton pattern
     */
    GyroGame(bool use_status_bar, bool use_navigation_bar);

    /**
     * @brief App Entry Point
     */
    bool run(void) override;

    /**
     * @brief Handle Back Event
     */
    bool back(void) override;

    /**
     * @brief Called when app closes
     */
    bool close(void) override;

    /**
     * @brief Called when app is minimized/backgrounded
     */
    bool pause(void) override;

    /**
     * @brief Called when app is restored from background
     */
    bool resume(void) override;

private:
    static GyroGame *_instance;

    // UI Elements
    lv_obj_t *_container;
    lv_obj_t *_box;
    lv_timer_t *_physics_timer;

    // Physics State
    float pos_x;
    float pos_y;
    float vel_x;
    float vel_y;
    int screen_width;
    int screen_height;
    int box_size;

    // IMU State
    bool imu_initialized;
    bool calibration_done;
    float accel_bias_x;
    float accel_bias_y;
    float _smooth_ax;
    float _smooth_ay;
    qmi8658_dev_t *_qmi_dev;

    // Internal methods
    void init_imu();
    void perform_calibration();
    void read_imu(float &acc_x, float &acc_y);
    void update_physics(lv_timer_t *timer);

    static void timer_cb(lv_timer_t *timer);
    static void event_handler(lv_event_t *e);
};

} // namespace esp_brookesia::apps
