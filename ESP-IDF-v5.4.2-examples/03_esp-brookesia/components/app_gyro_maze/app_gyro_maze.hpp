/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "systems/phone/esp_brookesia_phone_app.hpp"
#include "lvgl.h"
// Launcher icon declaration


// Fix M_PI redefinition warning between math.h and qmi8658.h
#ifdef M_PI
#undef M_PI
#endif
#include "qmi8658.h"
#include "driver/i2c_master.h"
#include <vector>

// Launcher icon declaration
LV_IMG_DECLARE(gyro_maze_icon);

namespace esp_brookesia::apps {

/**
 * @brief Gyro Maze App
 */
class GyroMaze: public systems::phone::App {
public:
    static GyroMaze *requestInstance(bool use_status_bar = false, bool use_navigation_bar = false);
    virtual ~GyroMaze();

protected:
    GyroMaze(bool use_status_bar, bool use_navigation_bar);
    bool run(void) override;
    bool back(void) override;
    bool close(void) override;
    bool pause(void) override;
    bool resume(void) override;

private:
    static GyroMaze *_instance;

    // UI Elements
    lv_obj_t *_container;
    lv_obj_t *_ball;
    lv_obj_t *_hole;
    lv_obj_t *_wall_container; // Container for wall blocks
    lv_timer_t *_game_timer;

    // Maze Configuration
    static const int ROWS = 12;
    static const int COLS = 12;
    struct Cell {
        bool wall_top = true;
        bool wall_right = true;
        bool wall_bottom = true;
        bool wall_left = true;
        bool visited = false;
        bool valid = true; // New: is cell part of the playable area?
    };
    Cell maze[ROWS][COLS];
    
    // Maze State
    int start_row;
    int start_col;
    int hole_row;
    int hole_col;

    // Physics State
    float pos_x;
    float pos_y;
    float vel_x;
    float vel_y;
    int screen_width;
    int screen_height;
    float cell_width;
    float cell_height;
    float ball_radius;

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
    void update_game(lv_timer_t *timer);
    
    // Maze Generation
    void generate_maze();
    void draw_maze();
    
    // Physics Helper
    bool check_collision(float new_x, float new_y);

    static void timer_cb(lv_timer_t *timer);
    static void event_handler(lv_event_t *e);
};

} // namespace esp_brookesia::apps
