/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cmath>
#include <stack>
#include <vector>
#include <cstdlib>
#include <ctime>
#include "app_gyro_maze.hpp"
#include "esp_brookesia.hpp"
#include "bsp/esp32_s3_touch_amoled_2_06.h"
#include "esp_log.h"
#include "esp_random.h"

// --- Logging ---
#ifdef ESP_UTILS_LOG_TAG
#   undef ESP_UTILS_LOG_TAG
#endif
#define ESP_UTILS_LOG_TAG "GyroMaze"
#include "esp_lib_utils.h"

#define GYRO_MAZE_APP_NAME "Gyro Maze"
#define GYRO_MAZE_LOG_TAG "GyroMaze"

// Physics constants
#define PHYSICS_FRICTION 0.90f
#define PHYSICS_ACCEL_FACTOR 3.5f
#define PHYSICS_BOUNCE 0.3f
#define PHYSICS_MAX_VEL 15.0f // Slower than the open game for better control in maze
#define INPUT_SMOOTHING 0.3f
#define CALIBRATION_DEADZONE 0.015f

using namespace std;
using namespace esp_brookesia::gui;
using namespace esp_brookesia::systems;

namespace esp_brookesia::apps {

GyroMaze *GyroMaze::_instance = nullptr;

GyroMaze *GyroMaze::requestInstance(bool use_status_bar, bool use_navigation_bar)
{
    if (_instance == nullptr) {
        _instance = new GyroMaze(use_status_bar, use_navigation_bar);
    }
    return _instance;
}

GyroMaze::GyroMaze(bool use_status_bar, bool use_navigation_bar):
    App(GYRO_MAZE_APP_NAME, &gyro_maze_icon, false, use_status_bar, use_navigation_bar),
    _container(nullptr), _ball(nullptr), _hole(nullptr), _wall_container(nullptr), _game_timer(nullptr),
    start_row(0), start_col(0), hole_row(0), hole_col(0),
    pos_x(0), pos_y(0), vel_x(0), vel_y(0),
    screen_width(0), screen_height(0), cell_width(0), cell_height(0), ball_radius(0),
    imu_initialized(false), calibration_done(false),
    accel_bias_x(0), accel_bias_y(0), 
    _smooth_ax(0), _smooth_ay(0),
    _qmi_dev(nullptr)
{
}

GyroMaze::~GyroMaze()
{
    if (_qmi_dev) {
        free(_qmi_dev);
    }
}

// --- Maze Generation (Recursive Backtracker) ---
void GyroMaze::generate_maze()
{
    // 1. Calculate Mask (valid/invalid cells) based on Rounded Corners
    // Config: 20% of min dimension. Use checking of cell corners to be "strict" (if touches, cut).
    const float CORNER_PERCENT = 0.20f; 
    float min_dim = std::min(screen_width, screen_height);
    float corner_radius = min_dim * CORNER_PERCENT;
    float corner_sq = corner_radius * corner_radius;
    
    ESP_LOGI(GYRO_MAZE_LOG_TAG, "Gen Maze: Screen %dx%d, Radius %.2f", screen_width, screen_height, corner_radius);

    for(int r=0; r<ROWS; r++) {
        for(int c=0; c<COLS; c++) {
            // Reset cell
            maze[r][c].wall_top = true;
            maze[r][c].wall_right = true;
            maze[r][c].wall_bottom = true;
            maze[r][c].wall_left = true;
            maze[r][c].visited = false;
            maze[r][c].valid = true;

            // Check Validity: If ANY part of the cell is in the invalid corner zone, mark invalid.
            // We check the cell corner closest to the screen corner.
            // If that corner point is in the "corner box" AND "outside the valid circle", it's invalid.
            
            bool invalid = false;
            
            // Top-Left: Check Cell Top-Left (c*w, r*h)
            // Invalid Box: x < R, y < R. Valid Circle Center: (R,R).
            float px = c * cell_width;
            float py = r * cell_height;
            if (px < corner_radius && py < corner_radius) {
                float dx = px - corner_radius;
                float dy = py - corner_radius;
                if (dx*dx + dy*dy > corner_sq) invalid = true;
            }

            // Top-Right: Check Cell Top-Right ((c+1)*w, r*h)
            // Invalid Box: x > W-R, y < R. Center: (W-R, R).
            if (!invalid) {
                px = (c + 1) * cell_width;
                py = r * cell_height;
                if (px > screen_width - corner_radius && py < corner_radius) {
                    float dx = px - (screen_width - corner_radius);
                    float dy = py - corner_radius;
                    if (dx*dx + dy*dy > corner_sq) invalid = true;
                }
            }

            // Bottom-Left: Check Cell Bottom-Left (c*w, (r+1)*h)
            // Invalid Box: x < R, y > H-R. Center: (R, H-R).
            if (!invalid) {
                px = c * cell_width;
                py = (r + 1) * cell_height;
                if (px < corner_radius && py > screen_height - corner_radius) {
                    float dx = px - corner_radius;
                    float dy = py - (screen_height - corner_radius);
                    if (dx*dx + dy*dy > corner_sq) invalid = true;
                }
            }

            // Bottom-Right: Check Cell Bottom-Right ((c+1)*w, (r+1)*h)
            // Invalid Box: x > W-R, y > H-R. Center: (W-R, H-R).
            if (!invalid) {
                px = (c + 1) * cell_width;
                py = (r + 1) * cell_height;
                if (px > screen_width - corner_radius && py > screen_height - corner_radius) {
                    float dx = px - (screen_width - corner_radius);
                    float dy = py - (screen_height - corner_radius);
                    if (dx*dx + dy*dy > corner_sq) invalid = true;
                }
            }

            if (invalid) {
                maze[r][c].valid = false;
                maze[r][c].visited = true; // Mark as visited so generator ignores it
            }
        }
    }

    // 2. Determine Start and Hole positions (First valid cells)
    start_row = -1; start_col = -1;
    // Scan for Start (Top-Left) - row by row
    for(int r=0; r<ROWS && start_row==-1; r++) {
        for(int c=0; c<COLS; c++) {
            if(maze[r][c].valid) {
                start_row = r;
                start_col = c;
                break;
            }
        }
    }

    hole_row = -1; hole_col = -1;
    // Scan for Hole (Bottom-Right) - row by row backwards
    for(int r=ROWS-1; r>=0 && hole_row==-1; r--) {
        for(int c=COLS-1; c>=0; c--) {
            if(maze[r][c].valid) {
                hole_row = r;
                hole_col = c;
                break;
            }
        }
    }
    
    if(start_row == -1 || hole_row == -1) {
        // Fallback: Use Center
        ESP_LOGE(GYRO_MAZE_LOG_TAG, "Failed to find valid positions, falling back to center.");
        start_row = ROWS/2; start_col = COLS/2;
        hole_row = ROWS/2; hole_col = COLS/2;
        maze[start_row][start_col].valid = true;
        maze[start_row][start_col].visited = false;
    }

    // 3. Recursive Backtracker
    std::stack<std::pair<int, int>> stack;
    
    // Mark start as visited for algorithm
    maze[start_row][start_col].visited = true;
    stack.push({start_row, start_col});

    while(!stack.empty()) {
        std::pair<int, int> current = stack.top();
        int r = current.first;
        int c = current.second;
        
        // Find unvisited neighbors that are VALID
        std::vector<int> neighbors; // 0: Top, 1: Right, 2: Bottom, 3: Left
        
        // Top
        if(r > 0 && maze[r-1][c].valid && !maze[r-1][c].visited) neighbors.push_back(0);
        // Right
        if(c < COLS-1 && maze[r][c+1].valid && !maze[r][c+1].visited) neighbors.push_back(1);
        // Bottom
        if(r < ROWS-1 && maze[r+1][c].valid && !maze[r+1][c].visited) neighbors.push_back(2);
        // Left
        if(c > 0 && maze[r][c-1].valid && !maze[r][c-1].visited) neighbors.push_back(3);

        if(!neighbors.empty()) {
            // Pick random neighbor
            int next_dir = neighbors[esp_random() % neighbors.size()];
            
            // Remove walls
            if(next_dir == 0) { // Top
                maze[r][c].wall_top = false;
                maze[r-1][c].wall_bottom = false;
                maze[r-1][c].visited = true;
                stack.push({r-1, c});
            } else if(next_dir == 1) { // Right
                maze[r][c].wall_right = false;
                maze[r][c+1].wall_left = false;
                maze[r][c+1].visited = true;
                stack.push({r, c+1});
            } else if(next_dir == 2) { // Bottom
                maze[r][c].wall_bottom = false;
                maze[r+1][c].wall_top = false;
                maze[r+1][c].visited = true;
                stack.push({r+1, c});
            } else if(next_dir == 3) { // Left
                maze[r][c].wall_left = false;
                maze[r][c-1].wall_right = false;
                maze[r][c-1].visited = true;
                stack.push({r, c-1});
            }
        } else {
            stack.pop();
        }
    }
}

// --- IMU Logic ---
void GyroMaze::perform_calibration() {
    if (!_qmi_dev) return;

    ESP_LOGI(GYRO_MAZE_LOG_TAG, "Starting calibration...");
    
    // UI Feedback could be added here
    lv_refr_now(NULL);

    const int samples = 100;
    float sum_x = 0;
    float sum_y = 0;
    qmi8658_data_t data;

    for (int i = 0; i < samples; i++) {
        if (qmi8658_read_sensor_data(_qmi_dev, &data) == ESP_OK) {
            sum_x += (data.accelX / 1000.0f);
            sum_y += (data.accelY / 1000.0f);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    accel_bias_x = sum_x / samples;
    accel_bias_y = sum_y / samples;
    _smooth_ax = 0;
    _smooth_ay = 0;
    calibration_done = true;

    ESP_LOGI(GYRO_MAZE_LOG_TAG, "Calibration done. Bias X: %.3f, Y: %.3f", accel_bias_x, accel_bias_y);
}

void GyroMaze::init_imu() {
    if (imu_initialized) return;

    i2c_master_bus_handle_t bus_handle = bsp_i2c_get_handle();
    if (!bus_handle) return;

    _qmi_dev = (qmi8658_dev_t*)malloc(sizeof(qmi8658_dev_t));
    if (!_qmi_dev) return;

    if (qmi8658_init(_qmi_dev, bus_handle, QMI8658_ADDRESS_HIGH) != ESP_OK) {
        free(_qmi_dev);
        _qmi_dev = nullptr;
        return;
    }

    qmi8658_set_accel_range(_qmi_dev, QMI8658_ACCEL_RANGE_2G); 
    qmi8658_set_accel_odr(_qmi_dev, QMI8658_ACCEL_ODR_500HZ);
    qmi8658_set_accel_unit_mps2(_qmi_dev, false); // 'g'
    qmi8658_write_register(_qmi_dev, QMI8658_CTRL5, 0x03); 

    imu_initialized = true;
    perform_calibration();
}

void GyroMaze::read_imu(float &acc_x, float &acc_y) {
    if (!imu_initialized) init_imu();
    if (!_qmi_dev) {
        acc_x = 0; acc_y = 0;
        return;
    }

    qmi8658_data_t data;
    if (qmi8658_read_sensor_data(_qmi_dev, &data) == ESP_OK) {
        float raw_x = data.accelX / 1000.0f;
        float raw_y = data.accelY / 1000.0f;

        if (calibration_done) {
            raw_x -= accel_bias_x;
            raw_y -= accel_bias_y;
        }

        _smooth_ax = _smooth_ax + INPUT_SMOOTHING * (raw_x - _smooth_ax);
        _smooth_ay = _smooth_ay + INPUT_SMOOTHING * (raw_y - _smooth_ay);

        if (fabsf(_smooth_ax) < CALIBRATION_DEADZONE) _smooth_ax = 0;
        if (fabsf(_smooth_ay) < CALIBRATION_DEADZONE) _smooth_ay = 0;

        acc_x = _smooth_ax;
        acc_y = _smooth_ay;
    }
}

// --- Physics & Game Logic ---

bool GyroMaze::check_collision(float new_x, float new_y) {
    // Check boundaries of cell
    // Ball occupies a square from (new_x, new_y) to (new_x+d, new_y+d) roughly
    float ball_d = ball_radius * 2;
    
    // Determine which cells the ball overlaps
    // Top-Left corner
    int c1 = (int)(new_x / cell_width);
    int r1 = (int)(new_y / cell_height);
    // Bottom-Right corner
    int c2 = (int)((new_x + ball_d) / cell_width);
    int r2 = (int)((new_y + ball_d) / cell_height);

    // Keep indices within bounds
    if(c1 < 0) { c1 = 0; }
    if(c1 >= COLS) { c1 = COLS-1; }
    if(r1 < 0) { r1 = 0; }
    if(r1 >= ROWS) { r1 = ROWS-1; }
    if(c2 < 0) { c2 = 0; }
    if(c2 >= COLS) { c2 = COLS-1; }
    if(r2 < 0) { r2 = 0; }
    if(r2 >= ROWS) { r2 = ROWS-1; }

    // Iterate over touched cells (usually 1, max 4)
    for(int r=r1; r<=r2; r++) {
        for(int c=c1; c<=c2; c++) {
            // Check walls of this cell
            float cell_x = c * cell_width;
            float cell_y = r * cell_height;
            
            // Wall thickness approximation for collision logic
            // Ideally, we treat walls as lines at the boundaries.
            
            // Check Top Wall
            if(maze[r][c].wall_top) {
                if(new_y < cell_y + 1) return true; // Hitting top wall
            }
            // Check Bottom Wall
            if(maze[r][c].wall_bottom) {
                 if(new_y + ball_d > cell_y + cell_height - 1) return true;
            }
            // Check Left Wall
            if(maze[r][c].wall_left) {
                 if(new_x < cell_x + 1) return true;
            }
            // Check Right Wall
            if(maze[r][c].wall_right) {
                 if(new_x + ball_d > cell_x + cell_width - 1) return true;
            }
        }
    }

    return false;
}

void GyroMaze::update_game(lv_timer_t *timer) {
    GyroMaze *app = (GyroMaze *)timer->user_data;
    
    float ax, ay;
    app->read_imu(ax, ay);

    // Coordinate mapping (same as GyroGame)
    // Device held naturally: X axis is vertical (force_y), Y axis is horizontal (force_x)
    float force_x = -ay;
    float force_y = ax;

    app->vel_x += force_x * PHYSICS_ACCEL_FACTOR;
    app->vel_y += force_y * PHYSICS_ACCEL_FACTOR;
    
    app->vel_x *= PHYSICS_FRICTION;
    app->vel_y *= PHYSICS_FRICTION;

    // Clamp Velocity
    if (app->vel_x > PHYSICS_MAX_VEL) app->vel_x = PHYSICS_MAX_VEL;
    if (app->vel_x < -PHYSICS_MAX_VEL) app->vel_x = -PHYSICS_MAX_VEL;
    if (app->vel_y > PHYSICS_MAX_VEL) app->vel_y = PHYSICS_MAX_VEL;
    if (app->vel_y < -PHYSICS_MAX_VEL) app->vel_y = -PHYSICS_MAX_VEL;

    // Proposed new position
    float next_x = app->pos_x + app->vel_x;
    float next_y = app->pos_y + app->vel_y;

    // Screen Boundaries (Hard limit)
    float ball_size = app->ball_radius * 2;
    if (next_x < 0) { next_x = 0; app->vel_x *= -PHYSICS_BOUNCE; }
    if (next_y < 0) { next_y = 0; app->vel_y *= -PHYSICS_BOUNCE; }
    if (next_x > app->screen_width - ball_size) { next_x = app->screen_width - ball_size; app->vel_x *= -PHYSICS_BOUNCE; }
    if (next_y > app->screen_height - ball_size) { next_y = app->screen_height - ball_size; app->vel_y *= -PHYSICS_BOUNCE; }

    // Maze Wall Collision
    // Simple logic: if new position collides, stop movement on that axis
    // Ideally check X and Y separately
    if(app->check_collision(next_x, app->pos_y)) {
         app->vel_x *= -PHYSICS_BOUNCE;
         next_x = app->pos_x; // Revert
    }
    if(app->check_collision(next_x, next_y)) {
         app->vel_y *= -PHYSICS_BOUNCE;
         next_y = app->pos_y; // Revert
    }

    app->pos_x = next_x;
    app->pos_y = next_y;

    // Update UI
    lv_obj_set_pos(app->_ball, (lv_coord_t)app->pos_x, (lv_coord_t)app->pos_y);

    // Win Condition
    int ball_r = (int)((app->pos_y + app->ball_radius) / app->cell_height);
    int ball_c = (int)((app->pos_x + app->ball_radius) / app->cell_width);

    if (ball_r == app->hole_row && ball_c == app->hole_col) {
        // WIN!
        // Reset or notify
        ESP_LOGI(GYRO_MAZE_LOG_TAG, "Level Cleared!");
        // Regenerate maze
        app->generate_maze();
        app->draw_maze();
        // Reset Position (Center of new start cell)
        app->pos_x = app->start_col * app->cell_width + (app->cell_width - app->ball_radius*2)/2;
        app->pos_y = app->start_row * app->cell_height + (app->cell_height - app->ball_radius*2)/2;
        app->vel_x = 0;
        app->vel_y = 0;
    }
}

void GyroMaze::timer_cb(lv_timer_t *timer) {
    GyroMaze *app = (GyroMaze *)timer->user_data;
    app->update_game(timer);
}

void GyroMaze::event_handler(lv_event_t *e) {
    GyroMaze *app = (GyroMaze *)lv_event_get_user_data(e);
    if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
         app->perform_calibration();
    }
}

// --- UI / Lifecycle ---

void GyroMaze::draw_maze() {
    // Clear previous walls
    lv_obj_clean(_wall_container);

    // Wall Thickness
    const int t = 2; // thickness

    for(int r=0; r<ROWS; r++) {
        for(int c=0; c<COLS; c++) {
            int cx = (int)(c * cell_width);
            int cy = (int)(r * cell_height);
            int cw = (int)cell_width;
            int ch = (int)cell_height;

            // If cell is invalid (corner cut), draw a solid block
            if(!maze[r][c].valid) {
                // Calculate exact dimensions to avoid gaps
                int next_cx = (int)((c + 1) * cell_width);
                int next_cy = (int)((r + 1) * cell_height);
                int w_fill = next_cx - cx;
                int h_fill = next_cy - cy;

                lv_obj_t *b = lv_obj_create(_wall_container);
                lv_obj_set_size(b, w_fill, h_fill);
                lv_obj_set_pos(b, cx, cy);
                lv_obj_set_style_bg_color(b, lv_color_hex(0x8B4513), 0);
                lv_obj_set_style_radius(b, 0, 0); // sharp corners
                lv_obj_set_style_border_width(b, 0, 0);
                continue; // Skip drawing walls (and normal pathing) for this cell
            }

            // Common style for walls
            // Since creating many objects is heavy, consider drawing lines or a canvas.
            // For now, use simple objects for robustness.
            
            if(maze[r][c].wall_top) {
                lv_obj_t *w = lv_obj_create(_wall_container);
                lv_obj_set_size(w, cw + t, t);
                lv_obj_set_pos(w, cx, cy);
                lv_obj_set_style_bg_color(w, lv_color_hex(0x8B4513), 0); // Brown
                lv_obj_set_style_border_width(w, 0, 0);
            }
            if(maze[r][c].wall_left) {
                lv_obj_t *w = lv_obj_create(_wall_container);
                lv_obj_set_size(w, t, ch + t);
                lv_obj_set_pos(w, cx, cy);
                lv_obj_set_style_bg_color(w, lv_color_hex(0x8B4513), 0);
                lv_obj_set_style_border_width(w, 0, 0);
            }
            
            // Only need Top and Left generally, plus Bottom/Right for boundary cells
            if(r == ROWS-1 && maze[r][c].wall_bottom) {
                lv_obj_t *w = lv_obj_create(_wall_container);
                lv_obj_set_size(w, cw + t, t);
                lv_obj_set_pos(w, cx, cy + ch);
                lv_obj_set_style_bg_color(w, lv_color_hex(0x8B4513), 0);
                lv_obj_set_style_border_width(w, 0, 0);
            }
            if(c == COLS-1 && maze[r][c].wall_right) {
                lv_obj_t *w = lv_obj_create(_wall_container);
                lv_obj_set_size(w, t, ch + t);
                lv_obj_set_pos(w, cx + cw, cy);
                lv_obj_set_style_bg_color(w, lv_color_hex(0x8B4513), 0);
                lv_obj_set_style_border_width(w, 0, 0);
            }
        }
    }
    
    // Position Hole
    int hx = (int)(hole_col * cell_width + (cell_width - ball_radius*2)/2);
    int hy = (int)(hole_row * cell_height + (cell_height - ball_radius*2)/2);
    lv_obj_set_pos(_hole, hx, hy);
}

// --- App Lifecycle ---

bool GyroMaze::run(void)
{
    ESP_UTILS_CHECK_FALSE_RETURN(startRecordResource(), false, "Start record failed");

    // Initialize screen dimensions early
    lv_obj_t *screen = lv_obj_create(NULL);
    lv_scr_load(screen);
    screen_width = lv_obj_get_width(screen);
    screen_height = lv_obj_get_height(screen);
    
    // Default Style for common UI
    _container = nullptr;
    _game_timer = nullptr;

    // Show Menu
    show_main_menu();

    ESP_UTILS_CHECK_FALSE_RETURN(endRecordResource(), false, "End record failed");
    return true;
}

void GyroMaze::clean_up_current_screen()
{
    // If there is an active container, clean it
    if (_container) {
        lv_obj_clean(_container);
        lv_obj_del(_container);
        _container = nullptr;
    }
    // Stop physics timer if running
    if (_game_timer) {
        lv_timer_del(_game_timer);
        _game_timer = nullptr;
    }
}

void GyroMaze::show_main_menu()
{
    _current_mode = MODE_MENU;
    clean_up_current_screen();

    lv_obj_t *screen = lv_scr_act();

    // Create a simple menu container
    _container = lv_obj_create(screen);
    lv_obj_set_size(_container, screen_width, screen_height);
    lv_obj_set_style_bg_color(_container, lv_color_black(), 0);
    lv_obj_set_style_border_width(_container, 0, 0);
    lv_obj_set_style_radius(_container, 0, 0);
    lv_obj_center(_container);

    // Title
    lv_obj_t *label_title = lv_label_create(_container);
    lv_label_set_text(label_title, "GYRO MAZE");
    lv_obj_set_style_text_font(label_title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(label_title, lv_color_white(), 0);
    lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 40);

    // Button: Classic
    lv_obj_t *btn_classic = lv_btn_create(_container);
    lv_obj_set_size(btn_classic, 180, 50);
    lv_obj_align(btn_classic, LV_ALIGN_CENTER, 0, -30);
    lv_obj_set_style_bg_color(btn_classic, lv_color_hex(0x444444), 0);
    
    lv_obj_t *lbl_classic = lv_label_create(btn_classic);
    lv_label_set_text(lbl_classic, "Juego Clásico");
    lv_obj_center(lbl_classic);

    // Button: Adventure
    lv_obj_t *btn_adv = lv_btn_create(_container);
    lv_obj_set_size(btn_adv, 180, 50);
    lv_obj_align(btn_adv, LV_ALIGN_CENTER, 0, 40);
    lv_obj_set_style_bg_color(btn_adv, lv_color_hex(0x222222), 0); // Darker to show disabled/beta
    
    lv_obj_t *lbl_adv = lv_label_create(btn_adv);
    lv_label_set_text(lbl_adv, "Modo Procedural");
    lv_obj_set_style_text_color(lbl_adv, lv_color_hex(0x888888), 0);
    lv_obj_center(lbl_adv);

    // Events
    lv_obj_add_event_cb(btn_classic, [](lv_event_t *e){
        GyroMaze *app = (GyroMaze *)lv_event_get_user_data(e);
        app->start_classic_game();
    }, LV_EVENT_CLICKED, this);

    lv_obj_add_event_cb(btn_adv, [](lv_event_t *e){
        // Placeholder
        lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e);
        // Visual feedback (flash red)
        lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), 0);
    }, LV_EVENT_CLICKED, this);
}

void GyroMaze::start_classic_game()
{
    _current_mode = MODE_CLASSIC;
    clean_up_current_screen();

    lv_obj_t *screen = lv_scr_act();

    // 2. Main Container (White Background for Classic)
    _container = lv_obj_create(screen);
    lv_obj_set_size(_container, screen_width, screen_height);
    lv_obj_set_style_bg_color(_container, lv_color_white(), 0);
    lv_obj_set_style_border_width(_container, 0, 0);
    lv_obj_set_style_pad_all(_container, 0, 0); // Remove default padding
    lv_obj_set_style_radius(_container, 0, 0);
    lv_obj_clear_flag(_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_center(_container);
    
    // Gesture pass through
    lv_obj_clear_flag(_container, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(_container, LV_OBJ_FLAG_GESTURE_BUBBLE);
    
    // Calculate dimensions based on screen
    cell_width = (float)screen_width / COLS;
    cell_height = (float)screen_height / ROWS;
    ball_radius = (std::min(cell_width, cell_height) / 2.0f) * 0.7f; // 70% of half-cell

    // 3. Wall Container
    _wall_container = lv_obj_create(_container);
    lv_obj_set_size(_wall_container, screen_width, screen_height);
    lv_obj_set_style_bg_opa(_wall_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(_wall_container, 0, 0);
    lv_obj_set_style_pad_all(_wall_container, 0, 0); // Remove default padding
    lv_obj_clear_flag(_wall_container, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    // 4. Hole (Black)
    _hole = lv_obj_create(_container);
    lv_obj_set_size(_hole, (lv_coord_t)(ball_radius * 2), (lv_coord_t)(ball_radius * 2));
    lv_obj_set_style_bg_color(_hole, lv_color_black(), 0);
    lv_obj_set_style_radius(_hole, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(_hole, 0, 0);

    // 5. Ball (Red)
    _ball = lv_obj_create(_container);
    lv_obj_set_size(_ball, (lv_coord_t)(ball_radius * 2), (lv_coord_t)(ball_radius * 2));
    lv_obj_set_style_bg_color(_ball, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_radius(_ball, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(_ball, 0, 0);
    lv_obj_clear_flag(_ball, LV_OBJ_FLAG_SCROLLABLE);

    // 6. Generate & Draw Maze
    generate_maze();
    draw_maze();

    // 7. Initial Position (Center of start cell)
    pos_x = start_col * cell_width + (cell_width - ball_radius*2)/2;
    pos_y = start_row * cell_height + (cell_height - ball_radius*2)/2;
    lv_obj_set_pos(_ball, (lv_coord_t)pos_x, (lv_coord_t)pos_y);

    // 8. Calibration clickable area (invisible button at bottom)
    lv_obj_t *btn = lv_btn_create(_container);
    lv_obj_set_size(btn, 100, 40);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_shadow_width(btn, 0, 0);
    lv_obj_add_event_cb(btn, event_handler, LV_EVENT_CLICKED, this);

    // 9. Start Logic
    _game_timer = lv_timer_create(timer_cb, 20, this); // 50Hz
}

bool GyroMaze::back(void)
{
    if (_current_mode == MODE_CLASSIC || _current_mode == MODE_ADVENTURE) {
        show_main_menu();
        return true; // Use back to go up one level
    }
    return notifyCoreClosed();
}

bool GyroMaze::close(void)
{
    return true;
}

bool GyroMaze::pause(void)
{
    if (_game_timer) lv_timer_pause(_game_timer);
    return true;
}

bool GyroMaze::resume(void)
{
    if (_game_timer) lv_timer_resume(_game_timer);
    return true;
}

ESP_UTILS_REGISTER_PLUGIN_WITH_CONSTRUCTOR(systems::base::App, GyroMaze, GYRO_MAZE_APP_NAME, []() {
    return std::shared_ptr<GyroMaze>(GyroMaze::requestInstance(), [](GyroMaze *p) {});
});

} // namespace esp_brookesia::apps
