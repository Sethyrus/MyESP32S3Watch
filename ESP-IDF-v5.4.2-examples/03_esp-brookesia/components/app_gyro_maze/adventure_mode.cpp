/*
 * Adventure Mode Implementation
 */
#include "adventure_mode.hpp"
#include "app_gyro_maze.hpp"
#include "esp_log.h"
#include "esp_random.h"
#include <cmath>
#include <algorithm>
#include <stack>

static const char *TAG = "AdventureMode";

namespace esp_brookesia::apps::gyro_maze {

AdventureMode::AdventureMode(esp_brookesia::apps::GyroMaze *parent)
    : _parent(parent),
      _exit_row(0), _exit_col(0),
      _start_row(MAZE_ROWS / 2), _start_col(MAZE_COLS / 2),
      _camera_x(0), _camera_y(0),
      _player_x(0), _player_y(0),
      _vel_x(0), _vel_y(0),
      _visible_rows(12), _visible_cols(12),
      _cell_width(0), _cell_height(0),
      _ball_radius(0),
      _screen_width(0), _screen_height(0),
      _container(nullptr), _ball(nullptr), _hole(nullptr), _wall_container(nullptr),
      _wall_pool_index(0)
{
}

AdventureMode::~AdventureMode() {
    cleanup();
}

void AdventureMode::init(lv_obj_t *container, int screen_w, int screen_h, int visible_rows, int visible_cols) {
    _container = container;
    _screen_width = screen_w;
    _screen_height = screen_h;
    _visible_rows = visible_rows;
    _visible_cols = visible_cols;

    // Calculate cell dimensions based on visible area
    _cell_width = (float)screen_w / _visible_cols;
    _cell_height = (float)screen_h / _visible_rows;
    _ball_radius = (std::min(_cell_width, _cell_height) / 2.0f) * 0.7f;

    ESP_LOGI(TAG, "Init: Screen %dx%d, Visible %dx%d, Cell %.1fx%.1f",
             screen_w, screen_h, visible_rows, visible_cols, _cell_width, _cell_height);

    // Wall Container (transparent, holds wall objects)
    _wall_container = lv_obj_create(_container);
    lv_obj_set_size(_wall_container, screen_w, screen_h);
    lv_obj_set_style_bg_opa(_wall_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(_wall_container, 0, 0);
    lv_obj_set_style_pad_all(_wall_container, 0, 0);
    lv_obj_clear_flag(_wall_container, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    // Pre-allocate wall object pool
    _wall_pool.reserve(MAX_WALL_OBJECTS);
    for (int i = 0; i < MAX_WALL_OBJECTS; ++i) {
        lv_obj_t *wall = lv_obj_create(_wall_container);
        lv_obj_set_style_bg_color(wall, lv_color_hex(0x8B4513), 0); // Brown
        lv_obj_set_style_border_width(wall, 0, 0);
        lv_obj_add_flag(wall, LV_OBJ_FLAG_HIDDEN);
        _wall_pool.push_back(wall);
    }

    // Hole (Black circle, will be positioned later)
    _hole = lv_obj_create(_container);
    lv_obj_set_size(_hole, (lv_coord_t)(_ball_radius * 2), (lv_coord_t)(_ball_radius * 2));
    lv_obj_set_style_bg_color(_hole, lv_color_black(), 0);
    lv_obj_set_style_radius(_hole, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(_hole, 0, 0);

    // Ball (Red circle, fixed at screen center)
    _ball = lv_obj_create(_container);
    lv_obj_set_size(_ball, (lv_coord_t)(_ball_radius * 2), (lv_coord_t)(_ball_radius * 2));
    lv_obj_set_style_bg_color(_ball, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_radius(_ball, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(_ball, 0, 0);
    lv_obj_set_pos(_ball,
        (lv_coord_t)((_screen_width - _ball_radius * 2) / 2),
        (lv_coord_t)((_screen_height - _ball_radius * 2) / 2));

    // Generate Maze
    generate_maze();

    // Reset Player to Center
    reset_player();

    // Initial Draw
    draw_visible_walls();
}

void AdventureMode::cleanup() {
    // Clear wall pool
    for (auto *wall : _wall_pool) {
        if (wall) lv_obj_del(wall);
    }
    _wall_pool.clear();
    _wall_pool_index = 0;

    // Clear other objects
    if (_ball) { lv_obj_del(_ball); _ball = nullptr; }
    if (_hole) { lv_obj_del(_hole); _hole = nullptr; }
    if (_wall_container) { lv_obj_del(_wall_container); _wall_container = nullptr; }
}

void AdventureMode::reset_player() {
    // Player starts at center of the maze (in maze coordinates)
    _start_row = MAZE_ROWS / 2;
    _start_col = MAZE_COLS / 2;
    
    // Find a valid starting cell near center
    while (!_maze[_start_row][_start_col].valid && _start_row > 0) {
        _start_row--;
    }

    _player_x = _start_col * _cell_width + _cell_width / 2;
    _player_y = _start_row * _cell_height + _cell_height / 2;
    _vel_x = 0;
    _vel_y = 0;

    update_camera();
    ESP_LOGI(TAG, "Player Reset: Pos (%.1f, %.1f), Camera (%.1f, %.1f)", _player_x, _player_y, _camera_x, _camera_y);
}

void AdventureMode::generate_maze() {
    ESP_LOGI(TAG, "Generating %dx%d maze...", MAZE_ROWS, MAZE_COLS);

    // 1. Initialize all cells
    for (int r = 0; r < MAZE_ROWS; ++r) {
        for (int c = 0; c < MAZE_COLS; ++c) {
            _maze[r][c].wall_top = true;
            _maze[r][c].wall_right = true;
            _maze[r][c].wall_bottom = true;
            _maze[r][c].wall_left = true;
            _maze[r][c].visited = false;
            _maze[r][c].valid = true;
        }
    }

    // 2. Apply Corner Masking (based on VISIBLE area, applied to all 4 corners of the TOTAL maze)
    apply_corner_mask();

    // 3. Determine Exit Position (one of the 4 corners, randomly)
    int corner = esp_random() % 4;
    switch (corner) {
        case 0: _exit_row = 1; _exit_col = 1; break; // Top-Left
        case 1: _exit_row = 1; _exit_col = MAZE_COLS - 2; break; // Top-Right
        case 2: _exit_row = MAZE_ROWS - 2; _exit_col = 1; break; // Bottom-Left
        case 3: _exit_row = MAZE_ROWS - 2; _exit_col = MAZE_COLS - 2; break; // Bottom-Right
    }
    // Ensure exit is valid
    _maze[_exit_row][_exit_col].valid = true;
    _maze[_exit_row][_exit_col].visited = false;
    ESP_LOGI(TAG, "Exit at (%d, %d)", _exit_row, _exit_col);

    // 4. Recursive Backtracker from center
    int start_r = MAZE_ROWS / 2;
    int start_c = MAZE_COLS / 2;
    while (!_maze[start_r][start_c].valid && start_r > 0) start_r--;

    std::stack<std::pair<int, int>> stack;
    _maze[start_r][start_c].visited = true;
    stack.push({start_r, start_c});

    auto get_unvisited_neighbors = [this](int r, int c) {
        std::vector<std::pair<int, int>> neighbors;
        if (r > 0 && !_maze[r - 1][c].visited && _maze[r - 1][c].valid)
            neighbors.push_back({r - 1, c});
        if (r < MAZE_ROWS - 1 && !_maze[r + 1][c].visited && _maze[r + 1][c].valid)
            neighbors.push_back({r + 1, c});
        if (c > 0 && !_maze[r][c - 1].visited && _maze[r][c - 1].valid)
            neighbors.push_back({r, c - 1});
        if (c < MAZE_COLS - 1 && !_maze[r][c + 1].visited && _maze[r][c + 1].valid)
            neighbors.push_back({r, c + 1});
        return neighbors;
    };

    while (!stack.empty()) {
        auto [r, c] = stack.top();
        auto neighbors = get_unvisited_neighbors(r, c);

        if (neighbors.empty()) {
            stack.pop();
        } else {
            auto [nr, nc] = neighbors[esp_random() % neighbors.size()];
            // Remove wall between current and neighbor
            if (nr == r - 1) { _maze[r][c].wall_top = false; _maze[nr][nc].wall_bottom = false; }
            if (nr == r + 1) { _maze[r][c].wall_bottom = false; _maze[nr][nc].wall_top = false; }
            if (nc == c - 1) { _maze[r][c].wall_left = false; _maze[nr][nc].wall_right = false; }
            if (nc == c + 1) { _maze[r][c].wall_right = false; _maze[nr][nc].wall_left = false; }
            _maze[nr][nc].visited = true;
            stack.push({nr, nc});
        }
    }

    ESP_LOGI(TAG, "Maze generation complete.");
}

void AdventureMode::apply_corner_mask() {
    // Corner radius based on VISIBLE area (same logic as classic mode)
    float corner_radius = std::min((float)_screen_width, (float)_screen_height) * CORNER_PERCENT;
    float corner_sq = corner_radius * corner_radius;

    ESP_LOGI(TAG, "Applying corner mask with radius %.1f to all 4 maze corners", corner_radius);

    // We need to mark cells invalid in the 4 corners of the TOTAL maze
    // The "visible size" determines how much we cut (e.g., 12 cells worth of screen = corner_radius pixels)
    // So we convert corner_radius from pixels to cells
    float cells_in_radius_x = corner_radius / _cell_width;
    float cells_in_radius_y = corner_radius / _cell_height;
    int check_cells = (int)std::ceil(std::max(cells_in_radius_x, cells_in_radius_y)) + 1;

    // Top-Left Corner (rows 0..check_cells, cols 0..check_cells)
    for (int r = 0; r < check_cells && r < MAZE_ROWS; ++r) {
        for (int c = 0; c < check_cells && c < MAZE_COLS; ++c) {
            float px = c * _cell_width;
            float py = r * _cell_height;
            if (px < corner_radius && py < corner_radius) {
                float dx = px - corner_radius;
                float dy = py - corner_radius;
                if (dx * dx + dy * dy > corner_sq) {
                    _maze[r][c].valid = false;
                    _maze[r][c].visited = true;
                }
            }
        }
    }

    // Top-Right Corner
    for (int r = 0; r < check_cells && r < MAZE_ROWS; ++r) {
        for (int c = MAZE_COLS - check_cells; c < MAZE_COLS; ++c) {
            if (c < 0) continue;
            float px = (c + 1) * _cell_width;
            float py = r * _cell_height;
            float screen_right = MAZE_COLS * _cell_width;
            if (px > screen_right - corner_radius && py < corner_radius) {
                float dx = px - (screen_right - corner_radius);
                float dy = py - corner_radius;
                if (dx * dx + dy * dy > corner_sq) {
                    _maze[r][c].valid = false;
                    _maze[r][c].visited = true;
                }
            }
        }
    }

    // Bottom-Left Corner
    for (int r = MAZE_ROWS - check_cells; r < MAZE_ROWS; ++r) {
        if (r < 0) continue;
        for (int c = 0; c < check_cells && c < MAZE_COLS; ++c) {
            float px = c * _cell_width;
            float py = (r + 1) * _cell_height;
            float screen_bottom = MAZE_ROWS * _cell_height;
            if (px < corner_radius && py > screen_bottom - corner_radius) {
                float dx = px - corner_radius;
                float dy = py - (screen_bottom - corner_radius);
                if (dx * dx + dy * dy > corner_sq) {
                    _maze[r][c].valid = false;
                    _maze[r][c].visited = true;
                }
            }
        }
    }

    // Bottom-Right Corner
    for (int r = MAZE_ROWS - check_cells; r < MAZE_ROWS; ++r) {
        if (r < 0) continue;
        for (int c = MAZE_COLS - check_cells; c < MAZE_COLS; ++c) {
            if (c < 0) continue;
            float px = (c + 1) * _cell_width;
            float py = (r + 1) * _cell_height;
            float screen_right = MAZE_COLS * _cell_width;
            float screen_bottom = MAZE_ROWS * _cell_height;
            if (px > screen_right - corner_radius && py > screen_bottom - corner_radius) {
                float dx = px - (screen_right - corner_radius);
                float dy = py - (screen_bottom - corner_radius);
                if (dx * dx + dy * dy > corner_sq) {
                    _maze[r][c].valid = false;
                    _maze[r][c].visited = true;
                }
            }
        }
    }
}

void AdventureMode::update_camera() {
    // Camera centered on player, but clamped to maze bounds
    float half_view_w = (_visible_cols * _cell_width) / 2.0f;
    float half_view_h = (_visible_rows * _cell_height) / 2.0f;
    float max_camera_x = (MAZE_COLS * _cell_width) - (_visible_cols * _cell_width);
    float max_camera_y = (MAZE_ROWS * _cell_height) - (_visible_rows * _cell_height);

    _camera_x = _player_x - half_view_w;
    _camera_y = _player_y - half_view_h;

    // Clamp
    _camera_x = std::max(0.0f, std::min(_camera_x, max_camera_x));
    _camera_y = std::max(0.0f, std::min(_camera_y, max_camera_y));
}

void AdventureMode::reset_wall_pool() {
    for (auto *wall : _wall_pool) {
        lv_obj_add_flag(wall, LV_OBJ_FLAG_HIDDEN);
    }
    _wall_pool_index = 0;
}

lv_obj_t* AdventureMode::get_wall_from_pool() {
    if (_wall_pool_index >= (int)_wall_pool.size()) {
        ESP_LOGW(TAG, "Wall pool exhausted!");
        return nullptr;
    }
    lv_obj_t *wall = _wall_pool[_wall_pool_index++];
    lv_obj_clear_flag(wall, LV_OBJ_FLAG_HIDDEN);
    return wall;
}

bool AdventureMode::is_cell_in_visible_area(int row, int col) const {
    float cell_x = col * _cell_width;
    float cell_y = row * _cell_height;
    float cell_x2 = cell_x + _cell_width;
    float cell_y2 = cell_y + _cell_height;

    float view_x2 = _camera_x + _visible_cols * _cell_width;
    float view_y2 = _camera_y + _visible_rows * _cell_height;

    return !(cell_x2 < _camera_x || cell_x > view_x2 || cell_y2 < _camera_y || cell_y > view_y2);
}

void AdventureMode::draw_visible_walls() {
    reset_wall_pool();

    const int wall_thickness = 2;

    // Determine visible cell range
    int start_row = (int)(_camera_y / _cell_height);
    int end_row = start_row + _visible_rows + 1;
    int start_col = (int)(_camera_x / _cell_width);
    int end_col = start_col + _visible_cols + 1;

    start_row = std::max(0, start_row);
    end_row = std::min(MAZE_ROWS, end_row);
    start_col = std::max(0, start_col);
    end_col = std::min(MAZE_COLS, end_col);

    for (int r = start_row; r < end_row; ++r) {
        for (int c = start_col; c < end_col; ++c) {
            // Position relative to camera
            int cx = (int)(c * _cell_width - _camera_x);
            int cy = (int)(r * _cell_height - _camera_y);
            int cw = (int)_cell_width;
            int ch = (int)_cell_height;

            // Draw solid block for invalid cells
            if (!_maze[r][c].valid) {
                lv_obj_t *block = get_wall_from_pool();
                if (!block) continue;
                lv_obj_set_size(block, cw, ch);
                lv_obj_set_pos(block, cx, cy);
                lv_obj_set_style_radius(block, 0, 0);
                continue;
            }

            // Draw walls
            if (_maze[r][c].wall_top) {
                lv_obj_t *w = get_wall_from_pool();
                if (!w) continue;
                lv_obj_set_size(w, cw + wall_thickness, wall_thickness);
                lv_obj_set_pos(w, cx, cy);
            }
            if (_maze[r][c].wall_left) {
                lv_obj_t *w = get_wall_from_pool();
                if (!w) continue;
                lv_obj_set_size(w, wall_thickness, ch + wall_thickness);
                lv_obj_set_pos(w, cx, cy);
            }
            // Bottom and Right only for boundary cells or if no neighbor
            if (r == MAZE_ROWS - 1 && _maze[r][c].wall_bottom) {
                lv_obj_t *w = get_wall_from_pool();
                if (!w) continue;
                lv_obj_set_size(w, cw + wall_thickness, wall_thickness);
                lv_obj_set_pos(w, cx, cy + ch);
            }
            if (c == MAZE_COLS - 1 && _maze[r][c].wall_right) {
                lv_obj_t *w = get_wall_from_pool();
                if (!w) continue;
                lv_obj_set_size(w, wall_thickness, ch + wall_thickness);
                lv_obj_set_pos(w, cx + cw, cy);
            }
        }
    }

    // Position Hole relative to camera
    float hole_x = _exit_col * _cell_width + (_cell_width - _ball_radius * 2) / 2 - _camera_x;
    float hole_y = _exit_row * _cell_height + (_cell_height - _ball_radius * 2) / 2 - _camera_y;
    lv_obj_set_pos(_hole, (lv_coord_t)hole_x, (lv_coord_t)hole_y);
    
    // Hide hole if off-screen
    if (hole_x < -_ball_radius * 2 || hole_x > _screen_width ||
        hole_y < -_ball_radius * 2 || hole_y > _screen_height) {
        lv_obj_add_flag(_hole, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(_hole, LV_OBJ_FLAG_HIDDEN);
    }
}

void AdventureMode::check_collision(float new_x, float new_y, float &out_x, float &out_y) {
    // Similar to classic mode, but in maze coordinates
    out_x = new_x;
    out_y = new_y;

    // Get current cell
    int col = (int)(new_x / _cell_width);
    int row = (int)(new_y / _cell_height);

    col = std::max(0, std::min(col, MAZE_COLS - 1));
    row = std::max(0, std::min(row, MAZE_ROWS - 1));

    float cell_left = col * _cell_width;
    float cell_right = cell_left + _cell_width;
    float cell_top = row * _cell_height;
    float cell_bottom = cell_top + _cell_height;

    // Check walls
    if (_maze[row][col].wall_left && new_x - _ball_radius < cell_left) {
        out_x = cell_left + _ball_radius;
        _vel_x = -_vel_x * BOUNCE;
    }
    if (_maze[row][col].wall_right && new_x + _ball_radius > cell_right) {
        out_x = cell_right - _ball_radius;
        _vel_x = -_vel_x * BOUNCE;
    }
    if (_maze[row][col].wall_top && new_y - _ball_radius < cell_top) {
        out_y = cell_top + _ball_radius;
        _vel_y = -_vel_y * BOUNCE;
    }
    if (_maze[row][col].wall_bottom && new_y + _ball_radius > cell_bottom) {
        out_y = cell_bottom - _ball_radius;
        _vel_y = -_vel_y * BOUNCE;
    }

    // Clamp to maze bounds
    float max_x = MAZE_COLS * _cell_width - _ball_radius;
    float max_y = MAZE_ROWS * _cell_height - _ball_radius;
    out_x = std::max(_ball_radius, std::min(out_x, max_x));
    out_y = std::max(_ball_radius, std::min(out_y, max_y));
}

void AdventureMode::update(float ax, float ay, float dt) {
    // Apply acceleration (from gyroscope)
    _vel_x += ax * ACCEL_FACTOR;
    _vel_y += ay * ACCEL_FACTOR;

    // Apply friction
    _vel_x *= FRICTION;
    _vel_y *= FRICTION;

    // Clamp velocity
    float speed = std::sqrt(_vel_x * _vel_x + _vel_y * _vel_y);
    if (speed > MAX_VELOCITY) {
        _vel_x = (_vel_x / speed) * MAX_VELOCITY;
        _vel_y = (_vel_y / speed) * MAX_VELOCITY;
    }

    // Calculate new position
    float new_x = _player_x + _vel_x;
    float new_y = _player_y + _vel_y;

    // Collision check
    float final_x, final_y;
    check_collision(new_x, new_y, final_x, final_y);

    _player_x = final_x;
    _player_y = final_y;

    // Update camera to follow player
    update_camera();

    // Redraw visible walls
    draw_visible_walls();
}

bool AdventureMode::check_win() const {
    // Check if player is in the exit cell
    int player_row = (int)(_player_y / _cell_height);
    int player_col = (int)(_player_x / _cell_width);
    return (player_row == _exit_row && player_col == _exit_col);
}

} // namespace esp_brookesia::apps::gyro_maze
