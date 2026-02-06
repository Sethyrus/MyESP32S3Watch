/*
 * Adventure Mode for Gyro Maze
 * - Large scrollable maze (50x50 default)
 * - Ball fixed at center, maze moves
 * - Efficient rendering: only visible cells drawn
 */
#pragma once

#include "lvgl.h"
#include <vector>
#include <cstdint>

// Forward declaration for GyroMaze in its actual namespace
namespace esp_brookesia::apps { class GyroMaze; }

namespace esp_brookesia::apps::gyro_maze {

class AdventureMode {
public:
    // Configuration
    static constexpr int MAZE_ROWS = 50;
    static constexpr int MAZE_COLS = 50;
    static constexpr float CORNER_PERCENT = 0.20f; // Same as classic mode

    AdventureMode(esp_brookesia::apps::GyroMaze *parent);
    ~AdventureMode();

    // Lifecycle
    void init(lv_obj_t *container, int screen_w, int screen_h, int visible_rows, int visible_cols);
    void cleanup();
    void update(float ax, float ay, float dt); // Physics + Camera update
    bool check_win() const;

    // Positioning
    void reset_player();

private:
    // Maze Cell Structure
    struct Cell {
        bool wall_top = true;
        bool wall_right = true;
        bool wall_bottom = true;
        bool wall_left = true;
        bool visited = false;
        bool valid = true;
    };

    // Parent app reference (for shared resources like IMU)
    esp_brookesia::apps::GyroMaze *_parent;

    // Maze Data
    Cell _maze[MAZE_ROWS][MAZE_COLS];
    int _exit_row, _exit_col;
    int _start_row, _start_col; // Center of maze

    // Camera & Player
    float _camera_x, _camera_y;  // Camera position in maze coordinates (top-left of view)
    float _player_x, _player_y;  // Player position in maze coordinates
    float _vel_x, _vel_y;        // Player velocity

    // Visible Area Config
    int _visible_rows, _visible_cols;
    float _cell_width, _cell_height;
    float _ball_radius;
    int _screen_width, _screen_height;

    // LVGL Objects
    lv_obj_t *_container;
    lv_obj_t *_ball;
    lv_obj_t *_hole;
    lv_obj_t *_wall_container;

    // Performance: Cell-aligned render window + cheap per-frame world translation
    int _render_start_row, _render_start_col;
    int _active_wall_count;
    lv_coord_t _last_world_offset_x, _last_world_offset_y;
    bool _hole_hidden;

    // Wall Object Pool (for efficient rendering)
    static constexpr int MAX_WALL_OBJECTS = 600; // More than enough for 12x12 * 4 walls
    std::vector<lv_obj_t*> _wall_pool;
    int _wall_pool_index;

    // Methods
    void generate_maze();
    void apply_corner_mask(); // Applies to visible area corners
    void draw_visible_walls(bool force = false);
    void update_world_transform();
    void update_camera();
    lv_obj_t* get_wall_from_pool();
    void check_collision(float new_x, float new_y, float &out_x, float &out_y);

    // Physics Constants (same as classic)
    static constexpr float FRICTION = 0.92f;
    static constexpr float ACCEL_FACTOR = 0.8f;
    static constexpr float MAX_VELOCITY = 15.0f;
    static constexpr float BOUNCE = 0.3f;
};

} // namespace esp_brookesia::apps::gyro_maze
