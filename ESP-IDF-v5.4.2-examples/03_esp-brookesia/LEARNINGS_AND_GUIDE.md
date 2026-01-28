# ESP-Brookesia Project Guide & Learnings

This document compiles key procedures and technical insights gained during the development of the "Gyro Game" app. It serves as a reference for creating future applications and working with the ESP32-S3-Touch-AMOLED-2.06 hardware.

## 1. Creating a New App Component

To create a new app that appears on the device's home screen (Launcher), follow these steps:

### A. Component Structure
Create a folder in `components/` (e.g., `components/my_new_app`) with following structure:
```text
my_new_app/
├── CMakeLists.txt
├── idf_component.yml
├── my_new_app.hpp
└── my_new_app.cpp
```

### B. Critical Registration (`WHOLE_ARCHIVE`)
The most important step for the app to appear on the Home Screen is preventing the linker from stripping the registration constructor. You **MUST** use `WHOLE_ARCHIVE` in your `CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "my_new_app.cpp"
    INCLUDE_DIRS "."
    REQUIRES brookesia_core lvgl # ... other deps
    WHOLE_ARCHIVE  # << CRITICAL: Forces linker to include static constructors
)
```

### C. App Registration Code
Use the `ESP_UTILS_REGISTER_PLUGIN_WITH_CONSTRUCTOR` macro at the end of your `.cpp` file:

```cpp
#include "esp_brookesia.hpp"

// ... implementation ...

ESP_UTILS_REGISTER_PLUGIN_WITH_CONSTRUCTOR(
    esp_brookesia::systems::base::App, 
    MyNewApp, 
    "App Name", 
    []() {
        return std::shared_ptr<MyNewApp>(MyNewApp::requestInstance(), [](MyNewApp *p) {});
    }
);
```

### D. Main Dependency
Ensure `main/CMakeLists.txt` requires your new component so it's part of the build:
```cmake
idf_component_register(SRCS "main.cpp" ... REQUIRES my_new_app)
```

---

## 2. IMU (Gyroscope/Accelerometer) Insights

We are using the **QMI8658** 6-axis IMU (not QMA7981 as initially thought).

### A. Driver Component
Use the official `waveshare__qmi8658` component. Add it to `idf_component.yml`:
```yaml
dependencies:
  waveshare__qmi8658:
    version: "*"
```

### B. Critical Implementation Details
*   **I2C Address**: The correct address is `QMI8658_ADDRESS_HIGH` (`0x6B`).
*   **Unit Scaling (Important)**: The driver returns accelerometer values in **milli-g**.
    *   **Raw Output**: ~1000 = 1g.
    *   **Normalization**: You MUST divide by `1000.0f` to work with standard `g` units (where 9.8m/s² = 1.0).
    *   **Deadzone**: A deadzone of `0.015f` (after normalization) works well for responsiveness.

### C. Physics & Coordinate Mapping
For this specific watch orientation:
*   **X-Axis**: Mapped to `force_y`.
*   **Y-Axis**: Mapped to `force_x` (inverted `-ay`).
*   **Smoothing**: A Simple Low Pass Filter (LPF) is essential to remove jitter.
    ```cpp
    smooth_val += ALPHA * (raw_val - smooth_val); // ALPHA ~0.1 to 0.3
    ```

### D. Calibration
The sensor has a non-zero bias. Always implement a calibration routine:
1.  Read ~200 samples at startup (when device is likely flat).
2.  Calculate average X and Y.
3.  Store these as `bias_x` / `bias_y`.
4.  Subtract this bias from every future reading.

---

## 3. UI & Layout (LVGL)

### A. Full Screen Container
To create a clean, edge-to-edge application background:
*   **Remove Borders**: Default LVGL objects have borders. Set `lv_obj_set_style_border_width(cont, 0, 0)`.
*   **Remove Radius**: Set `lv_obj_set_style_radius(cont, 0, 0)` to fill corners (especially on unique displays).
*   **Input Pass-through**: If you want gestures (like swiping up for the navigation bar) to work:
    ```cpp
    lv_obj_clear_flag(container, LV_OBJ_FLAG_CLICKABLE);     // Don't consume clicks
    lv_obj_add_flag(container, LV_OBJ_FLAG_GESTURE_BUBBLE);  // Pass gestures up
    ```

### B. Navigation Bar
*   **Enabling**: In your `App::requestInstance`, set `use_navigation_bar = true`.
*   **Gesture**: The system handles the "swipe up from bottom" to minimize. Your UI container must **not** block this gesture (see Input Pass-through above).

---

## 4. Debugging Tips

*   **Force Logs**: If logs aren't showing, force the tag level in your `init()`:
    ```cpp
    esp_log_level_set("GyroGame", ESP_LOG_INFO);
    ```
*   **Frequency**: IMU loops run fast (50Hz+). Avoid logging every frame. Use a counter to log every ~50th frame (1s) to avoid flooding the serial output.

---

## 5. Resource Recording for Recents Screen

For your app to appear in the multitask/recents screen, the framework must track all UI resources (screens, timers, animations).

### Correct Pattern
Wrap all UI creation in `run()` with resource recording. **Crucially, if you disabled `enable_default_screen`, you must create and load a screen first.**

```cpp
bool MyApp::run(void) {
    startRecordResource();  // FIRST: before any UI creation!
    
    // 1. Create and Load Screen (Required if use_default_screen = false)
    lv_obj_t *my_screen = lv_obj_create(NULL);
    lv_scr_load(my_screen);

    // 2. Create all UI on that screen
    lv_obj_t *container = lv_obj_create(my_screen);
    lv_timer_create(my_timer_cb, 100, this);
    
    endRecordResource();  // LAST: after all UI creation!
    return true;
}
```

## 6. Navigation Modes (Gestures vs. Bar)

The `use_navigation_bar` flag determines how the user exits or switches apps:

*   **`true` (Button Mode)**: Shows a handle at the bottom. swipes reveal a full Android-style bar with Back/Home/Recents buttons.
*   **`false` (Gesture Mode)**: No visual bar.
    *   **Swipe Up (Short)**: Home (Minimize).
    *   **Swipe Up (Hold)**: Recents/Multitask.
    *   **Swipe Right**: Back.

> [!NOTE]
> For Gesture Mode to work, your app **MUST** have the "Resource Recording" pattern (Section 5) implemented correctly. If the app screen isn't tracked, gestures will fail or not appear in recents.

> [!CAUTION]
> If you call `startRecordResource()` **after** creating UI elements, they will NOT be tracked and the app will not appear in the recents screen!

