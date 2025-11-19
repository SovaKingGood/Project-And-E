# Project-And-E
Remote Control Car

Competed in RC Classic 2025 at BCIT

## And-E Display - Final Code

This is the self-contained version of the ESP32 Square Line Studio display project with Bluepad32 support.

## Project Structure

```
FINALcode/
├── bluepad32/              # Bluepad32 library (self-contained)
│   ├── src/                # Bluepad32 source components
│   └── external/           # BTstack and dependencies
├── main/                   # Application source code
│   ├── ui/                 # Square Line Studio generated UI
│   ├── main.c              # Main application entry
│   ├── display.c           # Display and LVGL integration
│   ├── my_platform.c      # Bluepad32 custom platform
│   ├── espnow_task.c       # ESP-NOW communication
│   └── system_config.h     # Hardware pin definitions
├── managed_components/     # ESP-IDF managed components
│   ├── lvgl__lvgl/         # LVGL graphics library
│   ├── espressif__esp_lvgl_port/  # ESP LVGL port
│   ├── espressif__esp_lcd_touch/  # Touch driver base
│   └── atanisoft__esp_lcd_touch_xpt2046/  # XPT2046 touch driver
├── CMakeLists.txt          # Root CMake configuration
├── sdkconfig.defaults     # ESP-IDF default configuration
├── partitions.csv          # Partition table
└── lv_conf.h              # LVGL configuration
```

## Key Features

- **Square Line Studio UI**: Complete UI generated from Square Line Studio
- **Bluepad32 Integration**: PS4 controller support via Bluepad32
- **ESP-NOW Communication**: Wireless communication with RC car
- **ST7789 Display**: 320x240 TFT display with SPI
- **XPT2046 Touch**: Touch screen controller support
- **LVGL Graphics**: Modern UI framework

## Building

1. Ensure ESP-IDF is installed and configured
2. Navigate to the FINALcode directory
3. Run:
   ```bash
   idf.py build
   ```

## Configuration

- Hardware pins are defined in `main/system_config.h`
- ESP-NOW target MAC address is in `main/system_config.h`
- Display settings are in `lv_conf.h`
- ESP-IDF settings are in `sdkconfig.defaults`

## Dependencies

All dependencies are self-contained:
- Bluepad32 library is in `bluepad32/`
- Managed components are in `managed_components/`
- No external references needed

## Notes

- This project is completely self-contained and does not require the bluepad32 examples folder structure
- Bluepad32 is referenced from the local `bluepad32/` directory
- All Square Line Studio UI files are in `main/ui/`
