# Changelog

All notable changes to this project will be documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versions follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Renamed public functions to category-prefixed names:
  `ecl_algo_*`, `ecl_driver_*`, `ecl_motor_*`, `ecl_sensor_*`, and `ecl_io_*`.

## [0.1.0] - 2026-06-16

### Added

- `ultrasonic_sensor`: blocking HC-SR04 driver using ESP-IDF GPIO and `esp_timer`
  - `ecl_sensor_ultrasonic_sensor_default_config()` – sensible defaults (30 ms timeout, 343.2 m/s)
  - `ecl_sensor_ultrasonic_sensor_init()` – GPIO pin configuration
  - `ecl_sensor_ultrasonic_sensor_measure_distance_cm()` – blocking distance measurement
  - `ecl_sensor_ultrasonic_sensor_measure_pulse_us()` – raw echo pulse measurement
- `examples/basic_ultrasonic` – standalone ESP-IDF project using the driver
- `library.json` for PlatformIO registry compatibility
- `idf_component.yml` for ESP-IDF component manager compatibility
- Dual-mode `CMakeLists.txt` (ESP-IDF component + standalone CMake)
