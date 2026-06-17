if(ESP_PLATFORM)
    # Building as an ESP-IDF component (PlatformIO espidf / idf.py).
    idf_component_register(
        SRCS
            "src/sensor/imu.c"
            "src/driver/bmi160.c"
            "src/sensor/ir_line_sensor.c"
            "src/sensor/ultrasonic_sensor.c"
            "src/io/led.c"
            "src/motor/dc_motor.c"
            "src/motor/dc_motor_encoder.c"
            "src/motor/pcnt_encoder.c"
            "src/motor/motor_control.c"
            "src/driver/drv8833.c"
            "src/algo/pid.c"
            "src/algo/maze_solver.c"
            "src/algo/filter.c"
            "src/algo/odometry.c"
            "src/algo/motion_profile.c"
        INCLUDE_DIRS
            "include"
        REQUIRES
            esp_driver_gpio
            esp_timer
            esp_rom
            esp_adc
            esp_driver_i2c
            esp_driver_ledc
            esp_driver_pcnt
    )
else()
    # Building as a standalone CMake library (unit tests, host tooling, etc.).
    cmake_minimum_required(VERSION 3.16)
    project(ESP32EmbeddedCommonLib C)

    add_library(ESP32EmbeddedCommonLib STATIC
        src/sensor/imu.c
        src/driver/bmi160.c
        src/sensor/ir_line_sensor.c
        src/sensor/ultrasonic_sensor.c
        src/io/led.c
        src/motor/dc_motor.c
        src/motor/dc_motor_encoder.c
        src/motor/pcnt_encoder.c
        src/motor/motor_control.c
        src/driver/drv8833.c
        src/algo/pid.c
        src/algo/maze_solver.c
        src/algo/filter.c
        src/algo/odometry.c
        src/algo/motion_profile.c
    )

    target_include_directories(ESP32EmbeddedCommonLib
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )

    target_compile_options(ESP32EmbeddedCommonLib PRIVATE
        -Wall -Wextra -Wpedantic -Werror
    )
endif()
