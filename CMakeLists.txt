if(ESP_PLATFORM)
    # Building as an ESP-IDF component (PlatformIO espidf / idf.py).
    idf_component_register(
        SRCS
            "src/ultrasonic_sensor.c"
            "src/led.c"
            "src/ir_line_sensor.c"
            "src/bmi160.c"
        INCLUDE_DIRS
            "include"
        REQUIRES
            esp_driver_gpio
            esp_timer
            esp_rom
            esp_adc
            esp_driver_i2c
    )
else()
    # Building as a standalone CMake library (unit tests, host tooling, etc.).
    cmake_minimum_required(VERSION 3.16)
    project(ESP32EmbeddedCommonLib C)

    add_library(ESP32EmbeddedCommonLib STATIC
        src/ultrasonic_sensor.c
        src/led.c
        src/ir_line_sensor.c
        src/bmi160.c
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
