#ifndef ESP32_EMBEDDED_COMMON_LIB_HBRIDGE_H
#define ESP32_EMBEDDED_COMMON_LIB_HBRIDGE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generic single-motor H-bridge interface (vtable).
 *
 * Provides a chip-agnostic handle for one motor (one H-bridge channel).
 * Any chip driver (DRV8833, TB6612, L298N, …) can populate this struct via
 * its own bind() function so that motor_control and higher layers remain
 * completely chip-agnostic.
 *
 * Each instance represents exactly ONE motor direction.  For a two-channel
 * chip (e.g. DRV8833) bind once per channel to get two independent handles.
 *
 * @note  @p ctx must remain valid for the lifetime of the handle.
 */
typedef struct {
    /** Set motor speed [-100, +100] %. Positive = forward, negative = reverse. */
    esp_err_t (*set_speed)(void *ctx, int8_t speed_pct);
    /** Stop the motor (coast or brake — implementation-defined). */
    esp_err_t (*stop)(void *ctx);
    /** Opaque context pointer passed verbatim to set_speed() and stop(). */
    void *ctx;
} esp32_common_hbridge_t;

#ifdef __cplusplus
}
#endif

#endif /* ESP32_EMBEDDED_COMMON_LIB_HBRIDGE_H */
