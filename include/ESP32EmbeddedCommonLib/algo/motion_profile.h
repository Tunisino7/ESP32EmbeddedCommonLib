#ifndef ESP32_EMBEDDED_COMMON_LIB_ALGO_MOTION_PROFILE_H
#define ESP32_EMBEDDED_COMMON_LIB_ALGO_MOTION_PROFILE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Trapezoidal velocity motion profile.
 *
 * Generates a smooth velocity setpoint that:
 *   1. Ramps up from current velocity to cruise velocity at max_accel.
 *   2. Holds cruise velocity.
 *   3. Ramps down to target velocity at max_decel.
 *
 * Useful for avoiding wheel slip and mechanical shock on start/stop.
 *
 * Call esp32_common_motion_profile_update() at a fixed rate to get the next
 * velocity setpoint.
 */
typedef struct {
    float max_accel;    /**< Maximum acceleration magnitude (units/s²).       */
    float max_decel;    /**< Maximum deceleration magnitude (units/s²).       */
    float current_vel;  /**< Current velocity output (units/s).               */
    float target_vel;   /**< Desired final velocity (units/s).                */
} esp32_common_motion_profile_t;

/**
 * @brief Initialise a motion profile.
 *
 * @param mp           Instance to initialise.
 * @param max_accel    Maximum acceleration (must be > 0).
 * @param max_decel    Maximum deceleration (must be > 0).
 * @param initial_vel  Starting velocity (current actual velocity).
 */
void esp32_common_motion_profile_init(
    esp32_common_motion_profile_t *mp,
    float max_accel,
    float max_decel,
    float initial_vel
);

/**
 * @brief Set a new target velocity.
 *
 * The profile will ramp current_vel toward target_vel on subsequent
 * _update() calls.
 *
 * @param mp          Initialised profile.
 * @param target_vel  Desired velocity (same units as max_accel).
 */
void esp32_common_motion_profile_set_target(
    esp32_common_motion_profile_t *mp,
    float target_vel
);

/**
 * @brief Advance the profile by one time step.
 *
 * @param mp    Initialised profile.
 * @param dt_s  Time elapsed since the last call (seconds).
 * @return      Velocity setpoint for this time step.
 */
float esp32_common_motion_profile_update(
    esp32_common_motion_profile_t *mp,
    float dt_s
);

/**
 * @brief Returns true when current_vel == target_vel (profile settled).
 */
bool esp32_common_motion_profile_is_settled(
    const esp32_common_motion_profile_t *mp
);

#ifdef __cplusplus
}
#endif

#endif /* ESP32_EMBEDDED_COMMON_LIB_ALGO_MOTION_PROFILE_H */
