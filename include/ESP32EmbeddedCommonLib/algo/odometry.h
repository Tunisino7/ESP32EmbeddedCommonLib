#ifndef ESP32_EMBEDDED_COMMON_LIB_ALGO_ODOMETRY_H
#define ESP32_EMBEDDED_COMMON_LIB_ALGO_ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 2-D pose estimate for a differential-drive robot.
 *
 * Origin is the position at the last call to esp32_common_odometry_reset().
 * Heading theta is measured from the positive X-axis, counter-clockwise,
 * in radians.
 */
typedef struct {
    float x_m;     /**< X position (metres).              */
    float y_m;     /**< Y position (metres).              */
    float theta_rad; /**< Heading (radians, CCW positive). */
} esp32_common_pose_t;

/**
 * @brief Physical parameters of the differential-drive platform.
 */
typedef struct {
    float wheel_radius_m;   /**< Driven wheel radius (metres).                */
    float wheel_base_m;     /**< Distance between left and right wheel centres (metres). */
} esp32_common_odometry_config_t;

/**
 * @brief Odometry state.
 */
typedef struct {
    esp32_common_odometry_config_t config;
    esp32_common_pose_t            pose;
} esp32_common_odometry_t;

/**
 * @brief Initialise odometry.
 *
 * Sets pose to (0, 0, 0) and stores the physical config.
 *
 * @param odom    Instance to initialise.
 * @param config  Physical parameters.
 */
void esp32_common_odometry_init(
    esp32_common_odometry_t              *odom,
    const esp32_common_odometry_config_t *config
);

/**
 * @brief Update pose from wheel encoder tick counts.
 *
 * Call this at a fixed rate.  Pass the *delta* ticks since the last call
 * (not cumulative).
 *
 * @param odom          Initialised odometry instance.
 * @param delta_left    Ticks from left encoder since last update (+= forward).
 * @param delta_right   Ticks from right encoder since last update (+= forward).
 * @param ticks_per_rev Total encoder ticks per full wheel revolution.
 */
void esp32_common_odometry_update(
    esp32_common_odometry_t *odom,
    int32_t delta_left,
    int32_t delta_right,
    uint32_t ticks_per_rev
);

/**
 * @brief Get the current pose estimate.
 *
 * @param odom  Initialised odometry instance.
 * @param pose  Output pose (may not be NULL).
 */
void esp32_common_odometry_get_pose(
    const esp32_common_odometry_t *odom,
    esp32_common_pose_t           *pose
);

/**
 * @brief Reset pose to (0, 0, 0).
 */
void esp32_common_odometry_reset(esp32_common_odometry_t *odom);

#ifdef __cplusplus
}
#endif

#endif /* ESP32_EMBEDDED_COMMON_LIB_ALGO_ODOMETRY_H */
