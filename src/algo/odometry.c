/*
 * odometry.c — Differential-drive dead-reckoning odometry
 *
 * Estimates the robot's 2-D pose (x, y, θ) from wheel encoder tick deltas.
 *
 * Kinematics (per time step):
 *   dl, dr   = left / right wheel arc lengths (metres)
 *   dc       = (dl + dr) / 2          — distance travelled by robot centre
 *   dθ       = (dr − dl) / wheel_base  — change in heading
 *
 * Integration uses the mid-point heading (θ + dθ/2) to reduce error
 * compared to using the start-of-step heading.
 *
 * Limitations:
 *   - Dead reckoning accumulates error over time; use IMU fusion or
 *     external position fixes to correct drift.
 *   - Assumes no wheel slip.  Slip causes systematic drift in θ.
 */
#include "ESP32EmbeddedCommonLib/algo/odometry.h"

#include <math.h>
#include <stddef.h>

/* ── Public API ──────────────────────────────────────────────────────────── */

void esp32_common_odometry_init(
    esp32_common_odometry_t              *odom,
    const esp32_common_odometry_config_t *config)
{
    if (odom == NULL || config == NULL) return;
    odom->config = *config;
    esp32_common_odometry_reset(odom);
}

void esp32_common_odometry_update(
    esp32_common_odometry_t *odom,
    int32_t  delta_left,
    int32_t  delta_right,
    uint32_t ticks_per_rev)
{
    if (odom == NULL || ticks_per_rev == 0) return;

    /* Wheel circumference / ticks_per_rev gives metres per tick. */
    float metres_per_tick = (2.0f * (float)M_PI * odom->config.wheel_radius_m)
                            / (float)ticks_per_rev;
    float dl = (float)delta_left  * metres_per_tick;  /* left arc length  */
    float dr = (float)delta_right * metres_per_tick;  /* right arc length */

    /* Robot centre arc and heading change from differential kinematics. */
    float dc     = (dl + dr) * 0.5f;
    float dtheta = (dr - dl) / odom->config.wheel_base_m;

    /* Mid-point integration: heading at the midpoint of the step reduces
     * the linearisation error compared to using the start-of-step heading. */
    float theta_mid = odom->pose.theta_rad + dtheta * 0.5f;

    odom->pose.x_m      += dc * cosf(theta_mid);
    odom->pose.y_m      += dc * sinf(theta_mid);
    odom->pose.theta_rad += dtheta;

    /* Normalise heading to the half-open interval (-π, π] to keep
     * arithmetic consistent regardless of how many full rotations occur. */
    while (odom->pose.theta_rad >  (float)M_PI) odom->pose.theta_rad -= 2.0f * (float)M_PI;
    while (odom->pose.theta_rad <= -(float)M_PI) odom->pose.theta_rad += 2.0f * (float)M_PI;
}

void esp32_common_odometry_get_pose(
    const esp32_common_odometry_t *odom,
    esp32_common_pose_t           *pose)
{
    if (odom == NULL || pose == NULL) return;
    *pose = odom->pose;
}

void esp32_common_odometry_reset(esp32_common_odometry_t *odom)
{
    if (odom == NULL) return;
    odom->pose.x_m      = 0.0f;
    odom->pose.y_m      = 0.0f;
    odom->pose.theta_rad = 0.0f;
}
