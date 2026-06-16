#include "ESP32EmbeddedCommonLib/algo/odometry.h"

#include <stddef.h>
#include <string.h>

/* Avoid M_PI — define locally. */
#define ODOMETRY_TWO_PI  6.28318530717958647692f

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

    /* Convert ticks → arc lengths (metres). */
    float metres_per_tick = (ODOMETRY_TWO_PI * odom->config.wheel_radius_m)
                            / (float)ticks_per_rev;
    float dl = (float)delta_left  * metres_per_tick;
    float dr = (float)delta_right * metres_per_tick;

    /* Centre arc and heading change. */
    float dc    = (dl + dr) * 0.5f;
    float dtheta = (dr - dl) / odom->config.wheel_base_m;

    /* Integrate pose using mid-point heading. */
    float theta_mid = odom->pose.theta_rad + dtheta * 0.5f;

    /* Simple inline sin/cos approximation (Taylor, 5 terms) sufficient for
     * small dt.  Replace with sinf/cosf if math.h is available in your build. */
    /* For correctness in embedded environments without a full libm we use the
     * standard functions — esp-idf ships newlib which provides them. */
    float s, c;
    /* sinf / cosf are available via newlib on ESP-IDF. */
    {
        /* Use the portable approach: approximate via Maclaurin for small angles,
         * but for generality just call the runtime. */
        extern float sinf(float);
        extern float cosf(float);
        s = sinf(theta_mid);
        c = cosf(theta_mid);
    }

    odom->pose.x_m     += dc * c;
    odom->pose.y_m     += dc * s;
    odom->pose.theta_rad += dtheta;

    /* Normalise heading to (-π, π]. */
    while (odom->pose.theta_rad >  ODOMETRY_TWO_PI * 0.5f)
        odom->pose.theta_rad -= ODOMETRY_TWO_PI;
    while (odom->pose.theta_rad <= -ODOMETRY_TWO_PI * 0.5f)
        odom->pose.theta_rad += ODOMETRY_TWO_PI;
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
