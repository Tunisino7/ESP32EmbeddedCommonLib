#ifndef ECL_ALGO_PID_H
#define ECL_ALGO_PID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Discrete PID controller.
 *
 * Implements the standard position-form PID algorithm with:
 *   - Anti-windup: integral clamped to [-integral_limit, +integral_limit]
 *   - Derivative on measurement (not on error) to avoid derivative kick on
 *     setpoint changes
 *   - Output clamped to [output_min, output_max]
 *
 * Update at a fixed rate dt_s for correct behaviour.
 */
typedef struct {
    /* Gains */
    float kp;             /**< Proportional gain.                              */
    float ki;             /**< Integral gain.                                  */
    float kd;             /**< Derivative gain.                                */

    /* Limits */
    float output_min;     /**< Minimum controller output (e.g. -100).         */
    float output_max;     /**< Maximum controller output (e.g. +100).         */
    float integral_limit; /**< Anti-windup: caps |integral| to this value.    */

    /* Internal state (zero-initialise before first call to _update) */
    float integral;       /**< Accumulated integral term.                      */
    float prev_measurement; /**< Previous measurement for derivative term.    */
    bool  first_update;   /**< true until first _update call (skips derivative).*/
} ecl_pid_t;

/**
 * @brief Initialise a PID controller.
 *
 * @param pid             Controller instance.
 * @param kp              Proportional gain.
 * @param ki              Integral gain.
 * @param kd              Derivative gain.
 * @param output_min      Lower output clamp.
 * @param output_max      Upper output clamp.
 * @param integral_limit  Anti-windup integral clamp (absolute value).
 */
void ecl_pid_init(
    ecl_pid_t *pid,
    float kp, float ki, float kd,
    float output_min, float output_max,
    float integral_limit
);

/**
 * @brief Compute one PID iteration.
 *
 * @param pid          Initialised controller.
 * @param setpoint     Desired value.
 * @param measurement  Current measured value.
 * @param dt_s         Time elapsed since last call (seconds).
 * @return             Controller output, clamped to [output_min, output_max].
 */
float ecl_pid_update(
    ecl_pid_t *pid,
    float setpoint,
    float measurement,
    float dt_s
);

/**
 * @brief Reset integral and derivative state (e.g. on mode change).
 */
void ecl_pid_reset(ecl_pid_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* ECL_ALGO_PID_H */
