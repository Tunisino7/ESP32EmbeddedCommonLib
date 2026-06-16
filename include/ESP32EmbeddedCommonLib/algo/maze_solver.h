#ifndef ESP32_EMBEDDED_COMMON_LIB_ALGO_MAZE_SOLVER_H
#define ESP32_EMBEDDED_COMMON_LIB_ALGO_MAZE_SOLVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Wall-following direction preference.
 */
typedef enum {
    ESP32_COMMON_MAZE_FOLLOW_LEFT  = 0, /**< Left-hand rule: always prefer left.  */
    ESP32_COMMON_MAZE_FOLLOW_RIGHT = 1, /**< Right-hand rule: always prefer right.*/
} esp32_common_maze_follow_t;

/**
 * @brief Cardinal heading of the robot (grid-aligned).
 */
typedef enum {
    ESP32_COMMON_MAZE_NORTH = 0,
    ESP32_COMMON_MAZE_EAST  = 1,
    ESP32_COMMON_MAZE_SOUTH = 2,
    ESP32_COMMON_MAZE_WEST  = 3,
} esp32_common_maze_heading_t;

/**
 * @brief Turn instruction returned by the maze solver.
 */
typedef enum {
    ESP32_COMMON_MAZE_TURN_NONE     = 0, /**< Continue straight ahead.           */
    ESP32_COMMON_MAZE_TURN_LEFT     = 1, /**< Turn 90° left.                     */
    ESP32_COMMON_MAZE_TURN_RIGHT    = 2, /**< Turn 90° right.                    */
    ESP32_COMMON_MAZE_TURN_AROUND   = 3, /**< U-turn (dead end).                 */
} esp32_common_maze_turn_t;

/**
 * @brief Maze solver state.
 */
typedef struct {
    esp32_common_maze_follow_t  rule;    /**< Left-hand or right-hand.           */
    esp32_common_maze_heading_t heading; /**< Current cardinal heading.          */
} esp32_common_maze_solver_t;

/**
 * @brief Initialise the maze solver.
 *
 * @param solver          Instance to initialise.
 * @param rule            Left-hand or right-hand wall following.
 * @param initial_heading Starting heading of the robot.
 */
void esp32_common_maze_solver_init(
    esp32_common_maze_solver_t *solver,
    esp32_common_maze_follow_t  rule,
    esp32_common_maze_heading_t initial_heading
);

/**
 * @brief Compute the next turn given the current wall sensors.
 *
 * Call this once per decision point (intersection / dead-end).
 * The solver updates its internal heading automatically.
 *
 * @param solver      Initialised solver.
 * @param wall_left   true if a wall is detected to the left.
 * @param wall_front  true if a wall is detected straight ahead.
 * @param wall_right  true if a wall is detected to the right.
 * @return            Turn instruction to execute.
 */
esp32_common_maze_turn_t esp32_common_maze_solver_update(
    esp32_common_maze_solver_t *solver,
    bool wall_left,
    bool wall_front,
    bool wall_right
);

/**
 * @brief Apply a turn to the solver's internal heading.
 *
 * Call this after the robot has physically completed the turn.
 *
 * @param solver  Solver instance.
 * @param turn    The turn that was executed.
 */
void esp32_common_maze_solver_apply_turn(
    esp32_common_maze_solver_t *solver,
    esp32_common_maze_turn_t    turn
);

/**
 * @brief Return the current heading of the solver.
 */
esp32_common_maze_heading_t esp32_common_maze_solver_heading(
    const esp32_common_maze_solver_t *solver
);

#ifdef __cplusplus
}
#endif

#endif /* ESP32_EMBEDDED_COMMON_LIB_ALGO_MAZE_SOLVER_H */
