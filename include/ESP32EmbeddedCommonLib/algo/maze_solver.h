#ifndef ECL_ALGO_MAZE_SOLVER_H
#define ECL_ALGO_MAZE_SOLVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Wall-following direction preference.
 */
typedef enum {
    ECL_MAZE_FOLLOW_LEFT  = 0, /**< Left-hand rule: always prefer left.  */
    ECL_MAZE_FOLLOW_RIGHT = 1, /**< Right-hand rule: always prefer right.*/
} ecl_maze_follow_t;

/**
 * @brief Cardinal heading of the robot (grid-aligned).
 */
typedef enum {
    ECL_MAZE_NORTH = 0,
    ECL_MAZE_EAST  = 1,
    ECL_MAZE_SOUTH = 2,
    ECL_MAZE_WEST  = 3,
} ecl_maze_heading_t;

/**
 * @brief Turn instruction returned by the maze solver.
 */
typedef enum {
    ECL_MAZE_TURN_NONE     = 0, /**< Continue straight ahead.           */
    ECL_MAZE_TURN_LEFT     = 1, /**< Turn 90° left.                     */
    ECL_MAZE_TURN_RIGHT    = 2, /**< Turn 90° right.                    */
    ECL_MAZE_TURN_AROUND   = 3, /**< U-turn (dead end).                 */
} ecl_maze_turn_t;

/**
 * @brief Maze solver state.
 */
typedef struct {
    ecl_maze_follow_t  rule;    /**< Left-hand or right-hand.           */
    ecl_maze_heading_t heading; /**< Current cardinal heading.          */
} ecl_maze_solver_t;

/**
 * @brief Initialise the maze solver.
 *
 * @param solver          Instance to initialise.
 * @param rule            Left-hand or right-hand wall following.
 * @param initial_heading Starting heading of the robot.
 */
void ecl_algo_maze_init(
    ecl_maze_solver_t *solver,
    ecl_maze_follow_t  rule,
    ecl_maze_heading_t initial_heading
);

/**
 * @brief Compute the next turn given the current wall sensors.
 *
 * Call this once per decision point (intersection / dead-end).
 * This does not update heading; call ecl_algo_maze_apply_turn() after the
 * robot physically completes the returned turn.
 *
 * @param solver      Initialised solver.
 * @param wall_left   true if a wall is detected to the left.
 * @param wall_front  true if a wall is detected straight ahead.
 * @param wall_right  true if a wall is detected to the right.
 * @return            Turn instruction to execute.
 */
ecl_maze_turn_t ecl_algo_maze_next_turn(
    ecl_maze_solver_t *solver,
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
void ecl_algo_maze_apply_turn(
    ecl_maze_solver_t *solver,
    ecl_maze_turn_t    turn
);

/**
 * @brief Return the current heading of the solver.
 */
ecl_maze_heading_t ecl_algo_maze_get_heading(
    const ecl_maze_solver_t *solver
);

#ifdef __cplusplus
}
#endif

#endif /* ECL_ALGO_MAZE_SOLVER_H */
