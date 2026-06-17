/*
 * maze_solver.c — Left-hand / right-hand wall-following maze solver
 *
 * The wall-following rule guarantees exit from any simply-connected maze
 * (no loops).  The robot keeps one hand in contact with a wall and follows it.
 *
 * Left-hand rule decision priority:  left > straight > right > U-turn
 * Right-hand rule decision priority: right > straight > left > U-turn
 *
 * The solver tracks an absolute cardinal heading (N/E/S/W encoded 0-3).  This
 * lets callers map the robot's logical position on a grid if needed.
 */
#include "ESP32EmbeddedCommonLib/algo/maze_solver.h"

#include <stddef.h>

/* ── Private helpers ─────────────────────────────────────────────────────── */

/* Rotate heading 90° left (CCW).
 * N(0)→W(3)→S(2)→E(1)→N(0) — subtracting 1 mod 4, using +3 to avoid negatives. */
static ecl_maze_heading_t ecl_algo_maze_heading_turn_left(ecl_maze_heading_t h)
{
    return (ecl_maze_heading_t)((h + 3) % 4);
}

/* Rotate heading 90° right (CW).
 * N(0)→E(1)→S(2)→W(3)→N(0) — adding 1 mod 4. */
static ecl_maze_heading_t ecl_algo_maze_heading_turn_right(ecl_maze_heading_t h)
{
    return (ecl_maze_heading_t)((h + 1) % 4);
}

/* Rotate heading 180° (U-turn) — adding 2 mod 4. */
static ecl_maze_heading_t ecl_algo_maze_heading_turn_around(ecl_maze_heading_t h)
{
    return (ecl_maze_heading_t)((h + 2) % 4);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

/* Initialise the maze solver rule and starting cardinal heading. */
void ecl_algo_maze_init(
    ecl_maze_solver_t *solver,
    ecl_maze_follow_t  rule,
    ecl_maze_heading_t initial_heading)
{
    if (solver == NULL) return;
    solver->rule    = rule;
    solver->heading = initial_heading;
}

/* Choose the next wall-following turn from left/front/right wall readings. */
ecl_maze_turn_t ecl_algo_maze_next_turn(
    ecl_maze_solver_t *solver,
    bool wall_left,
    bool wall_front,
    bool wall_right)
{
    if (solver == NULL) return ECL_MAZE_TURN_NONE;

    ecl_maze_turn_t turn;

    if (solver->rule == ECL_MAZE_FOLLOW_LEFT) {
        /*
         * Left-hand rule priority: left > straight > right > U-turn
         */
        if (!wall_left) {
            turn = ECL_MAZE_TURN_LEFT;
        } else if (!wall_front) {
            turn = ECL_MAZE_TURN_NONE;
        } else if (!wall_right) {
            turn = ECL_MAZE_TURN_RIGHT;
        } else {
            turn = ECL_MAZE_TURN_AROUND; /* dead end */
        }
    } else {
        /*
         * Right-hand rule priority: right > straight > left > U-turn
         */
        if (!wall_right) {
            turn = ECL_MAZE_TURN_RIGHT;
        } else if (!wall_front) {
            turn = ECL_MAZE_TURN_NONE;
        } else if (!wall_left) {
            turn = ECL_MAZE_TURN_LEFT;
        } else {
            turn = ECL_MAZE_TURN_AROUND; /* dead end */
        }
    }

    return turn;
}

/* Apply a completed turn to the solver's stored absolute heading. */
void ecl_algo_maze_apply_turn(
    ecl_maze_solver_t *solver,
    ecl_maze_turn_t    turn)
{
    if (solver == NULL) return;

    switch (turn) {
        case ECL_MAZE_TURN_LEFT:
            solver->heading = ecl_algo_maze_heading_turn_left(solver->heading);
            break;
        case ECL_MAZE_TURN_RIGHT:
            solver->heading = ecl_algo_maze_heading_turn_right(solver->heading);
            break;
        case ECL_MAZE_TURN_AROUND:
            solver->heading = ecl_algo_maze_heading_turn_around(solver->heading);
            break;
        case ECL_MAZE_TURN_NONE:
        default:
            break;
    }
}

/* Return the solver's current cardinal heading, defaulting north on NULL. */
ecl_maze_heading_t ecl_algo_maze_get_heading(
    const ecl_maze_solver_t *solver)
{
    if (solver == NULL) return ECL_MAZE_NORTH;
    return solver->heading;
}
