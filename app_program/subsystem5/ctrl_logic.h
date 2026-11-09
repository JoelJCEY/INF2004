/*
 *	ctrl_logic.h
 *	Subsystem 5 -- pure parts of the Vehicle Controller (no kernel
 *	calls), unit-tested in subsystem5_test.c: which states accept which
 *	command, and how a profile becomes a list of bypass legs.
 */

#ifndef CTRL_LOGIC_H
#define CTRL_LOGIC_H

#include "subsystem5.h"

/* Is the state an "active manoeuvre" one (STOP / impact apply)? */
IMPORT BOOL ctrl_is_active(VEHICLE_STATE s);
/* Next state for a START / STOP / RESET in state s, or s if rejected. */
IMPORT VEHICLE_STATE ctrl_on_command(VEHICLE_STATE s, REMOTE_CMD c);
/* Next state for newly reported faults (FLT_xxx) in state s. */
IMPORT VEHICLE_STATE ctrl_on_faults(VEHICLE_STATE s, UINT faults);

/* ---------------------------------------------------------------------
 *  Manoeuvre steps executed one at a time through the motion API
 * --------------------------------------------------------------------- */
typedef enum {
	STEP_TURN	= 1,	/* arg: signed degrees, + = left            */
	STEP_MOVE	= 2,	/* arg: signed mm                           */
	STEP_SEARCH	= 3,	/* arg: mm; ends early when the line is seen */
	STEP_SEARCH_TURN = 4,	/* arg: signed degrees; ends early on the line */
	STEP_PASS	= 5	/* arg: max mm forward; ends early when scan_task
				   reports the obstacle is behind (31 Oct)   */
} STEP_OP;

typedef struct {
	UB	op;
	W	arg;
} CTRL_STEP;

#define CTRL_MAX_STEPS		10
#define CTRL_SEARCH_EXTRA_MM	150	/* search past the expected line */
#define CTRL_REVERSE_MM		100	/* before a rescan               */
#define CTRL_BYPASS_MM_S	80	/* 4 Nov: straight bypass legs (reverse,
					   sideways, pass).  Turns and the line
					   search stay at the normal 40 mm/s: the
					   turn calibration was measured there and
					   the search must stop on the line   */
#define CTRL_SENSOR_AHEAD_MM	70	/* IR sensors ahead of the wheel axle
					   (31 Oct estimate: measure the car;
					   too small is safer than too big)   */
#define CTRL_LAND_FIXED_DEG	30	/* first part of the final turn: the
					   sensors are leaving the line      */
#define CTRL_LAND_SEARCH_DEG	90	/* rest of it: stops on the line     */
#define CTRL_PASS_MAX_EXTRA_MM	0	/* pass leg may run this much past the
					   estimate (4 Nov: 0, was 100 then 250;
					   the car went far past the obstacle
					   when the end was not seen)        */
#define CTRL_BYPASS_BACKUP_MM	100	/* reverse before going around (28 Oct):
					   at a 10 cm stop the car cannot turn
					   without touching the obstacle */
#define CTRL_BARCODE_OFFSET_MM	0	/* execution point: to confirm (FR2) */

/* Rectangular detour (Version A approach, 5): turn out, move across,
   turn parallel, pass, turn back, search for the line, re-align. */
IMPORT INT ctrl_plan_bypass(UB side, W offset_mm, W depth_mm, CTRL_STEP *steps);
/* Barcode command -> steps (0 steps: just keep following) */
IMPORT INT ctrl_plan_barcode(BARCODE_CMD cmd, CTRL_STEP *steps);

/* ---------------------------------------------------------------------
 *  Barcode carried out at a cross-junction (demo rule, IF-15).
 *  The car pivots on its wheel axle, so the axle must be over the
 *  middle of the junction before it turns.  line_task reports where the
 *  middle of the crossing line was (NAV_JUNCTION_CROSS path_mm); by then
 *  the sensor row is already past it, so the car moves
 *      CTRL_SENSOR_AHEAD_MM - (distance already travelled past the middle)
 *  (backwards if that is negative), the same axle distance the bypass
 *  landing uses (measure it once, S5-M07).  Then it lands on the new line
 *  like the bypass does: a fixed turn short of the target, then a search
 *  turn that stops as soon as the sensors see the line, so a few degrees
 *  of turn error do not leave the car off the line.
 * --------------------------------------------------------------------- */
#define CTRL_JUNCTION_DEADBAND_MM 5	/* smaller corrections are skipped */
#define CTRL_JUNCTION_SEARCH_DEG 60	/* fixed turn stops this much short of
					   90/180, then a search turn of twice
					   this: the line is found within
					   +-CTRL_JUNCTION_SEARCH_DEG/2       */

/* Movement -> steps at a junction.  past_mm: how far the SENSOR ROW is
   already past the middle of the crossing line.  GO STRAIGHT and
   unrecognised letters give 0 steps (keep following straight on). */
IMPORT INT ctrl_plan_junction(BARCODE_CMD cmd, W past_mm, CTRL_STEP *steps);

#endif /* CTRL_LOGIC_H */
