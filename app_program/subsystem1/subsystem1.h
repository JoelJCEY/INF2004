/*
 *	subsystem1.h
 *	Subsystem 1 -- Wi-Fi communication and telemetry (Buddy 1).
 *
 *	BASIC VERSION.
 *	  - comms_task (lowest application priority) receives one telemetry
 *	    snapshot every 200 ms from ctrl_task through the Telemetry Queue
 *	    and prints it as one CSV line on the console.
 *	  - It watches the Wi-Fi link (read-only status; student tasks may
 *	    never call cyw43_/lwIP/MQTT functions themselves).
 *	  - No change to the shared MQTT library: building with
 *	    NET=lwip MQTT=1 still runs the kernel's own heartbeat.
 *	The complete version packs the snapshot into a versioned binary
 *	payload and publishes it over MQTT, and accepts START/STOP/RESET.
 *
 *	The snapshot uses plain C types on purpose: this is the
 *	"baseline-plain-types" build for the Week 13 before/after comparison.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

#define S1_PRI_COMMS		15	/* lowest application priority (NFR3) */
#define S1_PRINT_CSV		1	/* 0 = no telemetry lines on the console */
#define S1_TELEMETRY_DEPTH	16	/* snapshots buffered (offline buffer) */

/* Telemetry snapshot (FR5).  Written only by ctrl_task. */
typedef struct {
	UW	t_ms;		/* time since boot */
	UW	seq;
	W	state;		/* VEHICLE_STATE */
	W	last_barcode;	/* BARCODE_CMD */
	W	last_letter;	/* ASCII */
	UW	fault_mask;
	W	speed_l_mm_s;
	W	speed_r_mm_s;
	UW	enc_l;
	UW	enc_r;
	W	distance_mm;
	W	line_err;
	UW	line_flags;
	W	hump_active;
	W	hump_peak_mm;	/* highest hump of the run */
	W	obstacle_mm;	/* -1 = nothing within ~1 m */
	W	wifi_up;
} S1_TELEMETRY;

IMPORT ID	mbf_telemetry;	/* Telemetry Queue (owned by comms_task) */

/* Read by tests.  Written only by comms_task. */
IMPORT volatile UW	s1_rx_count;
IMPORT volatile BOOL	s1_wifi_up;

IMPORT ID	tid_comms;
IMPORT void	comms_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM1_H */
