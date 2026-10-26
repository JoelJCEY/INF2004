/*
 *	subsystem1.h
 *	Subsystem 1 -- Wi-Fi communication and telemetry (Buddy 1).
 *
 *	comms_task receives snapshots from ctrl_task, packs a versioned binary
 *	frame and gives it to the Wi-Fi service task through a message buffer.
 *	Only that service task calls MQTT. USB CSV is retained for debugging.
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
#define S1_MQTT_FRAME_SIZE	36	/* version 1; see telemetry_codec.h */
#define S1_MQTT_TX_DEPTH	2	/* recent snapshots while network task is busy */

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
IMPORT ID	mbf_mqtt_tx;	/* packed frames for the Wi-Fi service */
IMPORT ID	mbf_remote_stop;	/* remote STOP, separate from other commands */
IMPORT ID	mbf_remote_other;	/* remote START and RESET */

/* Called by the Wi-Fi service task, without blocking. Returns frame length
 * or zero when no frame is waiting. No lwIP function runs in comms_task. */
IMPORT INT	s1_mqtt_take_frame(UB *out, INT capacity);
/* Called only from the Wi-Fi service task's MQTT receive callback. */
IMPORT INT	s1_mqtt_submit_command(const UB *data, INT length);

/* Read by tests.  Written only by comms_task. */
IMPORT volatile UW	s1_rx_count;
IMPORT volatile BOOL	s1_wifi_up;

IMPORT ID	tid_comms;
IMPORT void	comms_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM1_H */
