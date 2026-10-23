/*
 *	subsystem1.h
 *	Subsystem 1 -- Wi-Fi/MQTT communication and telemetry (Buddy 1).
 *
 *	COMPLETE VERSION.
 *	  - ctrl_task sends one plain snapshot every 200 ms on the Telemetry
 *	    Queue.  comms_task (lowest application priority) packs it into
 *	    the versioned binary layout below and puts it in the MQTT outbox.
 *	  - The kernel's Wi-Fi service task publishes the outbox every 200 ms
 *	    (QoS 0) and passes received commands (QoS 1) to app hooks in
 *	    subsystem1/mqtt_hooks.c.  Those hooks run in the service task:
 *	    they never block (TMO_POL) and hold no mutex (NFR3).
 *	  - The outbox keeps the 16 most recent packed snapshots while
 *	    offline (oldest dropped and counted).
 *	  - Wi-Fi loss sets the WIFI LOSS fault bit: reported only, the car
 *	    keeps driving.
 *	NEEDS a small change to the shared MQTT client (lib/liblwip/sysdepend/
 *	pico_rp2040/mqtt_utk.c): see the patch notes in FULL_VERSION.md.
 *
 *	Packed telemetry v1 (little-endian bit stream, LSB first; field
 *	order and widths are the contract with the laptop decoder,
 *	tools/is11_decode.py):
 *	   8 version (=1)        4 state          3 barcode cmd    1 hump
 *	   8 fault mask          3 IR black L/C/R 1 line lost      12 line err +2048
 *	  11 speed L +1024      11 speed R +1024 16 distance cm   24 enc L (low bits)
 *	  24 enc R (low bits)    8 hump peak mm   8 obstacle cm (255 = none)
 *	   6 obstacle width cm   2 bypass side    3 motion event   1 watchdog reset
 *	   8 sequence            8 letter (ASCII)
 *	 = 170 bits = 22 bytes (design estimate was 14-16: encoder counts and
 *	   the decoded letter were added; measure in Week 13).
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

#define S1_PRI_COMMS		15
#define S1_PRINT_CSV		0	/* 1 = also print the plain snapshot */
#define S1_TELEMETRY_DEPTH	4	/* plain snapshots ctrl -> comms */
#define S1_OUTBOX_DEPTH		16	/* packed snapshots kept offline */

#define S1_PACKED_VERSION	1U
#define S1_PACKED_BYTES		22

/* Plain snapshot from ctrl_task (internal; plain C types). */
typedef struct {
	UW	t_ms;
	UW	seq;
	W	state;
	W	last_barcode;
	W	last_letter;
	UW	fault_mask;
	W	speed_l_mm_s;
	W	speed_r_mm_s;
	W	enc_l;
	W	enc_r;
	W	distance_mm;
	W	line_err;
	UW	line_flags;
	W	hump_active;
	W	hump_peak_mm;
	W	obstacle_mm;	/* -1 = nothing within ~1 m */
	W	obstacle_width_mm;
	W	bypass_side;
	W	motion_event;
	W	watchdog_reset;
	W	wifi_up;
} S1_TELEMETRY;

IMPORT ID	mbf_telemetry;	/* Telemetry Queue (owned by comms_task) */
IMPORT ID	mbf_outbox;	/* packed bytes; received by the MQTT hook */

/* Pack / unpack (pure functions, tested in subsystem1_test.c). */
IMPORT INT	s1_pack(const S1_TELEMETRY *p_t, UB out[S1_PACKED_BYTES]);
IMPORT void	s1_unpack(const UB in[S1_PACKED_BYTES], S1_TELEMETRY *p_t);

/* Hooks called by the shared MQTT client (in the Wi-Fi service task). */
IMPORT INT	app_mqtt_take_telemetry(UB *p_buf, INT max_len);
IMPORT void	app_mqtt_on_command(const UB *p_data, INT len);

IMPORT volatile UW	s1_rx_count;
IMPORT volatile UW	s1_dropped;
IMPORT volatile UW	s1_cmds_ok;
IMPORT volatile UW	s1_cmds_bad;
IMPORT volatile BOOL	s1_wifi_up;

IMPORT ID	tid_comms;
IMPORT void	comms_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM1_H */
