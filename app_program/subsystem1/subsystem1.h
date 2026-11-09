/*
 *	subsystem1.h
 *	Subsystem 1 -- Communication (comms_task).
 *	Buddy 1: 2501327 (Daryl).  Statechart 6.1, assumption A1.
 *
 *	Owns: nothing on the radio itself.  The CYW43439, lwIP and MQTT
 *	belong to the kernel's Wi-Fi service task (TEAM.md section 3), so
 *	comms code that must touch MQTT runs INSIDE that task through the
 *	bridge in comms_bridge.c, using only TMO_POL kernel calls.
 *	comms_task holds no mutex and never blocks on the network (NFR3).
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "../subsystem5/subsystem5.h"	/* REMOTE_MSG (IF-03) */

#define S1_PRI_COMMS		APP_PRI_COMMS
#define S1_PERIOD_MS		200	/* telemetry every 200 ms (NFR4) */
#define S1_OUTBOX_FRAMES	16	/* offline buffer: 16 newest frames */

/* ---------------------------------------------------------------------
 *  IF-01  Telemetry Queue (ctrl_task -> comms_task)
 *  Message buffer mbf_telemetry, owned by subsystem 1.  ctrl_task is the
 *  ONLY sender and sends a finished snapshot with TMO_POL; when the
 *  queue is full the newest snapshot is dropped and counted (6.1).
 *  The message buffer copies the whole struct, so comms_task never sees
 *  a half-written snapshot (design 10.4).
 * --------------------------------------------------------------------- */

/* Packed status word: explicit shifts and masks, never C bit-fields
   (design 10.2, Barr-C 5.5b).  Writer: ctrl_task. */
#define TLM_STATE_SHIFT		0	/* 4 bits VEHICLE_STATE      */
#define TLM_STATE_MASK		0xFU
#define TLM_BARCODE_SHIFT	4	/* 3 bits BARCODE_CMD        */
#define TLM_BARCODE_MASK	0x7U
#define TLM_REMOTE_SHIFT	7	/* 2 bits REMOTE_CMD (last)  */
#define TLM_REMOTE_MASK		0x3U
#define TLM_LINEF_SHIFT		9	/* 4 bits LINE_F_xxx         */
#define TLM_LINEF_MASK		0xFU
#define TLM_SERVO_SHIFT		13	/* 5 bits servo index        */
#define TLM_SERVO_MASK		0x1FU
#define TLM_HUMP_SHIFT		18	/* 1 bit hump active         */
#define TLM_OBST_SHIFT		19	/* 1 bit obstacle profiled   */
#define TLM_WDT_SHIFT		20	/* 1 bit watchdog reboot     */
#define TLM_SIDE_SHIFT		21	/* 2 bits bypass side        */
#define TLM_SIDE_MASK		0x3U
#define TLM_FAILS_SHIFT		23	/* 2 bits bypass failures    */
#define TLM_FAILS_MASK		0x3U
#define TLM_MEVENT_SHIFT	25	/* 3 bits motion event       */
#define TLM_MEVENT_MASK		0x7U
#define TLM_LINK_SHIFT		28	/* 1 bit MQTT connected      */
#define TLM_FIELD(w, f)		(((w) >> TLM_##f##_SHIFT) & TLM_##f##_MASK)
#define TLM_BIT(w, f)		(((w) >> TLM_##f##_SHIFT) & 1U)

typedef struct {
	UW	status;		/* packed fields above             */
	UB	fault_mask;	/* FLT_xxx latched until RESET     */
	UB	letter;		/* last barcode letter (ASCII)     */
	UB	hump_count;
	UB	reserved;
	H	speed_l_mm_s;
	H	speed_r_mm_s;
	W	enc_l;
	W	enc_r;
	UW	path_mm;
	H	line_err;	/* -1000..+1000                    */
	UH	hump_peak_01mm;	/* highest this run                */
	UH	obs_width_mm;
	UH	obs_closest_mm;
	UW	uptime_ms;
	UH	tlm_dropped;	/* snapshots dropped at the queue  */
	UH	reserved2;
} TLM_SNAPSHOT;
APP_STATIC_ASSERT(sizeof(TLM_SNAPSHOT) == 40, tlm_snapshot_size);

#define S1_TLM_QUEUE_MSGS	4
IMPORT ID	mbf_telemetry;

/* ---------------------------------------------------------------------
 *  MQTT wire format v1 (design 10.2: versioned, explicit byte order,
 *  little-endian, independent of the compiler's struct layout).
 *  Laptop decoder: tools/is11_mqtt.py (same golden vectors as S1-U01).
 *
 *  Telemetry frame, topic MQTT_TOPIC_TLM, QoS 0, 36 bytes:
 *    0     version (=1)          1     frame seq (wraps)
 *    2-5   status word (u32)     6     fault mask
 *    7     barcode letter        8-9   speed L mm/s (i16)
 *    10-11 speed R mm/s (i16)    12-15 encoder L (i32)
 *    16-19 encoder R (i32)       20-23 path mm (u32)
 *    24-25 line error (i16)      26-27 hump peak 0.1 mm (u16)
 *    28-29 obstacle width mm     30-31 obstacle closest mm
 *    32-35 uptime ms (u32)
 *
 *  Command frame, topic MQTT_TOPIC_CMD, QoS 1, 3 bytes:
 *    0 version (=1)   1 command (bits 0-1 REMOTE_CMD, bits 2-7 zero)
 *    2 sender sequence
 *  Code 0 (RC_NONE), any other version, length or set bit 2-7 is
 *  rejected and counted.
 * --------------------------------------------------------------------- */
#define WIRE_VERSION		1U
#define WIRE_TLM_LEN		36
#define WIRE_CMD_LEN		3

IMPORT INT comms_encode_tlm(const TLM_SNAPSHOT *s, UB seq, UB *buf);
IMPORT ER  comms_decode_cmd(const UB *buf, INT len, REMOTE_MSG *out);

/* ---------------------------------------------------------------------
 *  Barcode map frame (IF-15), same topic MQTT_TOPIC_CMD, QoS 1,
 *  published RETAINED by the dashboard so the broker hands it to the
 *  car again after a reboot or Wi-Fi drop.  7 bytes:
 *    0 version (=1)   1 opcode WIRE_OP_MAP (0x10)   2 sender sequence
 *    3 letter for GO STRAIGHT   4 letter for U-TURN
 *    5 letter for TURN LEFT     6 letter for TURN RIGHT
 *  Letters: 'A'-'Z' or '0'-'9', all four different.  The length and the
 *  opcode keep it apart from the 3-byte START/STOP/RESET frame, which
 *  comms_decode_cmd() still rejects for any other shape.
 * --------------------------------------------------------------------- */
#define WIRE_MAP_LEN		7
#define WIRE_OP_MAP		0x10U

/* Barcode test mode frame, same topic, QoS 1, NOT retained.  4 bytes:
     0 version (=1)   1 opcode WIRE_OP_BC_TEST (0x11)   2 sender seq
     3 0 = off, 1 = on */
#define WIRE_TEST_LEN		4
#define WIRE_OP_BC_TEST		0x11U
/* E_OK and *on (0/1) for a valid test frame, else E_PAR */
IMPORT ER  comms_decode_bc_test(const UB *buf, INT len, UW *on);

/* E_OK and *map (APP_BARCODE_MAP layout) for a valid frame, else E_PAR */
IMPORT ER  comms_decode_map(const UB *buf, INT len, UW *map);
/* TRUE if buf has the shape of a map frame (version, length, opcode) */
IMPORT BOOL comms_is_map_frame(const UB *buf, INT len);

/* Link state for telemetry (writer: comms_task) */
typedef enum {
	CS_DISCONNECTED		= 0,
	CS_WIFI_CONNECTING	= 1,
	CS_BROKER_CONNECTING	= 2,
	CS_ONLINE		= 3,
	CS_WAITING_TO_RETRY	= 4
} COMMS_STATE;
IMPORT volatile UW comms_state_word;	/* COMMS_STATE */

/* ---------------------------------------------------------------------
 *  Private to subsystem 1
 * --------------------------------------------------------------------- */
IMPORT ID	tid_comms;
IMPORT ID	mbf_outbox;		/* encoded frames waiting for MQTT */
IMPORT volatile UW comms_rx_rejected;
IMPORT volatile UW comms_mqtt_connected;	/* writer: Wi-Fi task via bridge */
IMPORT volatile UW comms_outbox_dropped;
IMPORT void	comms_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM1_H */
