/*
 *	comms_bridge.h
 *	Subsystem 1 -- functions the MQTT client (lib/liblwip/sysdepend/
 *	pico_rp2040/mqtt_utk.c) calls from INSIDE the kernel's Wi-Fi service
 *	task.  Plain C types only and no includes, because mqtt_utk.c mixes
 *	lwIP headers that must not meet tk/tkernel.h (size_t clash).
 *
 *	Every function returns at once: kernel calls inside use TMO_POL, so
 *	a slow application can never stall the radio (NFR3).
 *	mqtt_utk.c has weak no-op versions, so the library builds alone.
 */

#ifndef COMMS_BRIDGE_H
#define COMMS_BRIDGE_H

/* Copy the oldest encoded telemetry frame into buf.  Returns its length,
   or 0 when the outbox is empty. */
int  comms_bridge_take_frame(unsigned char *buf, int max);

/* A complete message arrived on the command topic (raw bytes). */
void comms_bridge_on_command(const unsigned char *data, int len);

/* MQTT session up (1) or down (0). */
void comms_bridge_set_connected(int connected);

/* Dashboard log: copy the oldest line (0 = none), pop it once published */
int  comms_bridge_peek_log(unsigned char *buf, int max);
void comms_bridge_pop_log(void);

/* Live IR text for the dashboard; returns length, 0 = nothing */
int  comms_bridge_ir_text(unsigned char *buf, int max);
/* Ultrasonic live text / last sweep (0 = nothing new) */
int  comms_bridge_us_text(unsigned char *buf, int max);
int  comms_bridge_sweep_text(unsigned char *buf, int max);
/* Last barcode read widths (0 = nothing new), topic is11/car/bc */
int  comms_bridge_bc_text(unsigned char *buf, int max);

#endif /* COMMS_BRIDGE_H */
