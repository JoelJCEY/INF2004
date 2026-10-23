/*
 *	mqtt_hooks.c
 *	Subsystem 1 -- the two functions the shared MQTT client calls.
 *
 *	They run inside the kernel's Wi-Fi service task (priority 4), so:
 *	never block (TMO_POL only), hold no mutex, do no printing in the
 *	normal path, and touch no hardware.
 *
 *	Command payload v1 (topic is11/cmd, QoS 1), 2 bytes:
 *	  byte 0: version = 1
 *	  byte 1: bits 0-1 = command (1 START, 2 STOP, 3 RESET), bits 2-7 = 0
 *	Anything else is rejected and counted.
 */

#include "subsystem1.h"
#include "../subsystem5/subsystem5.h"

EXPORT volatile UW	s1_cmds_ok = 0U;
EXPORT volatile UW	s1_cmds_bad = 0U;

/* Returns the number of bytes copied (0 = nothing to send). */
EXPORT INT app_mqtt_take_telemetry(UB *p_buf, INT max_len)
{
	UB	tmp[S1_PACKED_BYTES];
	INT	n;

	if (max_len < S1_PACKED_BYTES) {
		return 0;
	}
	n = tk_rcv_mbf(mbf_outbox, tmp, TMO_POL);
	if (n != S1_PACKED_BYTES) {
		return 0;
	}
	for (n = 0; n < S1_PACKED_BYTES; n++) {
		p_buf[n] = tmp[n];
	}
	return S1_PACKED_BYTES;
}

EXPORT void app_mqtt_on_command(const UB *p_data, INT len)
{
	REMOTE_MSG	msg;
	UB		c;

	if ((len != 2) || (p_data[0] != 1U) || ((p_data[1] & 0xFCU) != 0U)) {
		s1_cmds_bad++;
		return;
	}
	c = (UB)(p_data[1] & 0x03U);
	if (c == 0U) {
		s1_cmds_bad++;
		return;
	}
	msg.cmd = (REMOTE_CMD)c;
	if (s5_post(mbf_remote, &msg, (INT)sizeof(msg), FLG_REMOTE) == E_OK) {
		s1_cmds_ok++;
	} else {
		s1_cmds_bad++;
	}
}
