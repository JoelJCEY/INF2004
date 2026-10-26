/*
 *	subsystem1.c
 *	Subsystem 1 -- creates the Telemetry Queue and starts comms_task.
 */

#include "subsystem1.h"
#include "command_codec.h"
#include "../subsystem5/subsystem5.h"

EXPORT ID	tid_comms;
EXPORT ID	mbf_telemetry;
EXPORT ID	mbf_mqtt_tx;
EXPORT ID	mbf_remote_stop;
EXPORT ID	mbf_remote_other;

EXPORT INT s1_mqtt_submit_command(const UB *data, INT length)
{
	S1_COMMAND command;
	ID destination;
	ER result;
	UB one_byte;

	command = s1_decode_command(data, (length >= 0) ? (unsigned int)length : 0U);
	if (command == S1_CMD_INVALID) {
		return 0;
	}
	destination = (command == S1_CMD_STOP) ? mbf_remote_stop : mbf_remote_other;
	if (destination <= 0) {
		return 0;
	}
	one_byte = (UB)command;
	result = tk_snd_mbf(destination, &one_byte, 1, TMO_POL);
	/* One pending STOP is enough; repeating it must not fail. */
	if (result < E_OK &&
	    !(command == S1_CMD_STOP && result == E_TMOUT)) {
		return 0;
	}
	(void)tk_set_flg(flg_ctrl, FLG_REMOTE);
	return 1;
}

EXPORT INT s1_mqtt_take_frame(UB *out, INT capacity)
{
	if ((out == NULL) || (capacity < S1_MQTT_FRAME_SIZE) ||
	    (mbf_mqtt_tx <= 0)) {
		return 0;
	}
	return (tk_rcv_mbf(mbf_mqtt_tx, out, TMO_POL) == S1_MQTT_FRAME_SIZE)
		? S1_MQTT_FRAME_SIZE : 0;
}

EXPORT ER subsystem1_init(void)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.maxmsz = (INT)sizeof(S1_TELEMETRY);
	cmbf.bufsz = S1_TELEMETRY_DEPTH * ((INT)sizeof(S1_TELEMETRY) + 8);
	mbf_telemetry = tk_cre_mbf(&cmbf);
	if (mbf_telemetry < E_OK) {
		return mbf_telemetry;
	}
	cmbf.maxmsz = S1_MQTT_FRAME_SIZE;
	cmbf.bufsz = S1_MQTT_TX_DEPTH * (S1_MQTT_FRAME_SIZE + 8);
	mbf_mqtt_tx = tk_cre_mbf(&cmbf);
	if (mbf_mqtt_tx < E_OK) {
		return mbf_mqtt_tx;
	}
	cmbf.maxmsz = 1;
	cmbf.bufsz = 16; /* one STOP plus message-buffer bookkeeping */
	mbf_remote_stop = tk_cre_mbf(&cmbf);
	if (mbf_remote_stop < E_OK) {
		return mbf_remote_stop;
	}
	cmbf.bufsz = 4 * (1 + 8);
	mbf_remote_other = tk_cre_mbf(&cmbf);
	return (mbf_remote_other < E_OK) ? mbf_remote_other : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	tid_comms = app_start_task(comms_task, S1_PRI_COMMS, 0, "comms_task");
	return (tid_comms < E_OK) ? tid_comms : E_OK;
}
