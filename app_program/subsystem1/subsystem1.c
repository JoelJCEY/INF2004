/*
 *	subsystem1.c
 *	Subsystem 1 -- creates the Telemetry Queue and the MQTT outbox, starts
 *	comms_task.
 */

#include "subsystem1.h"

EXPORT ID	tid_comms;
EXPORT ID	mbf_telemetry;
EXPORT ID	mbf_outbox;

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
	cmbf.maxmsz = S1_PACKED_BYTES;
	cmbf.bufsz = S1_OUTBOX_DEPTH * (S1_PACKED_BYTES + 8);
	mbf_outbox = tk_cre_mbf(&cmbf);
	return (mbf_outbox < E_OK) ? mbf_outbox : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	tid_comms = app_start_task(comms_task, S1_PRI_COMMS, 0, "comms_task");
	return (tid_comms < E_OK) ? tid_comms : E_OK;
}
