/*
 *	subsystem1.c
 *	Subsystem 1 -- Communication: creates the Telemetry Queue and the
 *	outbox, and starts comms_task.
 */

#include "subsystem1.h"

#define COMMS_STACK_SIZE	2048

EXPORT ID	mbf_telemetry;		/* IF-01 */
EXPORT ID	mbf_outbox;
EXPORT ID	tid_comms;

EXPORT ER subsystem1_init(void)
{
	T_CMBF	cmbf = {0};

	/* IF-01 Telemetry Queue: a few complete snapshots */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)(S1_TLM_QUEUE_MSGS * (sizeof(TLM_SNAPSHOT) + sizeof(INT)));
	cmbf.maxmsz = (INT)sizeof(TLM_SNAPSHOT);
	mbf_telemetry = tk_cre_mbf(&cmbf);
	if (mbf_telemetry < E_OK) {
		return mbf_telemetry;
	}

	/* Offline buffer: the 16 newest encoded frames */
	cmbf.bufsz  = (SZ)(S1_OUTBOX_FRAMES * (WIRE_TLM_LEN + (INT)sizeof(INT)));
	cmbf.maxmsz = WIRE_TLM_LEN;
	mbf_outbox = tk_cre_mbf(&cmbf);
	return (mbf_outbox < E_OK) ? mbf_outbox : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	tid_comms = app_start_task_stk(comms_task, S1_PRI_COMMS, 0,
				       COMMS_STACK_SIZE, "comms_task");
	return (tid_comms < E_OK) ? tid_comms : E_OK;
}
