/*
 *	app_log.h
 *	Console log that is also sent to the dashboard (MQTT topic
 *	is11/car/log).  Use app_log() exactly like tm_printf().
 *
 *	Lines are kept in a RAM ring (newest APP_LOG_LINES) until the MQTT
 *	link takes them, so the boot messages (self-test, calibration)
 *	still reach the dashboard when Wi-Fi connects later.
 */

#ifndef APP_LOG_H
#define APP_LOG_H

#include <tk/tkernel.h>

#define APP_LOG_LINES	64	/* ring size                        */
#define APP_LOG_NOISE_MAX 8	/* ir:/enc: lines are queued only while
				   fewer than this wait, so they never push
				   boot messages out while Wi-Fi is down */
#define APP_LOG_LEN	200	/* bytes per line incl. "uptime "   */

#ifndef app_log	/* the PC test harness maps it to printf */
IMPORT INT  app_log(const UB *format, ...);
#endif
/* MQTT side (Wi-Fi task): copy the oldest line, then pop it once it
   was published.  Returns the length, 0 when the ring is empty. */
IMPORT INT  app_log_peek(UB *buf, INT max);
IMPORT void app_log_pop(void);
IMPORT volatile UW app_log_dropped;
/* 1: drop ir:/enc: chatter entirely (sensor bench prompts stay readable) */
IMPORT volatile UW app_log_quiet;

#endif /* APP_LOG_H */
