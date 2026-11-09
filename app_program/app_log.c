/*
 *	app_log.c
 *	See app_log.h.  Writers are tasks (any priority); the reader is the
 *	Wi-Fi task.  The ring is guarded by a short interrupt-disabled copy,
 *	so no task ever blocks on it.
 */

#include <stdarg.h>
#include "app.h"
#include "app_log.h"
#include <tm/tmonitor.h>

IMPORT INT tm_vsnprintf(UB *str, INT max, const UB *format, va_list ap);

LOCAL UB	ring[APP_LOG_LINES][APP_LOG_LEN];
LOCAL UH	ring_len[APP_LOG_LINES];
LOCAL UINT	head;		/* next slot to write */
LOCAL UINT	count;		/* lines waiting      */
EXPORT volatile UW app_log_dropped;
EXPORT volatile UW app_log_quiet;

LOCAL INT put_uint(UB *p, UW v)
{
	UB	tmp[10];
	INT	n = 0;
	INT	i;

	do {
		tmp[n++] = (UB)('0' + (v % 10U));
		v /= 10U;
	} while (v != 0U);
	for (i = 0; i < n; i++) {
		p[i] = tmp[n - 1 - i];
	}
	return n;
}

EXPORT INT app_log(const UB *format, ...)
{
	UB	text[APP_LOG_LEN];	/* the only big buffer: tasks have 2 KB */
	UB	pre[12];
	va_list	ap;
	INT	n;
	INT	s;
	INT	e;
	INT	k;
	INT	i;
	UINT	slot;
	UINT	imask;
	BOOL	noise;

	va_start(ap, format);
	n = tm_vsnprintf(text, (INT)sizeof(text), format, ap);
	va_end(ap);

	/* Dashboard copy: "uptime_ms text", without leading/trailing newlines */
	s = 0;
	while ((s < n) && ((text[s] == '\n') || (text[s] == '\r'))) {
		s++;
	}
	e = n;
	while ((e > s) && ((text[e - 1] == '\n') || (text[e - 1] == '\r'))) {
		e--;
	}
	if (e <= s) {
		(void)tm_putstring(text);	/* blank line: console only */
		return n;
	}
	noise = (((e - s) >= 3) && (text[s] == 'i') && (text[s + 1] == 'r') && (text[s + 2] == ':')) ||
		(((e - s) >= 4) && (text[s] == 'e') && (text[s + 1] == 'n') && (text[s + 2] == 'c') && (text[s + 3] == ':'));
	if (noise && (app_log_quiet != 0U)) {
		return n;
	}
	(void)tm_putstring(text);	/* console exactly as tm_printf */
	k = put_uint(pre, (UW)app_time_ms());
	pre[k++] = ' ';

	DI(imask);
	if (noise && (count >= APP_LOG_NOISE_MAX)) {
		EI(imask);
		return n;	/* sensor chatter: console only while backed up */
	}
	slot = head;
	for (i = 0; i < k; i++) {
		ring[slot][i] = pre[i];
	}
	while ((s < e) && (i < APP_LOG_LEN)) {
		ring[slot][i++] = text[s++];
	}
	ring_len[slot] = (UH)i;
	head = (head + 1U) % APP_LOG_LINES;
	if (count < APP_LOG_LINES) {
		count++;
	} else {
		app_log_dropped++;	/* oldest line overwritten */
	}
	EI(imask);
	return n;
}

EXPORT INT app_log_peek(UB *buf, INT max)
{
	UINT	imask;
	UINT	tail;
	INT	len;
	INT	i;

	DI(imask);
	if (count == 0U) {
		EI(imask);
		return 0;
	}
	tail = (head + APP_LOG_LINES - count) % APP_LOG_LINES;
	len = (INT)ring_len[tail];
	if (len > max) {
		len = max;
	}
	for (i = 0; i < len; i++) {
		buf[i] = ring[tail][i];
	}
	EI(imask);
	return len;
}

EXPORT void app_log_pop(void)
{
	UINT	imask;

	DI(imask);
	if (count > 0U) {
		count--;
	}
	EI(imask);
}
