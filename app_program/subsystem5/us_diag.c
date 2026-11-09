/*
 *	us_diag.c
 *	Ultrasonic diagnostic (build with APP_US_DIAG 1 in app.h).
 *
 *	Runs after start-up, repeats every few seconds and prints a verdict
 *	on the console and the dashboard car log ("usdiag:" lines):
 *	  1. ECHO pin (GP1) with nothing triggered: pulled down / up -> is a
 *	     wire there at all, is it stuck high, is it toggling by itself
 *	     (= connected to some other signal, e.g. an encoder)
 *	  2. 10 pings, timed by polling: did ECHO rise, how long after the
 *	     trigger, how long was the pulse -> distance
 *	  3. the same with TRIG/ECHO swapped (GP1 drives, GP0 listens)
 *	  4. a verdict in plain words
 *	Hold a hand at about 20 cm and then take it away: the distance must
 *	change.  The car never drives in this mode (scan_task is held).
 */

#include "../app.h"
#include "subsystem5.h"
#include "drv_ultrasonic.h"

#if APP_US_DIAG

#define PINGS		10
#define WAIT_RISE_US	5000U	/* echo must start within 5 ms */
#define WAIT_FALL_US	60000U	/* HC-SR04 gives up after ~38 ms */

typedef struct {
	INT	rose;		/* pings where ECHO went high      */
	INT	fell;		/* ... and came back low            */
	UW	rise_min, rise_max;	/* trigger -> rise, us          */
	UW	w[PINGS];	/* pulse widths, us (0 = none)      */
	INT	nw;
} PING_STATS;

LOCAL void pad_pull(UINT pin, BOOL up, BOOL down)
{
	clr_w(GPIO(pin), GPIO_PUE | GPIO_PDE);
	if (up) {
		set_w(GPIO(pin), GPIO_PUE);
	}
	if (down) {
		set_w(GPIO(pin), GPIO_PDE);
	}
}

/* Count level changes on a pin for ms milliseconds (nothing triggered) */
LOCAL INT count_edges(UINT pin, UINT ms, UINT *high_us_pct)
{
	UW	t0 = app_time_us();
	UW	last = gpio_get_val(pin);
	UW	v;
	INT	edges = 0;
	UW	samples = 0, highs = 0;

	while ((app_time_us() - t0) < (ms * 1000U)) {
		v = gpio_get_val(pin);
		samples++;
		if (v != 0U) {
			highs++;
		}
		if (v != last) {
			edges++;
			last = v;
		}
	}
	*high_us_pct = (samples > 0U) ? (UINT)((highs * 100U) / samples) : 0U;
	return edges;
}

/* One polled ping: trig pin pulses, echo pin is timed */
LOCAL void one_ping(UINT trig, UINT echo, PING_STATS *s)
{
	UW	t0, tr, tf;

	gpio_set_val(trig, 1);
	app_delay_us(10);
	gpio_set_val(trig, 0);
	t0 = app_time_us();

	while (gpio_get_val(echo) == 0U) {
		if ((app_time_us() - t0) > WAIT_RISE_US) {
			s->w[s->nw++] = 0;
			return;			/* never rose */
		}
	}
	tr = app_time_us();
	s->rose++;
	if ((tr - t0) < s->rise_min) s->rise_min = tr - t0;
	if ((tr - t0) > s->rise_max) s->rise_max = tr - t0;

	while (gpio_get_val(echo) != 0U) {
		if ((app_time_us() - tr) > WAIT_FALL_US) {
			s->w[s->nw++] = WAIT_FALL_US;	/* stuck high */
			return;
		}
	}
	tf = app_time_us();
	s->fell++;
	s->w[s->nw++] = tf - tr;
}

LOCAL void run_pings(UINT trig, UINT echo, PING_STATS *s)
{
	INT	i;

	s->rose = 0;
	s->fell = 0;
	s->nw = 0;
	s->rise_min = 0xFFFFFFFFU;
	s->rise_max = 0;
	for (i = 0; i < PINGS; i++) {
		one_ping(trig, echo, s);
		tk_dly_tsk(60);
	}
}

LOCAL UW median_mm(const PING_STATS *s, UW *lo_mm, UW *hi_mm)
{
	UW	v[PINGS];
	INT	n = 0, i, j;
	UW	t;

	for (i = 0; i < s->nw; i++) {
		if ((s->w[i] > 0U) && (s->w[i] < WAIT_FALL_US)) {
			v[n++] = s->w[i];
		}
	}
	if (n == 0) {
		*lo_mm = 0;
		*hi_mm = 0;
		return 0;
	}
	for (i = 1; i < n; i++) {
		t = v[i];
		for (j = i; (j > 0) && (v[j - 1] > t); j--) {
			v[j] = v[j - 1];
		}
		v[j] = t;
	}
	*lo_mm = drv_us_dur_to_mm(v[0]);
	*hi_mm = drv_us_dur_to_mm(v[n - 1]);
	return drv_us_dur_to_mm(v[n / 2]);
}

LOCAL void diag_round(INT round)
{
	PING_STATS	s, sx;
	UINT		pct_dn, pct_up, pct_idle;
	INT		e_dn, e_up, e_idle;
	UW		med, lo, hi, medx, lox, hix;

	app_log((UB*)"usdiag: ===== round %d (TRIG GP%d, ECHO GP%d) =====\n",
		round, US_TRIG_PIN, US_ECHO_PIN);

	/* 1. ECHO with nothing triggered */
	gpio_set_pin(US_TRIG_PIN, GPIO_MODE_OUT);
	gpio_set_val(US_TRIG_PIN, 0);
	gpio_set_pin(US_ECHO_PIN, GPIO_MODE_IN);
	pad_pull(US_ECHO_PIN, FALSE, TRUE);
	tk_dly_tsk(5);
	e_dn = count_edges(US_ECHO_PIN, 200U, &pct_dn);
	pad_pull(US_ECHO_PIN, TRUE, FALSE);
	tk_dly_tsk(5);
	e_up = count_edges(US_ECHO_PIN, 200U, &pct_up);
	pad_pull(US_ECHO_PIN, FALSE, TRUE);
	tk_dly_tsk(5);
	e_idle = count_edges(US_ECHO_PIN, 500U, &pct_idle);
	app_log((UB*)"usdiag: ECHO idle: pull-down %d%% high, pull-up %d%% high, %d changes in 0.5 s without any trigger\n",
		pct_dn, pct_up, e_idle);
	(void)e_dn; (void)e_up;

	/* 2. normal pings */
	run_pings(US_TRIG_PIN, US_ECHO_PIN, &s);
	med = median_mm(&s, &lo, &hi);
	app_log((UB*)"usdiag: normal: echo started %d/%d, ended %d/%d, start delay %d-%d us, distance %d mm (range %d-%d)\n",
		s.rose, PINGS, s.fell, PINGS,
		(s.rose > 0) ? (INT)s.rise_min : 0, (INT)s.rise_max, med, lo, hi);

	/* 3. swapped: GP1 drives, GP0 listens */
	gpio_set_pin(US_ECHO_PIN, GPIO_MODE_OUT);
	gpio_set_val(US_ECHO_PIN, 0);
	gpio_set_pin(US_TRIG_PIN, GPIO_MODE_IN);
	pad_pull(US_TRIG_PIN, FALSE, TRUE);
	tk_dly_tsk(5);
	run_pings(US_ECHO_PIN, US_TRIG_PIN, &sx);
	medx = median_mm(&sx, &lox, &hix);
	app_log((UB*)"usdiag: swapped test (GP1 as TRIG, GP0 as ECHO): echo started %d/%d, distance %d mm\n",
		sx.rose, PINGS, medx);
	/* restore */
	pad_pull(US_TRIG_PIN, FALSE, FALSE);
	gpio_set_pin(US_TRIG_PIN, GPIO_MODE_OUT);
	gpio_set_val(US_TRIG_PIN, 0);
	gpio_set_pin(US_ECHO_PIN, GPIO_MODE_IN);
	pad_pull(US_ECHO_PIN, FALSE, TRUE);
	(void)lox; (void)hix;

	/* 4. verdict */
	if (e_idle > 20) {
		app_log((UB*)"usdiag: VERDICT: GP1 changes by itself %d times in 0.5 s. It is wired to some OTHER signal (e.g. a wheel encoder), not the ultrasonic ECHO. Check which Grove port each cable is in.\n", e_idle);
	} else if (pct_dn > 90) {
		app_log((UB*)"usdiag: VERDICT: ECHO (GP1) is stuck HIGH. Wrong wire on GP1, or a 5 V module holding it high. Check the white wire goes to the sensor's ECHO pin.\n");
	} else if ((s.rose == 0) && (sx.rose >= (PINGS / 2))) {
		app_log((UB*)"usdiag: VERDICT: TRIG and ECHO are SWAPPED. Swap the yellow and white wires (yellow = TRIG = GP0, white = ECHO = GP1).\n");
	} else if ((s.rose == 0) && (pct_up > 90)) {
		app_log((UB*)"usdiag: VERDICT: nothing on GP1 (it just follows the pull resistor) and no echo. The sensor is not connected to GP0/GP1, or has no power (red/black wires).\n");
	} else if (s.rose == 0) {
		app_log((UB*)"usdiag: VERDICT: the sensor never answers. Check power (VCC/GND), the Grove port (must be GP0/GP1), and that it is a 3.3 V type (HC-SR04P / RCWL-1601).\n");
	} else if ((s.fell < s.rose) && (s.rise_min >= 100U)) {
		app_log((UB*)"usdiag: VERDICT: wiring is RIGHT (trigger -> echo start %d us, as a real HC-SR04 does), but the sensor never HEARS its echo: ECHO stays high until it gives up. A hand at 20 cm must end the pulse. If it does not: a 5 V-only HC-SR04 on 3.3 V (too weak to receive), or a faulty sensor. Use a 3.3 V type (HC-SR04P / RCWL-1601) or power it from 5 V with a divider on ECHO.\n", (INT)s.rise_min);
	} else if (s.fell < s.rose) {
		app_log((UB*)"usdiag: VERDICT: ECHO starts but does not end (stuck high). Typical of a 5 V-only HC-SR04 on 3.3 V, or a damaged module.\n");
	} else if ((s.rise_max < 100U) && (med < 60U)) {
		app_log((UB*)"usdiag: VERDICT: the echo starts almost at once (%d us) and is very short (%d mm). A real HC-SR04 waits ~200-500 us first. This is typical of a 5 V-only HC-SR04 on 3.3 V, or of electrical crosstalk between the TRIG and ECHO wires. Use a 3.3 V sensor (HC-SR04P).\n", (INT)s.rise_max, med);
	} else if ((hi - lo) > 100U) {
		app_log((UB*)"usdiag: VERDICT: answers, but readings jump around (%d-%d mm). Loose wire, or the beam is hitting something on the car. Hold a flat object at 20 cm and compare.\n", lo, hi);
	} else if (med < 50U) {
		app_log((UB*)"usdiag: VERDICT: answers steadily at %d mm. Something is RIGHT in front of the sensor (bracket, wires, car body) or it points at the floor. Move it or tilt it up, then move your hand: the distance must follow.\n", med);
	} else {
		app_log((UB*)"usdiag: VERDICT: OK - steady echo at %d mm. Move your hand in front (10, 20, 30 cm): the next rounds must follow it.\n", med);
	}
}

EXPORT void us_diag(void)
{
	INT	round = 1;

	/* run below every car task: the busy-wait timing must not starve
	   ctrl_task (it feeds the watchdog) or line_task */
	(void)tk_chg_pri(TSK_SELF, 30);
	scan_hold = 1;		/* scan_task stops pinging: we own GP0/GP1 */
	tk_dly_tsk(500);
	app_log((UB*)"usdiag: ultrasonic diagnostic. The car will not drive. A new round every 5 s.\n");
	while (1) {
		diag_round(round++);
		tk_dly_tsk(5000);
	}
}

#endif /* APP_US_DIAG */
