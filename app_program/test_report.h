/*
 *	test_report.h
 *	Minimal on-target test helpers.
 *
 *	Every check prints ONE line in a fixed format, so a saved console log
 *	can be searched for results ("grep '\[TEST\]' log.txt"):
 *
 *	    [TEST] S1-U01 PASS  ADC reads in range
 *	    [TEST] S1-U02 FAIL  queue rejects overflow   (task_x.c:57)
 *	    [TEST] subsystem1 SUMMARY 1 passed, 1 failed
 *
 *	Usage, inside a test function:
 *	    test_init();
 *	    TEST_CHECK("S1-U01", value >= 0 && value <= 4095, "ADC reads in
 *	    range");
 *	    test_summary("subsystem1");
 *
 *	Test IDs must match the ones in testing/<subsystem>/TEST_LOG.md.
 */

#ifndef TEST_REPORT_H
#define TEST_REPORT_H

#include <tm/tmonitor.h>
#include "app_log.h"	/* [TEST] lines also reach the dashboard log */

#if APP_TEST
/* Pass [0] / fail [1] counts of the test file that includes this header
   (Barr-C: functions, not macros, wherever a function can do the job) */
Inline INT *test_counts(void)
{
	static INT	counts[2];

	return counts;
}

Inline void test_init(void)
{
	test_counts()[0] = 0;
	test_counts()[1] = 0;
}

Inline void test_summary(const char *name)
{
	app_log((UB*)"[TEST] %s SUMMARY %d passed, %d failed\n",
		name, test_counts()[0], test_counts()[1]);
}
#endif

/* A macro, not a function: it reports the caller's __FILE__ and
   __LINE__ (Barr-C permits a macro where a function cannot do this) */
#define TEST_CHECK(id, cond, desc)					\
	do {								\
		if (cond) {						\
			test_counts()[0]++;				\
			app_log((UB*)"[TEST] %s PASS  %s\n", (id), (desc)); \
		} else {						\
			test_counts()[1]++;				\
			app_log((UB*)"[TEST] %s FAIL  %s   (%s:%d)\n",	\
				  (id), (desc), __FILE__, __LINE__);	\
		}							\
	} while (0)

#endif /* TEST_REPORT_H */
