/*
 * time.c
 *
 *  Created on: 29.06.2011
 *      Author: max
 */
#include "time.h"

static int next_exec_time[NUM_OF_COUNTERS]; ///< Software counter for mainloop time control
long const usec = 1000000;


long sys_time_clock_get_time_usec(void) {
	struct timeval tv;
	struct timezone tz;
	struct tm *tm;
	gettimeofday(&tv, &tz);
	tm = localtime(&tv.tv_sec);
//	return tm->tm_hour * 60 * 60 * usec + tm->tm_min * 60 * usec + tm->tm_sec
//			* usec + tv.tv_usec;
	return tv.tv_usec;
}

void us_run_init(void) {
	// Initialize counters for mainloop
	int counter_id;
	for (counter_id = 0; counter_id < NUM_OF_COUNTERS; counter_id++) {
		next_exec_time[counter_id] = 0;
	}
}

/**
 * @brief Check for periodic counter timeout
 *
 * This function can be called to check whether the counter associated with counter_id
 * has expired. If expired it the counter is set to the current time plus parameter ms
 * and TRUE is returned.
 *
 * @param us the interval to run the function with
 * @param counter_id the id of the counter to use - use one per interval
 * @param current_time the current system time
 */
int us_run_every(int us, counter_id_t counter_id, int current_time) {
	if (next_exec_time[counter_id] <= current_time
			|| next_exec_time[counter_id] - current_time > us) {
		next_exec_time[counter_id] = current_time + us;
		return 1;
	} else {
		return 0;
	}
}
