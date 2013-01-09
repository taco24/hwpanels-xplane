#ifndef TIME_H_
#define TIME_H_

#include <sys/time.h>

typedef enum {
	COUNTER1 = 0,
	COUNTER2,
	COUNTER3,
	COUNTER4,
	COUNTER5,
	COUNTER6,
	COUNTER7,
	COUNTER8,
	COUNTER9,
	COUNTER10,
	COUNTER11,
	COUNTER12,
	COUNTER13,
	COUNTER14,
	NUM_OF_COUNTERS
} counter_id_t; ///< Software counters for the mainloop


long sys_time_clock_get_time_usec(void);
void us_run_init(void);
/** @brief Software timer function for use in the mainloop */
int us_run_every(int us, counter_id_t counter_id, int current_time);

#endif /* TIME_H_ */
