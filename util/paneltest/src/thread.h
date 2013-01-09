/*
 * thread.h
 *
 */

#ifndef THREAD_H_
#define THREAD_H_

#include <pthread.h>

struct shared_data {
	int changed;
	int comFreq;
	int comFreqFraction;
	int comFreqStandby;
	int comFreqStandbyFraction;
	int flapIndicator;
	int selectedDevice;
	int board0;
	int board1;
	int board2;
	int board3;
	int board4;
	int isRunning;  // is calculation finished
	int thread_id; 	// id of the thread
	int stop;       // stop thread
};

void *run(void *ptr_shared_data);

#endif /* THREAD_H_ */
