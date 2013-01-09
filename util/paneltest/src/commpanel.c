/*
 ============================================================================
 Name        : HBridge.c
 Author      : colomboard
 Version     :
 Copyright   : colomboard
 Description : Hello World in C, Ansi-style
 ============================================================================

 read : targetPosition0 int16,
 boardValue0 int16,
 targetPosition1 int16,
 boardValue1 int16
 write: targetPosition0 int16,
 targetPosition1 int16,
 n/a  int16,
 n/a int16

 */
#if IBM
#include <windows.h>
#else
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "output.h"
#include "thread.h"
#include "getch.h"


#define MAX_LINE 256
#define VOR_FREQUENCY_MIN 108
#define VOR_FREQUENCY_MAX 117
#define VOR_FREQUENCY_DEF 114
#define VOR_FREQUENCY_FRACTION_DEF 85
#define VOR_FREQUENCY_STANDBY_DEF 114
#define VOR_FREQUENCY_STANDBY_FRACTION_DEF 30
#define FLAP_INDICATOR_MIN 1240
#define FLAP_INDICATOR_DEF 1240
#define FLAP_INDICATOR_MAX 4700


int countBytes = 0;
pthread_t g_thread;
int g_thread_id = 1;
int g_thread_return_code = 0;
struct shared_data g_shared_data;


int main(void) {
	int character = 0;

	initConsole();


	g_shared_data.thread_id = g_thread_id;
	g_shared_data.stop = 0;
	g_thread_return_code = pthread_create(&g_thread, NULL, run, (void *) &g_shared_data);
	if (g_thread_return_code) {
		closeConsole(); /* End curses mode		  */
		printf(
				"Commpanel error: return code from pthread_create() is %d\n",
				g_thread_return_code);
		return 0;
	}

	g_shared_data.comFreq = VOR_FREQUENCY_DEF;
	g_shared_data.comFreqFraction = VOR_FREQUENCY_FRACTION_DEF;
	g_shared_data.comFreqStandby = VOR_FREQUENCY_STANDBY_DEF;
	g_shared_data.comFreqStandbyFraction = VOR_FREQUENCY_STANDBY_FRACTION_DEF;
	g_shared_data.flapIndicator = FLAP_INDICATOR_DEF;

	while ((character = mygetch()) != 'q') {
		if (character == 'a') {
			g_shared_data.comFreq -= 1;
			g_shared_data.changed = 1;
		} else if (character == 'd') {
			g_shared_data.comFreq += 1;
			g_shared_data.changed = 1;
		} else if (character == 'w') {
			g_shared_data.comFreqFraction += 1;
			g_shared_data.changed = 1;
		} else if (character == 's') {
			g_shared_data.comFreqFraction -= 1;
			g_shared_data.changed = 1;
		} else if (character == 'j') {
			g_shared_data.comFreqStandby -= 1;
			g_shared_data.changed = 1;
		} else if (character == 'l') {
			g_shared_data.comFreqStandby += 1;
			g_shared_data.changed = 1;
		} else if (character == 'i') {
			g_shared_data.comFreqStandbyFraction += 1;
			g_shared_data.changed = 1;
		} else if (character == 'k') {
			g_shared_data.comFreqStandbyFraction -= 1;
			g_shared_data.changed = 1;
		} else if (character == 'm') {
			g_shared_data.flapIndicator -= 10;
			g_shared_data.changed = 1;
		} else if (character == 'n') {
			g_shared_data.flapIndicator += 10;
			g_shared_data.changed = 1;
		} else if (character == '0') {
			g_shared_data.selectedDevice = 0;
		} else if (character == '1') {
			g_shared_data.selectedDevice = 1;
		} else if (character == '2') {
			g_shared_data.selectedDevice = 2;
		} else if (character == '3') {
			g_shared_data.selectedDevice = 3;
		} else if (character == '4') {
			g_shared_data.selectedDevice = 4;
		} else {
			g_shared_data.changed = 0;
		}

		if (g_shared_data.comFreq < VOR_FREQUENCY_MIN) {
			g_shared_data.comFreq = VOR_FREQUENCY_MIN;
		} else if (g_shared_data.comFreq > VOR_FREQUENCY_MAX) {
			g_shared_data.comFreq = VOR_FREQUENCY_MAX;
		}
		g_shared_data.comFreqFraction %= 100;
		if (g_shared_data.comFreqFraction < 0) {
			g_shared_data.comFreqFraction = 99;
		}
		if (g_shared_data.comFreqStandby < VOR_FREQUENCY_MIN) {
			g_shared_data.comFreqStandby = VOR_FREQUENCY_MIN;
		} else if (g_shared_data.comFreqStandby > VOR_FREQUENCY_MAX) {
			g_shared_data.comFreqStandby = VOR_FREQUENCY_MAX;
		}
		g_shared_data.comFreqStandbyFraction %= 100;
		if (g_shared_data.comFreqStandbyFraction < 0) {
			g_shared_data.comFreqStandbyFraction = 99;
		}
		if (g_shared_data.flapIndicator < FLAP_INDICATOR_MIN) {
			g_shared_data.flapIndicator = FLAP_INDICATOR_MIN;
		} else if (g_shared_data.flapIndicator > FLAP_INDICATOR_MAX) {
			g_shared_data.flapIndicator = FLAP_INDICATOR_MAX;
		}
	}
	g_shared_data.stop = 1;

#ifdef __linux__
	sleep(1);
#else
	Sleep(1000);
#endif
	closeConsole(); /* End curses mode		  */

	return 0;
}
