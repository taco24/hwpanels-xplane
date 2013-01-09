/*
 * thread.c
 */
#if IBM
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "thread.h"
#include "output.h"
#include "colomboard.h"
#include "time.h"
#include "cb_driver.h"

#define MAX_LINE 256
int g_comFreq;
int g_comFreqFraction;
int g_comFreqStandby;
int g_comFreqStandbyFraction;
int g_flapIndicator;
int g_counter;
struct usb_data g_usb_data;
static const uint32_t min_mainloop_time = 5000;
static long last_mainloop_idle = 0;
struct shared_data *g_ptr_shared_data;
int g_board1 = 0;
int g_boardPrev1 = 0;
int g_board2 = 0;
int g_boardPrev2 = 0;

void update_screen() {
	char cTmp[MAX_LINE];
	int row = 1;
	int col = 0;
	writeConsole(row, col, "Colomboard v1.0 - Press 'q' to quit.");
	writeConsole(row + 1, col, "------------------------------------");
	sprintf(cTmp, "%d app %5d usb %3d twi %3d", g_ptr_shared_data->thread_id,
			g_counter % 10000, g_usb_data.usbCounter, g_usb_data.twiCounter);
	writeConsole(row + 2, col, cTmp);
	writeConsole(row + 4, col, "DataRef    board  driver  xplane");
	writeConsole(row + 5, col, "------------------------------------");
	sprintf(cTmp, "comFreq:  %3d.%02d  %3d.%02d  %3d.%02d",
			g_usb_data.board0, g_usb_data.board1, g_comFreq,
			g_comFreqFraction, g_ptr_shared_data->comFreq,
			g_ptr_shared_data->comFreqFraction);
	writeConsole(row + 6, col, cTmp);
	sprintf(cTmp, "comStby:  %3d.%02d  %3d.%02d  %3d.%02d",
			g_usb_data.board2, g_usb_data.board0,
			g_comFreqStandby, g_comFreqStandbyFraction,
			g_ptr_shared_data->comFreqStandby,
			g_ptr_shared_data->comFreqStandbyFraction);
	writeConsole(row + 7, col, cTmp);
	sprintf(cTmp, "flaps:      %4d    %4d    %4d", g_usb_data.flapIndicator,
			g_flapIndicator, g_ptr_shared_data->flapIndicator);
	writeConsole(row + 8, col, cTmp);
	if (g_usb_data.usbCounter % 100 == 0) {
		writeConsole(12, 0, "                                               ");
		writeConsole(13, 0, "                                               ");
	}

}

void updateHost() {
	// Board changed:
	g_comFreq = g_ptr_shared_data->comFreq;
	g_comFreqFraction = g_ptr_shared_data->comFreqFraction;
	g_comFreqStandby = g_ptr_shared_data->comFreqStandby;
	g_comFreqStandbyFraction = g_ptr_shared_data->comFreqStandbyFraction;

	if (g_flapIndicator != g_ptr_shared_data->flapIndicator) {
		g_flapIndicator = g_ptr_shared_data->flapIndicator;
	}
}

void updateBoard() {
	if (g_ptr_shared_data->changed == 1) {
		g_ptr_shared_data->changed = 0;
	} else {
		return;
	}
	g_usb_data.comFreq = g_ptr_shared_data->comFreq;
	g_usb_data.comFreqFraction = g_ptr_shared_data->comFreqFraction;
	g_usb_data.comFreqStandby = g_ptr_shared_data->comFreqStandby;
	g_usb_data.comFreqStandbyFraction = g_ptr_shared_data->comFreqStandbyFraction;
	g_usb_data.flapIndicator = g_ptr_shared_data->flapIndicator;
	g_usb_data.selectedDevice = g_ptr_shared_data->selectedDevice;
	writeDevice(&g_usb_data);
}

void processButtons(struct usb_data data) {
	g_board1 = data.board1 & CB_READ_COM1_ACT_STBY;
	if (g_board1 == 0 && g_boardPrev1 == 1) {
		int tempValue = g_ptr_shared_data->comFreq;
		g_ptr_shared_data->comFreq = g_ptr_shared_data->comFreqStandby;
		g_ptr_shared_data->comFreqStandby = tempValue;
		tempValue = g_ptr_shared_data->comFreqFraction;
		g_ptr_shared_data->comFreqFraction = g_ptr_shared_data->comFreqStandbyFraction;
		g_ptr_shared_data->comFreqStandbyFraction = tempValue;
		g_ptr_shared_data->changed = 1;
	}
	g_boardPrev1 = g_board1;
	g_board2 = data.board2;
	if (g_board2 == 0 && g_boardPrev2 == 1) {
		g_ptr_shared_data->comFreq += 1;
		g_ptr_shared_data->changed = 1;
	}
	g_boardPrev2 = g_board2;
}

void *run(void *ptr_shared_data) {
	char cTmp[MAX_LINE];

	int deviceInitialized = 0;
	int l_change = 0;

	deviceInitialized = initDevice();

	g_ptr_shared_data = (struct shared_data *) ptr_shared_data;
	// Initialize datarefs
	g_comFreq = g_ptr_shared_data->comFreq;

	last_mainloop_idle = sys_time_clock_get_time_usec();
	// while stop == 0 calculate position.
	while (g_ptr_shared_data->stop == 0) {
		g_counter++;
		long loop_start_time = sys_time_clock_get_time_usec();

		///////////////////////////////////////////////////////////////////////////
		/// CRITICAL FAST 20 Hz functions
		///////////////////////////////////////////////////////////////////////////
		if (us_run_every(50000, COUNTER1, loop_start_time)) {
			// read usb board values
			l_change = readDevice(&g_usb_data);
			processButtons(g_usb_data);
			// Update xplane
			updateHost();
			// Update board
			updateBoard();
		}
		///////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 10 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(100000, COUNTER2, loop_start_time)) {
			update_screen();
		}
		///////////////////////////////////////////////////////////////////////////

		if (loop_start_time - last_mainloop_idle >= 100000) {
			writeConsole(10, 0, "CRITICAL WARNING! CPU LOAD TOO HIGH.");
			last_mainloop_idle = loop_start_time;//reset to prevent multiple messages
		} else {
			writeConsole(10, 0, "CPU LOAD OK.");
		}

		// wait 1 milliseconds
#if IBM
		Sleep(10);
#endif
#if LIN
		usleep(10);
#endif
	}
	sprintf(cTmp, "thread closing usb device %d...", 0);
	writeConsole(0, 0, cTmp);
	closeDevice();
	sprintf(cTmp, "thread info : stopping thread #%d, %d!\n",
			g_ptr_shared_data->thread_id, g_counter);
	writeConsole(0, 0, cTmp);
	pthread_exit(NULL);
	return 0;
}
