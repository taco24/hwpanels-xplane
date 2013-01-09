/*
 * colomboard.c
 *
 */
#include <stdio.h>
#include "cb_driver.h"
#include "thread.h"
#include "colomboard.h"
#include "output.h"
#define MAX_LINE 256

int writeCounter = 0;

int usb_status = 0;
int returnCode = 0;

int initDevice();
int closeDevice();
int readDevice();
int writeDevice(void *ptr_usb_data);

int writeDevice(void *ptr_usb_data) {
	char cTmp[MAX_LINE];
	if (usb_status < 0) {
		return usb_status;
	}
	struct usb_data *l_ptr_usb_data;
	l_ptr_usb_data = (struct usb_data *) ptr_usb_data;
	unsigned char tmp_buffer_out[12];
	// write colomboard
	tmp_buffer_out[0] = 0x00;
	tmp_buffer_out[1] = l_ptr_usb_data->selectedDevice; // Device id
	tmp_buffer_out[2] = l_ptr_usb_data->comFreq / 100;
	tmp_buffer_out[3] = (l_ptr_usb_data->comFreq / 10) % 10;
	tmp_buffer_out[4] = l_ptr_usb_data->comFreq % 10;
	tmp_buffer_out[5] = l_ptr_usb_data->comFreqFraction / 10;
	tmp_buffer_out[6] = l_ptr_usb_data->comFreqFraction % 10;
	tmp_buffer_out[7] = l_ptr_usb_data->comFreqStandby / 100;
	tmp_buffer_out[8] = (l_ptr_usb_data->comFreqStandby / 10) % 10;
	tmp_buffer_out[9] = l_ptr_usb_data->comFreqStandby % 10;
	tmp_buffer_out[10] = l_ptr_usb_data->comFreqStandbyFraction / 10;
	tmp_buffer_out[11] = l_ptr_usb_data->comFreqStandbyFraction % 10;
	if ((usb_status = cb_panel_write(tmp_buffer_out))
			< sizeof(tmp_buffer_out)) {
		sprintf(cTmp, "error: interrupt write failed, %d, %d\n", usb_status, l_ptr_usb_data->usbCounter);
		writeConsole(12, 0, cTmp);
	}
	l_ptr_usb_data->usbCounter++;
	return 0;
}

int getValue(unsigned char a, unsigned char b) {
	int result = ((int) a) * 255 + ((int) b);
	return result;
}

int readDevice(void *ptr_usb_data) {
	char cTmp[MAX_LINE];
	int countBytes = 0;
	if (usb_status < 0) {
		return usb_status;
	}
	struct usb_data *l_ptr_usb_data;
	l_ptr_usb_data = (struct usb_data *) ptr_usb_data;
	// read colomboard
	int result = 0;
	unsigned char buf[5];
	countBytes = cb_panel_read_non_blocking(buf);
	l_ptr_usb_data->usbCounter++;
	if (countBytes <= 0) {
		result = countBytes;
	} else if (countBytes != sizeof(buf)) {
		sprintf(cTmp, "error: interrupt read failed, %d, %d\n", countBytes, l_ptr_usb_data->usbCounter);
		writeConsole(13, 0, cTmp);
		result = -1;
	} else {
		l_ptr_usb_data->board0 = buf[0];
		l_ptr_usb_data->board1 = buf[1];
		l_ptr_usb_data->board2 = buf[2];
		l_ptr_usb_data->board3 = buf[3];
		l_ptr_usb_data->board4 = buf[4];
		result = countBytes;
	}
	return result;
}

int initDevice() {
	usb_status = cb_panel_open();
	return usb_status;
}

int closeDevice() {
	usb_status = cb_panel_close();
	return usb_status;
}
