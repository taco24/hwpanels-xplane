#if IBM
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <stdio.h>
#include "cb_driver.h"
#include "hidapi.h"
#include "output.h"

#define MAX_STR 255

enum {
	HID_ERROR = -1,
	CB_VENDOR_ID = 0xF055,
	CB_PROD_ID = 0x5500,
	CB_ERROR_THRESH = 40,
	PANEL_CHECK_INTERVAL = 5
// seconds
};

hid_device *cbHandle;
static char tmp[100];
unsigned char cb_in_buf[CB_IN_BUF_SIZE];
unsigned char cb_zero_panel[CB_OUT_BUF_SIZE] = { 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F,
		0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F };

unsigned char cb_blank_panel[CB_OUT_BUF_SIZE] = { 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F,
		0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F };
static unsigned char tempInbuf[100];

int cb_panel_open() {
	printf("\n\n\n\n\n\n\n\n\n\n\n");
	int res = 0;

	cbHandle = hid_open(CB_VENDOR_ID, CB_PROD_ID, NULL);

	if (!cbHandle) {
		printf("-> CP: cb_driver.panel_open: unable to open device.\n");
		return -1;
	}
	wchar_t wstr[MAX_STR];
	res = hid_get_manufacturer_string(cbHandle, wstr, MAX_STR);
	sprintf(tmp, "-> CP: cb_driver.panel_open: Manufacturer String %ls\n", wstr);
    printf(tmp);

	cb_panel_read_non_blocking(tempInbuf);
	cb_panel_read_non_blocking(tempInbuf);
	cb_panel_read_non_blocking(tempInbuf);
	res = cb_panel_write(cb_blank_panel);
//		res = hid_send_feature_report(cbHandle, cb_blank_panel, CB_OUT_BUF_SIZE);
	if (res < 0) {
		sprintf(tmp, "-> CP: cb_driver.panel_open: Error: %ls\n", hid_error(
				cbHandle));
		printf(tmp);
	}
	return 0;
}

int cb_panel_write(unsigned char buf[]) {
	int res = 0;
	if (cbHandle) {
		res = hid_write(cbHandle, buf, CB_OUT_BUF_SIZE);
		if (res < 0) {
			sprintf(tmp, "-> CP: cb_driver.panel_write: Error: %ls\n",
					hid_error(cbHandle));
			printf(tmp);
		}
	}
	return res;
}

int cb_panel_write_empty() {
	int res = 0;
	unsigned char cb_empty_buf[0];
	if (cbHandle) {
		res = hid_send_feature_report(cbHandle, cb_empty_buf, 0);
		if (res < 0) {
			sprintf(tmp, "-> CP: cb_driver.panel_write_empty: Error: %ls\n",
					hid_error(cbHandle));
			printf(tmp);
		}
	}
	return res;
}

int cb_panel_read_blocking(unsigned char *buf) {
	int res = 0;
	if (cbHandle) {
		hid_set_nonblocking(cbHandle, 0);
		res = hid_read_timeout(cbHandle, buf, CB_IN_BUF_SIZE, 100);
		if (res < 0) {
			sprintf(tmp, "-> CP: cb_driver.panel_read_blocking: Error: %ls\n",
					hid_error(cbHandle));
			printf(tmp);
		}
	}
	return res;
}

int cb_panel_read_non_blocking(unsigned char *buf) {
	int res = 0;
	if (cbHandle) {
		hid_set_nonblocking(cbHandle, 1);
		res = hid_read(cbHandle, buf, CB_IN_BUF_SIZE);
		if (res < 0) {
			sprintf(tmp,
					"-> CP: cb_driver.panel_read_non_blocking: Error: %ls\n",
					hid_error(cbHandle));
			printf(tmp);
		}
	}
	return res;
}

int cb_panel_close() {
	int res = 0;
	if (cbHandle) {
		cb_panel_write(cb_blank_panel);
		//		res = hid_send_feature_report(cbHandle, cb_blank_panel, CB_OUT_BUF_SIZE);
		//		if (res < 0) {
		//		    sprintf(tmp, "-> CP: cb_driver.panel_close: Error: %ls\n", hid_error(cbHandle));
		//			XPLMDebugString(tmp);
		//		}
		hid_close(cbHandle);
		printf("-> CP: cb_driver.panel_close: panel closed.\n");
		cbHandle = NULL;
	}
	return res;
}

