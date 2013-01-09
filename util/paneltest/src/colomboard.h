/*
 * colomboard.h
 */

#ifndef COLOMBOARD_H_
#define COLOMBOARD_H_

/*
 *
 * 	if (xpl_command == 10) {
		DataArray[0] = xpl_command;
		DataArray[1] = xpl_sub_command;
		DataArray[2] = freq10;
		DataArray[3] = freq11;
		DataArray[4] = freq20;
		DataArray[5] = freq21;
		DataArray[6] = flap_indicator / 255;
		DataArray[7] = flap_indicator % 255;
	} else if (xpl_command == 20) {
		DataArray[0] = xpl_command;
		DataArray[1] = xpl_sub_command;
		DataArray[2] = freq30;
		DataArray[3] = freq31;
		DataArray[4] = freq40;
		DataArray[5] = freq41;
		DataArray[6] = usbCounter % 255;
		DataArray[7] = twiCounter % 255;
 *
 *
 *	if (xpl_command == 10) {
		com_freq = DataArray[2];
		com_freq_fraction = DataArray[3];
		com_freq_standby = DataArray[4];
		com_freq_fraction_standby = DataArray[5];
		flap_indicator = DataArray[6] * 255;
		flap_indicator += DataArray[7];
		flap_trigger = 0;
	} else if (xpl_command == 20) {
		nav_freq = DataArray[2];
		nav_freq_fraction = DataArray[3];
		nav_freq_standby = DataArray[4];
		nav_freq_fraction_standby = DataArray[5];
	} else {
		// not implemented
	}
 *
 *
 */
struct usb_data {
	int comFreq;  // COM 118.00 ... 136.975
	int comFreqFraction;
	int comFreqStandby;
	int comFreqStandbyFraction;
	int flapIndicator; // FlapIndicator 1240 .. 4700
	int usbCounter;
	int twiCounter;
	int selectedDevice;
	
	int board0;
	int board1;
	int board2;
	int board3;
	int board4;
};

int writeDeviceCom(void *ptr_usb_data);
int writeDeviceNav(void *ptr_usb_data);
int writeDevice(void *ptr_usb_data);
int readDevice(void *ptr_usb_data);
char* getVersionDevice();
int initDevice();
int closeDevice();

#endif /* COLOMBOARD_H_ */
