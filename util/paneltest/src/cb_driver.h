#ifndef CB_DRIVER_H_
#define CB_DRIVER_H_

#define CB_OUT_BUF_SIZE              (12)
#define CB_IN_BUF_SIZE               (5)

#define CB_READ_UPPER_KNOB_MODE_MASK (0x000000)
#define CB_READ_UPPER_KNOB_COM1      (0x000000)
#define CB_READ_UPPER_KNOB_COM2      (0x000000)
#define CB_READ_UPPER_KNOB_NAV1      (0x000000)
#define CB_READ_UPPER_KNOB_NAV2      (0x000000)
#define CB_READ_UPPER_KNOB_ADF       (0x000000)
#define CB_READ_UPPER_KNOB_DME       (0x000000)
#define CB_READ_UPPER_KNOB_TRANSPNDR (0x000000)
#define CB_READ_LOWER_KNOB_MODE_MASK (0x000000)
#define CB_READ_LOWER_KNOB_COM1      (0x000000)
#define CB_READ_LOWER_KNOB_COM2      (0x000000)
#define CB_READ_LOWER_KNOB_NAV1      (0x000000)
#define CB_READ_LOWER_KNOB_NAV2      (0x000000)
#define CB_READ_LOWER_KNOB_ADF       (0x000000)
#define CB_READ_LOWER_KNOB_DME       (0x000000)
#define CB_READ_LOWER_KNOB_TRANSPNDR (0x000000)
#define CB_READ_UPPER_ACT_STBY       (0x000001)
#define CB_READ_LOWER_ACT_STBY       (0x000000)
#define CB_READ_UPPER_FINE_TUNING_MASK (0xFF0000)
#define CB_READ_UPPER_COARSE_TUNING_MASK (0x00FF00)
#define CB_READ_UPPER_FINE_RIGHT     (0xF00000)
#define CB_READ_UPPER_FINE_LEFT      (0x0F0000)
#define CB_READ_UPPER_COARSE_RIGHT   (0x00F000)
#define CB_READ_UPPER_COARSE_LEFT    (0x000F00)
#define CB_READ_LOWER_FINE_TUNING_MASK (0x000000)
#define CB_READ_LOWER_COARSE_TUNING_MASK (0x000000)
#define CB_READ_LOWER_FINE_RIGHT     (0x000000)
#define CB_READ_LOWER_FINE_LEFT      (0x000000)
#define CB_READ_LOWER_COARSE_RIGHT   (0x000000)
#define CB_READ_LOWER_COARSE_LEFT    (0x000000)
#define CB_READ_COM1_ACT_STBY        (0x00000121)

extern int cb_panel_open();
extern int cb_panel_write(unsigned char *buf);
extern int cb_panel_write_empty();
extern int cb_panel_read_blocking(unsigned char *buf);
extern int cb_panel_read_non_blocking(unsigned char *buf);
extern int cb_panel_close();

extern unsigned char cb_blank_panel[CB_OUT_BUF_SIZE];
extern unsigned char cb_zero_panel[CB_OUT_BUF_SIZE];


#endif /* CB_DRIVER_H_ */
