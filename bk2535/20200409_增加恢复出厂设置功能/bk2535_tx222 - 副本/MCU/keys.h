
#ifndef _KEYS_H_
#define _KEYS_H_

#include "utils.h"

#define KEY_PORT_MASK      0xff

#define KEY_COL_NUM        15

sbit SCL_I = P2^3;
sbit SCL_O = P2^3;
sbit SDA_I = P2^4;
sbit SDA_O = P2^4;


extern XDATA UINT8 up_power000;

extern unsigned char is_keydown(void);
extern void wait_all_key_release(void);

extern void process_mode_switch(void);
extern void read_keys_pin(void);

extern void key_proc(void);
extern void mouse_proc(void);
extern UINT8 BMI160_Init(void);
extern UINT8 BMI160_Sleep(void);

#define KEY_POWER	 (key_array[14]&0x01)
#define KEY_SLEEP	 (key_array[14]&0x02)
#define KEY_WAKEUP	 (key_array[14]&0x04)

#define KEY_LCTRL	 (key_array[13]&0x01)
#define KEY_LSHIFT	 (key_array[13]&0x02)
#define KEY_LALT	 (key_array[13]&0x04)
#define KEY_LWIN	 (key_array[13]&0x08)
#define KEY_RCTRL	 (key_array[13]&0x10)
#define KEY_RSHIFT	 (key_array[13]&0x20)
#define KEY_RALT	 (key_array[13]&0x40)
#define KEY_RWIN	 (key_array[13]&0x80)


#define KEY_LMOUSE	 (key_array[14]&0x10)
#define KEY_RMOUSE	 (key_array[14]&0x20)
//#define KEY_MMOUSE	 (key_array[14]&0x40)


//#define KEY_TV		 (key_array[1]&0x02)
//#define KEY_HOME     	 (key_array[1]&0x04)
//#define KEY_SMARTTV	 (key_array[1]&0x08)
//
////#define KEY_MOUSE_SW (key_array[1]&0x01)
//
//#define KEY_VOLP	 (key_array[2]&0x01)
//#define KEY_VOLM	 (key_array[2]&0x02)
//#define KEY_MUTE	 (key_array[2]&0x04)
//#define KEY_BACK	 (key_array[2]&0x08)
//
//
////#define KEY_MENU	 (key_array[4]&0x20)
//#define KEY_MENU	 (key_array[3]&0x02)
//#define KEY_GUIDE	 (key_array[4]&0x20)
//
//#define KEY_NC		 (key_array[3]&0x20)
//
//
//#define KEY_RED		 (key_array[4]&0x40)
//#define KEY_GREEN	 (key_array[4]&0x80)
//#define KEY_YELLOW	 (key_array[5]&0x01)
//#define KEY_BLUE	 (key_array[5]&0x02)
//
//#define KEY_PRE_TRACE (key_array[0]&0x20)
//
//#define KEY_ALT		 (key_array[9]&0x04)
//
//#define KEY_UP		 (key_array[1]&0x10)
//#define KEY_DOWN	 (key_array[1]&0x20)
//#define KEY_LEFT	 (key_array[1]&0x40)
//#define KEY_RIGHT	 (key_array[1]&0x80)
typedef union {
    UINT8 CHAR_data[2];
    UINT16 INT_data;
} cyp;
#ifndef gyroscope_en

void gyroscope_MOSE(void);
void gyro_read_defult(short *FIFO);
void Kalman_filter(short *acc,short *gyro);
void Kalman_four_buff_filter(short *acc);
short Kalman_filter_arg(short gy_ang,short acc_ang,UINT8 Hk);
short Kalman_axis_limit(short gy,short angle);
UINT8 Kalman_filter_Hk(UINT16 r);
UINT8 arg_bit16_to_bit8(UINT16 buff);
char Kalman_filter_get_angle(short X,short Y,short limit);
UINT8 angle_get_arg(short  *angle);
void angle_line_arg(short*gyro,short angx);
UINT8 _swap_(UINT8 a);
void tolerance_sub(short *p);
void gyro_div(short*p, char dpi);
void touch_filter_sub(short *FIFO);
UINT8 first_order_filter(short *now, short *old,UINT8 flag, UINT8 *coe, UINT8 *count);



#endif
bit I2C_TX(BYTE a);
bit SMBus_tran(BYTE p, BYTE dat);
bit SMBus_rece(BYTE a, BYTE addr,BYTE *p);
UINT8 I2C_RX(void);




#endif

