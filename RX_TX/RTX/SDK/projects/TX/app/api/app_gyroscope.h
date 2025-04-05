#ifndef _APP_GYROSCOPE_H_
#define _APP_GYROSCOPE_H_
#include "i2c.h"

#define ACCEL_DATA_X1 0x0B
#define GYRO_DATA_X1  0x11


#define mouse_max   32767
#define mouse_min   0

extern const uint16_t angle_Tab[46];

extern uint8_t data[12];
extern uint8_t acc_FIFO[3];
extern uint8_t gyro_FIFO[3];


typedef struct {
    int16_t x;
    int16_t y;
} PlaneData;

// º¯ÊýÉùÃ÷
void floatToString(double num, char *str, int decimalPlaces);
void sensor_init();
uint8_t read_sensor_data(uint8_t reg_addr);
void write_sensor_data(uint8_t reg_addr, uint8_t data);
void get_gyro_data(short *acc_FIFO, short *gyro_FIFO);
void gyro_read_defult(short *FIFO);

void Kalman_filter(short *acc, short *gyro);
uint8_t Kalman_filter_Hk(uint16_t r);
int16_t map_value(int16_t value, int16_t from_min, int16_t from_max, int16_t to_min, int16_t to_max);
uint8_t arg_bit16_to_bit8(uint16_t buff);
short Kalman_filter_arg(short gy_ang, short acc_ang, uint8_t Hk);
void Kalman_four_buff_filter(short *acc);
short Kalman_axis_limit(short gy, short angle);
char Kalman_filter_get_angle(short X, short Y, short limit);
void angle_line_arg(short *gyro, short angx);
uint8_t _swap_(uint8_t a);
uint8_t angle_get_arg(short *angle);
void tolerance_sub(short *p);
void gyro_div(short *p, char dpi);
void touch_filter_sub(short *FIFO);
uint8_t first_order_filter(short *now, short *old, uint8_t flag, uint8_t *coe, uint8_t *count);

void mouse_init();
void mouse();

#endif 