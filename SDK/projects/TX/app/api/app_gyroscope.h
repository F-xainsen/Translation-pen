#ifndef _APP_GYROSCOPE_H_
#define _APP_GYROSCOPE_H_
#include "i2c.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define I2C_ADDR      0x69
#define ACCEL_DATA_X1 0x0B
#define GYRO_DATA_X1  0x11
#define ACCEL_SSF     8192.0f      //±4g
#define GYRO_SSF      65.5       //65.5      // ±500dps


/**
三角函数计算
 */
int common_sin_val_calculate(int angle);
int common_cos_val_calculate(int angle);
int common_tan_val_calculate(int angle);
#define SIN(val) common_sin_val_calculate(val) / 100.0
#define COS(val) common_cos_val_calculate(val) / 100.0
#define TAN(val) common_tan_val_calculate(val) / 100.0

/**
滑动窗口
 */
#define WIN_NUM 5



typedef struct
{
	int16_t i_gyro[3];
	int16_t i_acc[3];
	float f_gyro[3];
	float f_acc[3];
	int16_t gyro_zero[3];
	int16_t acc_zero[3];

} imu_t;
extern imu_t imu_9;
	
extern uint8_t cursor_speed;

// 函数声明
void sensor_init(void);
uint8_t read_sensor_data(uint8_t reg_addr);
void write_sensor_data(uint8_t reg_addr, uint8_t data);
void get_gyro_data(int16_t *gyro_raw, int16_t *acc_raw);
void Filter_threshold();
float calculate_acc_angle(int16_t* acc_raw, int16_t* gyro_raw);

void calculate_displacement(float *gyro_g, float *angular_displacement, float *linear_displacement);
void rotate_point(float ax, float az, float degrees, float* new_ax, float* new_az);
void mouse_init(void);
void mouse(void);
#endif
