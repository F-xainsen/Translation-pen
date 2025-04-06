#ifndef ACC_GRY_H_
#define ACC_GRY_H_

#include "main.h"
#include "i2c.h"

#define I2C_ADDR      0x69
#define ACCEL_DATA_X1 0x0B
#define GYRO_DATA_X1  0x11

//初始化  读取原始数据
void write_sensor_data(uint8_t reg_addr, uint8_t data);
void imu_sample_data(int16_t *gyro_raw, int16_t *acc_raw);


uint8_t acc_gyro_init(void);
void acc_gyro_sample_data(int16_t *gyro,int16_t *acc);
void set_acc_gyro_offset(void);
#endif /* ACC_GRY_H_ */
