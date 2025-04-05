#ifndef _APP_GYROSCOPE_H_
#define _APP_GYROSCOPE_H_
#include "i2c.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define I2C_ADDR      0x69
#define ACCEL_DATA_X1 0x0B
#define GYRO_DATA_X1  0x11
#define ACCEL_SSF     8192      //±4g
#define GYRO_SSF      65.5      // ±500dps
#define max_data      8100
#define min_data	   -8100
#define LP_ALPHA 0.1          // 低通滤波系数
#define G 9.81                // 重力加速度（m/s?）
#define Q_ANGLE 0.001
#define Q_BIAS 0.003
#define R_MEASURE 0.03
#define DATA_LENGTH 3

typedef struct {
    int16_t x;
    int16_t y;
} PlaneData;


typedef struct {
    float q0, q1, q2, q3;
} Quaternion;

// 简化卡尔曼滤波
typedef struct {
    float angle;
    float bias;
    float P[2][2];
} Kalman;

// 函数声明
void floatToString(float num, char *str, int decimalPlaces);

void sensor_init();
uint8_t read_sensor_data(uint8_t reg_addr);
void write_sensor_data(uint8_t reg_addr, uint8_t data);
void get_gyro_data(int16_t *acc_FIFO, int16_t *gyro_FIFO);
void raw_to_physical(int16_t* raw_acc, int16_t* raw_gyro, float* acc, float* gyro);
void calibrate_gyro(void);
float low_pass_filter(float current, float prev_filtered, float alpha);
void kalman_init(Kalman *kf, float init_angle);
float kalman_filter(Kalman* kf, float acc_angle, float gyro_rate, float dt);
float calculate_acc_angle(float acc_angle);
void compute_displacement(float acc_g, float gravity_component, float *velocity, float *displacement, float dt);

void rotate_point(float x, float y, float degrees, float* new_ax, float* new_ay);


#endif 