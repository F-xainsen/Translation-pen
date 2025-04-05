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
#define GYRO_SSF      65.5       //65.5      // ±500dps
#define ACCEL_MAX          8192
#define ACCEL_MIN          -8192
#define GYRO_MAX				   8600
#define GYRO_MIN				   -8600
#define MOUSE_MAX					32767
#define MOUSE_MIN					-32767
#define LP_ALPHA 0.1          // 低通滤波系数
#define G 0.981                // 重力加速度（m/s?）
#define Q_ANGLE 0.001
#define Q_BIAS 0.003
#define R_MEASURE 0.03
#define MOUSE_DATA_LENGTH 3

#define Kp 10.0f                        // 这里的KpKi是用于调整加速度计修正陀螺仪的速度
#define Ki 0.008f                        
#define halfT 0.005f             // 采样周期的一半，用于求解四元数微分方程时计算角增量

int common_sin_val_calculate(int angle);
int common_cos_val_calculate(int angle);
int common_tan_val_calculate(int angle);
 
#define SIN(val) common_sin_val_calculate(val) / 100.0
#define COS(val) common_cos_val_calculate(val) / 100.0
#define TAN(val) common_tan_val_calculate(val) / 100.0


typedef struct {
    float estimate;  // 当前估计值
    float error;     // 当前估计误差
} KalmanFilter;

	
// 函数声明
void floatToString(float num, char *str, int decimalPlaces);
void printFloats(float *numbers, int count, int decimalPlaces);
void sensor_init(void);
uint8_t read_sensor_data(uint8_t reg_addr);
void write_sensor_data(uint8_t reg_addr, uint8_t data);
void get_gyro_data(int16_t *gyro_raw, int16_t *acc_raw);
void Filter_threshold(int16_t *gyro_raw, int16_t *gyro_filter);
float calculate_acc_angle(int16_t* acc_raw, int16_t* gyro_raw);
void raw_to_physical(int16_t* acc_raw, int16_t* gyro_raw, float* acc_g, float* gyro_g );
void calibrate_gyro(void);
float low_pass_filter(float current, float prev_filtered, float alpha);
void kalman_init(KalmanFilter *filter, float initial_estimate, float initial_error);
float kalman_update(KalmanFilter *filter, float measurement);
void process_data_with_kalman(int16_t *gyro_raw, int16_t *acc_raw, float *gyro_filtered, float *acc_filtered);
void calculate_displacement(float *gyro_g, float *angular_displacement, float *linear_displacement);
void rotate_point(float ax, float az, float degrees, float* new_ax, float* new_az);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
extern uint8_t cursor_speed;
void mouse_init(void);
void mouse(void);
#endif
