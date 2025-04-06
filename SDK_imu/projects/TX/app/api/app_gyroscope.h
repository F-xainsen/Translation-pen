#ifndef _APP_GYROSCOPE_H_
#define _APP_GYROSCOPE_H_
#include "i2c.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

#define USE_FUSION    0
#define MOTION_THRESHOLD  1.025f

typedef struct
{
	int16_t i_gyro[3];
	int16_t i_acc[3];
	int16_t i_mag[3];
	float f_gyro[3];
	float f_acc[3];
	float f_mag[3];
	float mag_xsf;
	float mag_ysf;
	int16_t gyro_zero[3];
	int16_t acc_zero[3];
	int16_t mag_zero[3];

	float pitch;
	float roll;
	float yaw;
	float mag_yaw_test;
	float yaw_temp;
	float velocity[3];
	float displacement[3];
	uint8_t cali_flag;
	float accelerationMagnitude;
	uint8_t shell_cmd_ok;
	uint8_t output_mode;
	uint8_t output_flag;
	uint16_t output_freq;
	uint32_t time_tick;

} imu_t;

extern imu_t imu_9;

void imu_final_data_get(void);
void Kalman_cal_yaw_angle(void);
void imu_9_shell_cmd_to_do(void);
void data_output_mode(uint8_t mode);

extern float numbers[100];
// 打印函数
void floatToString(float num, char *str, int decimalPlaces);
void printFloats(float *numbers, int count, int decimalPlaces);

void sensor_init(void);

//滑动滤波
void imu_data_transition(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz);
void Filter_threshold(int16_t *gyro_raw, int16_t *gyro_filter);
float calculate_acc_angle(int16_t* acc_raw, int16_t* abs_angle);
void raw_to_physical(int16_t* acc_raw, int16_t* gyro_raw, float* acc_g, float* gyro_g );
void calibrate_gyro(void);
void low_pass_filter(int16_t* abs_angle, int16_t *gyro_raw, float *linear_displacement);
void process_data_with_kalman(int16_t *gyro_raw, int16_t *acc_raw, float *gyro_filtered, float *acc_filtered);
void calculate_displacement(float *gyro_g, float *angular_displacement, float *linear_displacement);
void rotate_point(float ax, float az, float degrees, float* new_ax, float* new_ay);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
extern uint8_t cursor_speed;
void mouse_init(void);
void mouse(void);
#endif
