#include "app_gyroscope.h"
#include "user_handle.h"
#include "i2c.h"
#include "app.h"
#include "gpio.h"
#include <math.h>
#include <stdint.h> // 引入标准整数类型定义
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "data_filter.h"
#include "acc_gry.h"
#include "attitude.h"
imu_t imu_9;


int16_t window_ax[WIN_NUM];
int16_t window_ay[WIN_NUM];
int16_t window_az[WIN_NUM];

int16_t window_gx[WIN_NUM];
int16_t window_gy[WIN_NUM];
int16_t window_gz[WIN_NUM];

float numbers[100];
uint8_t cursor_speed = 1;
extern int16_t mouse_X;
extern int16_t mouse_Y;
int16_t acc_raw[3], gyro_raw[3], gyro_filter[3];
float acc_g[3], gyro_g[3], angular_displacement[3], linear_displacement[3];
float new_ax, new_ay;
float gyro_filtered[MOUSE_DATA_LENGTH], acc_filtered[MOUSE_DATA_LENGTH];
float init_angle = 0;

int16_t angle_Tab2[46] = // 4gmax  8bit  1=31.25mg
    {
        8192, // 90
        8187, // 88°
        8172, // 86°
        8147, // 84°
        8113, // 82°
        8067, // 80°
        8013, // 78°
        7949, // 76°

        7875, // 74°
        7791, // 72°
        7698, // 70°
        7596, // 68°
        7483, // 66°
        7363, // 64°
        7233, // 62°
        7094, // 60°

        6947, // 58°
        6791, // 56°
        6627, // 54°
        6455, // 52
        6275, // 50°
        6087, // 48°
        5893, // 46°
        5691, // 44°

        5481, // 42°
        5266, // 40°
        5044, // 38°
        4815, // 36°
        4581, // 34°
        4341, // 32°
        4096, // 30°
        3946, // 28°

        3591, // 26°
        3332, // 24°
        3069, // 22°
        2802, // 20°
        2531, // 18°
        2258, // 16°
        1982, // 14°
        1703, // 12°

        1422, // 10°
        1140, // 8°
        856,  // 6°
        572,  // 4°
        286,  // 2°
        0     // 0°
};

static const int sinTable[91] = {0, 2, 3, 5, 7, 9, 10, 12, 14, 16, 17, 19, 21, 22, 24, 26, 28, 29, 31,
                                 33, 34, 36, 37, 39, 41, 42, 44, 45, 47, 48, 50, 52, 53, 54, 56, 57, 59, 60, 62, 63, 64, 66, 67, 68,
                                 69, 70, 72, 73, 74, 75, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 87, 88, 89, 90, 91, 91, 92, 93,
                                 93, 94, 95, 95, 96, 96, 97, 97, 97, 98, 98, 98, 99, 99, 99, 99, 100, 100, 100, 100, 100, 100};

static const int cosTable[91] = {100, 100, 100, 100, 100, 100, 99, 99, 99, 99, 98, 98, 98, 97, 97, 97,
                                 96, 96, 95, 95, 94, 93, 93, 92, 91, 91, 90, 89, 88, 87, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77,
                                 75, 74, 73, 72, 71, 69, 68, 67, 66, 64, 63, 62, 60, 59, 57, 56, 54, 53, 52, 50, 48, 47, 45, 44, 42,
                                 41, 39, 37, 36, 34, 33, 31, 29, 28, 26, 24, 22, 21, 19, 17, 16, 14, 12, 10, 9, 7, 5, 3, 2, 0};


void floatToString(float num, char *str, int decimalPlaces)
{
    // 处理负数
    if (num < 0)
    {
        num = -num;
        *str++ = '-';
    }

    // 计算四舍五入因子
    float roundingFactor = 0.5;
    for (int i = 0; i < decimalPlaces; i++)
    {
        roundingFactor /= 10.0;
    }
    num += roundingFactor;

    // 分离整数和小数部分
    int integerPart = (int)num;
    float fractionalPart = num - integerPart;

    // 转换整数部分
    str += sprintf(str, "%d", integerPart);

    // 处理小数部分
    if (decimalPlaces > 0)
    {
        *str++ = '.';
        for (int i = 0; i < decimalPlaces; i++)
        {
            fractionalPart *= 10;
            int digit = (int)fractionalPart;
            *str++ = '0' + digit;
            fractionalPart -= digit;
        }
    }
    *str = '\0';
}

// 批量打印浮点数
void printFloats(float *numbers, int count, int decimalPlaces)
{
    char buffer[50];
    for (int i = 0; i < count; i++)
    {
        floatToString(numbers[i], buffer, decimalPlaces);
        uart_printf("Number %d: %s\r\n", i + 1, buffer);
    }
}




/**
 * @brief 获取陀螺仪和加速度计数据
 * @param acc_raw 存储加速度计数据的数组
 * @param gyro_raw 存储陀螺仪数据的数组
gyro_raw: 0->y  1->x  2->x   角度
acc_raw：0->x, 1->y, 2->z  加速度
 */

void imu_data_transition(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz){
	imu_9.f_acc[0] = (float)(ax-imu_9.acc_zero[0]) / 8192.0f;  // 加速度量程为:±2G      获取到的加速度计数据 除以 16393 ，可以转化为带物理单位的数据，单位：g(m/s^2)
	imu_9.f_acc[1] = (float)(ay-imu_9.acc_zero[1]) / 8192.0f;
	imu_9.f_acc[2] = (float)(az-imu_9.acc_zero[2]) / 8192.0f;

	imu_9.f_gyro[0] = (float)(gx-imu_9.gyro_zero[0]) / 57.1f; //  陀螺仪量程为:±500dps  获取到的陀螺仪数据除以 57.1，    可以转化为带物理单位的数据，单位为：°/s
	imu_9.f_gyro[1] = (float)(gy-imu_9.gyro_zero[1]) / 57.1f;
	imu_9.f_gyro[2] = (float)(gz-imu_9.gyro_zero[2]) / 57.1f;
	//uart_printf("acc_filter: 	X=%d, Y=%d, Z=%d \r\n", ax, ay, az);
//    numbers[0] = imu_9.f_acc[0];
//    numbers[1] = imu_9.f_acc[1];
//    numbers[2] = imu_9.f_acc[2];
//    printFloats(numbers, 3, 2);
	
}


void imu_final_data_get(void)
{
	int16_t filter_ax ,filter_ay,filter_az ;
	int16_t filter_gx ,filter_gy,filter_gz ;
	int16_t filter_mx ,filter_my,filter_mz ;

	  imu_sample_data(imu_9.i_gyro,imu_9.i_acc); //采样得到九轴原始数据
	  //原始数据窗口滤波
	  filter_ax = window_filter(imu_9.i_acc[0],window_ax,WIN_NUM);
	  filter_ay = window_filter(imu_9.i_acc[1],window_ay,WIN_NUM);
	  filter_az = window_filter(imu_9.i_acc[2],window_az,WIN_NUM);

	  filter_gx = window_filter(imu_9.i_gyro[0],window_gx,WIN_NUM);
	  filter_gy = window_filter(imu_9.i_gyro[1],window_gy,WIN_NUM);
	  filter_gz = window_filter(imu_9.i_gyro[2],window_gz,WIN_NUM);

		
   //uart_printf("acc_filter: 	X=%d, Y=%d, Z=%d \r\n", filter_ax, filter_ay, filter_az);
   //uart_printf("gyro_filter: 	X=%d, Y=%d, Z=%d \r\n", filter_gx, filter_gy, filter_gz);
	

	  //转换成实际物理量
	  imu_data_transition(filter_ax,filter_ay,filter_az,filter_gx,filter_gy,filter_gz);
}



// 定义采样时间间隔（单位：秒）
#define SAMPLE_TIME 0.005f    //0.01/2
#define MOUSE_DATA_LENGTH 3
// 假设轮子的半径（单位：米）
#define WHEEL_RADIUS 1.0f

void calculate_displacement(float *gyro_g, float *angular_displacement, float *linear_displacement)
{
    static float prev_gyro[MOUSE_DATA_LENGTH] = {0}; // 上一次的角速度值

    for (int i = 0; i < MOUSE_DATA_LENGTH; i++)
    {
        // 将原始角速度数据转换为浮点数
        float current_gyro = gyro_g[i] * 20;

        angular_displacement[i] = 0;
        // 使用梯形法则进行积分，计算角位移
        angular_displacement[i] += (prev_gyro[i] + current_gyro) * SAMPLE_TIME;

        // 计算直线位移（假设是轮子的旋转，位移=半径×角位移）
        linear_displacement[i] = WHEEL_RADIUS * angular_displacement[i];

        // 更新上一次的角速度值
        prev_gyro[i] = current_gyro;


    }
}

// 低通滤波函数：简单指数滤波
void low_pass_filter(int16_t* abs_angle, int16_t *gyro_raw, float *linear_displacement)
{
    if((*abs_angle > 0 && *abs_angle < 10) && abs(gyro_raw[2]) > 2000){
			linear_displacement[2] = linear_displacement[2];
			linear_displacement[0] = linear_displacement[0] * 0.01;
			linear_displacement[1] = linear_displacement[1] * 0.01;
		}else if((*abs_angle > 80 && *abs_angle < 90) && abs(gyro_raw[2]) > 2000 ){
			linear_displacement[0] = linear_displacement[0];
			linear_displacement[1] = linear_displacement[1] * 0.01;
			linear_displacement[2] = linear_displacement[2] * 0.01;
		}else{
			linear_displacement[0] = linear_displacement[0];
		  linear_displacement[1] = linear_displacement[1];
		  linear_displacement[2] = linear_displacement[2];
		}
}



static float last_angle = 0.0f;
static int last_ax = 0, last_az = 0;
int ax, az, temp;
int16_t value, abs_angle;
// 根据加速度计 X 与 Z 数据计算 pitch 角（单位?）
float calculate_acc_angle(int16_t* acc_raw , int16_t* abs_angle) {  // 0->x, 2->z
	
		ax = acc_raw[0];
    az = acc_raw[2];
        // 原有的逻辑
        for (int i = 0; i < 46; i++) {
            temp = abs(ax);
            if (temp > angle_Tab2[i + 1] && temp <= angle_Tab2[i]) {
                *abs_angle = 90 - i * 2;
                break;
            }
        }
				
			//float value =  ax * 90.0 / 8192;
			if(ax > 0 && az < 0){
				 value = *abs_angle;  // 1
			}else if(ax > 0 && az > 0){      // 2
				 value = 180 - *abs_angle;
			}else if(ax < 0 && az > 0){      // 3
				 value = -180 + *abs_angle;
			}else if(ax < 0 && az < 0){            // 4
				 value = - *abs_angle;
			}
			
			return value;
	 }	


//一阶互补滤波
float K1 =0.1; // 对加速度计取值的权重
float dt=0.01;//注意：dt的取值为滤波器采样时间
float angle_P,angle_R;

float yijiehubu_P(float angle_m, float gyro_m)//采集后计算的角度和角加速度
{
		if(fabs(gyro_m) > 1e-6){
			angle_P = K1 * angle_m + (1-K1) * (angle_P + gyro_m * dt);  // 
		}else {
			angle_P = angle_m;
		}
		
    return angle_P;	    
}
 
float yijiehubu_R(float angle_m, float gyro_m)//采集后计算的角度和角加速度
{
     angle_R = K1 * angle_m + (1-K1) * (angle_R + gyro_m * dt);
         return angle_R;
}



// 三角函数的计算
int common_sin_val_calculate(int angle)
{
    angle = (angle + 3600) % 3600; // 归一化角度到0 ~ 3600范围内

    if ((angle >= 0) && (angle <= 900))
    {
        return sinTable[angle / 10];
    }
    else if ((angle > 900) && (angle <= 1800))
    {
        return cosTable[(angle - 900) / 10];
    }
    else if ((angle > 1800) && (angle <= 2700))
    {
        return -sinTable[(angle - 1800) / 10];
    }
    else
    {
        return -cosTable[(angle - 2700) / 10];
    }
}

/* 计算余弦值
 * angle: 输入角度（0 ~ 3600，对应0 ~ 360度，每个单位表示0.1度）
 * return: cos(angle) * 100
 */
int common_cos_val_calculate(int angle)
{
    angle = (angle + 3600) % 3600; // 归一化角度到0 ~ 3600范围内

    if ((angle >= 0) && (angle <= 900))
    {
        return cosTable[angle / 10];
    }
    else if ((angle > 900) && (angle <= 1800))
    {
        return -sinTable[(angle - 900) / 10];
    }
    else if ((angle > 1800) && (angle <= 2700))
    {
        return -cosTable[(angle - 1800) / 10];
    }
    else
    {
        return sinTable[(angle - 2700) / 10];
    }
}
void rotate_point(float ax, float az, float degrees, float *new_ax, float *new_ay)
{
    // 将角度转化为弧度
    //	float radians = degrees *M_PI / 180.0f;
    //	float cos_theta = cosf(radians);
    //	float sin_theta = sinf(radians);
    //
    //		*new_ax = ax * cos_theta - az * sin_theta;
    //    *new_ay = ax * sin_theta + az * cos_theta;
    degrees = degrees * 10;
    *new_ax = ax * COS(degrees) - az * SIN(degrees);
    *new_ay = ax * SIN(degrees) + az * COS(degrees);

//    numbers[0] = degrees;
//    numbers[1] = COS(degrees);
//    numbers[2] = SIN(degrees);
//    printFloats(numbers, 3, 2);
}

void Filter_threshold(int16_t *gyro_raw, int16_t *gyro_filter)
{
    for (int i = 0; i < 3; i++)
    {
        if (gyro_raw[i] < 500 && gyro_raw[i] > -500)
        {
            gyro_filter[i] = 0; // 设为 0
        }
        else
        {
            gyro_filter[i] = gyro_raw[i];
        }
    }
}




// 姿态估计结构体  
typedef struct {  
    float q0, q1, q2, q3;           // 四元数  
    float exInt, eyInt, ezInt;        // 积分误差  
    float yaw, pitch, roll;          // 欧拉角  
    float accelBias[3];              // 加速度计偏置  
    float gyroBias[3];               // 陀螺仪偏置  
} AttitudeEstimator;  

// 卡尔曼滤波结构体  
typedef struct {  
    float Q;                          // 过程噪声  
    float R;                          // 测量噪声  
    float P, S, K;                    // 状态、协方差、卡尔曼增益  
    float x[3];                       // 状态（姿态角）  
    float z[3];                       // 测量值  
} KalmanFilter;  


// 卡尔曼滤波初始化  
void kalmanFilterInit(KalmanFilter* kf, float Q, float R) {  
    kf->Q = Q;  
    kf->R = R;  
    kf->P = 1.0f;  
    kf->x[0] = kf->x[1] = kf->x[2] = 0.0f;  
}  

// 预测步  
void kalmanPredict(KalmanFilter* kf, float* gyro, float dt) {  
    // 状态转移矩阵  
    float F[3][3] = {{1, -gyro[2]*dt, gyro[1]*dt},  
                     {gyro[2]*dt, 1, -gyro[0]*dt},  
                     {-gyro[1]*dt, gyro[0]*dt, 1}};  
    
    float x_pred[3];  
    for (int i = 0; i < 3; i++) {  
        x_pred[i] = 0;  
        for (int j = 0; j < 3; j++) {  
            x_pred[i] += F[i][j] * kf->x[j];  
        }  
    }  
    memcpy(kf->x, x_pred, sizeof(float) * 3);  

    // 协方差预测  
    // 由于手动计算F * P * F^T比较复杂，这里简化处理，实际中应完整计算。  
    // 为了简化，可以使用矩阵运算库或手动展开计算。  
    // 此处仅为示例，正确计算需要更详细的矩阵操作。  
    // 暂时保留P += Q * dt，建议用户重新实现协方差计算。  
}  

// 更新步  
void kalmanUpdate(KalmanFilter* kf, float* acc, float dt) {  
    // 观测模型  
    float z_pred[3];  
    z_pred[0] = acc[0] - kf->x[1] * kf->x[2] * dt;  
    z_pred[1] = acc[1] - kf->x[2] * kf->x[0] * dt;  
    z_pred[2] = acc[2] - kf->x[0] * kf->x[1] * dt;  
    
    // 计算卡尔曼增益  
    kf->S = kf->P + kf->R;  
    kf->K = kf->S * kf->S;  

    // 更新状态  
    for (int i = 0; i < 3; i++) {  
        kf->x[i] += (acc[i] - z_pred[i]) * kf->K;  
    }  
}  

//void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) {  
//    static AttitudeEstimator est;  
//    static KalmanFilter kf;  
//    static bool initialized = false;  
//    
//    if (!initialized) {  
//        attitudeEstimatorInit(&est);  
//        kalmanFilterInit(&kf, 0.001f, 0.1f);  
//        initialized = true;  
//    }  
//    
//    // step 1: 去除加速度偏置  
//    ax -= est.accelBias[0];  
//    ay -= est.accelBias[1];  
//    az -= est.accelBias[2];  
//    
//    // step 2: 使用卡尔曼滤波对加速度和陀螺仪数据进行优化  
//    float acc[3] = {ax, ay, az};  
//    float gyro[3] = {gx, gy, gz};  
//    
//    // 预测步  
//    kalmanPredict(&kf, gyro, 0.01f);  
//    
//    // 更新步  
//    kalmanUpdate(&kf, acc, 0.01f);  
//    
//    // 使用卡尔曼滤波后的姿态估计结果  
//    est.yaw = kf.x[0];  
//    est.pitch = kf.x[1];  
//    est.roll = kf.x[2];  
//    
//    // four元数更新  
//    float q0 = est.q0;  
//    float q1 = est.q1;  
//    float q2 = est.q2;  
//    float q3 = est.q3;  
//    
//    // Compute quaternion derivative  
//    float dq0 = (0.5f * q0) * (gyro[0] + gyro[1] + gyro[2]);  
//    float dq1 = 0.5f * ( - q1 * gx - q2 * gy - q3 * gz);  
//    float dq2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);  
//    float dq3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);  
//    
//    // Update quaternions  
//    est.q0 += dq0 * dt;  
//    est.q1 += dq1 * dt;  
//    est.q2 += dq2 * dt;  
//    est.q3 += dq3 * dt;  
//    
//    // Normalize quaternion  
//    float norm = sqrt(est.q0*est.q0 + est.q1*est.q1 + est.q2*est.q2 + est.q3*est.q3);  
//    est.q0 /= norm;  
//    est.q1 /= norm;  
//    est.q2 /= norm;  
//    est.q3 /= norm;  
//    
//    // Update Euler angles  
//    est.yaw = atan2(2 * (est.q1 * est.q2 + est.q0 * est.q3),   
//                    -2 * (est.q2 * est.q2 + est.q3 * est.q3) + 1) * 57.3;  
//    est.pitch = asin(2 * (est.q0 * est.q2 - est.q1 * est.q3)) * 57.3;  
//    est.roll = atan2(2 * (est.q2 * est.q3 + est.q0 * est.q1),   
//                    -2 * (est.q1 * est.q1 + est.q2 * est.q2) + 1) * 57.3;  
////    numbers[0] = est.pitch;
////    numbers[1] = est.roll;
////    printFloats(numbers, 2, 2);
////		uart_printf("------------ \r\n");
//}  


void mouse_init()
{
    // 初始化I2C，设置从机地址为0x69，波特率为400kHz
    i2c_init(I2C_ADDR, 0);

    // 配置SDA和SCL引脚
    gpio_config(02, SC_FUN, PULL_NONE); // 配置SDA引脚
    gpio_config(03, SC_FUN, PULL_NONE); // 配置SCL引脚

    // 设置陀螺仪量程为±500 dps
    uint8_t gyro_range = 0x49; // 根据数据表中的GYRO_CONFIG0寄存器设置  //0~3 0101: 1.6k Hz    5~6    000: ±2000 dps
    write_sensor_data(0x20, gyro_range);

    // 设置加速度计量程为±4g
    uint8_t accel_range = 0x49; // 根据数据表中的ACCEL_CONFIG0寄存器设置
    write_sensor_data(0x21, accel_range);

    // 配置电源管理寄存器，使能陀螺仪和加速度计
    uint8_t pwr_mgmt = 0x0F; // PWR_MGMT0寄存器，使能所有传感器
    write_sensor_data(0x1F, pwr_mgmt);

    i2c_init(I2C_ADDR, 0);
}

double az_sum = 0;
double ax_sum = 0;
double pos_t = 0;

double pre_x = 0;
double pre_y = 0;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
void mouse()
{

  imu_final_data_get(); //原始数据采样
	calculate_attitude(&attitude, 1) ; //姿态解算
//    get_gyro_data(gyro_raw, acc_raw);
//    Filter_threshold(gyro_raw, gyro_filter);
//   // uart_printf("gyro_raw: 	X=%d, Y=%d, Z=%d \r\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
//    //uart_printf("acc_raw: 	X=%d, Y=%d, Z=%d \r\n", acc_g[0], acc_g[1], acc_g[2]);
//    raw_to_physical(acc_raw, gyro_filter, acc_g, gyro_g);
////		numbers[0] = acc_g[0];
////		numbers[1] = acc_g[1];
////		numbers[2] = acc_g[2];
////    numbers[3] = gyro_g[0];
////    numbers[4] = gyro_g[1];
////    numbers[5] = gyro_g[2];
////    printFloats(numbers, 6, 2);
//    calculate_displacement(gyro_g, angular_displacement, linear_displacement);

//    float acc_angle_x = calculate_acc_angle(acc_raw, &abs_angle);
//		low_pass_filter(&abs_angle, gyro_raw, linear_displacement);
//		float angle_p = yijiehubu_P(acc_angle_x, gyro_g[1]);
//		numbers[0] = angle_P;
//    numbers[1] = acc_angle_x;
//    numbers[2] = gyro_g[1];
//    printFloats(numbers, 3, 2);
//		IMUupdate(gyro_g[0], gyro_g[1], gyro_g[2], acc_g[0], acc_g[1], acc_g[2]);

//    rotate_point(linear_displacement[2], linear_displacement[0], angle_p, &new_ax, &new_ay);
//		
//    ax_sum += new_ax * cursor_speed;
//    az_sum += new_ay * cursor_speed; // cursor_speed
//    mouse_X = ax_sum;
//    mouse_Y = -az_sum;

//    ax_sum -= mouse_X;
//    az_sum += mouse_Y;

//    if (single_monitor_flag)
//    {
//        current_mouse_x += mouse_X;
//        current_mouse_y += mouse_Y;
//        if (current_mouse_x < 0)
//            current_mouse_x = 0;
//        if (current_mouse_y < 0)
//            current_mouse_y = 0;
//        if (current_mouse_x > resolution_Width)
//            current_mouse_x = resolution_Width;
//        if (current_mouse_y > resolution_Height)
//            current_mouse_y = resolution_Height;

//        pos_t = current_mouse_x / pre_x;
//        mouse_X = (int16_t)pos_t;

//        pos_t = current_mouse_y / pre_y;
//        mouse_Y = (int16_t)pos_t;

//    }

    // mouse_X = (mouse_X > MOUSE_MAX)?MOUSE_MAX :(mouse_X < MOUSE_MIN ? MOUSE_MIN : mouse_X);
    // mouse_Y = (mouse_Y > MOUSE_MAX)?MOUSE_MAX :(mouse_Y < MOUSE_MIN ? MOUSE_MIN : mouse_Y);

}
