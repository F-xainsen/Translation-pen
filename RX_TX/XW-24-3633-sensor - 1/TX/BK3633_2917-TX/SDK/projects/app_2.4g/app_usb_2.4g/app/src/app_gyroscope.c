#include "app_gyroscope.h"
#include "i2c.h"
#include "app.h"
#include "gpio.h"
#include <math.h>
#include <stdint.h>  // 引入标准整数类型定义
#include <stdlib.h>
#include <stdio.h>

       // 根据传感器数据表，ICM-42607-P的I2C地址是0x68

// 静态变量用于卡尔曼滤波
static short GYRO_OFFSET[3] = {0, 0, 0};
static char DPI_DATA = 0;
// 低通滤波变量
static float filtered_pitch = 0.0;
// z 轴位移相关变量
static float velocity_x = 0.0, displacement_x = 0.0;
static float velocity_z = 0.0, displacement_z = 0.0;

void floatToString(float num, char *str, int decimalPlaces) {
    // 处理负数符号
    int isNegative = 0;
    if (num < 0) {
        isNegative = 1;
        num = -num; // 取绝对值进行处理
    }

    // 处理整数部分
    int integerPart = (int)num;
    char temp[50];
    if (isNegative) {
        sprintf(temp, "-%d", integerPart);
    } else {
        sprintf(temp, "%d", integerPart);
    }
    strcpy(str, temp);

    // 处理小数部分
    if (decimalPlaces > 0) {
        strcat(str, ".");
        float fractionalPart = num - integerPart;
        for (int i = 0; i < decimalPlaces + 1; i++) { // 多取一位用于四舍五入
            fractionalPart *= 10;
            int digit = (int)fractionalPart;
            if (i < decimalPlaces) {
                char tempDigit[2];
                sprintf(tempDigit, "%d", digit);
                strcat(str, tempDigit);
            } else {
                // 四舍五入处理
                if (digit >= 5) {
                    // 需要进位
                    int len = strlen(str);
                    for (int j = len - 1; j >= 0; j--) {
                        if (str[j] == '.') continue;
                        if (str[j] == '-') continue;
                        str[j]++; // 当前进位
                        if (str[j] <= '9') break; // 如果没有进位溢出，则停止
                        str[j] = '0'; // 如果进位溢出，继续向前进位
                    }
                }
            }
            fractionalPart -= digit;
        }
    }
}

/**
 * @brief 初始化传感器
 */
void sensor_init() {
    // 初始化I2C，设置从机地址为0x69，波特率为400kHz
    i2c_init(I2C_ADDR, 0);

    // 配置SDA和SCL引脚
    gpio_config(02, SC_FUN, PULL_NONE); // 配置SDA引脚
    gpio_config(03, SC_FUN, PULL_NONE); // 配置SCL引脚

    // 设置陀螺仪量程为±500 dps
    uint8_t gyro_range = 0x4C; // 根据数据表中的GYRO_CONFIG0寄存器设置  //0~3 0101: 1.6k Hz    5~6    000: ±2000 dps
    write_sensor_data(0x20, gyro_range);

    // 设置加速度计量程为±4g
    uint8_t accel_range = 0x4C; // 根据数据表中的ACCEL_CONFIG0寄存器设置
    write_sensor_data(0x21, accel_range);

    // 配置电源管理寄存器，使能陀螺仪和加速度计
    uint8_t pwr_mgmt = 0x0F; // PWR_MGMT0寄存器，使能所有传感器
    write_sensor_data(0x1F, pwr_mgmt);

    // 配置FIFO模式
    // uint8_t fifo_config = 0x01; // FIFO_CONFIG1寄存器，设置为STOP-on-FULL模式
    // write_sensor_data(0x28, fifo_config);

    // 其他初始化设置...
}

/**
 * @brief 读取传感器寄存器数据
 * @param reg_addr 寄存器地址
 * @return 读取的数据
 */
uint8_t read_sensor_data(uint8_t reg_addr) {
    uint8_t data;
    i2c_read(I2C_ADDR, reg_addr, &data, 1);
    return data;
}

/**
 * @brief 写入传感器寄存器数据
 * @param reg_addr 寄存器地址
 * @param data 要写入的数据
 */
void write_sensor_data(uint8_t reg_addr, uint8_t data) {
    i2c_write(I2C_ADDR, reg_addr, &data, 1);
}


/**
 * @brief 获取陀螺仪和加速度计数据
 * @param acc_raw 存储加速度计数据的数组
 * @param gyro_raw 存储陀螺仪数据的数组
 */
void get_gyro_data(int16_t *acc_raw, int16_t *gyro_raw) {
    uint8_t accel_x0 = 0, accel_y0 = 0, accel_z0 = 0, 
            accel_x1 = 0, accel_y1 = 0, accel_z1 = 0; 
    uint8_t gyro_x0 = 0,  gyro_y0 = 0,  gyro_z0 = 0,  
            gyro_x1 = 0,  gyro_y1 = 0,  gyro_z1 = 0;  

    // 示例：读取加速度计数据
    accel_x1 = read_sensor_data(ACCEL_DATA_X1);    
    accel_x0 = read_sensor_data(ACCEL_DATA_X1 + 1); 
    accel_y1 = read_sensor_data(ACCEL_DATA_X1 + 2);  
    accel_y0 = read_sensor_data(ACCEL_DATA_X1 + 3); 
    accel_z1 = read_sensor_data(ACCEL_DATA_X1 + 4);   
    accel_z0 = read_sensor_data(ACCEL_DATA_X1 + 5);
    acc_raw[0] = (accel_x1 << 8) | accel_x0;
    acc_raw[1] = (accel_y1 << 8) | accel_y0;
    acc_raw[2] = (accel_z1 << 8) | accel_z0;
    // 示例：读取陀螺仪数据
    gyro_x1 = read_sensor_data(GYRO_DATA_X1);    
    gyro_x0 = read_sensor_data(GYRO_DATA_X1 + 1);   
    gyro_y1 = read_sensor_data(GYRO_DATA_X1 + 2);   
    gyro_y0 = read_sensor_data(GYRO_DATA_X1 + 3);
    gyro_z1 = read_sensor_data(GYRO_DATA_X1 + 4);   
    gyro_z0 = read_sensor_data(GYRO_DATA_X1 + 5);
    gyro_raw[0] = (gyro_x1 << 8) | gyro_x0;
    gyro_raw[1] = (gyro_y1 << 8) | gyro_y0;
    gyro_raw[2] = (gyro_z1 << 8) | gyro_z0;
    // 解析加速度计数据
    for(int i = 0; i < DATA_LENGTH; i++){
        acc_raw[i] = (acc_raw[i] > max_data) ? max_data : ((acc_raw[i] < min_data) ? min_data : acc_raw[i]);
        gyro_raw[i] = (gyro_raw[i] > max_data) ? max_data : ((gyro_raw[i] < min_data) ? min_data : gyro_raw[i]);

    }
//    		uart_printf("Acc: 	X=%d, Y=%d ,Z=%d,\n", 		acc_raw[0], acc_raw[1], acc_raw[2]);
//   		uart_printf("Gyro:X=%d, Y=%d,	Z=%d\r\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]); 
}

// 原始数据转物理量
void raw_to_physical(int16_t* acc_raw, int16_t* gyro_raw, float* acc_g, float* gyro_dps) {
    // 将数据转换为实际物理量并保留两位小数
    for(int i = 0 ; i < 3; i++){
    acc_g[i] =  (float)acc_raw[i]  / ACCEL_SSF;
    gyro_dps[i] = (float)gyro_raw[i] / GYRO_SSF;	
    }

//    for(int i = 0; i < 3; i++){
//    char buffer[50];
//    floatToString(acc_raw[i] , buffer, 2);
//    uart_printf("acc_raw[%d]: %s\n",i, buffer);

//    floatToString(gyro_raw[i] , buffer, 2);
//    uart_printf("gyro_raw[%d]: %s\r\n", i, buffer);
//    }
//    uart_printf("\r\n");
}

// 根据加速度计 X 与 Z 数据计算 pitch 角（单位?）
float calculate_acc_angle(float acc_angle) {
    return acc_angle * 90.0 / max_data;
}

void rotate_point(float x, float y, float degrees, float* new_ax, float* new_ay) {
    // 将角度转换为弧度
    float radians = degrees * M_PI / 180.0f;
    float cos_theta = cosf(M_PI - radians);
    float sin_theta = sinf(radians);
    
    if(degrees < 0){  // 标准旋转矩阵（逆时针）
			*new_ax = x * cos_theta - y * sin_theta;
			*new_ay = x * sin_theta + y * cos_theta;
		}else if(degrees > 0){  //		顺时针
			*new_ax = x * cos_theta + y * sin_theta;
			*new_ay = -x * sin_theta + y * cos_theta;
		}
}

// 陀螺仪校准（静止时采样100次取均值）
void calibrate_gyro(void) {
    const int num_samples = 100;
    long sum[3] = {0, 0, 0};
    int16_t gyro_raw[3];
    
    for (int i = 0; i < num_samples; i++) {
        get_gyro_data(NULL, gyro_raw);
        sum[0] += gyro_raw[0];
        sum[1] += gyro_raw[1];
        sum[2] += gyro_raw[2];
        Delay_ms(1);
    }
    
    GYRO_OFFSET[0] = sum[0] / num_samples;
    GYRO_OFFSET[1] = sum[1] / num_samples;
    GYRO_OFFSET[2] = sum[2] / num_samples;
}

// 低通滤波函数：简单指数滤波
float low_pass_filter(float current, float prev_filtered, float alpha) {
    return alpha * current + (1.0 - alpha) * prev_filtered;
}


// 初始化卡尔曼滤波器
void kalman_init(Kalman *kf, float init_angle) {
    kf->angle = init_angle;
    kf->bias = 0.0;
    kf->P[0][0] = 0.0; kf->P[0][1] = 0.0;
    kf->P[1][0] = 0.0; kf->P[1][1] = 0.0;
}


float kalman_filter(Kalman* kf, float acc_angle, float gyro_rate, float dt) {
    // 预测
    kf->angle += dt * (gyro_rate - kf->bias);
    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + Q_ANGLE);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += Q_BIAS * dt;

    // 更新
    float S = kf->P[0][0] + R_MEASURE;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;
    float y = acc_angle - kf->angle;
    
    kf->angle += K0 * y;
    kf->bias  += K1 * y;
    
    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];
    
    kf->P[0][0] -= K0 * P00;
    kf->P[0][1] -= K0 * P01;
    kf->P[1][0] -= K1 * P00;
    kf->P[1][1] -= K1 * P01;
    
    return kf->angle;
}


// 计算位移（封装 x 轴 & z 轴计算）
void compute_displacement(float acc_g, float gravity_component, float *velocity, float *displacement, float dt) {
    float a_sensor = acc_g * G;                     // 将 g 转换为 m/s?
    float dynamic_a = a_sensor - gravity_component; // 计算动态加速度
*velocity += dynamic_a * dt;                        // 计算速度
*displacement += *velocity * dt;                    // 计算位移
}