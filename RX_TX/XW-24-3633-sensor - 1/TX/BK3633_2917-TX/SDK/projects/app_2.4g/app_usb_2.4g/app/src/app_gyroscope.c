#include "app_gyroscope.h"
#include "i2c.h"
#include "app.h"
#include "gpio.h"
#include <math.h>
#include <stdint.h>  // �����׼�������Ͷ���
#include <stdlib.h>
#include <stdio.h>

       // ���ݴ��������ݱ�ICM-42607-P��I2C��ַ��0x68

// ��̬�������ڿ������˲�
static short GYRO_OFFSET[3] = {0, 0, 0};
static char DPI_DATA = 0;
// ��ͨ�˲�����
static float filtered_pitch = 0.0;
// z ��λ����ر���
static float velocity_x = 0.0, displacement_x = 0.0;
static float velocity_z = 0.0, displacement_z = 0.0;

void floatToString(float num, char *str, int decimalPlaces) {
    // ����������
    int isNegative = 0;
    if (num < 0) {
        isNegative = 1;
        num = -num; // ȡ����ֵ���д���
    }

    // ������������
    int integerPart = (int)num;
    char temp[50];
    if (isNegative) {
        sprintf(temp, "-%d", integerPart);
    } else {
        sprintf(temp, "%d", integerPart);
    }
    strcpy(str, temp);

    // ����С������
    if (decimalPlaces > 0) {
        strcat(str, ".");
        float fractionalPart = num - integerPart;
        for (int i = 0; i < decimalPlaces + 1; i++) { // ��ȡһλ������������
            fractionalPart *= 10;
            int digit = (int)fractionalPart;
            if (i < decimalPlaces) {
                char tempDigit[2];
                sprintf(tempDigit, "%d", digit);
                strcat(str, tempDigit);
            } else {
                // �������봦��
                if (digit >= 5) {
                    // ��Ҫ��λ
                    int len = strlen(str);
                    for (int j = len - 1; j >= 0; j--) {
                        if (str[j] == '.') continue;
                        if (str[j] == '-') continue;
                        str[j]++; // ��ǰ��λ
                        if (str[j] <= '9') break; // ���û�н�λ�������ֹͣ
                        str[j] = '0'; // �����λ�����������ǰ��λ
                    }
                }
            }
            fractionalPart -= digit;
        }
    }
}

/**
 * @brief ��ʼ��������
 */
void sensor_init() {
    // ��ʼ��I2C�����ôӻ���ַΪ0x69��������Ϊ400kHz
    i2c_init(I2C_ADDR, 0);

    // ����SDA��SCL����
    gpio_config(02, SC_FUN, PULL_NONE); // ����SDA����
    gpio_config(03, SC_FUN, PULL_NONE); // ����SCL����

    // ��������������Ϊ��500 dps
    uint8_t gyro_range = 0x4C; // �������ݱ��е�GYRO_CONFIG0�Ĵ�������  //0~3 0101: 1.6k Hz    5~6    000: ��2000 dps
    write_sensor_data(0x20, gyro_range);

    // ���ü��ٶȼ�����Ϊ��4g
    uint8_t accel_range = 0x4C; // �������ݱ��е�ACCEL_CONFIG0�Ĵ�������
    write_sensor_data(0x21, accel_range);

    // ���õ�Դ����Ĵ�����ʹ�������Ǻͼ��ٶȼ�
    uint8_t pwr_mgmt = 0x0F; // PWR_MGMT0�Ĵ�����ʹ�����д�����
    write_sensor_data(0x1F, pwr_mgmt);

    // ����FIFOģʽ
    // uint8_t fifo_config = 0x01; // FIFO_CONFIG1�Ĵ���������ΪSTOP-on-FULLģʽ
    // write_sensor_data(0x28, fifo_config);

    // ������ʼ������...
}

/**
 * @brief ��ȡ�������Ĵ�������
 * @param reg_addr �Ĵ�����ַ
 * @return ��ȡ������
 */
uint8_t read_sensor_data(uint8_t reg_addr) {
    uint8_t data;
    i2c_read(I2C_ADDR, reg_addr, &data, 1);
    return data;
}

/**
 * @brief д�봫�����Ĵ�������
 * @param reg_addr �Ĵ�����ַ
 * @param data Ҫд�������
 */
void write_sensor_data(uint8_t reg_addr, uint8_t data) {
    i2c_write(I2C_ADDR, reg_addr, &data, 1);
}


/**
 * @brief ��ȡ�����Ǻͼ��ٶȼ�����
 * @param acc_raw �洢���ٶȼ����ݵ�����
 * @param gyro_raw �洢���������ݵ�����
 */
void get_gyro_data(int16_t *acc_raw, int16_t *gyro_raw) {
    uint8_t accel_x0 = 0, accel_y0 = 0, accel_z0 = 0, 
            accel_x1 = 0, accel_y1 = 0, accel_z1 = 0; 
    uint8_t gyro_x0 = 0,  gyro_y0 = 0,  gyro_z0 = 0,  
            gyro_x1 = 0,  gyro_y1 = 0,  gyro_z1 = 0;  

    // ʾ������ȡ���ٶȼ�����
    accel_x1 = read_sensor_data(ACCEL_DATA_X1);    
    accel_x0 = read_sensor_data(ACCEL_DATA_X1 + 1); 
    accel_y1 = read_sensor_data(ACCEL_DATA_X1 + 2);  
    accel_y0 = read_sensor_data(ACCEL_DATA_X1 + 3); 
    accel_z1 = read_sensor_data(ACCEL_DATA_X1 + 4);   
    accel_z0 = read_sensor_data(ACCEL_DATA_X1 + 5);
    acc_raw[0] = (accel_x1 << 8) | accel_x0;
    acc_raw[1] = (accel_y1 << 8) | accel_y0;
    acc_raw[2] = (accel_z1 << 8) | accel_z0;
    // ʾ������ȡ����������
    gyro_x1 = read_sensor_data(GYRO_DATA_X1);    
    gyro_x0 = read_sensor_data(GYRO_DATA_X1 + 1);   
    gyro_y1 = read_sensor_data(GYRO_DATA_X1 + 2);   
    gyro_y0 = read_sensor_data(GYRO_DATA_X1 + 3);
    gyro_z1 = read_sensor_data(GYRO_DATA_X1 + 4);   
    gyro_z0 = read_sensor_data(GYRO_DATA_X1 + 5);
    gyro_raw[0] = (gyro_x1 << 8) | gyro_x0;
    gyro_raw[1] = (gyro_y1 << 8) | gyro_y0;
    gyro_raw[2] = (gyro_z1 << 8) | gyro_z0;
    // �������ٶȼ�����
    for(int i = 0; i < DATA_LENGTH; i++){
        acc_raw[i] = (acc_raw[i] > max_data) ? max_data : ((acc_raw[i] < min_data) ? min_data : acc_raw[i]);
        gyro_raw[i] = (gyro_raw[i] > max_data) ? max_data : ((gyro_raw[i] < min_data) ? min_data : gyro_raw[i]);

    }
//    		uart_printf("Acc: 	X=%d, Y=%d ,Z=%d,\n", 		acc_raw[0], acc_raw[1], acc_raw[2]);
//   		uart_printf("Gyro:X=%d, Y=%d,	Z=%d\r\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]); 
}

// ԭʼ����ת������
void raw_to_physical(int16_t* acc_raw, int16_t* gyro_raw, float* acc_g, float* gyro_dps) {
    // ������ת��Ϊʵ����������������λС��
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

// ���ݼ��ٶȼ� X �� Z ���ݼ��� pitch �ǣ���λ?��
float calculate_acc_angle(float acc_angle) {
    return acc_angle * 90.0 / max_data;
}

void rotate_point(float x, float y, float degrees, float* new_ax, float* new_ay) {
    // ���Ƕ�ת��Ϊ����
    float radians = degrees * M_PI / 180.0f;
    float cos_theta = cosf(M_PI - radians);
    float sin_theta = sinf(radians);
    
    if(degrees < 0){  // ��׼��ת������ʱ�룩
			*new_ax = x * cos_theta - y * sin_theta;
			*new_ay = x * sin_theta + y * cos_theta;
		}else if(degrees > 0){  //		˳ʱ��
			*new_ax = x * cos_theta + y * sin_theta;
			*new_ay = -x * sin_theta + y * cos_theta;
		}
}

// ������У׼����ֹʱ����100��ȡ��ֵ��
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

// ��ͨ�˲���������ָ���˲�
float low_pass_filter(float current, float prev_filtered, float alpha) {
    return alpha * current + (1.0 - alpha) * prev_filtered;
}


// ��ʼ���������˲���
void kalman_init(Kalman *kf, float init_angle) {
    kf->angle = init_angle;
    kf->bias = 0.0;
    kf->P[0][0] = 0.0; kf->P[0][1] = 0.0;
    kf->P[1][0] = 0.0; kf->P[1][1] = 0.0;
}


float kalman_filter(Kalman* kf, float acc_angle, float gyro_rate, float dt) {
    // Ԥ��
    kf->angle += dt * (gyro_rate - kf->bias);
    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + Q_ANGLE);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += Q_BIAS * dt;

    // ����
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


// ����λ�ƣ���װ x �� & z ����㣩
void compute_displacement(float acc_g, float gravity_component, float *velocity, float *displacement, float dt) {
    float a_sensor = acc_g * G;                     // �� g ת��Ϊ m/s?
    float dynamic_a = a_sensor - gravity_component; // ���㶯̬���ٶ�
*velocity += dynamic_a * dt;                        // �����ٶ�
*displacement += *velocity * dt;                    // ����λ��
}