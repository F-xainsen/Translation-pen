#include "app_gyroscope.h"
#include "i2c.h"
#include "gpio.h"
#include <math.h>
#include <stdint.h>  // �����׼�������Ͷ���
#include <stdlib.h>
#include <stdio.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define I2C_ADDR        0x69       // ���ݴ��������ݱ�ICM-42607-P��I2C��ַ��0x68

// ��̬�������ڿ������˲�
static short GYRO_OFFSET[3];
static char DPI_DATA = 0;


void floatToString(double num, char *str, int decimalPlaces) {
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
        double fractionalPart = num - integerPart;
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
 * @brief �ǶȲ��ұ����ڿ������˲�����
 */
const uint16_t angle_Tab[46] = {
    8192,  // 90��
    8187,  // 88��
    8172,  // 86��
    8147,  // 84��

    8113,  // 82��
    8067,  // 80��
    8013,  // 78��
    7949,  // 76��

    7875,  // 74��
    7791,  // 72��
    7698,  // 70��
    7596,  // 68��

    7483,  // 66��
    7363,  // 64��
    7233,  // 62��
    7094,  // 60��

    6947,  // 58��
    6791,  // 56��
    6627,  // 54��
    6455,  // 52��

    6275,  // 50��
    6087,  // 48��
    5893,  // 46��
    5691,  // 44��

    5481,  // 42��
    5266,  // 40��
    5044,  // 38��
    4815,  // 36��

    4581,  // 34��
    4341,  // 32��
    4096,  // 30��
    3946,  // 28��

    3591,  // 26��
    3332,  // 24��
    3069,  // 22��
    2802,  // 20��

    2531,  // 18��
    2258,  // 16��
    1982,  // 14��
    1703,  // 12��

    1422,  // 10��
    1140,  // 8��
    856,   // 6��
    572,   // 4��

    286,   // 2��
    0      // 0��
};

/**
 * @brief ��ʼ��������
 */
void sensor_init() {
    // ��ʼ��I2C�����ôӻ���ַΪ0x68��������Ϊ400kHz
    i2c_init(I2C_ADDR, 0);

    // ����SDA��SCL����
    gpio_config(02, SC_FUN, PULL_NONE); // ����SDA����
    gpio_config(03, SC_FUN, PULL_NONE); // ����SCL����

    // ��������������Ϊ��2000 dps
    uint8_t gyro_range = 0x4C; // �������ݱ��е�GYRO_CONFIG0�Ĵ�������  //0~3 0101: 1.6k Hz    5~6    000: ��2000 dps
    write_sensor_data(0x20, gyro_range);

    // ���ü��ٶȼ�����Ϊ��16g
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
 * @param acc_FIFO �洢���ٶȼ����ݵ�����
 * @param gyro_FIFO �洢���������ݵ�����
 */
void get_gyro_data(short *acc_FIFO, short *gyro_FIFO) {
    uint8_t accel_x0 = 0, accel_y0 = 0, accel_z0 = 0, 
            accel_x1 = 0, accel_y1 = 0, accel_z1 = 0; 
    uint16_t accel_x = 0, accel_y = 0, accel_z = 0;
    uint8_t gyro_x0 = 0,  gyro_y0 = 0,  gyro_z0 = 0,  
            gyro_x1 = 0,  gyro_y1 = 0,  gyro_z1 = 0;  
    uint16_t gyro_x = 0,  gyro_y = 0,  gyro_z = 0;

    // ʾ������ȡ���ٶȼ�����
    accel_x1 = read_sensor_data(ACCEL_DATA_X1);    
    accel_x0 = read_sensor_data(ACCEL_DATA_X1 + 1); 
    accel_y1 = read_sensor_data(ACCEL_DATA_X1 + 2);  
    accel_y0 = read_sensor_data(ACCEL_DATA_X1 + 3); 
    accel_z1 = read_sensor_data(ACCEL_DATA_X1 + 4);   
    accel_z0 = read_sensor_data(ACCEL_DATA_X1 + 5);
    accel_x = (accel_x1 << 8) | accel_x0;
    accel_y = (accel_y1 << 8) | accel_y0;
    accel_z = (accel_z1 << 8) | accel_z0;
    acc_FIFO[0] = accel_x;
    acc_FIFO[1] = accel_y;
    acc_FIFO[2] = accel_z;

    // ʾ������ȡ����������
    gyro_x1 = read_sensor_data(GYRO_DATA_X1);    
    gyro_x0 = read_sensor_data(GYRO_DATA_X1 + 1);   
    gyro_y1 = read_sensor_data(GYRO_DATA_X1 + 2);   
    gyro_y0 = read_sensor_data(GYRO_DATA_X1 + 3);
    gyro_z1 = read_sensor_data(GYRO_DATA_X1 + 4);   
    gyro_z0 = read_sensor_data(GYRO_DATA_X1 + 5);
    gyro_x  = (gyro_x1 << 8) | gyro_x0;
    gyro_y  = (gyro_y1 << 8) | gyro_y0;
    gyro_z  = (gyro_z1 << 8) | gyro_z0;
    // �������ٶȼ�����
    gyro_FIFO[0] = gyro_x;
    gyro_FIFO[1] = gyro_y;
    gyro_FIFO[2] = gyro_z;
}

/**
 * @brief ��ȡ������Ĭ��ֵ������ƫ�Ʋ���
 * @param FIFO ���������������
 */
void gyro_read_defult(short *gyro_FIFO) {
    static short gyro_defult[3] = {0, 0, 0}; // ��̬��������������Ĭ��ֵ
    static uint8_t Len = 0; // ��̬������
    uint8_t i = 0;
    short a;

    // ���������������
    for (i = 0; i < 3; i++) {
        a = gyro_FIFO[i] - gyro_defult[i]; // ���㵱ǰֵ��Ĭ��ֵ�Ĳ�ֵ
        a = abs(a); // ȡ����ֵ

        // �����ֵ������ֵ������Ĭ��ֵ������ѭ��
        if (a > 0x001a) {
            Len = 0;
            gyro_defult[0] = gyro_FIFO[0];
            gyro_defult[1] = gyro_FIFO[1];
            gyro_defult[2] = gyro_FIFO[2];
            break;
        }
    }

    // ���������������ֵ������ƫ��������������������
    if (Len > 0x80) {
        Len = 0;
        for (i = 0; i < 3; i++) {
            GYRO_OFFSET[i] = (gyro_defult[i] + gyro_FIFO[i]) >> 1; // ����ƫ����
            gyro_FIFO[i] = 0; // ���õ�ǰֵ
        }
    } else {
        // ��ÿ��������ݽ��д���
        for (i = 0; i < 3; i++) {
            a = gyro_FIFO[i]; // ����ԭʼֵ
            gyro_FIFO[i] = gyro_FIFO[i] - GYRO_OFFSET[i]; // ��ȥƫ����

            // ������ݷ���ı��Ҿ���ֵ�ϴ󣬽��б��ʹ���
            if ((gyro_FIFO[i] ^ a) & 0x8000) {
                if (abs(a) > 0x0400) {
                    gyro_FIFO[i] = (a & 0x8000) ? 0x8000 : 0x7FFF;
                }
            }
        }
    }

    Len++; // ���Ӽ�����
}

/**
 * @brief �������˲��㷨
 * @param acc ���ٶȼ�����
 * @param gyro ����������
 * @param plane_data ���ڴ洢ƽ�� x �� y ���ݵĽṹ��ָ��
 */
void Kalman_filter(short *acc, short *gyro) {
    static short anglex, pre_accx[6], pre_accz[6], pre_gyrox[6], pre_gyroy[6], pre_gyroz[6];
    uint8_t i;
    short M, anglex_acc;
    uint16_t Rx;

    // ���»�����
    for (i = 0; i < 5; i++) {
        pre_accx[i] = pre_accx[i + 1];
        pre_accz[i] = pre_accz[i + 1];
        pre_gyrox[i] = pre_gyrox[i + 1];
        pre_gyroy[i] = pre_gyroy[i + 1];
        pre_gyroz[i] = pre_gyroz[i + 1];
    }
    pre_accx[5] = acc[0];
    pre_accz[5] = acc[2];
    pre_gyrox[5] = gyro[0];
    pre_gyroy[5] = gyro[1];
    pre_gyroz[5] = gyro[2];

    // �˲�����
    Kalman_four_buff_filter(pre_accx);
    pre_accx[5] = Kalman_filter_arg(pre_accx[4], pre_accx[5], 30);
    Kalman_four_buff_filter(pre_accz);
    pre_accz[5] = Kalman_filter_arg(pre_accz[4], pre_accz[5], 30);
    Kalman_four_buff_filter(pre_gyrox);
    Kalman_four_buff_filter(pre_gyroy);
    Kalman_four_buff_filter(pre_gyroz);

    // ����Ƕ�����
    M = Kalman_axis_limit(pre_gyroy[5], anglex);

    // ����Rxֵ
    Rx = abs(pre_gyrox[5]) >> 5;
    Rx += abs(pre_gyroy[5]) >> 5;
    Rx += abs(pre_gyroz[5]) >> 5;
    if (Rx / 256)
        Rx = 0xff;
    else
        Rx &= 0x00ff;
    Rx = Rx * Rx;

    // ����Hkֵ
    uint8_t Hk = Kalman_filter_Hk(Rx);

    // ����Ƕ�
    anglex_acc = Kalman_filter_get_angle(pre_accx[5], pre_accz[5], M);
    anglex_acc *= 256;
    anglex = Kalman_filter_arg(M, anglex_acc, Hk);

    // Ӧ�ýǶ�У��
    angle_line_arg(gyro, anglex);
}

/**
 * @brief �������˲��������� - ����Hkֵ
 * @param r �������ֵ
 * @return Hkֵ
 */
uint8_t Kalman_filter_Hk(uint16_t r) {
    static uint8_t anglex_cov = 10;
    static uint8_t pre_Hk = 0xff;
    uint8_t Hk;

    anglex_cov += 0x05;
    r += anglex_cov;
    Hk = anglex_cov / arg_bit16_to_bit8(r);

    anglex_cov = arg_bit16_to_bit8((256 - Hk) * anglex_cov);
    if (Hk > pre_Hk) {
        pre_Hk += 12;
        Hk = pre_Hk;
    } else {
        pre_Hk = Hk;
    }

    if (Hk < 40)
        Hk = 2;
    else
        Hk = 40;

    return Hk;
}

/**
 * @brief �������� - ��16λ����ת��Ϊ8λ
 * @param buff ��������
 * @return ת�����8λ����
 */
uint8_t arg_bit16_to_bit8(uint16_t buff) {
    union {
        uint16_t INT_data;
        uint8_t CHAR_data[2];
    } val;

    val.INT_data = buff;
    if (val.CHAR_data[1] & 0x80)
        val.CHAR_data[0]++;
    return val.CHAR_data[0];
}

// �������� - ��ֵ��һ����Χӳ�䵽��һ����Χ
int16_t map_value(int16_t value, int16_t from_min, int16_t from_max, int16_t to_min, int16_t to_max) {
    return (int16_t)((value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min);
}

/**
 * @brief �������˲��������� - �����˲�����
 * @param gy_ang �����ǽǶ�
 * @param acc_ang ���ٶȼƽǶ�
 * @param Hk Hkֵ
 * @return �˲���ĽǶ�
 */
short Kalman_filter_arg(short gy_ang, short acc_ang, uint8_t Hk) {
    uint8_t flag1;

    acc_ang -= gy_ang; // �������
    if (acc_ang & 0x8000) {  // ����
        acc_ang = -acc_ang;  // ȡ��
        flag1 = 1;
    } else {
        flag1 = 0;
    }

    acc_ang = acc_ang / (256 / Hk); // �������
    if (flag1)
        acc_ang = -acc_ang;

    return gy_ang + acc_ang; // �����˲���ĽǶ�
}

/**
 * @brief �������˲��������� - �Ļ������˲�
 * @param acc �������ݻ�����
 */
void Kalman_four_buff_filter(short *acc) {
    int32_t sum = 0;

    // �ۼӻ���������
    for (int i = 0; i < 6; i++) {
        sum += acc[i];
    }

    // ����ƽ��ֵ
    acc[5] = sum / 6;
}

/**
 * @brief �������˲��������� - ����Ƕ�����
 * @param gy ����������
 * @param angle ��ǰ�Ƕ�
 * @return ���ƺ�ĽǶ�
 */
/*-------------------------------------------------------------- 
 2000��=0x80*8ms  ��������ʵ���ƶ��Ƕ�=2000��/(0x80*125)*x=x*1/8
 ��angle��ϵΪ0x40=90��  1��=0x40/90��	�����x*1/8*0x40/90�� 
 x/11.25Ϊ��Ӧ�Ƕ�ֵ
 --------------------------------------------------------------*/

short Kalman_axis_limit(short gy, short angle) {
    gy = gy / 11; // ��������������
    return angle + gy; // �������ƺ�ĽǶ�
}

/**
 * @brief �������˲��������� - �������սǶ�
 * @param X ���ٶȼ�X������
 * @param Y ���ٶȼ�Y������
 * @param limit �Ƕ�����
 * @return ���սǶ�
 */
char Kalman_filter_get_angle(short X, short Y, short limit) {
    uint8_t flag1 = 0, flag2 = 0, flag3 = 0;
    char i;
    short P;
    short trig_x[2];

    trig_x[0] = limit;
    i = angle_get_arg(trig_x);

    if (trig_x[1] < 5793)
        flag3 = 0;
    else
        flag3 = 1;

    if (Y & 0x8000) {
        flag2 = 1;
        Y = -Y;
    }

    if (X & 0x8000) {
        X = -X;
        if (X < 8192)
            flag1 = 1;
        else if (X < 16384) {
            flag1 = 1;
            flag2 = ~flag2;
            X = 0x4000 - X;
        }
    } else {
        if (X > 0x2000) {
            flag2 = ~flag2;
            X = 0x4000 - X;
        }
    }

    short A = flag3 ? Y : X;

    for (i = 0; i < 46; i++) {
        if (A >= angle_Tab[i])
            break;
    }

    if (flag3)
        i = 45 - i;

    A = i * 64;
    i = A / 45;

    if (flag1)
        i = 0x80 - i;

    P = i;
    if (flag2)
        P = -P;

    return P;
}

/**
 * @brief �������˲��������� - Ӧ�ýǶ�У��
 * @param gyro ����������
 * @param angx ��ǰ�Ƕ�
 */
void angle_line_arg(short *gyro, short angx) {
    short trig_x[2];
    uint8_t m, flag = 0;
    int32_t B_word1, B_word2, B_word3, B_word4, B_word5;

    trig_x[0] = angx;
    m = angle_get_arg(trig_x);

    m = _swap_(m);
    m &= 0xf0;

    flag ^= 0x08;

    if (m & 0x10)
        flag ^= 0x00;
    if (m & 0x20)
        flag ^= 0x06;
    if (m & 0x40)
        flag ^= 0x0f;
    if (m & 0x80)
        flag ^= 0x09;

    if (gyro[2] & 0x8000) {
        flag ^= 0x05;
        gyro[2] = -gyro[2];
    }
    if (gyro[0] & 0x8000) {
        flag ^= 0x0a;
        gyro[0] = -gyro[0];
    }

    B_word4 = trig_x[0];
    B_word5 = gyro[2];
    B_word1 = B_word4 * B_word5;
    B_word1 = B_word1 / 4096;

    if (flag & 0x01)
        B_word1 = -B_word1;

    B_word4 = trig_x[1];
    B_word5 = gyro[0];
    B_word2 = B_word4 * B_word5;
    B_word2 = B_word2 / 4096;

    if (flag & 0x02)
        B_word2 = -B_word2;

    B_word3 = B_word1 + B_word2;

    if (B_word3 & 0x80000000) {
        B_word3 = -B_word3;
        if (B_word3 > 0x3f00)
            gyro[1] = 0xc100;
        else
            gyro[1] = B_word1 + B_word2;
    } else {
        if (B_word3 > 0x3F00)
            gyro[1] = 0x3F00;
        else
            gyro[1] = B_word1 + B_word2;
    }

    B_word4 = trig_x[1];
    B_word5 = gyro[2];
    B_word1 = B_word5 * B_word4;
    B_word1 = B_word1 / 4096;

    if (flag & 0x04)
        B_word1 = -B_word1;

    B_word4 = trig_x[0];
    B_word5 = gyro[0];
    B_word2 = B_word5 * B_word4;
    B_word2 = B_word2 / 4096;

    if (flag & 0x08)
        B_word2 = -B_word2;

    B_word3 = B_word1 + B_word2;

    if (B_word3 & 0x80000000) {
        B_word3 = -B_word3;
        if (B_word3 > 0x3f00)
            gyro[0] = 0xc100;
        else
            gyro[0] = B_word1 + B_word2;
    } else {
        if (B_word3 > 0x3F00)
            gyro[0] = 0x3F00;
        else
            gyro[0] = B_word1 + B_word2;
    }
}

/**
 * @brief �����ֽڵĸߵ�4λ
 * @param a �����ֽ�
 * @return ��������ֽ�
 */
uint8_t _swap_(uint8_t a) {
    uint8_t swapped = (a << 4) | (a >> 4);
    return swapped;
}

/**
 * @brief ����Ƕȵ����Һ�����ֵ
 * @param angle ����Ƕ����飨����һ���Ƕ�ֵ��
 * @return ��־λ����ʾ�Ƕ����ڵ�����
 */
uint8_t angle_get_arg(short *angle) {
    uint8_t flag = 0;
    uint16_t A;

    // ����Ƕ�Ϊ������ȡ����ֵ�����ñ�־λ
    if (angle[0] & 0x8000) {
        angle[0] = -angle[0];
        if (angle[0] > 0x4000) {
            angle[0] = 0x8000 - angle[0];
            flag |= 0x04;  // ��������
        } else {
            flag |= 0x08;  // ��������
        }
    } else {
        if (angle[0] > 0x4000) {
            angle[0] = 0x8000 - angle[0];
            flag |= 0x02;  // �ڶ�����
        } else {
            flag |= 0x01;  // ��һ����
        }
    }

    // ���Ƕ�ӳ�䵽���ұ�����
    A = angle[0] * 45;
    A >>= 6;

    // ����������ȡ���Һ�����ֵ
    angle[0] = angle_Tab[45 - A];  // ����ֵ
    angle[1] = angle_Tab[A];       // ����ֵ

    return flag;
}

/**
 * @brief �ݲ����
 * @param p �����ָ��
 */
void tolerance_sub(short *p) {
    short a;
    a = abs(*p);
    if (a > 0x10)
        a -= 0x10;
    else
        a = 0;
    if (*p & 0x8000)
        *p = (~a) + 1;
    else
        *p = a;
}

/**
 * @brief ���������ݴ�����
 * @param p �����ָ��
 * @param dpi ��������
 */
void gyro_div(short *p, char dpi) {
    char i;
    short m;
    m = *p >> 1;
    if (*p & 0x8000) {
        *p = -*p;
        *p <<= 1;
        *p = -*p;
    } else
        *p <<= 1;          // Ĭ�ϷŴ�1.5��
    if (dpi > 0) {
        if (dpi > 5)
            dpi = 5;
        for (i = 0; i < dpi; i++)
            *p += m;
    } else if (dpi & 0x80) {
        dpi = -dpi;
        if (dpi > 4)
            dpi = 4;
        for (i = 0; i < dpi; i++) {
            if (i > 1)
                *p = *p >> 1;
            else
                *p = *p - m;
        }
    }
}

/**
 * @brief �����˲��Ӻ���
 * @param FIFO ���������������
 */
void touch_filter_sub(short *FIFO) {
    static short gyro_pre_FIFO[2];
    static uint8_t gyro_coe[2], gyro_ct[2], gy_use_x, gy_use_y;
    uint8_t i;

    if (FIFO[0] || FIFO[1]) {
        gy_use_x = first_order_filter(&FIFO[0], &gyro_pre_FIFO[0], gy_use_x, &gyro_coe[0], &gyro_ct[0]);
        gy_use_y = first_order_filter(&FIFO[1], &gyro_pre_FIFO[1], gy_use_y, &gyro_coe[1], &gyro_ct[1]);
    } else {
        i = 0;
        do {
            gyro_pre_FIFO[i] = 0;
            gyro_ct[i] = 0;
            gyro_coe[i] = 80; // FITITER_COE_Min;
            i++;
        } while (i < 2);
    }
}

/**
 * @brief һ���˲�����
 * @param now ��ǰ����ָ��
 * @param old ��һ������ָ��
 * @param flag ��־λ
 * @param coe ϵ��ָ��
 * @param count ������ָ��
 * @return �˲���ı�־λ
 */
uint8_t first_order_filter(short *now, short *old, uint8_t flag, uint8_t *coe, uint8_t *count) {
    uint8_t m;
    short data0;

    if (*now == 0) { // ����Ϊ0����ǰ���������
        *count = 0;
        *coe = 80; // FITITER_COE_Min;
        *old = 0;
        return 1;
    }
    data0 = *now - *old;
    m = 1;
    if (data0 & 0x8000)
        m = 0;
    if (m == flag) {
        *count += data0 / 1024;
        *count += 1;
        if (*count > 8) {
            *count = 0;
            *coe += 8;
            if (*coe > 180) // FITITER_COE_Max)
                *coe = 180; // FITITER_COE_Max;
        }
    } else {
        *count = 0;                 // ����ı� ��������0��ϵ��ȡ��С
        *coe = 80; // FITITER_COE_Min;
    }

    data0 = data0 / 256;
    *old += *coe * data0;
    *now = *old / 256;

    if (m)
        return 1;
    else
        return 0;
}


short acc_data[3];
short gyro_data[3];
char buffer[50];
int16_t a[2];      // ���ڴ洢����������������
void mouse_init(){
		sensor_init();
}

extern int16_t mouse_X;
extern int16_t mouse_Y;
void mouse(){   //acc_data[i] i: 0,1,2 -> X Y Z��  gyro_data[i] i:0,1,2 -> 
			get_gyro_data(acc_data, gyro_data);
			gyro_read_defult(gyro_data);
			Kalman_filter(acc_data, gyro_data); 
			
			for(int i=0;i<2;i++) {
            tolerance_sub(&gyro_data[i]);
            gyro_div(&gyro_data[i], DPI_DATA);
        }

        touch_filter_sub(gyro_data);
        a[0] = gyro_data[0];
        a[1] = gyro_data[1];

        if(a[0] || a[1]) {
            a[1] = -a[1];
//					mouse_X = (mouse_X + 32767)/2;
//					mouse_Y = (mouse_Y + 32767)/2;
					
					mouse_X += a[0];
					mouse_Y += a[1];
					if(mouse_X + a[0] > 32767) mouse_X = 32767;
					else if(mouse_X + a[0] < 0) mouse_X = 0;
					else mouse_X += a[0];
					
					if(mouse_Y + a[1] > 32767) mouse_Y = 32767;
					else if(mouse_Y + a[1] < 0) mouse_Y = 0;
					else mouse_Y += a[1];
					//uart_printf("mouse_X: %d\r\n",mouse_X);
					//uart_printf("mouse_X: %d\r\n",mouse_Y);
				}
}