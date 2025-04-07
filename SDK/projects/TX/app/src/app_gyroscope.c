#include "app_gyroscope.h"
#include "user_handle.h"
#include "i2c.h"
#include "app.h"
#include "gpio.h"
#include <math.h>
#include <stdint.h> // �����׼�������Ͷ���
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

imu_t imu_9;
float numbers[100];
uint8_t cursor_speed = 1;
extern int16_t mouse_X;
extern int16_t mouse_Y;
int16_t acc_raw[3], gyro_raw[3], gyro_filter[3];
float acc_g[3], gyro_g[3], angular_displacement[3], linear_displacement[3];
float new_ax, new_az;
float gyro_filtered[MOUSE_DATA_LENGTH], acc_filtered[MOUSE_DATA_LENGTH];
KalmanFilter kf;
float init_angle = 0;

int16_t angle_Tab2[46] = // 4gmax  8bit  1=31.25mg
    {
        8192, // 90
        8187, // 88��
        8172, // 86��
        8147, // 84��
        8113, // 82��
        8067, // 80��
        8013, // 78��
        7949, // 76��

        7875, // 74��
        7791, // 72��
        7698, // 70��
        7596, // 68��
        7483, // 66��
        7363, // 64��
        7233, // 62��
        7094, // 60��

        6947, // 58��
        6791, // 56��
        6627, // 54��
        6455, // 52
        6275, // 50��
        6087, // 48��
        5893, // 46��
        5691, // 44��

        5481, // 42��
        5266, // 40��
        5044, // 38��
        4815, // 36��
        4581, // 34��
        4341, // 32��
        4096, // 30��
        3946, // 28��

        3591, // 26��
        3332, // 24��
        3069, // 22��
        2802, // 20��
        2531, // 18��
        2258, // 16��
        1982, // 14��
        1703, // 12��

        1422, // 10��
        1140, // 8��
        856,  // 6��
        572,  // 4��
        286,  // 2��
        0     // 0��
};

static const int sinTable[91] = {0, 2, 3, 5, 7, 9, 10, 12, 14, 16, 17, 19, 21, 22, 24, 26, 28, 29, 31,
                                 33, 34, 36, 37, 39, 41, 42, 44, 45, 47, 48, 50, 52, 53, 54, 56, 57, 59, 60, 62, 63, 64, 66, 67, 68,
                                 69, 70, 72, 73, 74, 75, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 87, 88, 89, 90, 91, 91, 92, 93,
                                 93, 94, 95, 95, 96, 96, 97, 97, 97, 98, 98, 98, 99, 99, 99, 99, 100, 100, 100, 100, 100, 100};

static const int cosTable[91] = {100, 100, 100, 100, 100, 100, 99, 99, 99, 99, 98, 98, 98, 97, 97, 97,
                                 96, 96, 95, 95, 94, 93, 93, 92, 91, 91, 90, 89, 88, 87, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77,
                                 75, 74, 73, 72, 71, 69, 68, 67, 66, 64, 63, 62, 60, 59, 57, 56, 54, 53, 52, 50, 48, 47, 45, 44, 42,
                                 41, 39, 37, 36, 34, 33, 31, 29, 28, 26, 24, 22, 21, 19, 17, 16, 14, 12, 10, 9, 7, 5, 3, 2, 0};

// // ��̬�������ڿ������˲�
short GYRO_OFFSET[3] = {0, 0, 0};
// static char DPI_DATA = 0;
// // ��ͨ�˲�����
// static float filtered_pitch = 0.0;
// // z ��λ����ر���
// static float velocity_x = 0.0, displacement_x = 0.0;
// static float velocity_z = 0.0, displacement_z = 0.0;

void floatToString(float num, char *str, int decimalPlaces)
{
    // ������
    if (num < 0)
    {
        num = -num;
        *str++ = '-';
    }

    // ����������������
    float roundingFactor = 0.5;
    for (int i = 0; i < decimalPlaces; i++)
    {
        roundingFactor /= 10.0;
    }
    num += roundingFactor;

    // ����������С������
    int integerPart = (int)num;
    float fractionalPart = num - integerPart;

    // ת����������
    str += sprintf(str, "%d", integerPart);

    // ����С������
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

// ������ӡ������
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
 * @brief ��ȡ�������Ĵ�������
 * @param reg_addr �Ĵ�����ַ
 * @return ��ȡ������
 */
uint8_t read_sensor_data(uint8_t reg_addr)
{
    uint8_t data;
    i2c_read(I2C_ADDR, reg_addr, &data, 1);
    return data;
}

/**
 * @brief д�봫�����Ĵ�������
 * @param reg_addr �Ĵ�����ַ
 * @param data Ҫд�������
 */
void write_sensor_data(uint8_t reg_addr, uint8_t data)
{
    i2c_write(I2C_ADDR, reg_addr, &data, 1);
}

/**
 * @brief ��ȡ�����Ǻͼ��ٶȼ�����
 * @param acc_raw �洢���ٶȼ����ݵ�����
 * @param gyro_raw �洢���������ݵ�����
gyro_raw: 0->y  1->x  2->x   �Ƕ�
acc_raw��0->x, 1->y, 2->z  ���ٶ�

 */
void get_gyro_data(int16_t *gyro_raw, int16_t *acc_raw)
{
    uint8_t *acc_raw_8 = (uint8_t *)acc_raw;
    uint8_t *gyro_raw_8 = (uint8_t *)gyro_raw;
    i2c_read(I2C_ADDR, ACCEL_DATA_X1, acc_raw_8, 6); // ���
    i2c_read(I2C_ADDR, GYRO_DATA_X1, gyro_raw_8, 6);
    
    acc_raw[0] = (acc_raw_8[0] << 8) + acc_raw_8[1];
    acc_raw[1] = (acc_raw_8[2] << 8) + acc_raw_8[3];
    acc_raw[2] = (acc_raw_8[4] << 8) + acc_raw_8[5];
    
    gyro_raw[0] = (gyro_raw_8[0] << 8) + gyro_raw_8[1];
    gyro_raw[1] = (gyro_raw_8[2] << 8) + gyro_raw_8[3];
    gyro_raw[2] = (gyro_raw_8[4] << 8) + gyro_raw_8[5];


//    for (int i = 0; i < MOUSE_DATA_LENGTH; i++)
//    {
//        acc_raw[i] = (acc_raw[i] > ACCEL_MAX) ? ACCEL_MAX : ((acc_raw[i] < ACCEL_MIN) ? ACCEL_MIN : acc_raw[i]);
//        gyro_raw[i] = (gyro_raw[i] > MOUSE_MAX) ? MOUSE_MAX : ((gyro_raw[i] < MOUSE_MIN) ? MOUSE_MIN : gyro_raw[i]);
//    }
}

// ԭʼ����ת������
void raw_to_physical(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
	imu_9.f_acc[0] = (float)(ax-imu_9.acc_zero[0])/ ACCEL_SSF;  // ���ٶ�����Ϊ:��2G      ��ȡ���ļ��ٶȼ����� ���� 16393 ������ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
	imu_9.f_acc[1] = (float)(ay-imu_9.acc_zero[1]) / ACCEL_SSF;
	imu_9.f_acc[2] = (float)(az-imu_9.acc_zero[2]) / ACCEL_SSF;

	imu_9.f_gyro[0] = (float)(gx-imu_9.gyro_zero[0])/65.5; //  ����������Ϊ:��500dps  ��ȡ�������������ݳ��� 57.1��    ����ת��Ϊ������λ�����ݣ���λΪ����/s
	imu_9.f_gyro[1] = (float)(gy-imu_9.gyro_zero[1])/65.5;
	imu_9.f_gyro[2] = (float)(gz-imu_9.gyro_zero[2])/65.5;

}

// �������ʱ��������λ���룩
#define SAMPLE_TIME 0.005f
#define MOUSE_DATA_LENGTH 3
// �������ӵİ뾶����λ���ף�
#define WHEEL_RADIUS 1.0f

void calculate_displacement(float *gyro_g, float *angular_displacement, float *linear_displacement)
{
    static float prev_gyro[MOUSE_DATA_LENGTH] = {0}; // ��һ�εĽ��ٶ�ֵ

    for (int i = 0; i < MOUSE_DATA_LENGTH; i++)
    {
        // ��ԭʼ���ٶ�����ת��Ϊ������
        float current_gyro = gyro_g[i] * 20;

        angular_displacement[i] = 0;
        // ʹ�����η�����л��֣������λ��
        angular_displacement[i] += (prev_gyro[i] + current_gyro) * SAMPLE_TIME;

        // ����ֱ��λ�ƣ����������ӵ���ת��λ��=�뾶����λ�ƣ�
        linear_displacement[i] = WHEEL_RADIUS * angular_displacement[i];

        // ������һ�εĽ��ٶ�ֵ
        prev_gyro[i] = current_gyro;

        //			numbers[0] = gyro_g[0] ;
        //			numbers[1] = gyro_g[2] ;
        //			numbers[2] = linear_displacement[0] ;
        //			numbers[3] = linear_displacement[2] ;
        //		 printFloats(numbers, 4, 2);
    }
}
static float last_angle = 0.0f;
static int last_ax = 0, last_az = 0;
int ax, az, temp;
int16_t value;
// ���ݼ��ٶȼ� X �� Z ���ݼ��� pitch �ǣ���λ?��
float calculate_acc_angle(int16_t* acc_raw , int16_t* gyro_raw) {  // 0->x, 2->z
	
		ax = acc_raw[0];
    az = acc_raw[2];
        // ԭ�е��߼�
        for (int i = 0; i < 46; i++) {
            temp = abs(ax);
            if (temp >= angle_Tab2[i + 1] && temp < angle_Tab2[i]) {
                value = 90 - i * 2;
                break;
            }
        }
				
			//float value =  ax * 90.0 / 8192;
			if(ax > 0 && az < 0){
				 value = value;  // 1
			}else if(ax > 0 && az > 0){      // 2
				 value = 180 - value;
			}else if(ax < 0 && az > 0){      // 3
				 value = 180 + value;
			}else if(ax < 0 && az < 0){            // 4
				 value = 360 - value;
			}
			
			return value;
	 }	


//һ�׻����˲�
float K1 =0.1; // �Լ��ٶȼ�ȡֵ��Ȩ��
float dt=0.001;//ע�⣺dt��ȡֵΪ�˲�������ʱ��
float angle_P,angle_R;

float yijiehubu_P(float angle_m, float gyro_m)//�ɼ������ĽǶȺͽǼ��ٶ�
{
     angle_P = K1 * angle_m + (1-K1) * (angle_P + gyro_m * dt);
         return angle_P;
}
 
float yijiehubu_R(float angle_m, float gyro_m)//�ɼ������ĽǶȺͽǼ��ٶ�
{
     angle_R = K1 * angle_m + (1-K1) * (angle_R + gyro_m * dt);
         return angle_R;
}



// ���Ǻ����ļ���
int common_sin_val_calculate(int angle)
{
    angle = (angle + 3600) % 3600; // ��һ���Ƕȵ�0 ~ 3600��Χ��

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

/* ��������ֵ
 * angle: ����Ƕȣ�0 ~ 3600����Ӧ0 ~ 360�ȣ�ÿ����λ��ʾ0.1�ȣ�
 * return: cos(angle) * 100
 */
int common_cos_val_calculate(int angle)
{
    angle = (angle + 3600) % 3600; // ��һ���Ƕȵ�0 ~ 3600��Χ��

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
void rotate_point(float ax, float az, float degrees, float *new_ax, float *new_az)
{
    // ���Ƕ�ת��Ϊ����
    //	float radians = degrees *M_PI / 180.0f;
    //	float cos_theta = cosf(radians);
    //	float sin_theta = sinf(radians);
    //
    //		*new_ax = ax * cos_theta - az * sin_theta;
    //    *new_az = ax * sin_theta + az * cos_theta;
    degrees = degrees * 10;
    *new_ax = ax * COS(degrees) - az * SIN(degrees);
    *new_az = ax * SIN(degrees) + az * COS(degrees);

//    numbers[0] = degrees;
//    numbers[1] = COS(degrees);
//    numbers[2] = SIN(degrees);
//    printFloats(numbers, 3, 2);
}

void Filter_threshold(int16_t *gyro_raw, int16_t *gyro_filter)
{
    for (int i = 0; i < 3; i++)
    {
        if (gyro_raw[i] < 1000 && gyro_raw[i] > -1000)
        {
            gyro_filter[i] = 0; // ��Ϊ 0
        }
        else
        {
            gyro_filter[i] = gyro_raw[i];
        }
    }
}

// ������У׼����ֹʱ����100��ȡ��ֵ��
void calibrate_gyro(void)
{
    const int num_samples = 100;
    long sum[3] = {0, 0, 0};
    int16_t gyro_raw[3];

    for (int i = 0; i < num_samples; i++)
    {
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
float low_pass_filter(float current, float prev_filtered, float alpha)
{
    return alpha * current + (1.0 - alpha) * prev_filtered;
}

//�����˲�
int16_t window_filter(int16_t data, int16_t *buf, uint8_t len)
{
	uint8_t i;
	int32_t sum = 0;

	for (i = 1; i < len; i++)
	{
		buf[i - 1] = buf[i];
	}
	buf[len - 1] = data;

	for (i = 0; i < len; i++)
	{
		sum += buf[i];
	}

	sum /= len;

	return sum;
}


int16_t window_ax[WIN_NUM];
int16_t window_ay[WIN_NUM];
int16_t window_az[WIN_NUM];

int16_t window_gx[WIN_NUM];
int16_t window_gy[WIN_NUM];
int16_t window_gz[WIN_NUM];
void imu_final_data_get(void)
{
	int16_t filter_ax ,filter_ay,filter_az ;
	int16_t filter_gx ,filter_gy,filter_gz ;

	  get_gyro_data(imu_9.i_gyro,imu_9.i_acc); //�����õ�����ԭʼ����
	  //ԭʼ���ݴ����˲�
	  filter_ax = window_filter(imu_9.i_acc[0],window_ax,WIN_NUM);
	  filter_ay = window_filter(imu_9.i_acc[1],window_ay,WIN_NUM);
	  filter_az = window_filter(imu_9.i_acc[2],window_az,WIN_NUM);

	  filter_gx = window_filter(imu_9.i_gyro[0],window_gx,WIN_NUM);
	  filter_gy = window_filter(imu_9.i_gyro[1],window_gy,WIN_NUM);
	  filter_gz = window_filter(imu_9.i_gyro[2],window_gz,WIN_NUM);
	  //ת����ʵ��������
		raw_to_physical(filter_ax,filter_ay,filter_az,filter_gx,filter_gy,filter_gz);
}

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // ��ʼ��̬��Ԫ��������ƪ�����ᵽ�ı任��Ԫ����ʽ����
float exInt = 0, eyInt = 0, ezInt = 0; // ��ǰ�ӼƲ�õ��������ٶ��������ϵķ���
                                       // ���õ�ǰ��̬��������������������ϵķ��������Ļ���

static float yaw = 0;
static float pitch = 0;
static float roll = 0;
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) // g�������ǣ�a��Ӽ�
{
    float q0temp, q1temp, q2temp, q3temp; // ��Ԫ���ݴ���������΢�ַ���ʱҪ��
    float norm;                           // ʸ����ģ����Ԫ���ķ���
    float vx, vy, vz;                     // ��ǰ��̬��������������������ϵķ���
    float ex, ey, ez;                     // ��ǰ�ӼƲ�õ��������ٶ��������ϵķ���
																					// ���õ�ǰ��̬��������������������ϵķ��������
    // �Ȱ���Щ�õõ���ֵ���
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

//    if (ax * ay * az == 0) // �Ӽƴ�����������״̬ʱ��������̬���㣬��Ϊ�������ĸ���������
//        return;
		

    norm = sqrt(ax * ax + ay * ay + az * az); // ��λ�����ٶȼƣ�
    ax = ax / norm;                           // �������������Ҳ����Ҫ�޸�KP��������Ϊ�����һ����
    ay = ay / norm;
    az = az / norm;
    // �õ�ǰ��̬������������������ϵķ�����
    // �ο�����nϵת������������bϵ������Ԫ����ʾ�ķ������Ҿ�������м��ǣ�����һ�����ᵽ��
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    // �����õ������������������������������Ա�ʾ��һ���
    // ԭ�����������Ϊ���������ǵ�λ������sin0����0
    // ����Ҫ�Ǽн���180����~�����û���
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
		// �������������Ki����Ư������ǿ�ȣ�
    exInt = exInt + ex * Ki; // �������л���
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;
		//���������ǲ���ֵ��Kp���Ƽ��ٶȼ�Ȩ�أ�
    // adjusted gyroscope measurements
    gx = gx + Kp * ex + exInt; // �����PI�󲹳��������ǣ����������Ư��
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt; // �����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�
    // ���������̬�ĸ��£�Ҳ������Ԫ��΢�ַ��̵����
    q0temp = q0; // �ݴ浱ǰֵ���ڼ���
    q1temp = q1; // ���ϴ�������㷨���û��ע��������⣬�ڴ˸���
    q2temp = q2;
    q3temp = q3;
    // ����һ�ױϿ��ⷨ�����֪ʶ�ɲμ���������������Ե���ϵͳ��P212
    q0 = q0temp + (-q1temp * gx - q2temp * gy - q3temp * gz) * halfT;
    q1 = q1temp + (q0temp * gx + q2temp * gz - q3temp * gy) * halfT;
    q2 = q2temp + (q0temp * gy - q1temp * gz + q3temp * gx) * halfT;
    q3 = q3temp + (q0temp * gz + q1temp * gy - q2temp * gx) * halfT;
    // ��λ����Ԫ���ڿռ���תʱ�������죬������ת�Ƕȣ����������Դ�����������任
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    // ��Ԫ����ŷ���ǵ�ת������ʽ�Ƶ�������һ
    // ����YAW��������ڼ��ٶȼƶ���û���������ã���˴˴�ֱ���������ǻ��ִ���
		
    yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;  // unit:degree
    pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                // unit:degree
    roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // unit:degree
		numbers[0] = roll;
//    numbers[1] = pitch;
//    numbers[2] = yaw;
    printFloats(numbers, 1, 2);
}


void mouse_init()
{
    // ��ʼ��I2C�����ôӻ���ַΪ0x69��������Ϊ400kHz
    i2c_init(I2C_ADDR, 0);

    // ����SDA��SCL����
    gpio_config(02, SC_FUN, PULL_NONE); // ����SDA����
    gpio_config(03, SC_FUN, PULL_NONE); // ����SCL����

    // ��������������Ϊ��500 dps
    uint8_t gyro_range = 0x45; // �������ݱ��е�GYRO_CONFIG0�Ĵ�������  //0~3 0101: 1.6k Hz    5~6    000: ��2000 dps
    write_sensor_data(0x20, gyro_range);

    // ���ü��ٶȼ�����Ϊ��4g
    uint8_t accel_range = 0x45; // �������ݱ��е�ACCEL_CONFIG0�Ĵ�������
    write_sensor_data(0x21, accel_range);

    // ���õ�Դ����Ĵ�����ʹ�������Ǻͼ��ٶȼ�
    uint8_t pwr_mgmt = 0x0F; // PWR_MGMT0�Ĵ�����ʹ�����д�����
    write_sensor_data(0x1F, pwr_mgmt);

    i2c_init(I2C_ADDR, 0);
}

double az_sum = 0;
double ax_sum = 0;
double pos_t = 0;

double pre_x = 0;
double pre_y = 0;

void mouse()
{

    get_gyro_data(gyro_raw, acc_raw);
    Filter_threshold(gyro_raw, gyro_filter);
    //uart_printf("gyro_raw: 	X=%d, Y=%d, Z=%d \r\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
    //uart_printf("gyro_filter: 	X=%d, Y=%d, Z=%d \r\n", acc_raw[0], acc_raw[1], acc_raw[2]);

    
		//IMUupdate(gyro_g[0], gyro_g[1], gyro_g[2], acc_g[0], acc_g[1], acc_g[2]);

    calculate_displacement(gyro_g, angular_displacement, linear_displacement);
		uart_printf("gyro_filter: 	X=%d, Y=%d, Z=%d \r\n", (int16)gyro_g[0], (int16)gyro_g[1], (int16)gyro_g[2]);
    float acc_angle_x = calculate_acc_angle(acc_raw, gyro_raw);
		float angle_p = yijiehubu_P(gyro_g[2], acc_angle_x);
    rotate_point(linear_displacement[2], linear_displacement[0], angle_p, &new_ax, &new_az);

    ax_sum += linear_displacement[2] * cursor_speed;
    az_sum += linear_displacement[0] * cursor_speed; // cursor_speed
    mouse_X = ax_sum;
    mouse_Y = -az_sum;

    ax_sum -= mouse_X;
    az_sum += mouse_Y;

    if (single_monitor_flag)
    {
        current_mouse_x += mouse_X;
        current_mouse_y += mouse_Y;
        if (current_mouse_x < 0)
            current_mouse_x = 0;
        if (current_mouse_y < 0)
            current_mouse_y = 0;
        if (current_mouse_x > resolution_Width)
            current_mouse_x = resolution_Width;
        if (current_mouse_y > resolution_Height)
            current_mouse_y = resolution_Height;

        pos_t = current_mouse_x / pre_x;
        mouse_X = (int16_t)pos_t;

        pos_t = current_mouse_y / pre_y;
        mouse_Y = (int16_t)pos_t;

    }

    // mouse_X = (mouse_X > MOUSE_MAX)?MOUSE_MAX :(mouse_X < MOUSE_MIN ? MOUSE_MIN : mouse_X);
    // mouse_Y = (mouse_Y > MOUSE_MAX)?MOUSE_MAX :(mouse_Y < MOUSE_MIN ? MOUSE_MIN : mouse_Y);

//    numbers[0] = roll;
//    numbers[1] = pitch;
//    numbers[2] = yaw;
//    numbers[3] = new_ax;
//    numbers[4] = new_az;
//    numbers[5] = mouse_X;
//    numbers[6] = mouse_Y;
//    printFloats(numbers, 3, 2);
}
