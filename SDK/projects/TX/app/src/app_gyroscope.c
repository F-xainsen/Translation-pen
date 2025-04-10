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
uint8_t cursor_speed = 1;
extern int16_t mouse_X;
extern int16_t mouse_Y;
//int16_t acc_raw[3], gyro_raw[3];
float angular_displacement[3], linear_displacement[3];
float new_ax, new_az;

static const int sinTable[91] = {0, 2, 3, 5, 7, 9, 10, 12, 14, 16, 17, 19, 21, 22, 24, 26, 28, 29, 31,
                                 33, 34, 36, 37, 39, 41, 42, 44, 45, 47, 48, 50, 52, 53, 54, 56, 57, 59, 60, 62, 63, 64, 66, 67, 68,
                                 69, 70, 72, 73, 74, 75, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 87, 88, 89, 90, 91, 91, 92, 93,
                                 93, 94, 95, 95, 96, 96, 97, 97, 97, 98, 98, 98, 99, 99, 99, 99, 100, 100, 100, 100, 100, 100};

static const int cosTable[91] = {100, 100, 100, 100, 100, 100, 99, 99, 99, 99, 98, 98, 98, 97, 97, 97,
                                 96, 96, 95, 95, 94, 93, 93, 92, 91, 91, 90, 89, 88, 87, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77,
                                 75, 74, 73, 72, 71, 69, 68, 67, 66, 64, 63, 62, 60, 59, 57, 56, 54, 53, 52, 50, 48, 47, 45, 44, 42,
                                 41, 39, 37, 36, 34, 33, 31, 29, 28, 26, 24, 22, 21, 19, 17, 16, 14, 12, 10, 9, 7, 5, 3, 2, 0};


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
	
}

// ԭʼ����ת������
void raw_to_physical(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
	imu_9.f_acc[0] = (float)(ax-imu_9.acc_zero[0])/ ACCEL_SSF;  // ���ٶ�����Ϊ:��2G      ��ȡ���ļ��ٶȼ����� ���� 16393 ������ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
	imu_9.f_acc[1] = (float)(ay-imu_9.acc_zero[1]) / ACCEL_SSF;
	imu_9.f_acc[2] = (float)(az-imu_9.acc_zero[2]) / ACCEL_SSF;

	imu_9.f_gyro[0] = (float)(gx-imu_9.gyro_zero[0])/GYRO_SSF; //  ����������Ϊ:��500dps  ��ȡ�������������ݳ��� 57.1��    ����ת��Ϊ������λ�����ݣ���λΪ����/s
	imu_9.f_gyro[1] = (float)(gy-imu_9.gyro_zero[1])/GYRO_SSF;
	imu_9.f_gyro[2] = (float)(gz-imu_9.gyro_zero[2])/GYRO_SSF;
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
    }
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
    degrees = degrees * 10;
    *new_ax = ax * COS(degrees) - az * SIN(degrees);
    *new_az = ax * SIN(degrees) + az * COS(degrees);
}

void Filter_threshold()
{
 if(abs(imu_9.i_gyro[0]) < 500 && abs(imu_9.i_gyro[1]) < 500 && abs(imu_9.i_gyro[2])< 500){
		imu_9.i_gyro[0] = imu_9.i_gyro[1] = imu_9.i_gyro[2] = 0;
	}
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
		Filter_threshold();
		
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

//����ŵ���-----��һЩ
static float invSqrt(float x){
	float halfx=0.5f*x;
	float y=x;
	long i=*(long*)&y;
	i=0x5f3759df-(i>>1);
	y=*(float*)&i;
	y=y*(1.5f-(halfx*y*y));
	return y;
}



typedef struct 
{
    float LastP;//�ϴι���Э���� ��ʼ��ֵΪ0.02
    float Now_P;//��ǰ����Э���� ��ʼ��ֵΪ0
    float out;//�������˲������ ��ʼ��ֵΪ0
    float Kg;//���������� ��ʼ��ֵΪ0
    float Q;//��������Э���� ��ʼ��ֵΪ0.001
    float R;//�۲�����Э���� ��ʼ��ֵΪ0.543
}KFP;//Kalman Filter parameter
#define Kp 15.0f                  // �����KpKi�����ڵ������ٶȼ����������ǵ��ٶ�
#define Ki 0.002f
#define dt   0.005
#define halfT 0.01f

KFP KFP_accelX={0.02,0,0,0,0.001,0.543};
KFP KFP_accelY={0.02,0,0,0,0.001,0.543};
KFP KFP_accelZ={0.02,0,0,0,0.001,0.543};
KFP KFP_gyroX={0.02,0,0,0,0.001,0.543};
KFP KFP_gyroY={0.02,0,0,0,0.001,0.543};
KFP KFP_gyroZ={0.02,0,0,0,0.001,0.543};
/**
 *�������˲���
 *@param KFP *kfp �������ṹ�����
 *   float input ��Ҫ�˲��Ĳ����Ĳ���ֵ�����������Ĳɼ�ֵ��
 *@return �˲���Ĳ���������ֵ��
 */
 float kalmanFilter(KFP *kfp,float input)
 {
     //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
     kfp->Now_P = kfp->LastP + kfp->Q;
     //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
     //����Э�����: ���ε�ϵͳЭ����� kfp->LastP ����һ������׼����
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
     return kfp->out;
 }
 
static float yaw = 0, pitch = 0, roll = 0;
//a���ٶ� g���ٶ�  a����>g   g->dps
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
void algorithm(float ax,float ay,float az,float gx,float gy,float gz){
	gx=gx*0.0174f;//��dpsת��Ϊ����
	gy=gy*0.0174f;
	gz=gz*0.0174f;
	
	float recip;//ƽ��������
	recip=invSqrt(ax*ax+ay*ay+az*az);
	//��ȡ��̬�������������
	float Vx,Vy,Vz;
	Vx=2*(q1*q3-q0*q2);
	Vy=2*(q0*q1+q2*q3);
	Vz=1-2*q1*q1-2*q2*q2;
	//����̬���
	float ex,ey,ez;
	ex=ay*Vz-az*Vy;
	ey=az*Vx-ax*Vz;
	ez=ax*Vy-ay*Vx;
	//�������
	float accex=0,accey=0,accez=0;
  accex=accex+ex*Ki*dt;
	accey=accey+ey*Ki*dt;
	accez=accez+ez*Ki*dt;
	//�����˲���ʹ��pid�����������
	gx=gx+Kp*ex+accex;
	gy=gy+Kp*ey+accey;
	gz=gz+Kp*ez+accez;
	//����Ԫ��΢�ַ�������
	gx=gx*(0.5f*dt);
	gy=gy*(0.5f*dt);
	gz=gz*(0.5f*dt);
	q0=q0+(-q1*gx-q2*gy-q3*gz);
	q1=q1+(q0*gx+q2*gz-q3*gy);
	q2=q2+(q0*gy-q1*gz+q3*gx);
	q3=q3+(q0*gz+q1*gy-q2*gx);
	//��һ��
	recip=invSqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0=q0*recip;
	q1=q1*recip;
	q2=q2*recip;
	q3=q3*recip;
	//������̬��
  float g1,g2,g3,g4,g5;
	g1=2.0f*(q1*q3-q0*q2);
	g2=2.0f*(q0*q1+q2*q3);
	g3=q0*q0-q1*q1-q2*q2+q3*q3;
	g4=2.0f*(q1*q2+q0*q3);
	g5=q0*q0+q1*q1-q2*q2-q3*q3;
	
  pitch=-asinf(g1)*57.3f;
//	roll=atanf(g2/g3)*57.3f;
//	yaw=atanf(g4/g5)*57.3f;
		roll=atan2f(g2,g3)*57.3f;
		yaw=atan2f(g4,g5)*57.3f;
  uart_printf("%d, %d, %d\r\n",(int16_t)(pitch),(int16_t)(roll),(int16_t)(yaw));
}


// ���ݼ��ٶȼ� X �� Z ���ݼ��� pitch �ǣ���λ?��
float calculate_acc_angle(int16_t* acc_raw , int16_t* gyro_raw) {  // 0->x, 2->z
	int ax, az;
		ax = imu_9.i_acc[0];
    az = imu_9.i_acc[2];

				roll = fabs(pitch);
				//uart_printf("fabs_roll:%d\r\n",(int16_t)(roll * 10));
			if(ax > 0 && az < 0){                // 1
				roll = roll;
			}else if(ax > 0 && az > 0){      // 2
				roll = 180 - roll;
			}else if(ax < 0 && az > 0){      // 3
				roll = 180 + roll;
			}else if(ax < 0 && az < 0){       // 4
				roll = 360 - roll;
			}
			
			if(roll > 70 && roll < 110)roll = 90;
			if(roll > 250 && roll < 290)roll = 270;
			
			if((roll < 20 && roll > 0) || (roll < 360 && roll > 340))roll = 0;
			if(roll > 160 && roll < 200)roll = 180;
			uart_printf("roll:%d\r\n",(int16_t)(roll * 10));
			return roll;
	 }

void mouse_init()
{
    // ��ʼ��I2C�����ôӻ���ַΪ0x69��������Ϊ400kHz
    i2c_init(I2C_ADDR, 0);

    // ����SDA��SCL����
    gpio_config(02, SC_FUN, PULL_NONE); // ����SDA����
    gpio_config(03, SC_FUN, PULL_NONE); // ����SCL����

    // ��������������Ϊ��500 dps
    uint8_t gyro_range = 0x49; // �������ݱ��е�GYRO_CONFIG0�Ĵ�������  //0~3 0101: 100k Hz    5~6    000: ��2000 dps
    write_sensor_data(0x20, gyro_range);

    // ���ü��ٶȼ�����Ϊ��4g
    uint8_t accel_range = 0x49; // �������ݱ��е�ACCEL_CONFIG0�Ĵ�������
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
float ax,ay,az,gx,gy,gz;
float krm[6]={0};
void mouse()
{
		imu_final_data_get();

		krm[0]=kalmanFilter(&KFP_accelX,imu_9.f_acc[0]);//�������˲�
		krm[1]=kalmanFilter(&KFP_accelY,imu_9.f_acc[1]);
		krm[2]=kalmanFilter(&KFP_accelZ,imu_9.f_acc[2]);
		krm[3]=kalmanFilter(&KFP_gyroX,imu_9.f_gyro[0]);
		krm[4]=kalmanFilter(&KFP_gyroY,imu_9.f_gyro[1]);
		krm[5]=kalmanFilter(&KFP_gyroZ,imu_9.f_gyro[2]);
		algorithm(krm[0],krm[1],krm[2],krm[3],krm[4],krm[5]);//���ÿ������˲�

		roll = calculate_acc_angle((int16_t *)imu_9.i_acc, (int16_t *)imu_9.i_gyro);
    calculate_displacement(imu_9.f_gyro, angular_displacement, linear_displacement);
		
    rotate_point(linear_displacement[2], linear_displacement[0], roll, &new_ax, &new_az);

    ax_sum += new_ax * cursor_speed;
    az_sum += new_az * cursor_speed; // cursor_speed
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
}
