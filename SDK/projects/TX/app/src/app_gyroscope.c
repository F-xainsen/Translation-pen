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
 * @brief 读取传感器寄存器数据
 * @param reg_addr 寄存器地址
 * @return 读取的数据
 */
uint8_t read_sensor_data(uint8_t reg_addr)
{
    uint8_t data;
    i2c_read(I2C_ADDR, reg_addr, &data, 1);
    return data;
}
/**
 * @brief 写入传感器寄存器数据
 * @param reg_addr 寄存器地址
 * @param data 要写入的数据
 */
void write_sensor_data(uint8_t reg_addr, uint8_t data)
{
    i2c_write(I2C_ADDR, reg_addr, &data, 1);
}

/**
 * @brief 获取陀螺仪和加速度计数据
 * @param acc_raw 存储加速度计数据的数组
 * @param gyro_raw 存储陀螺仪数据的数组
gyro_raw: 0->y  1->x  2->x   角度
acc_raw：0->x, 1->y, 2->z  加速度
 */
void get_gyro_data(int16_t *gyro_raw, int16_t *acc_raw)
{
    uint8_t *acc_raw_8 = (uint8_t *)acc_raw;
    uint8_t *gyro_raw_8 = (uint8_t *)gyro_raw;
    i2c_read(I2C_ADDR, ACCEL_DATA_X1, acc_raw_8, 6); // 大端
    i2c_read(I2C_ADDR, GYRO_DATA_X1, gyro_raw_8, 6);
    
    acc_raw[0] = (acc_raw_8[0] << 8) + acc_raw_8[1];
    acc_raw[1] = (acc_raw_8[2] << 8) + acc_raw_8[3];
    acc_raw[2] = (acc_raw_8[4] << 8) + acc_raw_8[5];
    
    gyro_raw[0] = (gyro_raw_8[0] << 8) + gyro_raw_8[1];
    gyro_raw[1] = (gyro_raw_8[2] << 8) + gyro_raw_8[3];
    gyro_raw[2] = (gyro_raw_8[4] << 8) + gyro_raw_8[5];
	
}

// 原始数据转物理量
void raw_to_physical(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
	imu_9.f_acc[0] = (float)(ax-imu_9.acc_zero[0])/ ACCEL_SSF;  // 加速度量程为:±2G      获取到的加速度计数据 除以 16393 ，可以转化为带物理单位的数据，单位：g(m/s^2)
	imu_9.f_acc[1] = (float)(ay-imu_9.acc_zero[1]) / ACCEL_SSF;
	imu_9.f_acc[2] = (float)(az-imu_9.acc_zero[2]) / ACCEL_SSF;

	imu_9.f_gyro[0] = (float)(gx-imu_9.gyro_zero[0])/GYRO_SSF; //  陀螺仪量程为:±500dps  获取到的陀螺仪数据除以 57.1，    可以转化为带物理单位的数据，单位为：°/s
	imu_9.f_gyro[1] = (float)(gy-imu_9.gyro_zero[1])/GYRO_SSF;
	imu_9.f_gyro[2] = (float)(gz-imu_9.gyro_zero[2])/GYRO_SSF;
}

// 定义采样时间间隔（单位：秒）
#define SAMPLE_TIME 0.005f
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


//滑动滤波
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

	  get_gyro_data(imu_9.i_gyro,imu_9.i_acc); //采样得到九轴原始数据
		Filter_threshold();
		
	  //原始数据窗口滤波
	  filter_ax = window_filter(imu_9.i_acc[0],window_ax,WIN_NUM);
	  filter_ay = window_filter(imu_9.i_acc[1],window_ay,WIN_NUM);
	  filter_az = window_filter(imu_9.i_acc[2],window_az,WIN_NUM);
		
	  filter_gx = window_filter(imu_9.i_gyro[0],window_gx,WIN_NUM);
	  filter_gy = window_filter(imu_9.i_gyro[1],window_gy,WIN_NUM);
	  filter_gz = window_filter(imu_9.i_gyro[2],window_gz,WIN_NUM);
	  //转换成实际物理量
		raw_to_physical(filter_ax,filter_ay,filter_az,filter_gx,filter_gy,filter_gz);
}

//算根号倒数-----快一些
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
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001
    float R;//观测噪声协方差 初始化值为0.543
}KFP;//Kalman Filter parameter
#define Kp 15.0f                  // 这里的KpKi是用于调整加速度计修正陀螺仪的速度
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
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
 float kalmanFilter(KFP *kfp,float input)
 {
     //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
     kfp->Now_P = kfp->LastP + kfp->Q;
     //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
     //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
     return kfp->out;
 }
 
static float yaw = 0, pitch = 0, roll = 0;
//a加速度 g角速度  a——>g   g->dps
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
void algorithm(float ax,float ay,float az,float gx,float gy,float gz){
	gx=gx*0.0174f;//将dps转换为弧度
	gy=gy*0.0174f;
	gz=gz*0.0174f;
	
	float recip;//平方根倒数
	recip=invSqrt(ax*ax+ay*ay+az*az);
	//提取姿态矩阵的重力分量
	float Vx,Vy,Vz;
	Vx=2*(q1*q3-q0*q2);
	Vy=2*(q0*q1+q2*q3);
	Vz=1-2*q1*q1-2*q2*q2;
	//求姿态误差
	float ex,ey,ez;
	ex=ay*Vz-az*Vy;
	ey=az*Vx-ax*Vz;
	ez=ax*Vy-ay*Vx;
	//积分误差
	float accex=0,accey=0,accez=0;
  accex=accex+ex*Ki*dt;
	accey=accey+ey*Ki*dt;
	accez=accez+ez*Ki*dt;
	//互补滤波，使用pid进行误差修正
	gx=gx+Kp*ex+accex;
	gy=gy+Kp*ey+accey;
	gz=gz+Kp*ez+accez;
	//解四元数微分方程如下
	gx=gx*(0.5f*dt);
	gy=gy*(0.5f*dt);
	gz=gz*(0.5f*dt);
	q0=q0+(-q1*gx-q2*gy-q3*gz);
	q1=q1+(q0*gx+q2*gz-q3*gy);
	q2=q2+(q0*gy-q1*gz+q3*gx);
	q3=q3+(q0*gz+q1*gy-q2*gx);
	//归一化
	recip=invSqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0=q0*recip;
	q1=q1*recip;
	q2=q2*recip;
	q3=q3*recip;
	//计算姿态角
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


// 根据加速度计 X 与 Z 数据计算 pitch 角（单位?）
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
    // 初始化I2C，设置从机地址为0x69，波特率为400kHz
    i2c_init(I2C_ADDR, 0);

    // 配置SDA和SCL引脚
    gpio_config(02, SC_FUN, PULL_NONE); // 配置SDA引脚
    gpio_config(03, SC_FUN, PULL_NONE); // 配置SCL引脚

    // 设置陀螺仪量程为±500 dps
    uint8_t gyro_range = 0x49; // 根据数据表中的GYRO_CONFIG0寄存器设置  //0~3 0101: 100k Hz    5~6    000: ±2000 dps
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
float ax,ay,az,gx,gy,gz;
float krm[6]={0};
void mouse()
{
		imu_final_data_get();

		krm[0]=kalmanFilter(&KFP_accelX,imu_9.f_acc[0]);//卡尔曼滤波
		krm[1]=kalmanFilter(&KFP_accelY,imu_9.f_acc[1]);
		krm[2]=kalmanFilter(&KFP_accelZ,imu_9.f_acc[2]);
		krm[3]=kalmanFilter(&KFP_gyroX,imu_9.f_gyro[0]);
		krm[4]=kalmanFilter(&KFP_gyroY,imu_9.f_gyro[1]);
		krm[5]=kalmanFilter(&KFP_gyroZ,imu_9.f_gyro[2]);
		algorithm(krm[0],krm[1],krm[2],krm[3],krm[4],krm[5]);//采用卡尔曼滤波

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
