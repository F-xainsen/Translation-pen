#include "acc_gry.h"
//#include "imu_9.h"
#include "app_gyroscope.h"




void write_sensor_data(uint8_t reg_addr, uint8_t data)
{
    i2c_write(I2C_ADDR, reg_addr, &data, 1);
}


/**
 * @brief  6轴采样
 * @author
 * @param  gyro-脱落仪数据 acc-加速度数据
 * @return void
 */
void imu_sample_data(int16_t *gyro_raw, int16_t *acc_raw)
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


void set_acc_gyro_offset(void)
{
    uint8_t i;
    int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
    int32_t ax_offset_sum, ay_offset_sum, az_offset_sum, gx_offset_sum, gy_offset_sum, gz_offset_sum;
    int16_t gyro_off[3],acc_off[3];

    ax_offset_sum = 0;
    ay_offset_sum = 0;
    az_offset_sum = 0;
    gx_offset_sum = 0;
    gy_offset_sum = 0;
    gz_offset_sum = 0;

    for (i = 0; i < 20; i++)
   {
	  imu_sample_data(gyro_off,acc_off);

	   ax_offset = acc_off[0];
	   ay_offset = acc_off[1];
	   az_offset = acc_off[2];
	   ax_offset_sum = ax_offset_sum + ax_offset;
	   ay_offset_sum = ay_offset_sum + ay_offset;
	   az_offset_sum = az_offset_sum + az_offset;

	   gx_offset = gyro_off[0];
	   gy_offset = gyro_off[1];
	   gz_offset = gyro_off[2];
	   gx_offset_sum = gx_offset_sum + gx_offset;
	   gy_offset_sum = gy_offset_sum + gy_offset;
	   gz_offset_sum = gz_offset_sum + gz_offset;
   }
    imu_9.acc_zero[0] = ax_offset_sum/20;
    imu_9.acc_zero[1] = ay_offset_sum/20;
    imu_9.acc_zero[2] = az_offset_sum/20;
    imu_9.gyro_zero[0] = gx_offset_sum/20;
    imu_9.gyro_zero[1] = gy_offset_sum/20;
    imu_9.gyro_zero[2] = gz_offset_sum/20;

}
