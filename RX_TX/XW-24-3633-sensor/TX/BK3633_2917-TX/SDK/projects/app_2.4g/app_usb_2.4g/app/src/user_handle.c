#include "user_handle.h"
#include "app.h"
#include "bk3633_reglist.h"
#include "icu.h"
#include "driver_timer.h"
#include "application_mode.h"
#include "application_mode_test.h"
#include "application_mode_PCBA.h"
#include <string.h>
#include "driver_sensor.h"

#include "gpio.h"

#include <string.h>

#include "driver_usb.h"
#include "usb.h"

#include "app_key.h"
#include "app_gyroscope.h"
extern SYSTEM_STRUCT_DATA system_data;
extern void f24Init(void);
extern void fReadRFAddr(void);
extern void  xvr_reg_initial_24(void);
	
extern uint8_t rf_fifo_data[MAX_PACKET_LEN];  

extern uint32_t RF_flag;
uint32 user_rf_address_normal[RF_ADDRESS_LEN] = {0x20, 0x34, 0x56, 0x42, 0x46};    // {0x20, 0x34, 0x56, 0x00, 0x00};

uint32 user_rf_address_page[RF_ADDRESS_LEN] = {0x15, 0x59, 0x23, 0xC6, 0x29};    // {0x20, 0x34, 0x56, 0x00, 0x00};


void user_application_initial(void)
{
	SENSOR_TYPE eType;
	eType = system_data.sensor_val.type;
	memset(&system_data, 0, sizeof(SYSTEM_STRUCT_DATA));
	system_data.sensor_val.type = eType;
    fReadRFAddr();
    f24Init();

    Timer_Initial(0,1,15); // T0,1M
    Timer_Initial(1,0,0); // T1,32k
    Timer0_Start(1,1000000); // 2ms
    //注册IO回调
	timer_cb_register(0,1,app_timer0_1_int_cb);
	
/******************************************/
		//LED
		gpio_config(LED_R, OUTPUT, PULL_NONE);  //0x10
  	gpio_config(LED_W, OUTPUT, PULL_NONE);	//0x11  *
//		gpio_config(MOTOR, OUTPUT, PULL_NONE);  //0x04  *
		gpio_set(LED_R,1);
		gpio_set(LED_W,1);

		//key
		gpio_config(ROW1, OUTPUT, PULL_NONE);
		gpio_config(ROW2, OUTPUT, PULL_NONE);
		gpio_config(ROW3, OUTPUT, PULL_NONE);
		gpio_config(COL1, INPUT, PULL_HIGH);  //PULL_LOW   PULL_HIGH
		gpio_config(COL2, INPUT, PULL_HIGH);  
		gpio_set(ROW1,1);
		gpio_set(ROW2,1);
		gpio_set(ROW3,1);
		
		
/******************************************/

    //  WHEEL_WAKEUP_EX_INTERRUPT_FALLING_EDGE;
    SYS_REG0X10_INT_EN |= POS_SYS_REG0X10_INT_EN_TIMER0 |POS_SYS_REG0X10_INT_EN_GPIO;
}

void user_rf_receive_set(void)
{
	uart_printf("user_rf_receive_set\r\n");
	
	TRX_SETUP_RETR = 0x08;	   // 250us,redo 8times
	FLUSH_RX;
	FLUSH_TX;
	TRX_IRQ_STATUS = 0x70;
	
	driver_rf_ouput_power_value_set(POWER_VALUE_5DBM);    // 
	memcpy_2461(&TRX_RX_ADDR_P0_0, user_rf_address_normal, RF_ADDRESS_LEN);
	memcpy_2461(&TRX_TX_ADDR_0, user_rf_address_normal, RF_ADDRESS_LEN);
	TRX_RF_CH = 0x80+80;   // 100;	 

    SwitchToTxMode();
	Delay_ms(1);
}




void app_data_status_check(){
		if((RF_flag & (flag_key_pressed | flag_key_released)) || \
        (system_data.wheel != 0x0) || (system_data.sensor_val.x != 0x0 ) || (system_data.sensor_val.y != 0x0))
    {
        RF_flag &= ~flag_key_released;
        RF_flag |= flag_rf_send ;
    }
}

void app_rf_send(){

		memset(rf_fifo_data, 0, 10);
	
    RF_CHIP_ENABLE;
    {
        rf_fifo_data[0] = DATATYPE_MOUSE;
        rf_fifo_data[0] |= system_data.rf_connect;
        rf_fifo_data[1] = system_data.key_current;
		}
		
		if(driver_rf_data_send_ACK(2))				   //receive dongle send end package to mouse
		{
			uart_printf("send_ok \r\n");
    }

}
void app_rf_send_check(){
    if(RF_flag & flag_rf_send )
    {
        RF_flag &= ~flag_rf_send;
        RF_flag &= ~flag_rf_hop_16ch;
        system_data.time.tick_rf_send_fail = SYSTEM_SEND_FAIL_TIME_MOUSE;
//        uart_printf("f=%x,sNO=%x\n",(RF_flag & flag_rf_last_state),sync_count);
        app_rf_send();
		}
}


void user_application_normal_mode(void)
{
	static uint8_t cnt = 0;
	
	uart_printf("user_application_normal_mode \r\n");
	
	user_rf_receive_set();
	
			sensor_init();
			Delay_ms(500);
			uart_printf("sensor ok!\r\n");
			short acc_data[3];
			short gyro_data[3];
			PlaneData plane_data;
		
			// 全局变量声明
			char DPI_DATA = 0; // DPI 数据
			int16_t a[2];      // 用于存储处理后的陀螺仪数据
			APP_MOUSE_T system;    // 系统结构体，包含鼠标数据
    while(system_data.system_mode == SYSTEM_NORMAL)
    {
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
            // 将 gyro_data 映射到 0 到 32767 的范围
            system.mouse[1] = map_value(a[1], -32768, 32767, 0, 32767);
            system.mouse[2] = map_value(a[0], -32768, 32767, 0, 32767);
        }
			
			uart_printf("Gyro: X=%d, Y=%d, Z=%d  \r\n", gyro_data[0], gyro_data[1], gyro_data[2]);
			//uart_printf("平面坐标  X: %d, Y: %d\r\n", system.mouse[1], system.mouse[2]);
			Delay_ms(80);  // 每秒读取一次数据
		
    }

}



void user_rf_receive_page_set(void)
{
	TRX_SETUP_RETR = 0x08;	   // 250us,redo 8times
	FLUSH_RX;
	FLUSH_TX;
	TRX_IRQ_STATUS = 0x70;
	
	driver_rf_ouput_power_value_set(POWER_VALUE_5DBM);    // 
	memcpy_2461(&TRX_RX_ADDR_P0_0, user_rf_address_page, RF_ADDRESS_LEN);
	memcpy_2461(&TRX_TX_ADDR_0, user_rf_address_page, RF_ADDRESS_LEN);
	TRX_RF_CH = 0x80+80;   // 100;	 

	SwitchToRxMode();
	Delay_ms(1);
}

void user_application_page_mode(void)
{
	user_rf_receive_page_set();
}


void user_fn24_main(void)
{
    xvr_reg_initial_24();
	
	user_application_initial();
	

	system_data.system_mode = SYSTEM_NORMAL;

	while(1)
	{
        if(system_data.system_mode==SYSTEM_NORMAL)
        {
            user_application_normal_mode();
        }
        else if(system_data.system_mode == SYSTEM_PAGE)
        {
            user_application_page_mode();
        }
	}
	

}

