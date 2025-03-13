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

extern SYSTEM_STRUCT_DATA system_data;
extern void f24Init(void);
extern void fReadRFAddr(void);
extern void  xvr_reg_initial_24(void);


extern uint8_t rf_fifo_data[MAX_PACKET_LEN];

extern uint32_t RF_flag;
uint32 user_rf_address_normal[RF_ADDRESS_LEN] = {0x20, 0x34, 0x56, 0x42, 0x46};    // {0x20, 0x34, 0x56, 0x00, 0x00};

uint32 user_rf_address_page[RF_ADDRESS_LEN] = {0x15, 0x59, 0x23, 0xC6, 0x29};    // {0x20, 0x34, 0x56, 0x00, 0x00};

uint8_t sub_key_buf[8];
uint8_t receive_key_flag = 0;
uint8_t usb_key_send_flag = 0;
uint8_t usb_key_send_zero_flag = 0;
uint8_t usb_flagw_en = 0;
uint8_t use_key_buf[9];


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
    //×¢²áIO»Øµ÷
	timer_cb_register(0,1,app_timer0_1_int_cb);

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

    SwitchToRxMode();
	Delay_ms(1);
}




void user_rf_receive_handle(void)
{
	driver_rf_receive_package();
	if(RF_flag & flag_rf_receive_page)
	{
		RF_flag &= ~flag_rf_receive_page ;
		#if 1
		uart_printf("receive_ok = ");
		for(uint8_t a = 0;a<5;a++)
		{
			uart_printf("%02x ",rf_fifo_data[a]);
		}
		uart_printf("\r\n");
		#endif

		if(0x01 & rf_fifo_data[0])
		{
			receive_key_flag = 1;
		}
	}
}

void user_analysis_handle(void)
{
   #if 0
	if(1 == usb_key_send_zero_flag)
	{
		usb_key_send_zero_flag = 0;
		sub_key_buf[0] = 0;
		sub_key_buf[2] = 0;
		usb_key_send_flag  = 1;
	}
	#endif

	if(1 == receive_key_flag)
	{
		receive_key_flag = 0;
		sub_key_buf[0] = rf_fifo_data[2];
		sub_key_buf[2] = rf_fifo_data[1];
		usb_key_send_flag  = 1;
		usb_key_send_zero_flag = 1;
	}
	
}

void user_send_usb_handle(void)
{
	if(1 == usb_key_send_flag)
	{
		usb_key_send_flag = 0;
		if(usb_flagw_en)
		{
			
			uart_printf("sub_key_buf = ");
			for(uint8_t a = 0;a<8;a++)
			{
				uart_printf("%02x ",sub_key_buf[a]);
			}
			uart_printf("\r\n");
			memset(use_key_buf,0x0,sizeof(use_key_buf));
			
			use_key_buf[0]= 0x02;
			memcpy(&use_key_buf[1],sub_key_buf,sizeof(sub_key_buf));
			AplUsb_StartTx(USB_ENDPID_Hid_MSE, use_key_buf, 9);
		}
			
		memset(sub_key_buf,0,8);
	}
	
#if 0
	if(usb_flagw_en)
	{
	   usb_flagw_en = 0;
	   static int16_t deltaX,deltaY;
				  
	  //  if(ms_buf[1]== 0)
		{
		  {
	
			  static uint8_t dcount = 100;
			  dcount++;
			  if(dcount>=100)
			  {
				  dcount = 0;
	
				  if ((deltaX == 0) && (deltaY == 4))
				  {
					  deltaX=4;
					  deltaY = 0;
				  }
				  else if ((deltaX == 4) && (deltaY == 0))
				  {
					  deltaX = 0;
					  deltaY = -4;
				  }
				  else if ((deltaX == 0) && (deltaY== -4))
				  {
					  deltaX = -4;
					  deltaY = 0;
				  }
				  else if((deltaX == -4) && (deltaY == 0))
				  {
					  deltaX = 0;
					  deltaY = 4;
				  }
				  else
				  {
					  deltaX = 4;
					  deltaY = 0;
				  }
			  }
	
		  }
	
			  ms_buf[1]= 0x00;
			  ms_buf[2] = deltaX;
			  ms_buf[3] = ((deltaY<<4)&0xf0)|((deltaX>>8)&0x0f);
			  ms_buf[4] = deltaY>>4;
			  AplUsb_StartTx(USB_ENDPID_Hid_MSE, ms_buf, 6);
		}
	  #endif
}

void user_application_normal_mode(void)
{
	user_rf_receive_set();
    while(system_data.system_mode == SYSTEM_NORMAL)
    {
		user_rf_receive_handle();
		
		user_analysis_handle();
		
		user_send_usb_handle();
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

