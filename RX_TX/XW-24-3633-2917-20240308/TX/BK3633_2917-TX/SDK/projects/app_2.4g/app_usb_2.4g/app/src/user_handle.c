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
    //ע��IO�ص�
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

  SwitchToTxMode();
	Delay_ms(1);
}

extern uint8_t local_addr[6];
void user_application_normal_mode(void)
{
	static uint8_t cnt = 0;
	
	uart_printf("user_application_normal_mode \r\n");
	
	user_rf_receive_set();
    while(system_data.system_mode == SYSTEM_NORMAL)
    {
    	cnt += 1;

		if(cnt & 0x01)
		{
			
			          rf_fifo_data[0] = DATATYPE_PAGE_START;
                rf_fifo_data[1] = DATATYPE_MOUSE;
                rf_fifo_data[2] = local_addr[0];    // low byte of mac address//rf_address_mouse1;
                rf_fifo_data[3] = local_addr[1];    // high byte of mac address rf_address_mouse2;
                rf_fifo_data[4] = local_addr[2];
//			rf_fifo_data[0] = 0x01;
//			rf_fifo_data[1] = 0x4b;
//			rf_fifo_data[2] = 0x00;
//			rf_fifo_data[3] = 0x00;
//			rf_fifo_data[4] = 0x00;
		}
//		else
//		{
//			rf_fifo_data[0] = 0x01;
//			rf_fifo_data[1] = 0x00;
//			rf_fifo_data[2] = 0x00;
//			rf_fifo_data[3] = 0x00;
//			rf_fifo_data[4] = 0x00;
//		}
		
		if(driver_rf_data_send_ACK(5))				   //receive dongle send end package to mouse
		{
			uart_printf("send_ok = ");
			
			for(uint8_t a = 0;a<5;a++)
			{
				uart_printf("%02x ",rf_fifo_data[a]);
			}
			uart_printf("\r\n");
		}
		Delay_ms(100);
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
	
	SwitchToTxMode();
  //SwitchToRxMode();
	Delay_ms(1);
}

#define    cFlashProtect1         0x253A
#define    cFlashProtect2         0x55AA
#define    cFlashProtect3         0xAA55

uint8_t local_addr[6];

void user_application_page_mode(void)
{
    uint8_t uBindStep = 0x0;
    uint8_t uReTXCNT;
    uint8_t szRXAddr[8];	
	  uint32_t u32FlashProtect1,u32FlashProtect2,u32FlashProtect3;
    u32FlashProtect1 = cFlashProtect1;
    u32FlashProtect2 = 0x0;
    u32FlashProtect3 = 0x0;
	
	user_rf_receive_set();
    while(1)
    {

				if(uBindStep == 0)
				{
						u32FlashProtect2 = 0x0;
					
						rf_fifo_data[0] = DATATYPE_PAGE_START;
						rf_fifo_data[1] = DATATYPE_MOUSE;
						rf_fifo_data[2] = local_addr[0];    // low byte of mac address//rf_address_mouse1;
						rf_fifo_data[3] = local_addr[1];    // high byte of mac address rf_address_mouse2;
						rf_fifo_data[4] = local_addr[2];
						if(driver_rf_data_send_ACK(5))				   //receive dongle send end package to mouse
						{
							uart_printf("send_ok = ");
							
							for(uint8_t a = 0;a<5;a++)
							{
								uart_printf("%02x ",rf_fifo_data[a]);
							}
							uart_printf("\r\n");
							SwitchToRxMode();
							uart_printf("tx s1 ok\r\n");
							//Delay_ms(15);    // 15 = 15ms;(4);                // 4ms
							RF_flag &= ~flag_rf_receive_page ;
							driver_rf_receive_package();
							if(RF_flag & flag_rf_receive_page)
							{
									if((uRXDataLens == 0x6) && (rf_fifo_data[0] == DATATYPE_PAGE_START) \
											&& (rf_fifo_data[1] == DATATYPE_MOUSE))
									{
											szRXAddr[0] = rf_fifo_data[2];
											szRXAddr[1] = rf_fifo_data[3];
											szRXAddr[2] = rf_fifo_data[4];
											szRXAddr[3] = rf_fifo_data[5];

											uBindStep = 1;
											uReTXCNT = 0x0;

											u32FlashProtect2 = cFlashProtect2;
									}
							}							
						}
		
				}
				else
				{
					uart_printf("aaaa\r\n");
						u32FlashProtect3 = 0x0;

						rf_fifo_data[0] = DATATYPE_PAGE_END_MOUSE;
						rf_fifo_data[1] = DATATYPE_MOUSE;
						rf_fifo_data[2] = local_addr[0];
						rf_fifo_data[3] = local_addr[1];
						rf_fifo_data[4] = local_addr[2];
//            rf_fifo_data[5] = local_addr[3];

						rf_fifo_data[5] = szRXAddr[0];
						rf_fifo_data[6] = szRXAddr[1];
						rf_fifo_data[7] = szRXAddr[2];
						rf_fifo_data[8] = szRXAddr[3];

						uReTXCNT++;
						if(driver_rf_data_send_ACK(9))                 //receive dongle send end package to mouse
						{
								SwitchToRxMode();
								Delay_ms(15);    // 15 = 15ms;(4);                // 4ms
								RF_flag &= ~flag_rf_receive_page ;
								driver_rf_receive_package();
								if(RF_flag & flag_rf_receive_page)
								{
										if((uRXDataLens == 9) && (rf_fifo_data[0] == DATATYPE_PAGE_END_MOUSE) && (rf_fifo_data[1] == DATATYPE_MOUSE) \
												&& (rf_fifo_data[2] == local_addr[0]) && (rf_fifo_data[3] == local_addr[1]) && (rf_fifo_data[4] == local_addr[2]) \
												&& (szRXAddr[0] == rf_fifo_data[5]) && (szRXAddr[1] == rf_fifo_data[6]) && (szRXAddr[2] == rf_fifo_data[7]) && (szRXAddr[3] == rf_fifo_data[8]))
										{

												RF_flag |= flag_rf_paged;
												u32FlashProtect3 = cFlashProtect3;
												break;
										}
										else
										{
												uBindStep = 0;
												uart_printf("page confirm resp err\r\n");
										}
								}
								else
								{
										uart_printf("no rx page resp\r\n");
								}
						}
						else
						{
								uart_printf("page confirm tx err\r\n");
						}

						if(uReTXCNT > 2)
						{
								uBindStep = 0;
						}
				}
		

		Delay_ms(100);
    }
}


void user_fn24_main(void)
{
  xvr_reg_initial_24();
	
	user_application_initial();
	
	system_data.system_mode = SYSTEM_PAGE;

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

