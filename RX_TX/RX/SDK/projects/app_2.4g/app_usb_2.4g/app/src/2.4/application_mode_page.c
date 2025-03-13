#include "app.h"
#include "driver_sensor.h"
#include "application_mode.h"
#include "flash.h"
#include "wdt.h"
#include <string.h>

#define UART_PRINTF    uart_printf
int uart_printf(const char *fmt,...);

#define    cFlashProtect1         0x253A
#define    cFlashProtect2         0x55AA
#define    cFlashProtect3         0xAA55

extern uint32_t RF_flag;
extern uint8_t rf_fifo_data[MAX_PACKET_LEN];             //rf fifo
extern SYSTEM_STRUCT_DATA system_data;
extern APP_VAL_T app_val;

uint8_t local_addr[6];

uint32 rf_address_tx_page[RF_ADDRESS_LEN] = {0x20, 0x34, 0x56, 0x42, 0x46};    // {0x20, 0x34, 0x56, 0x00, 0x00};
uint32 rf_address_tx_page1[RF_ADDRESS_LEN] = {0x15, 0x59, 0x23, 0xC6, 0x29};    // {0x20, 0x34, 0x56, 0x00, 0x00};



void application_page_mode(void)
{

    uint16_t page_loop_count = 0;                     //0-749 -10dbm, 750-1499, -5dbm, 1500-2250 0dbm; 750*4ms=3s, 3000*3ms=9s
    uint8_t uBindStep = 0x0;
    uint8_t uReTXCNT;
    uint8_t szRXAddr[8];
    uint32_t u32FlashProtect1,u32FlashProtect2,u32FlashProtect3;

    u32FlashProtect1 = cFlashProtect1;
    u32FlashProtect2 = 0x0;
    u32FlashProtect3 = 0x0;
	
    TRX_SETUP_RETR = 0x08;        // 250us,redo 8times
		FLUSH_RX;
    FLUSH_TX;
    TRX_IRQ_STATUS = 0x70;
	
    driver_rf_ouput_power_value_set(POWER_VALUE_5DBM);    // POWER_VALUE_MINUS_15DBM, POWER_VALUE_MINUS_20DBM
    memcpy_2461(&TRX_RX_ADDR_P0_0, rf_address_tx_page, RF_ADDRESS_LEN);
    memcpy_2461(&TRX_TX_ADDR_0, rf_address_tx_page, RF_ADDRESS_LEN);
    TRX_RF_CH = 0x80+80;    // 100;        // 2450MHZ
	
		SwitchToRxMode();
		Delay_ms(1);
				
    while(1)
    {
//        if(RF_flag & flag_tick_8ms_loop)
//        {
//            RF_flag &= ~flag_tick_8ms_loop;
//            app_val.power_on_cnt++;

            if(uBindStep == 0)
           {
                u32FlashProtect2 = 0x0;
                RF_flag &= ~flag_rf_receive_page;
                driver_rf_receive_package();
                //uart_printf("1111111\r\n");
                if(RF_flag & flag_rf_receive_page)
                {
										uart_printf("123\r\n");
                    if((uRXDataLens == 5) && (rf_fifo_data[0] == DATATYPE_PAGE_START) \
                        && (rf_fifo_data[1] == DATATYPE_MOUSE))
                    {
                        szRXAddr[0] = rf_fifo_data[2];
                        szRXAddr[1] = rf_fifo_data[3];
                        szRXAddr[2] = rf_fifo_data[4];

                        rf_fifo_data[0] = DATATYPE_PAGE_START;
                        rf_fifo_data[1] = DATATYPE_MOUSE;
                        rf_fifo_data[2] = local_addr[0];
                        rf_fifo_data[3] = local_addr[1];
                        rf_fifo_data[4] = local_addr[2];
                        rf_fifo_data[5] = local_addr[3];

                        SwitchToTxMode();
                        if(driver_rf_data_send_ACK(6))
                        {
                            uBindStep = 1;
                            uReTXCNT = 0x0;
                            u32FlashProtect2 = cFlashProtect2;
                            uart_printf("rx s1 ok\r\n");
                        }
                    }
                }
            }
            else
           {
                u32FlashProtect3 = 0x0;
                RF_flag &= ~flag_rf_receive_page;
                driver_rf_receive_package();

                if(RF_flag & flag_rf_receive_page)
                {
                    if((uRXDataLens == 9) && (rf_fifo_data[0] == DATATYPE_PAGE_END_MOUSE) \
                        && (rf_fifo_data[1] == DATATYPE_MOUSE) \
                        && (rf_fifo_data[2] == szRXAddr[0]) \
                        && (rf_fifo_data[3] == szRXAddr[1]) \
                        && (rf_fifo_data[4] == szRXAddr[2]))
                    {
                        rf_fifo_data[0] = DATATYPE_PAGE_END_MOUSE;
                        rf_fifo_data[1] = DATATYPE_MOUSE;
                        rf_fifo_data[2] = szRXAddr[0];
                        rf_fifo_data[3] = szRXAddr[1];
                        rf_fifo_data[4] = szRXAddr[2];
                        rf_fifo_data[5] = local_addr[0];
                        rf_fifo_data[6] = local_addr[1];
                        rf_fifo_data[7] = local_addr[2];
                        rf_fifo_data[8] = local_addr[3];

                        SwitchToTxMode();
                        if(driver_rf_data_send_ACK(9))
                        {
                            RF_flag |= flag_rf_paged;
                            u32FlashProtect3 = cFlashProtect3;
														uart_printf("uBindStep ok\r\n");
                            break;
                        }
                    }
                }

                uReTXCNT++;
                if(uReTXCNT > 2)
                {
                    uBindStep = 0;
                }
            }
        //}

        Delay_us(10);
    }
    LED_Ctrl(cLEDOff);

    if((RF_flag & flag_rf_paged) && (u32FlashProtect1 == cFlashProtect1) && (u32FlashProtect2 == cFlashProtect2) && (u32FlashProtect3 == cFlashProtect3))
    {
        uart_printf("ID=%x,%x,%x,%x\r\n",szRXAddr[0],szRXAddr[1],szRXAddr[2],szRXAddr[3]);
        Delay_ms(100);
        if((u32FlashProtect1 == cFlashProtect1) && (u32FlashProtect2 == cFlashProtect2) && (u32FlashProtect3 == cFlashProtect3))
        {
          /*   flash_wp_256k(); // 有 bug 1、 保护空间不对，2、写完没有恢复保护

            flash_read(0,0x7d210, 14,temp, (void*)0);//the addr is the word,so the addr +20
            flash_erase(0,0x7d210,0x1000,0);*/
            szRXAddr[4] = szRXAddr[0] ^ 0xff;
            szRXAddr[5] = szRXAddr[1] ^ 0xff;
            szRXAddr[6] = szRXAddr[2] ^ 0xff;
            szRXAddr[7] = szRXAddr[3] ^ 0xff;

        /*    memcpy(temp,szRXAddr,8);

            flash_write(0,0x7d210,14,szRXAddr,(void*)0);
            flash_wp_ALL();*/
            flash_write_some_data(szRXAddr,0x7d210,8);

            P0_Address[1] = szRXAddr[0];
            P0_Address[2] = szRXAddr[1];
            P0_Address[3] = szRXAddr[2];
            P0_Address[4] = szRXAddr[3];
            P0_Address[0] = RF_ADDRESS_FIRST_BYTE;

            app_val.powered_bind = 0;
            app_sensor_init();        //清buff数据，免得对码成功瞬间光标会移动
        }
    }
    memcpy_2461(&TRX_RX_ADDR_P0_0, P0_Address, RF_ADDRESS_LEN);
    memcpy_2461(&TRX_TX_ADDR_0, P0_Address, RF_ADDRESS_LEN);


    f24GetFreqConfigValue();
    SwitchToTxMode();
    driver_rf_ouput_power_value_set(cRFPWR);
    system_data.system_mode = SYSTEM_NORMAL;
    RF_CMD_FLUSH_TX;
    RF_CMD_FLUSH_RX;
}
