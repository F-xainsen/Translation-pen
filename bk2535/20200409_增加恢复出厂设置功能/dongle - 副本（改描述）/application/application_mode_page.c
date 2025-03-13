/****************************************************************************
**
** Copyright (C) 2011 Beken Corporation, All rights reserved.
**
** Contact: Beken Corporation (www.beken.com)
**
** Author:  river
**
** History: 2012/03/07 
**
** Version: 1.0
**
****************************************************************************/
/*! \file application_mode_page.c
    \brief The page mode of system, which executes the pairing process between dongle and mouse.
*/

#include "headfile\includes.h"

#ifdef MODE_PAGE

//=======================================================
void application_page_mode(void)
{

    UINT8 power_value;
    UINT8 page_send_count;
#if 0

    if(rf_fifo_data[0] == DATATYPE_PAGE_START)              //pipe 0 start
    {
        flag_page_need_ack = 0;
        
        power_value = rf_fifo_data[4];
        
        if(rf_fifo_data[1] == DATATYPE_MOUSE)
        {
            if((!flag_rf_paged_mouse)&&(power_value!=POWER_VALUE_5DBM))                        //have NOT paged before
            {
                system_data.rf_address_paged_mouse1 = rf_fifo_data[2];
                system_data.rf_address_paged_mouse2 = rf_fifo_data[3];
            }
            if((system_data.rf_address_paged_mouse1 == rf_fifo_data[2]) && (system_data.rf_address_paged_mouse2 == rf_fifo_data[3]))
                flag_page_need_ack = 1;                     //page again with the same mouse
            rf_fifo_data[0] = DATATYPE_PAGE_END_MOUSE;
            rf_fifo_data[3] = rf_address_rx1[2] | 0x01;     //rf_address_rx1[2] = RF_ADDRESS_FIRST_BYTE, mouse
            rf_fifo_data[4] = flag_rf_old_mouse;
            rf_fifo_data[5] = system_data.rf_address_paged_mouse1;
            rf_fifo_data[6] = system_data.rf_address_paged_mouse2;
        }
        
        if(rf_fifo_data[1] == DATATYPE_KEYBOARD)
        {
            if((!flag_rf_paged_keyboard)&&(power_value!=POWER_VALUE_5DBM))                     //have NOT paged before
            {
                system_data.rf_address_paged_keyboard1 = rf_fifo_data[2];
                system_data.rf_address_paged_keyboard2 = rf_fifo_data[3];
                //driver_rf_page_write_address();
            }
            if((system_data.rf_address_paged_keyboard1 == rf_fifo_data[2]) && (system_data.rf_address_paged_keyboard2 == rf_fifo_data[3]))
                flag_page_need_ack = 1;                     //page again with the same mouse
            rf_fifo_data[0] = DATATYPE_PAGE_END_KEYBOARD;
            rf_fifo_data[3] = rf_address_rx1[2] | 0x02;     //keyboard
            if(flag_rf_old_keyboard)
                rf_fifo_data[4] = system_data.numlock_state | 0x80;
			else
                rf_fifo_data[4] = system_data.numlock_state & 0x7f;
            rf_fifo_data[5] = system_data.rf_address_paged_keyboard1;
            rf_fifo_data[6] = system_data.rf_address_paged_keyboard2;
        }

        if(flag_page_need_ack)
        {
            rf_fifo_data[1] = rf_address_rx1[0];
            rf_fifo_data[2] = rf_address_rx1[1];

            driver_rf_spi_write_buffer((WRITE_REG|TX_ADDR), rf_address_rx0, RF_ADDRESS_LEN);
            driver_rf_ouput_power_value_set(power_value);
            driver_rf_spi_set_mode_tx();
            
            page_send_count = 5;
            while(page_send_count)
            {
//                if(driver_rf_data_send(7))
                    break;
                page_send_count--;
        		if(system_data.rf_page_fail_wait)
        			driver_delay_us(system_data.rf_page_fail_wait*100);
            }
                
            driver_rf_spi_set_mode_rx();
        }

        driver_rf_ouput_power_value_set(POWER_VALUE_5DBM);
    }
#endif

//    DEG(("Page Mode\n"));
   
	rf_fifo_data[0]=rf_address_rx0[0];
	rf_fifo_data[1]=rf_address_rx0[1];
	rf_fifo_data[2]=rf_address_rx0[2];
	rf_fifo_data[3]=rf_address_rx0[3];
	rf_fifo_data[4]=rf_address_rx0[4];
	rf_fifo_data[5]=0xcc;
	//driver_rf_spi_write_register(WRITE_REG|RF_CH,54);
	rf_pipe_set_public();
	driver_rf_spi_write_register(WRITE_REG|RF_CH,65);
	driver_rf_spi_set_mode_tx();
	driver_delay_us(100);
	page_send_count=3;
	while(page_send_count)
    {
	   if(driver_rf_data_send(6))
	   {
	      flag_page_flag=1;
//		  DEG(("Page ok\n"));
          break;
	   }
	   //driver_delay_us(1);
	   page_send_count--;
	}
	driver_rf_spi_write_buffer((WRITE_REG|RX_ADDR_P0), rf_address_rx0, RF_ADDRESS_LEN);
    driver_rf_spi_write_buffer((WRITE_REG|TX_ADDR), rf_address_rx0, RF_ADDRESS_LEN);
    driver_rf_spi_write_buffer((WRITE_REG|RX_ADDR_P1), rf_address_rx0, RF_ADDRESS_LEN);
	driver_rf_spi_set_mode_rx();
	system_data.system_mode = SYSTEM_NORMAL;
}

#endif//MODE_PAGE

/***********************************************************
						end file
***********************************************************/
