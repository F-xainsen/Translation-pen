/***********************************************************
Copyright(C), 2004-2011, Beken Technology
FileName		: 
Author		: beken  
Date		  	: 2011/10/20
Description	: 
Version		: V1.0
History		: Author      Date                        Ver       Note
                       beken       2011/10/20  1.0       1.0       created
***********************************************************/
/*! \file application_mode_test.c
    \brief The test mode of system used on the product line. The customer can do their own test design according to their requirements.
*/

#include "headfile\includes.h"

#ifdef MODE_TEST

/*! \fn void application_test_mode(void)
    \brief The test mode of system used on the product line.
 
    Dongle: \n
               When received the enter command from PC test software, it will switch to the test mode.\n
               When received the exit command from PC test software, it will switch to the normal mode.\n
               Execute single wave/rf data sending test according to the command from PC test software.\n
    \param void
    \return void
*/
void application_test_mode(void)
{
#if 0
    UINT8 flag_revice=0,status=0;
    driver_rf_spi_write_register((WRITE_REG|RF_SETUP), 0x03);
    driver_rf_spi_set_mode_tx();
    driver_rf_spi_write_register(WRITE_REG|RF_CH, RF_TEST_CHANNEL[1]);
    driver_rf_mode_set_single_wave(RF_MODE_SINGLE);
    //driver_rf_ouput_power_value_set(1);
    RF_CHIP_DISABLE;
    RF_CHIP_ENABLE;

    system_data.time.tick_system = 0;                   //used for 8ms loop
    while(system_data.system_mode == SYSTEM_TEST)
    {
        if(system_data.time.tick_system < 4)
        {
            system_data.time.tick_system = 0;
            if(!flag_test_single_wave)  		
            {                        
                rf_fifo_data[0] = 0;
                rf_fifo_data[1] = 1;	
                rf_fifo_data[2] = 2;
                rf_fifo_data[3] = 3;
                rf_fifo_data[4] = 4;
                packet_count = 5;
                while(packet_count)
                {
                    driver_rf_spi_write_buffer(W_TX_PAYLOAD_NOACK_CMD, rf_fifo_data, 5);
                    driver_delay_us(50);
                    packet_count--;
                }
            }


			if(flag_revice)
			{
			    status=driver_rf_spi_read_register(STATUS);
				if(status & STATUS_RX_DR)
				{
				   driver_rf_spi_write_register(FLUSH_RX,0);
				   //driver_rf_spi_write_register(FLUSH_TX, 0);
				   driver_rf_spi_write_register(WRITE_REG|STATUS,status);
				}
			}
		
        }

        //usb data dispose
        if(flag_usb_endpoint0_accept)
        {
            flag_usb_endpoint0_accept = 0;
            if(COMMAND_TEST == system_xdata.usbp0_data.set_report_data[0])
            {
                flag_usb_endpoint0_ready_report = 1; 
                if(system_xdata.usbp0_data.set_report_data[1] == COM_TEST_SINGLE_WAVE)
                {
                    flag_test_single_wave = 1;
					flag_revice=0;
					driver_rf_spi_set_mode_tx();
                    driver_rf_spi_write_register((WRITE_REG|RF_CH), RF_TEST_CHANNEL[system_xdata.usbp0_data.set_report_data[2]]);
                    driver_rf_mode_set_single_wave(RF_MODE_SINGLE);
                }
                else if(system_xdata.usbp0_data.set_report_data[1] == COM_TEST_DAT)
                {
                    flag_test_single_wave = 0; 
					flag_revice=0;
					driver_rf_spi_set_mode_tx();
                    driver_rf_spi_write_register((WRITE_REG|RF_CH), RF_TEST_CHANNEL[system_xdata.usbp0_data.set_report_data[2]]);
                    driver_rf_mode_set_single_wave(RF_MODE_NORMAL);
                }
                else if(system_xdata.usbp0_data.set_report_data[1] == COM_TEST_EXIT)
                    system_data.system_mode = SYSTEM_NORMAL; 
				else if(system_xdata.usbp0_data.set_report_data[1]==COM_TEST_RX)  //0827
				{
				   flag_test_single_wave = 1;
                   flag_revice=1;
				   driver_rf_spi_set_mode_rx();
				   
				   driver_rf_spi_write_register((WRITE_REG|RF_CH), RF_TEST_CHANNEL[system_xdata.usbp0_data.set_report_data[2]]);
				   driver_rf_mode_set_single_wave(RF_MODE_NORMAL);				   
				   
				}
                else
                    flag_usb_endpoint0_ready_report = 0; 
            }
        }
    }

    driver_rf_initial();
#endif	
}

/*! \fn void application_test_check_power_up(void)
    \brief The test mode when system powers up, used on the product line.
 
    Dongle: \n
               When DP and DN is input a defined wave, system will enter into test mode.\n
               1) Only do for bk2423.\n
               2) Execute rf data sending for 200 times on 2406 channel, if receive ack 199 times we think it as succeed.\n
    \param void
    \return void
*/
void application_test_check_power_up(void)
{
#if 0
    UINT8 packet_succeed = 0,loop = 0,index = 1;
    UINT8 status;

    DISABLE_INTERRUPT;
    driver_clock_initial();
    driver_timer0_initial();
    driver_buffer_set((UINT8 *)&system_data, 0, sizeof(SYSTEM_STRUCT_DATA));
    driver_rf_initial();                                            //rf initial with rf address
    driver_rf_spi_write_register((WRITE_REG|RF_CH), 0x65); 
    driver_rf_spi_write_register((WRITE_REG|RF_SETUP), 0x00);       //-5dB, LNA 20dB down
    driver_rf_spi_write_register((WRITE_REG|SETUP_AW), 0x03);       //5 bytes address
    driver_rf_spi_write_buffer((WRITE_REG | RX_ADDR_P0), RF_TEST_ADDRESS, RF_ADDRESS_LEN);
    driver_rf_spi_write_buffer((WRITE_REG | TX_ADDR), RF_TEST_ADDRESS, RF_ADDRESS_LEN);
    driver_rf_spi_set_mode_tx();
    driver_rf_mode_set_single_wave(RF_MODE_NORMAL);
    driver_usb_initial();
    EX4 = 0;
    ENABLE_INTERRUPT;
    
    system_data.time.tick_system = 0;                               //200ms
    while(1)
    {
        if(system_data.time.tick_system >= 100)
        {
            flag_test_power_up = 1;
            if((P_count >= 23) && (N_count >= 23))
            {
                LED = 1;
                //if(flag_rf_bk2423)
                //{
                    while(1)
                    {
                        packet_count = 200;
                        while(packet_count)
                        {
                            rf_fifo_data[0] = loop;
                            rf_fifo_data[1] = index++;	
                    rf_fifo_data[2] = 2;
                    rf_fifo_data[3] = 3;
                    rf_fifo_data[4] = 4;
                            
                        system_data.time.tick_system = 0;           // 4ms
                        driver_rf_spi_write_buffer(WR_TX_PLOAD,  rf_fifo_data,  5);
                        packet_count--;
                        do
                        {
                            status = driver_rf_spi_read_register(STATUS);
                        }while((status & STATUS_RX_TX_MAX) == 0x00);
                        if(status & STATUS_TX_DS)
                            packet_succeed++;
                            RF_CMD_FLUSH_TX;
                        driver_rf_spi_write_register(WRITE_REG|STATUS,status);
                        while(system_data.time.tick_system < 2);
                    }
                    
                    if(packet_succeed >= 199)
                    {
                            //while(1)
                        {
                               LED = 0;                             //send succeed
                                //system_data.time.tick_system = 0;
                                //while(system_data.time.tick_system < 25);
                        }
                    }
                    else
                    {
                        LED = 1;                                    //send fail
                            //system_data.time.tick_system = 0;
                            //while(system_data.time.tick_system < 250);
                            //LED = 0;
                            //while(1);
                        }
                        packet_succeed = 0;
                        loop++;
                        index = 1;

                    }
               // }
                //else
                //{
                    //driver_delay_us(100*300);
                    //LED = 1;
                    //while(1);
                //}
            }
            else
            {
                break;
            }
        }
    }
#endif	
}

#endif//#ifdef MODE_TEST

/***********************************************************
						end file
***********************************************************/

