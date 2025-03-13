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
/*! \file application_main.c
    \brief The exclusive entry of system, which has main and application initialization.
*/

#include "headfile\includes.h"


//=========================================================
void application_initial(void)
{
    driver_delay_us(10);

    //application_test_check_power_up();

    DISABLE_INTERRUPT;
	RF_CHIP_DISABLE;

    driver_clock_initial();
    driver_timer0_initial();
    driver_buffer_set((UINT8 *)&system_data, 0, sizeof(SYSTEM_STRUCT_DATA));
    driver_buffer_set((UINT8 *)&system_xdata, 0, sizeof(SYSTEM_STRUCT_XDATA));	
	
    driver_usb_initial();
	
	driver_rf_initial();                                           //rf initial with rf address
    
		
#if defined(UART_LOOP) || defined(UART_INTERRUPT)
    driver_uart_initial();
#endif

    ENABLE_INTERRUPT;	                                          //rf need 2ms timer to delay
}


//=========================================================
int main(void)
{
    UINT8 value,i=0;
	
    application_initial();
	

//	   while(1)
//		{
//			rf_fifo_data[0] =55;
//			rf_fifo_data[1] =44;
//	        driver_rf_data_send(2);	            	        
//			driver_delay_us(1);
//	
//	    }

	/*
    P2_IOSEL = P2_IOSEL_CFG_DONGLE;
    P2_OPDR = P2_OPDR_CFG_DONGLE;
    P2_PU = P2_PU_CFG_DONGLE;
    P2_PD = P2_PD_CFG_DONGLE;
    P2_WUEN = P2_WAKEUP_CFG_DONGLE;    
      */
	//if(!key_test)
	if(0)
    {
   //    RF_POWER_UP;
  //  driver_delay_us(50*300);  
        driver_rf_spi_switch_bank(0);
        driver_rf_spi_set_mode_tx();
        for(i=0;i<=3;i++)
          driver_rf_spi_write_register((WRITE_REG | RF_BANK0_TSET_REG[i][0]), RF_BANK0_TSET_REG[i][1]);

        driver_rf_spi_switch_bank(1);
    	driver_rf_spi_write_bank1_reg((WRITE_REG | 4), 0xf9968221);
 //       driver_rf_spi_set_mode_tx();


        
        while(1);
    }
    system_data.system_mode = SYSTEM_NORMAL;
    while(1)
    {

	  //  driver_rf_receive_packet();
	  /*
		if(rf_fifo_data[9]	==	5)
		{
			test_pin0		=	0;
		}else if(rf_fifo_data[9]	==	0)
		{
			test_pin0		=	1;
		}
		*/
#if 1

		
    	switch(system_data.system_mode)
    	{
            case SYSTEM_NORMAL:
            	application_normal_mode();
            	break;
            /*    
        #ifdef MODE_TEST 
            case SYSTEM_TEST:
            	application_test_mode();
            	break;
        #endif         
            case SYSTEM_PAGE:
				application_page_mode();
				break;
				*/
            default:
            	system_data.system_mode = SYSTEM_NORMAL;
            	break;
    	}
#endif
    	
    }

    return 0;
}


/***********************************************************
						end file
***********************************************************/
