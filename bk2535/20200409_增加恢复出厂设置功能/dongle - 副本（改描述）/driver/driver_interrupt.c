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
/*! \file driver_interrupt.c
    \brief The driver interface of initialization module.
*/

#include "headfile\includes.h"

/*! \fn void driver_isr_timer0(void) interrupt 1
    \brief The interrupt service routine of timer0 module.

    Dongle:     2ms tick in osc16mhz, reload intialization value to T0.\n
    \param void
    \return void
*/
void driver_isr_timer0(void) interrupt 1 
{
    TF0 = 0;
    TR0 = 0; 
  
    TL0 = TIMER0_2MS_CNT_16MHZ & 0x00FF;
    TH0 = TIMER0_2MS_CNT_16MHZ >> 8;	
    TR0 = 1; 

    system_data.time.tick_system++;
    system_data.time.tick_2ms++;
    system_data.time.tick_led_rf_received++;
    system_data.time.tick_led_rf_no_page++;

    system_data.usbp1_data.tick++;                   //used for keyboard pressed event timeout, 200ms, system will notify pc to release the key
    if(system_data.usbp1_data.tick >= 100) 
    {
        system_data.usbp1_data.tick = 0;
        flag_usb_endpoint1_null_data = 1;
//		if(flag_page_flag==0)
//		   test_pin0=~test_pin0;
//		else
//		   test_pin0=1;
    }
	if(flag_page_flag)
	{
		Normal_to_pair_cont++;
		if(Normal_to_pair_cont >= 1000)
		{
			Normal_to_pair_cont = 0;
			flag_page_flag = 0;				//在连接过后，2S未连接，就可以重新对码
			//flag_mouse_ctrl = 0;
		}
	}
	if(flag_rf_link_ok == 0)
	{
		Not_link_cont ++;
		if(Not_link_cont > 2000)
		{
			Not_link_cont = 0;
			flag_rf_link_status = 0;
			flag_mouse_ctrl = 0;
			flag_grose_status = 0;
			//tx_send_cont = 0;
		}	
	}
	else
	{
		flag_rf_link_ok = 0;
		Not_link_cont = 0;
	}
//	if(flag_mouse_ctrl)
//	{
//		if(flag_page_flag == 0)
//		{
//			Not_link_cont++;
//			if(Not_link_cont > 1000)
//			{
//				Not_link_cont = 0;
//				flag_mouse_ctrl = 0;
//			}
//		}
//		else
//			Not_link_cont = 0;	
//	}
	g_FrqUpdataTimer++;
	TimeFlag = 1;
	
#ifdef MODE_TEST
    if(!flag_test_power_up)
    {
        if(P_flag == (USB_PWR_CN & 0x20))
        {
            P_status_count++;     
        }
        else
        {
            P_flag = USB_PWR_CN & 0x20;
            if(P_status_count >= 2)
            {
                P_status_count = 0;
                P_count++;
            }
            else 
                P_status_count = 0;
        }
    
        if(N_flag == (USB_PWR_CN & 0x40))
        {
            N_status_count++;
        }
        else
        {
            N_flag = USB_PWR_CN & 0x40;
            if(N_status_count >= 2)
            {
                N_status_count = 0;
                N_count++;
            }
            else
                N_status_count = 0;
        }
    }
#endif
}

void driver_isr_timer1(void) interrupt 3
{
}

void driver_isr_int0(void) interrupt 0
{
}

void driver_isr_int1(void) interrupt 2
{
}

void driver_isr_int2(void) interrupt 8    
{
    EXIF = 0x00;
}

void driver_isr_int3(void) interrupt 9  
{
    EXIF = 0x00;
}

void driver_isr_int5(void) interrupt 11 
{
    EXIF = 0x00;
}

void driver_isr_int6(void) interrupt 12  
{
    EXIF = 0x00;
}

void driver_isr_int7(void) interrupt 13  
{
    EXIF = 0x00;
}

void driver_isr_int8(void) interrupt 14  
{
    EXIF &= ~0x40;
}

void driver_isr_int9(void) interrupt 15  
{
    EXIF = 0x00;
}

/*! \fn void driver_isr_usb(void) interrupt 10
    \brief The interrupt service routine of usb module.

    Call \a driver_usb_isr_data_control to process the data from endpoint0.\n
    Call \a driver_usb_isr_data_receive to process the data from endpoint1-7.\n
    Call \a driver_usb_isr_data_send to send the data to PC from endpoint1-7.\n
    Call \a driver_usb_isr_reset to reset usb.\n
    Call \a driver_usb_isr_suspend to suspend usb.\n
    Call \a driver_usb_isr_resume to restore usb by \a driver_usb_isr_suspend.\n
    Call \a driver_usb_isr_suspend to suspend usb.\n
    Call \a driver_usb_isr_sof to process sof event.\n
    Call \a driver_usb_isr_unsupported_token to process the unsupported token.\n
    Call \a driver_usb_isr_sof to process sof event.\n
    Call \a driver_usb_isr_crc_error to process crc error event.\n
    Call \a driver_usb_isr_overtime to process overtime event.\n
    Call \a driver_usb_isr_data_pid_error to process pid error event.
    \param void
    \return void
*/
void driver_isr_usb(void) interrupt 10
{ 		 
    if(( USBINT0 & 0x80)||( USBINT0 & 0x40))  // Handle ep0 receive_data and send_data interrupt
        driver_usb_isr_data_control();	 
				
    if( USBINT0 & 0x20)            // Handle ep1-7 receive_data interrupt
        driver_usb_isr_data_receive();             

    if( USBINT0 & 0x10)            // Handle ep1-7 send_data interrupt
        driver_usb_isr_data_send();

    if( USBINT0 & 0x08)            // USB_rst interrupt
        driver_usb_isr_reset();                               

    if( USBINT0 & 0x04)            // Handle Suspend interrupt
        driver_usb_isr_suspend();

    if( USBINT0 & 0x02)            // Handle resume interrupt
        driver_usb_isr_resume();

    if( USBINT0 & 0x01)            // Handle SOF interrupt
        driver_usb_isr_sof();

    if( USBINT1 & 0x80)            // Handle SOF interrupt
        driver_usb_isr_unsupported_token();

    if( USBINT1 & 0x40)            // Handle SOF interrupt
        driver_usb_isr_crc_error();

    if( USBINT1 & 0x20)            // Handle SOF interrupt
        driver_usb_isr_overtime();

    if( USBINT1 & 0x10)            // Handle SOF interrupt
        driver_usb_isr_data_pid_error();
      
    EXIF &= ~ 0x04;    // clr mcu flag 
	  
}

void driver_isr_pfi(void)  interrupt 6
{  
    EICON &= ~0x10;     
}

#ifdef UART_INTERRUPT
/*! \fn void driver_isr_uart(void)  interrupt 4
    \brief The interrupt service routine of uart module.

    Receive the uart data from PC debug software, and store them in uart_data_receive buffer.\n
    Continue to send the left bytes in uart_data_send buffer to PC debug software after \a driver_uart_send_start started the sending.
    \param void
    \return void
*/
void driver_isr_uart(void)  interrupt 4
{  
    if(RI)
    {
        RI = 0;
        if(uart_data_len_receive >= UART_DATA_LEN)
            uart_data_len_receive = 0;
        uart_data_receive[uart_data_len_receive++] = SBUF0;
    }
/*	
    else if(TI)
    {
        TI = 0;
        if(uart_data_len_send--)
            SBUF0 = uart_data_send[uart_data_count_send++];
    }
*/    
}
#endif

/***********************************************************
						end file
***********************************************************/
