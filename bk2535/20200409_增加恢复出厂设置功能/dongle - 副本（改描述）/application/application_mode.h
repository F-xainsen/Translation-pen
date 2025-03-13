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
/*! \file application_mode.h
    \brief The application head file of system mode, including normal mode, page mode, configuration mode and debug mode.
*/

#ifndef _APPLICATION_MODE_H_
#define _APPLICATION_MODE_H_

void application_normal_mode_initial(void);
void application_normal_mode(void);
#ifdef MODE_PAGE
void application_page_mode(void);
#endif
#ifdef MODE_TEST
void application_test_mode(void);
void application_test_check_power_up(void);
#endif

//dongle part
void application_dongle_normal_submode_suspend(void);
void application_dongle_usb_data_send_to_pc(void);
void application_dongle_rf_interference_detect(void);
void application_dongle_rf_data_received_check(void);
void application_dongle_rf_data_received_analysis(void);
void application_dongle_rf_data_received_analysis_mouse(void);
void application_dongle_rf_data_received_analysis_keyboard(void);
void application_dongle_usb_data_received_analysis(void);
//void driver_send_keyboard_led_status(void);//keyboard led status

#ifdef UART_INTERRUPT
void application_uart_data_analysis(void);
#endif

#endif

/***********************************************************
						end file
***********************************************************/
