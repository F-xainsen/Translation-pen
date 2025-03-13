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

#ifndef _INCLUDES_H_
#define _INCLUDES_H_

//System Macro Definition for Module
#define UART_LOOP
//#define UART_INTERRUPT
#define DEBUG

#define BK5100_FEATURE
//#define WATCHDOG_ENABLE

#define SetAirDataRate_250Kbps
#define MODE_NORMAL
#define MODE_PAGE
//#define MODE_TEST

//#define MOUSE_SWING
//#define KEYBOARD_MCE

#include <stdio.h>
#include <intrins.h>

#include "headfile\define_system.h"              
#include "headfile\define_register.h"            

#include "driver\driver_initial.h"                     
#include "driver\driver_rf.h"                          
#include "driver\driver_uart.h"                        
#include "driver\driver_usb_descriptor.h"                  
#include "driver\driver_usb_interrupt.h"   
#include "driver\driver_usb_standard_requests.h"           
#include "driver\driver_timer.h"                       
#include "driver\driver_memory.h"                      

#include "application\application_mode.h"
#include "application\application_rf.h"

#endif

/***********************************************************
						end file
***********************************************************/
