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
/*! \file driver_initial.h
    \brief The driver headfile of initialization module.
*/

#ifndef _DRIVER_INITIAL_H_
#define _DRIVER_INITIAL_H_

void driver_clock_initial(void);
void driver_gpio_initial(void);
void driver_usb_initial(void);
void driver_buffer_set(UINT8 *ptr, UINT8 value, UINT8 len);
void driver_buffer_copy(UINT8 *dest, UINT8 *src, UINT8 len);

#endif

/***********************************************************
						end file
***********************************************************/
