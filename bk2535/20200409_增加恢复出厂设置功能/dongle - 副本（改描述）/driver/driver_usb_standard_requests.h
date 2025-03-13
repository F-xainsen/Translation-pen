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
/*! \file driver_usb_standard_requests.h
    \brief The driver headfile of usb standard requests module.
*/

#ifndef _USB_STANDARD_REQUESTS_H_
#define _USB_STANDARD_REQUESTS_H_

void driver_usb_get_status(void);
void driver_usb_clear_feature(void);
void driver_usb_set_feature(void);
void driver_usb_set_address(void);
void driver_usb_get_descriptor(void);
void driver_usb_get_configuration(void);
void driver_usb_set_configuration(void);
void driver_usb_get_interface(void);
void driver_usb_set_interface(void);
void driver_usb_get_idle(void);
void driver_usb_get_protocol(void);
void driver_usb_set_protocol(void);
void driver_usb_set_idle(void);
void driver_usb_get_report(void);
void driver_usb_set_report(void);

#endif

/***********************************************************
						end file
***********************************************************/
