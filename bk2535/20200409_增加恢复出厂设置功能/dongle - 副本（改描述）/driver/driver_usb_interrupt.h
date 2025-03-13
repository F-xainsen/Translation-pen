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
/*! \file driver_usb_interrupt.h
    \brief The driver headfile of usb interrupt module.
*/

#ifndef _USB_INTERRUPT_SERVICE_ROUTINE_H_
#define _USB_INTERRUPT_SERVICE_ROUTINE_H_

void driver_usb_isr_data_control(void);
void driver_usb_isr_data_receive(void);
void driver_usb_isr_data_send(void);
void driver_usb_isr_reset(void);
void driver_usb_isr_suspend(void);
void driver_usb_isr_resume(void);
void driver_usb_isr_sof(void);
void driver_usb_isr_unsupported_token(void);
void driver_usb_isr_crc_error(void);
void driver_usb_isr_overtime(void);
void driver_usb_isr_data_pid_error(void);
void driver_usb_force_stall(void);
void driver_usb_fifo_read(UINT8 index, UINT8 len, UINT8 *pointer);
void driver_usb_fifo_write(UINT8 index, UINT8 len, UINT8 *pointer);
void driver_usb_set_wait_status(void);
void driver_usb_set_wait_address(void);
void driver_usb_set_current_endpoint(UINT8 ep_index) ;
void driver_usb_set_endpoint0_directory(UINT8 flag) ;

#endif

/***********************************************************
						end file
***********************************************************/
