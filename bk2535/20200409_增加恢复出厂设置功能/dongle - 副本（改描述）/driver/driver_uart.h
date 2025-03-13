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
/*! \file driver_uart.h
    \brief The driver headfile of uart module.
*/

#ifndef _DRIVER_UART_H_
#define _DRIVER_UART_H_

#if defined(UART_LOOP) || defined(UART_INTERRUPT)

void driver_uart_initial(void);

#ifdef UART_INTERRUPT
void driver_uart_send_start(UINT8 len);
#endif

#endif //defined(UART_LOOP) || defined(UART_INTERRUPT)

#ifdef DEBUG
#define DEG(x) printf x
#else
#define DEG(x)
#endif

#endif

/***********************************************************
						end Uart.h
************************************************************/
