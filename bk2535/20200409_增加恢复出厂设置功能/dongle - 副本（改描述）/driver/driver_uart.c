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
/*! \file driver_uart.c
    \brief The driver interface of uart module.
*/

#include "headfile\includes.h"

#if defined(UART_LOOP) || defined(UART_INTERRUPT)

#define UART_BAUD_RATE_9600_16MHZ			            0xffcc      //(65536-(SYS_CLOCK/(32*9600)))
#define UART_BAUD_RATE_9600_8MHZ		                0xffe6

/*! \fn void driver_uart_initial(void)
    \brief Initialize uart with the baud rate UART_BAUD_RATE. If the macro UART_INTERRUPT is defined, uart interrupt will be enabled.

    \param void
    \return void
*/
void driver_uart_initial(void)
{
    CLK_EN_CFG |= 0x60;	    //timer & uart clock
    SCON0 = 0x50;                   //mode 1
    {
        RCAP2H = UART_BAUD_RATE_9600_16MHZ>>8;
        RCAP2L = UART_BAUD_RATE_9600_16MHZ & 0xff;
        TH2 = UART_BAUD_RATE_9600_16MHZ>>8;
        TL2 = UART_BAUD_RATE_9600_16MHZ & 0xff;
    }
    T2CON  = 0x30;              // rclk tclk = 1
    TR2 = 1;                        // start
    SM2 = 0;                        // stop bit will NOT matter
    PALT |= 0x80; 

    #ifdef UART_INTERRUPT
    ES0 = 1;                        // enable uart interrupt for read and write register debug
    driver_buffer_set(uart_data_receive, 0, UART_DATA_LEN);
    uart_data_len_receive = 0;
    uart_data_len_send = 0;
    #else
    ES0 = 0;                        // disable uart interrupt for read and write register debug
    #endif

    TI = 1;                          // TI:  set TI to send first char of UART   
}

#ifdef UART_INTERRUPT
/*! \fn void driver_uart_send_start(UINT8 len)
    \brief Send \a len bytes data stored in uart_data_send array by uart interrupt.

    \param len - the length of data
    \return void
*/
void driver_uart_send_start(UINT8 len)
{
    uart_data_len_send = len;
    if(uart_data_len_send--)
    {
        uart_data_count_send = 0;
        SBUF0 = uart_data_send[uart_data_count_send++];
    }
}
#endif

#endif //defined(UART_LOOP) || defined(UART_INTERRUPT)

        
/***********************************************************
						end Uart.c
************************************************************/
