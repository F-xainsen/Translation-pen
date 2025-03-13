/***********************************************************
FileName		: Uart.h
Date			: 2010-11-03
Description		: 
Version			: 1.0bate
Function List	: 
--
History			: 
<author> <time>	<version> <desc>
************************************************************/

#ifndef _UART_H_
#define _UART_H_

#ifdef DEBUG
	void UartOpen(void);
	#define DEG(x) printf x
#else
    #define DEG(x)
#endif

extern char uart_send(char c);
#define uart_putc(c)  uart_send(c)
extern void uart_puts(char *s);

extern int uart_get(void);
#define uart_getc()  uart_get()




extern char idata uart_rxbuf[16];
extern unsigned char uart_rxbuf_in, uart_rxbuf_out;

extern char idata uart_txbuf[10];
extern unsigned char uart_txbuf_len, uart_txbuf_ptr;

extern void uart_buf_init(void);

#define is_rxbuf_full()  (((uart_rxbuf_in+1)&0xf) == uart_rxbuf_out) 
#define is_rxbuf_empty() (uart_rxbuf_in == uart_rxbuf_out) 



#endif

/***********************************************************
						end Uart.h
************************************************************/