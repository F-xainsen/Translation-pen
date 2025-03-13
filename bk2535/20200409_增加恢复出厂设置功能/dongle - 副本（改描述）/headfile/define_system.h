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

#ifndef _DEFINE_APPLICATION_H_
#define _DEFINE_APPLICATION_H_

#include "headfile\define_register.h"

//#################Type Definition Start#################
//type redefine
typedef void				VOID, *PVOID;
typedef char				BOOL;
typedef char				BOOLEAN;
typedef unsigned char		BYTE, UINT8, UCHAR, *PUCHAR;
typedef char				INT8;
typedef short				INT16;
typedef unsigned short		UINT16;
typedef unsigned long		UINT32;
typedef union _WORD
{
    unsigned int i; 
    unsigned char c[2]; 
} WORD;

#define VOLATILE              volatile
#define XDATA                   xdata
#define PDATA                   pdata
#define IDATA                    idata
#define DATA	                     data
#define CODE                      code

#define wait() _nop_()

//#################Type Definition End##################

//#################Constant Definition Start###############
// constant definition
#define FALSE			0
#define TRUE			       1

#define	B_0000_0000			0x00
#define	B_0000_0001			0x01
#define	B_0000_0010			0x02
#define	B_0000_0011			0x03
#define	B_0000_0100			0x04
#define	B_0000_0101			0x05
#define	B_0000_0110			0x06
#define	B_0000_0111			0x07
#define	B_0000_1000			0x08
#define	B_0000_1001			0x09
#define	B_0000_1010			0x0A
#define	B_0000_1011			0x0B
#define	B_0000_1100			0x0C
#define	B_0000_1101			0x0D
#define	B_0000_1110			0x0E
#define	B_0000_1111			0x0F
#define	B_0001_0000			0x10
#define	B_0001_0001			0x11
#define	B_0001_0010			0x12
#define	B_0001_0011			0x13
#define	B_0001_0100			0x14
#define	B_0001_0101			0x15
#define	B_0001_0110			0x16
#define	B_0001_0111			0x17
#define	B_0001_1000			0x18
#define	B_0001_1001			0x19
#define	B_0001_1010			0x1A
#define	B_0001_1011			0x1B
#define	B_0001_1100			0x1C
#define	B_0001_1101			0x1D
#define	B_0001_1110			0x1E
#define	B_0001_1111			0x1F
#define	B_0010_0000			0x20
#define	B_0010_0001			0x21
#define	B_0010_0010			0x22
#define	B_0010_0011			0x23
#define	B_0010_0100			0x24
#define	B_0010_0101			0x25
#define	B_0010_0110			0x26
#define	B_0010_0111			0x27
#define	B_0010_1000			0x28
#define	B_0010_1001			0x29
#define	B_0010_1010			0x2A
#define	B_0010_1011			0x2B
#define	B_0010_1100			0x2C
#define	B_0010_1101			0x2D
#define	B_0010_1110			0x2E
#define	B_0010_1111			0x2F
#define	B_0011_0000			0x30
#define	B_0011_0001			0x31
#define	B_0011_0010			0x32
#define	B_0011_0011			0x33
#define	B_0011_0100			0x34
#define	B_0011_0101			0x35
#define	B_0011_0110			0x36
#define	B_0011_0111			0x37
#define	B_0011_1000			0x38
#define	B_0011_1001			0x39
#define	B_0011_1010			0x3A
#define	B_0011_1011			0x3B
#define	B_0011_1100			0x3C
#define	B_0011_1101			0x3D
#define	B_0011_1110			0x3E
#define	B_0011_1111			0x3F
#define	B_0100_0000			0x40
#define	B_0100_0001			0x41
#define	B_0100_0010			0x42
#define	B_0100_0011			0x43
#define	B_0100_0100			0x44
#define	B_0100_0101			0x45
#define	B_0100_0110			0x46
#define	B_0100_0111			0x47
#define	B_0100_1000			0x48
#define	B_0100_1001			0x49
#define	B_0100_1010			0x4A
#define	B_0100_1011			0x4B
#define	B_0100_1100			0x4C
#define	B_0100_1101			0x4D
#define	B_0100_1110			0x4E
#define	B_0100_1111			0x4F
#define	B_0101_0000			0x50
#define	B_0101_0001			0x51
#define	B_0101_0010			0x52
#define	B_0101_0011			0x53
#define	B_0101_0100			0x54
#define	B_0101_0101			0x55
#define	B_0101_0110			0x56
#define	B_0101_0111			0x57
#define	B_0101_1000			0x58
#define	B_0101_1001			0x59
#define	B_0101_1010			0x5A
#define	B_0101_1011			0x5B
#define	B_0101_1100			0x5C
#define	B_0101_1101			0x5D
#define	B_0101_1110			0x5E
#define	B_0101_1111			0x5F
#define	B_0110_0000			0x60
#define	B_0110_0001			0x61
#define	B_0110_0010			0x62
#define	B_0110_0011			0x63
#define	B_0110_0100			0x64
#define	B_0110_0101			0x65
#define	B_0110_0110			0x66
#define	B_0110_0111			0x67
#define	B_0110_1000			0x68
#define	B_0110_1001			0x69
#define	B_0110_1010			0x6A
#define	B_0110_1011			0x6B
#define	B_0110_1100			0x6C
#define	B_0110_1101			0x6D
#define	B_0110_1110			0x6E
#define	B_0110_1111			0x6F
#define	B_0111_0000			0x70
#define	B_0111_0001			0x71
#define	B_0111_0010			0x72
#define	B_0111_0011			0x73
#define	B_0111_0100			0x74
#define	B_0111_0101			0x75
#define	B_0111_0110			0x76
#define	B_0111_0111			0x77
#define	B_0111_1000			0x78
#define	B_0111_1001			0x79
#define	B_0111_1010			0x7A
#define	B_0111_1011			0x7B
#define	B_0111_1100			0x7C
#define	B_0111_1101			0x7D
#define	B_0111_1110			0x7E
#define	B_0111_1111			0x7F
#define	B_1000_0000			0x80
#define	B_1000_0001			0x81
#define	B_1000_0010			0x82
#define	B_1000_0011			0x83
#define	B_1000_0100			0x84
#define	B_1000_0101			0x85
#define	B_1000_0110			0x86
#define	B_1000_0111			0x87
#define	B_1000_1000			0x88
#define	B_1000_1001			0x89
#define	B_1000_1010			0x8A
#define	B_1000_1011			0x8B
#define	B_1000_1100			0x8C
#define	B_1000_1101			0x8D
#define	B_1000_1110			0x8E
#define	B_1000_1111			0x8F
#define	B_1001_0000			0x90
#define	B_1001_0001			0x91
#define	B_1001_0010			0x92
#define	B_1001_0011			0x93
#define	B_1001_0100			0x94
#define	B_1001_0101			0x95
#define	B_1001_0110			0x96
#define	B_1001_0111			0x97
#define	B_1001_1000			0x98
#define	B_1001_1001			0x99
#define	B_1001_1010			0x9A
#define	B_1001_1011			0x9B
#define	B_1001_1100			0x9C
#define	B_1001_1101			0x9D
#define	B_1001_1110			0x9E
#define	B_1001_1111			0x9F
#define	B_1010_0000			0xA0
#define	B_1010_0001			0xA1
#define	B_1010_0010			0xA2
#define	B_1010_0011			0xA3
#define	B_1010_0100			0xA4
#define	B_1010_0101			0xA5
#define	B_1010_0110			0xA6
#define	B_1010_0111			0xA7
#define	B_1010_1000			0xA8
#define	B_1010_1001			0xA9
#define	B_1010_1010			0xAA
#define	B_1010_1011			0xAB
#define	B_1010_1100			0xAC
#define	B_1010_1101			0xAD
#define	B_1010_1110			0xAE
#define	B_1010_1111			0xAF
#define	B_1011_0000			0xB0
#define	B_1011_0001			0xB1
#define	B_1011_0010			0xB2
#define	B_1011_0011			0xB3
#define	B_1011_0100			0xB4
#define	B_1011_0101			0xB5
#define	B_1011_0110			0xB6
#define	B_1011_0111			0xB7
#define	B_1011_1000			0xB8
#define	B_1011_1001			0xB9
#define	B_1011_1010			0xBA
#define	B_1011_1011			0xBB
#define	B_1011_1100			0xBC
#define	B_1011_1101			0xBD
#define	B_1011_1110			0xBE
#define	B_1011_1111			0xBF
#define	B_1100_0000			0xC0
#define	B_1100_0001			0xC1
#define	B_1100_0010			0xC2
#define	B_1100_0011			0xC3
#define	B_1100_0100			0xC4
#define	B_1100_0101			0xC5
#define	B_1100_0110			0xC6
#define	B_1100_0111			0xC7
#define	B_1100_1000			0xC8
#define	B_1100_1001			0xC9
#define	B_1100_1010			0xCA
#define	B_1100_1011			0xCB
#define	B_1100_1100			0xCC
#define	B_1100_1101			0xCD
#define	B_1100_1110			0xCE
#define	B_1100_1111			0xCF
#define	B_1101_0000			0xD0
#define	B_1101_0001			0xD1
#define	B_1101_0010			0xD2
#define	B_1101_0011			0xD3
#define	B_1101_0100			0xD4
#define	B_1101_0101			0xD5
#define	B_1101_0110			0xD6
#define	B_1101_0111			0xD7
#define	B_1101_1000			0xD8
#define	B_1101_1001			0xD9
#define	B_1101_1010			0xDA
#define	B_1101_1011			0xDB
#define	B_1101_1100			0xDC
#define	B_1101_1101			0xDD
#define	B_1101_1110			0xDE
#define	B_1101_1111			0xDF
#define	B_1110_0000			0xE0
#define	B_1110_0001			0xE1
#define	B_1110_0010			0xE2
#define	B_1110_0011			0xE3
#define	B_1110_0100			0xE4
#define	B_1110_0101			0xE5
#define	B_1110_0110			0xE6
#define	B_1110_0111			0xE7
#define	B_1110_1000			0xE8
#define	B_1110_1001			0xE9
#define	B_1110_1010			0xEA
#define	B_1110_1011			0xEB
#define	B_1110_1100			0xEC
#define	B_1110_1101			0xED
#define	B_1110_1110			0xEE
#define	B_1110_1111			0xEF
#define	B_1111_0000			0xF0
#define	B_1111_0001			0xF1
#define	B_1111_0010			0xF2
#define	B_1111_0011			0xF3
#define	B_1111_0100			0xF4
#define	B_1111_0101			0xF5
#define	B_1111_0110			0xF6
#define	B_1111_0111			0xF7
#define	B_1111_1000			0xF8
#define	B_1111_1001			0xF9
#define	B_1111_1010			0xFA
#define	B_1111_1011			0xFB
#define	B_1111_1100			0xFC
#define	B_1111_1101			0xFD
#define	B_1111_1110			0xFE
#define	B_1111_1111			0xFF

//#################Constant Definition End###############

//###########System Definition Shared Part Start#############
// usb
#ifndef  MSB
#define  MSB 0
#endif

#ifndef  LSB
#define  LSB 1
#endif

// mcu clk
#define CKCON_CLK_16MHZ  0x00
#define CKCON_CLK_8MHZ  0x40
#define CKCON_CLK_4MHZ  0x80
#define CKCON_CLK_2MHZ  0xc0

#define CKCON_TIMER2CLK_1_12 0x00
#define CKCON_TIMER2CLK_1_4 0x20
#define CKCON_TIMER1CLK_1_12 0x00
#define CKCON_TIMER1CLK_1_4 0x10
#define CKCON_TIMER0CLK_1_12 0x00
#define CKCON_TIMER0CLK_1_4 0x08

//timer const
#define TIMER0_8MS_CNT_16MHZ  33536
#define TIMER0_7MS_CNT_16MHZ  37000
#define TIMER0_6MS_CNT_16MHZ  41536
#define TIMER0_5MS_CNT_16MHZ  45536
#define TIMER0_4MS_CNT_16MHZ  49536
#define TIMER0_3MS_CNT_16MHZ  53536
#define TIMER0_2MS_CNT_16MHZ  57552     
#define TIMER0_1_5MS_CNT_16MHZ  58000
#define TIMER0_1MS_CNT_16MHZ 61536

#define TIMER0_8MS_CNT_8MHZ  49536
#define TIMER0_7MS_CNT_8MHZ  51000
#define TIMER0_6MS_CNT_8MHZ  53536
#define TIMER0_5MS_CNT_8MHZ  55536
#define TIMER0_4MS_CNT_8MHZ  57536
#define TIMER0_3MS_CNT_8MHZ  59536
#define TIMER0_2MS_CNT_8MHZ  61555      
#define TIMER0_1_5MS_CNT_8MHZ  62000
#define TIMER0_1MS_CNT_8MHZ 63536

#define TIMER0_32K_4MS_CNT 65504
#define TIMER0_32K_5MS_CNT 65496
#define TIMER0_32K_6MS_CNT 65488
#define TIMER0_32K_2MS_CNT 65520
#define TIMER0_32K_2MS5_CNT 65516
#define TIMER0_32K_3MS_CNT 65512
#define TIMER0_32K_1MS_CNT 65528
#define TIMER0_32K_1MS5_CNT 65524
#define TIMER0_32K_200MS_CNT 63936
#define TIMER0_32K_100MS_CNT 64736

// mcu idle sleep
#define MCU_IDLE_OSCXMHZ    PCON &= ~0x18; PCON = 0x01
#define MCU_IDLE_OSC32KHZ  PCON &= ~0x18; PCON = 0x05
#define MCU_IDLE_RC32KHZ    PCON &= ~0x18; PCON = 0x03

//interrupt
#define ENABLE_INTERRUPT    EA = 1
#define DISABLE_INTERRUPT   EA = 0

#define ENABLE_RF_INT	EX5 = 1
#define DISABLE_RF_INT  EX5 = 0

#define ADC_ENABLE CLK_EN_CFG |= 0x80
#define ADC_DISABLE CLK_EN_CFG &= ~0x80

#define TX_MODE 0x00
#define RX_MODE 0x01

#ifdef UART_INTERRUPT
#define UART_DATA_LEN 8
#define UART_DATA_START 0x55

#define UART_REGISTER_READ 0xB0
#define UART_REGISTER_WRITE 0xA0
#define UART_REGISTER_WRITE_MORE 0xC0

#define UART_SFR_WRITE 0x80
#define UART_SFR_READ 0x81

#define UART_BANK0_READ 0x00
#define UART_BANK1_READ 0x01
#define UART_BANK0_WRITE 0x20
#define UART_BANK1_WRITE 0x21

#define UART_IDATA_READ  0xD6
#define UART_IDATA_WRITE 0xD7
#endif

//###########System Definition Start Shared Part End#########

//###########System Struct Definition Shared Part Start#######
// System  mode
typedef enum _APPLICATION_MODE
{
    SYSTEM_NORMAL = 0x00,	   		     
    SYSTEM_PAGE,					     
    SYSTEM_CONFIGURE,					            
    SYSTEM_DEBUG,
    SYSTEM_TEST,
    SYSTEM_MODE_MAX
} SYSTEM_MODE;

//device id
typedef enum _DEVICE
{
    DEVICE_NULL=0x00,
    MOUSE,		  		
    KEYBOARD,			
    PRESENTER,			
    REMOTE,				
    DEVICE_MAX
}DEVICES;

// data type
typedef enum _DATATYPE
{
    DATATYPE_NULL = 0x80,
    DATATYPE_MOUSE = 0x00,		
    DATATYPE_KEYBOARD = 0x01,	
    DATATYPE_KB_CELL = 0x03,
    DATATYPE_PAGE_START = 0xA5,
    DATATYPE_PAGE_END_MOUSE = 0x34,
    DATATYPE_PAGE_END_KEYBOARD = 0x36,
    DATATYPE_MOUSE_SEARCH_DONGLE = 0x50,	
    KEYBOARD_LED_STATUS=0x41,//add  keyboard led status
    DATATYPE_MAX
}DATATYPE;

typedef enum _MOUSE_DATA_TYPE
{
    MOUSE_DATA_STANDARD = 0x00,
    MOUSE_DATA_REMOTE,
    MOUSE_DATA_MEDIA,
    MOUSE_DATA_ACTIVE,
    MOUSE_DATA_MAX
}MOUSE_DATA_TYPE;

/*
byte0                    byte1           byte2       byte3       byte4       byte5       byte6

bit 3-1 0
          KEYBOARD=1
          MOUSE=0
EP1/3
standard key = 0
control key = 1

EP3
media key = 2
mce key = 3
power key = 4 

null key
*/    
#define KEYBOARD_DATA_STANDARD      0x02
#define KEYBOARD_DATA_MEDIA         0x04
#define KEYBOARD_DATA_MCE           0x08
#define KEYBOARD_DATA_POWER         0x10

#define DELAY_LED_RF_REVEIVED_COUNT 10
#define DELAY_LED_RF_NO_PAGE_COUNT  50
#define DELAY_PAGE_NEW_WAIT         10
#define RF_PAGE_CREATE_BUDDY    	0xA3

// timer struct definition
typedef struct _TIME_STRUCT
{
//6B
    UINT8   tick_system;		        // system timer tick, 8ms
    UINT8   tick_2ms;                   // 2ms, 2*4=8ms
    UINT8   tick_led_rf_received;       // 2ms, 2*10=20ms
    UINT8   tick_led_rf_no_page;        // 2ms, 2*100=200ms
#ifdef MOUSE_SWING
    UINT8   tick_mouse_key;
    UINT8   tick_mouse_key_max;
#endif
}TIME_STRUCT;

// system timer struct
typedef struct _TIMER
{
// 4B
    UINT32 value_expired; 			//time out value
} TIMER;

//###########System Struct Definition Shared Part End######

//###########System Definition Dongle Part Start##########
//CD status
#define CD_INT 0x01

//IO initial
/*
P0.2	IRQ	
P0.3	MISO	RF- BK2423
P0.4	MOSI	RF- BK2423
P0.5	SCK	RF- BK2423
P0.6	CSN	RF- BK2423
P0.7	CE	RF- BK2423
P2.0	XXX	UART-RXD
P2.1	XXX	UART-TXD
*/
#define	P0_IOSEL_CFG_DONGLE				B_0000_1111 // 0.7 ce, 6 cs, 5 clk, 4 mosi, 3 miso, 2 irq
#define	P0_OPDR_CFG_DONGLE				B_0000_0000
#define	P0_PU_CFG_DONGLE				B_0000_1111
#define	P0_PD_CFG_DONGLE				B_0000_0000
#define	P0_WAKEUP_CFG_DONGLE			B_0000_0000

#define	P1_IOSEL_CFG_DONGLE				B_0000_0000
#define	P1_OPDR_CFG_DONGLE				B_0000_0000
#define	P1_PU_CFG_DONGLE				B_0000_0000
#define	P1_PD_CFG_DONGLE				B_0000_0000
#define	P1_WAKEUP_CFG_DONGLE			B_0000_0000

#define	P2_IOSEL_CFG_DONGLE				B_0000_1110 
#define	P2_OPDR_CFG_DONGLE				B_0000_0000
#define	P2_PU_CFG_DONGLE				B_0000_1110
#define	P2_PD_CFG_DONGLE				B_0000_0000
#define	P2_WAKEUP_CFG_DONGLE			B_0000_0000

#define	P3_IOSEL_CFG_DONGLE				B_1111_1111
#define	P3_OPDR_CFG_DONGLE				B_0000_0000
#define	P3_PU_CFG_DONGLE				B_1111_1111
#define	P3_PD_CFG_DONGLE				B_0000_0000
#define	P3_WAKEUP_CFG_DONGLE			B_0000_0000

#define	P4_IOSEL_CFG_DONGLE				B_1111_1111
#define	P4_OPDR_CFG_DONGLE				B_0000_0000
#define	P4_PU_CFG_DONGLE				B_1111_1111
#define	P4_PD_CFG_DONGLE				B_0000_0000
#define	P4_WAKEUP_CFG_DONGLE			B_0000_0000

sbit LED = P2^0;

sbit test_pin0 = P1^0; 
sbit test_pin1 = P1^1; 
sbit test_pin2 = P1^2; 
sbit test_pin3 = P1^3; 
sbit test_pin4 = P1^4; 
sbit test_pin5 = P1^5; 
sbit test_pin6 = P1^6; 
sbit test_pin7 = P1^7; 

//	mouse stand key flag
//byte0
#define MOUSE_STANDARD_KEY_LEFT                 0x01
#define MOUSE_STANDARD_KEY_RIGHT               0x02
#define MOUSE_STANDARD_KEY_MIDDLE             0x04
#define MOUSE_STANDARD_KEY_BACK                 0x08
#define MOUSE_STANDARD_KEY_FORAWARD       0x10
#define MOUSE_STANDARD_KEY_DPI                   0x20
#define MOUSE_STANDARD_KEY_SWING_LEFT    0x40
#define MOUSE_STANDARD_KEY_SWING_RIGHT  0x80

//byte1
#define MOUSE_STANDARD_KEY_CUS1                 0x40
#define MOUSE_STANDARD_KEY_CUS2                 0x20

//	mouse multi key flag
#define MOUSE_MEDIA_KEY_MIDLLE                     0x40
#define MOUSE_MEDIA_KEY_RIGHT                       0x20
#define MOUSE_MEDIA_KEY_LEFT                         0x10
#define MOUSE_MEDIA_KEY_SWING_LEFT            0x08
#define MOUSE_MEDIA_KEY_FORWARD                 0x04
#define MOUSE_MEDIA_KEY_BACK                         0x02
#define MOUSE_MEDIA_KEY_PLAY                         0x01
#define MOUSE_MEDIA_KEY_SWING_RIGHT          0x80
#define MOUSE_MEDIA_KEY_NT                             0x40
#define MOUSE_MEDIA_KEY_PT                             0x20
#define MOUSE_MEDIA_KEY_STOP                         0x10

//	mouse remote key flag
#define MOUSE_REMOTE_KEY_MUTE                       0x40
#define MOUSE_REMOTE_KEY_RIGHT                      0x20
#define MOUSE_REMOTE_KEY_LEFT                        0x10
#define MOUSE_REMOTE_KEY_SWING_LEFT           0x08
#define MOUSE_REMOTE_KEY_PT                            0x04
#define MOUSE_REMOTE_KEY_NT                            0x02
#define MOUSE_REMOTE_KEY_3D                            0x01
#define MOUSE_REMOTE_KEY_SWING_RIGHT         0x80
#define MOUSE_REMOTE_KEY_MEDI                        0x40
#define MOUSE_REMOTE_KEY_PLAY                        0x20

//   mouse key value
#define MOUSE_KEY_VALUE_FORWARD                   0x10
#define MOUSE_KEY_VALUE_BACK                           0x08
#define MOUSE_KEY_VALUE_MIDDLE                       0x04
#define MOUSE_KEY_VALUE_RIGHT                         0x02
#define MOUSE_KEY_VALUE_LEFT                           0x01
#define MOUSE_KEY_VALUE_SWING_RIGHT            0x01
#define MOUSE_KEY_VALUE_SWING_LEFT              0xff

//	send config
#define MOUSE_DATA_TYPE_BATTERY                     0x20
#define MOUSE_DATA_TYPE_CPI                              0x40
#define MOUSE_KEYBOARD_DATA_TYPE_CONFIG    0x80

#define MOUSE_DATA_TYPE_SYNCHRONIZE	           0x02
#define MOUSE_DATA_TYPE_END	                         0x04
#define MOUSE_DATA_TYPE_HOP                             0x08

// mouse data num
#define MOUSE_INDEX_KEY	0
#define MOUSE_INDEX_X          1
#define MOUSE_INDEX_XH        2
#define MOUSE_INDEX_Y          3
#define MOUSE_INDEX_YH        4
#define MOUSE_INDEX_WHEEL 5
#define MOUSE_INDEX_SWING 6

//	pc connect
#define PC_GET_REPORT_1 0xba
#define PC_GET_REPORT_2 0x01

//#define PC_SET_REPORT_1 0xba
//#define PC_SET_REPORT_2 0x02
#define PC_SET_REPORT_1 0x33
#define PC_SET_REPORT_2 0x03

#define PC_GET_DATA 0x01
#define PC_GET_NONDATA 0x00

#define COMMAND_MODE  0xA0 
#define COM_MODE_TEST 0x82 
#define COM_REG_READ 0x85

//juzi command				巨资命令
#define COMMAND_HEAD 		0x33
#define ZXD_CONTRL_COMMAND  0x31				  //准心灯控制指令
#define LED_CONTRL_COMMAND  0x32				  //指示灯控制指令
#define ADCVOL_CHECK_COMMAND  0x33				  //电量查询指令
#define CHANGE_CHECK_COMMAND  0x34				  //充电状态查询指令
#define MOUSE_SWICTH_COMMAND  0x35				  //空鼠数据发送控制
#define LED_CTRL_COMMAND	0x01					//控制发射端LED显示
#define SLEEPTIME_CTRL_COMMAND	0x37				//控制发射端静止休眠时间
#define CHECK_LINK_COMMAND	0x38				//查询发射端的链接状态
#define WRITE_FLASH_COMMAND	0x39				//让发射端记忆收到的滚码
#define RESET_ALL_COMMAND		0X3A				// 让发射端恢复出厂设置
#define READ_GROSE_COMMAND		0X3B			// 读取陀螺仪的状态

//TEST mode command
#define COMMAND_TEST 0xA2 
#define COM_TEST_SINGLE_WAVE  0x81  
#define COM_TEST_CD  0x82  
#define COM_TEST_DAT 0x83 
#define COM_TEST_LOW_CH 0x84
#define COM_TEST_MID_CH 0x85
#define COM_TEST_HIG_CH 0x86
#define COM_TEST_SW_HIG 0x87
#define COM_TEST_SW_LOW 0x88
#define COM_TEST_SW_STOP 0x89
#define COM_TEST_POW_5  0x8A
#define COM_TEST_POW_0  0x8B
#define COM_TEST_POW_N5  0x8C
#define COM_TEST_POW_N10  0x8D
#define COM_TEST_EXIT   0x8E

#define COM_TEST_RX     0x8F

// key code table
//#define NULL	0x00

//character
#define KEY_A 0x04		//A
#define KEY_B 0x05		//B
#define KEY_C 0x06		//C
#define KEY_D 0x07		//D
#define KEY_E 0x08 		//E
#define KEY_F 0x09		//F
#define KEY_G 0x0A		//G
#define KEY_H 0x0B		//H
#define KEY_I 0x0C		//I
#define KEY_J 0x0D		//J
#define KEY_K 0x0E	       //K
#define KEY_L 0x0F		//L
#define KEY_M 0x10		//M
#define KEY_N 0x11		//N
#define KEY_O 0x12  	       //O
#define KEY_P 0x13		//P
#define KEY_Q 0x14 		//Q
#define KEY_R 0x15		//R
#define KEY_S 0x16	       //S
#define KEY_T 0x17		//T
#define KEY_U 0x18		//U
#define KEY_V 0x19		//V
#define KEY_W 0x1A 		//W
#define KEY_X 0x1B		//X
#define KEY_Y 0x1C		//Y
#define KEY_Z 0x1D		//Z

//F1 -- F12
#define KEY_F1  0x3A	       //F1
#define KEY_F2  0x3B	      //F2
#define KEY_F3  0x3C	      //F3
#define KEY_F4  0x3D	      //F4
#define KEY_F5  0x3E	      //F5
#define KEY_F6  0x3F	      //F6
#define KEY_F7  0x40	      //F7
#define KEY_F8  0x41	      //F8
#define KEY_F9  0x42	      //F9
#define KEY_F10 0x43	      //F10
#define KEY_F11 0x44	      //F11
#define KEY_F12 0x45	      //F12

//shift + number
#define KEY_SHIFT_0 0x27		//) 0
#define KEY_SHIFT_1 0x1E		//! 1
#define KEY_SHIFT_2 0x1F		//@ 2
#define KEY_SHIFT_3 0x20		//# 3
#define KEY_SHIFT_4 0x21		//$ 4
#define KEY_SHIFT_5 0x22		//%5
#define KEY_SHIFT_6 0x23		//^6
#define KEY_SHIFT_7 0x24		//& 7
#define KEY_SHIFT_8 0x25		//* 8
#define KEY_SHIFT_9 0x26		//( 9
#define KEY_SHIFT_SUBTRACT 0x2D	//_-
#define KEY_SHIFT_PLUS    0x2E	       //+=

// num pad
#define KEY_NUM_LOCK 	      0x53		//Num lock
#define KEY_NUM_MULTIPLY   0x55		//*
#define KEY_NUM_DIVIDE       0x54		// /
#define KEY_NUM_PLUS           0x57	       //+
#define KEY_NUM_SUBTRACT  0x56		//-
#define KEY_NUM_9 	 0x61			// 9
#define KEY_NUM_8 	 0x60			// 8
#define KEY_NUM_7 	 0x5F   		       // 7
#define KEY_NUM_6 	 0x5E			// 6
#define KEY_NUM_5 	 0x5D			// 5
#define KEY_NUM_4 	 0x5C			// 4
#define KEY_NUM_3 	 0x5B			// 3
#define KEY_NUM_2 	 0x5A			// 2
#define KEY_NUM_1 	 0x59			// 1
#define KEY_NUM_0 	 0x62			// 0
#define KEY_NUM_DOT      0x63		       // .
#define KEY_NUM_ENTER  0x58			// small Enter

//control key
#define KEY_HOME 	 0x4A	//Home
#define KEY_INSERT 	 0x49	//insert
#define KEY_DELETE	 0x4C	//delete
#define KEY_END 	        0x4D	//end
#define KEY_PAGEUP   0x4B	//page up
#define KEY_PAGEDOWN 0x4E	//page down

#define KEY_ESC 	 	       0x29	//ESC
#define KEY_CAPS 	       0x39	//Caps lock
#define KEY_TAB                0x2B	//tab
#define KEY_SCROLL 		0x47	//Scroll lock
#define KEY_PRINT_SCREEN 0x46	//prcsc sysrq
#define KEY_PAUSE 		0x48	//Pause break
#define KEY_BACK_SPACE 	0x2A	//Back  space
#define KEY_ENTER 		0x28	//Enter
#define KEY_SPACE 		0x2C	//space

#define KEY_LEFT_CONTROL 	 0x01	//left ctrl control
#define KEY_LEFT_ALT     	        0x04      //left Alt
#define KEY_LEFT_SHIFT  	        0x02	//left Shift
#define KEY_LEFT_WINDOWS 	 0x08	//left windows
#define KEY_RIGHT_WINDOWS  0x80	//right windows
#define KEY_RIGHT_SHIFT 	 0x20	//right Shift
#define KEY_RIGHT_ALT 		 0x40	//right Alt
#define KEY_RIGHT_CONTROL 	 0x10	//right Ctrl
#define KEY_APPLICATION 	 0x65	//Application

#define KEY_VERTICAL_LINE	       0x31	//|"\"
#define KEY_DOUBLE_QUOTATION 	0x34    //" '
#define KEY_COLON 		              0x33	//: ;
#define KEY_WAVE 		              0x35	//~ `
#define KEY_SHIFT_COMMA 		0x36	//<,
#define KEY_SHIFT_DOT 		       0x37	//>.
#define KEY_QUESTION 		       0x38	//?/
#define KEY_RIGHT_BRACKET 		0x30	//}]
#define KEY_LEFT_BRACKET 		0x2F	//{[

#define KEY_UP		0x52		//Up arrow
#define KEY_LEFT 	0x50		//Left arrow
#define KEY_RIGHT 	0x4F		//Right arrow
#define KEY_DOWN 	0x51		//Down arrow

#define KEY_0x32	0x32		//
#define KEY_0x64	0x64		//
#define KEY_0x85	0x85		//
#define KEY_0x87	0x87		//
#define KEY_0x88 	0x88		//
#define KEY_0x89	0x89		//international key
#define KEY_0x8a	0x8a		//
#define KEY_0x8b	0x8b 		//
#define KEY_0x90	0x90		//
#define KEY_0x91	0x91		//

//keyboard standard layout
//R0
#define R0_C0 0x00
#define R0_C1 0x01
#define R0_C2 0x02
#define R0_C3 0x03
#define R0_C4 0x04
#define R0_C5 0x05
#define R0_C6 0x06
#define R0_C7 0x07
#define R0_C8 0x08
#define R0_C9 0x09
#define R0_C10 0x0A
#define R0_C11 0x0B
#define R0_C12 0x0C
#define R0_C13 0x0D
#define R0_C14 0x0E
#define R0_C15 0x0F
#define R0_C16 0x10
#define R0_C17 0x11
#define R0_C18 0x12
//R1
#define R1_C0 0x20
#define R1_C1 0x21
#define R1_C2 0x22
#define R1_C3 0x23
#define R1_C4 0x24
#define R1_C5 0x25
#define R1_C6 0x26
#define R1_C7 0x27
#define R1_C8 0x28
#define R1_C9 0x29
#define R1_C10 0x2A
#define R1_C11 0x2B
#define R1_C12 0x2C
#define R1_C13 0x2D
#define R1_C14 0x2E
#define R1_C15 0x2F
#define R1_C16 0x30
#define R1_C17 0x31
#define R1_C18 0x32
//R2
#define R2_C0 0x40
#define R2_C1 0x41
#define R2_C2 0x42
#define R2_C3 0x43
#define R2_C4 0x44
#define R2_C5 0x45
#define R2_C6 0x46
#define R2_C7 0x47
#define R2_C8 0x48
#define R2_C9 0x49
#define R2_C10 0x4A
#define R2_C11 0x4B
#define R2_C12 0x4C
#define R2_C13 0x4D
#define R2_C14 0x4E
#define R2_C15 0x4F
#define R2_C16 0x50
#define R2_C17 0x51
#define R2_C18 0x52
//R3
#define R3_C0 0x60
#define R3_C1 0x61
#define R3_C2 0x62
#define R3_C3 0x63
#define R3_C4 0x64
#define R3_C5 0x65
#define R3_C6 0x66
#define R3_C7 0x67
#define R3_C8 0x68
#define R3_C9 0x69
#define R3_C10 0x6A
#define R3_C11 0x6B
#define R3_C12 0x6C
#define R3_C13 0x6D
#define R3_C14 0x6E
#define R3_C15 0x6F
#define R3_C16 0x70
#define R3_C17 0x71
#define R3_C18 0x72
//R4
#define R4_C0 0x80
#define R4_C1 0x81
#define R4_C2 0x82
#define R4_C3 0x83
#define R4_C4 0x84
#define R4_C5 0x85
#define R4_C6 0x86
#define R4_C7 0x87
#define R4_C8 0x88
#define R4_C9 0x89
#define R4_C10 0x8A
#define R4_C11 0x8B
#define R4_C12 0x8C
#define R4_C13 0x8D
#define R4_C14 0x8E
#define R4_C15 0x8F
#define R4_C16 0x90
#define R4_C17 0x91
#define R4_C18 0x92
//R5
#define R5_C0 0xA0
#define R5_C1 0xA1
#define R5_C2 0xA2
#define R5_C3 0xA3
#define R5_C4 0xA4
#define R5_C5 0xA5
#define R5_C6 0xA6
#define R5_C7 0xA7
#define R5_C8 0xA8
#define R5_C9 0xA9
#define R5_C10 0xAA
#define R5_C11 0xAB
#define R5_C12 0xAC
#define R5_C13 0xAD
#define R5_C14 0xAE
#define R5_C15 0xAF
#define R5_C16 0xB0
#define R5_C17 0xB1
#define R5_C18 0xB2
//R6
#define R6_C0 0xC0
#define R6_C1 0xC1
#define R6_C2 0xC2
#define R6_C3 0xC3
#define R6_C4 0xC4
#define R6_C5 0xC5
#define R6_C6 0xC6
#define R6_C7 0xC7
#define R6_C8 0xC8
#define R6_C9 0xC9
#define R6_C10 0xCA
#define R6_C11 0xCB
#define R6_C12 0xCC
#define R6_C13 0xCD
#define R6_C14 0xCE
#define R6_C15 0xCF
#define R6_C16 0xD0
#define R6_C17 0xD1
#define R6_C18 0xD2
//R7
#define R7_C0 0xE0
#define R7_C1 0xE1
#define R7_C2 0xE2
#define R7_C3 0xE3
#define R7_C4 0xE4
#define R7_C5 0xE5
#define R7_C6 0xE6
#define R7_C7 0xE7
#define R7_C8 0xE8
#define R7_C9 0xE9
#define R7_C10 0xEA
#define R7_C11 0xEB
#define R7_C12 0xEC
#define R7_C13 0xED
#define R7_C14 0xEE
#define R7_C15 0xEF
#define R7_C16 0xF0
#define R7_C17 0xF1
#define R7_C18 0xF2

// ct key
#define LEFT_CONTROL       0xC0
#define LEFT_ALT               0x69
#define LEFT_SHIFT           0x2F 
#define LEFT_WINDOWS     0x30 
#define RIGHT_CONTROL    0x80
#define RIGHT_ALT             0xA9
#define RIGHT_SHIFT         0x4F
#define RIGHT_WINDOWS  0x51
 
//fn key
#define FN_F1 0xC2
#define FN_F2 0xC3
#define FN_F3 0x23
#define FN_F4 0x63
#define FN_F5 0xE0
#define FN_F6 0x66
#define FN_F7 0x27
#define FN_F8 0xC7
#define FN_F9 0xCA
#define FN_F10 0xEA  
#define FN_F11 0x6A
#define FN_F12 0xAA

#define FN_INS 0xCC

#define FN_UP 0x6E
#define FN_DOWN 0xAB
#define FN_LEFT 0xAE
#define FN_RIGHT 0xAC

#define FN_7 0xE5
#define FN_8 0xE6
#define FN_9 0xE7
#define FN_ZERO 0xE8    //*
#define FN_U 0x05         //4
#define FN_I 0x06           // 5
#define FN_O 0x07          // 6
#define FN_P 0x08          // -
#define FN_J 0x45          // 1
#define FN_K 0x46         // 2
#define FN_L 0x47        //3
#define FN_COLON 0x48   // +
#define FN_M 0x85         // 0
#define FN_GTS 0x87     // .
#define FN_QM  0xA8     // /

// end package
#define END_NULL 0xFF

// endpoint number
#define  EP_0          0x0
#define  EP_1          0x1
#define  EP_2          0x2
#define  EP_3          0x3
#define  EP_4          0x4
#define  EP_5          0x5
#define  EP_6          0x6
#define  EP_7          0x7

// Standard Descriptor Types
#define  DSC_DEVICE              0x01      // Device Descriptor
#define  DSC_CONFIG             0x02      // Configuration Descriptor
#define  DSC_STRING             0x03      // String Descriptor
#define  DSC_INTERFACE       0x04      // Interface Descriptor
#define  DSC_ENDPOINT         0x05      // Endpoint Descriptor

// Standard Request Codes
#define  GET_STATUS                 0x00  // Code for Get Status
#define  CLEAR_FEATURE           0x01  // Code for Clear Feature
#define  SET_FEATURE               0x03  // Code for Set Feature
#define  SET_ADDRESS               0x05  // Code for Set Address
#define  GET_DESCRIPTOR          0x06  // Code for Get Descriptor
#define  SET_DESCRIPTOR          0x07  // Code for Set Descriptor(not used)
#define  GET_CONFIGURATION   0x08  // Code for Get Configuration
#define  SET_CONFIGURATION   0x09  // Code for Set Configuration
#define  GET_INTERFACE           0x0A  // Code for Get Interface
#define  SET_INTERFACE           0x0B  // Code for Set Interface
#define  SYNCH_FRAME              0x0C  // Code for Synch Frame(not used)

// Define wIndex bitmaps
#define  IN_EP1                0x81      // Index values used by Set and Clear
#define  OUT_EP1             0x01      // commands for Endpoint_Halt
#define  IN_EP2                0x82
#define  OUT_EP2             0x02
#define  IN_EP3                0x83
#define  OUT_EP3             0x03
#define  IN_EP4                0x84
#define  OUT_EP4             0x04
#define  IN_EP5                0x85
#define  OUT_EP5             0x05
#define  IN_EP6                0x86
#define  OUT_EP6             0x06
#define  IN_EP7                0x87
#define  OUT_EP7             0x07

#define EP_IDLE                 0x00
#define EP_TX                     0x11
#define EP_RX                     0x10
#define EP_SET_ADDRESS  0x21
#define EP_WAIT_STATUS  0x20
#define EP_STALL               0xaa

#define  EP0_PACKET_SIZE         0x40       // this value can be 8/16/32/64, depending on device speed, see USB spec
#define  EP1_PACKET_SIZE         0x0040   // can range 0 - 1024 depending on data and transfer type
#define  EP1_PACKET_SIZE_LE   0x4000   // this should be Little-Endian version of EP1_PACKET_SIZE
#define  EP2_PACKET_SIZE         0x0040   // can range 0 - 1024 depending on data and transfer type
#define  EP2_PACKET_SIZE_LE   0x4000   // this should be Little-Endian version of EP2_PACKET_SIZE
#define  EP3_PACKET_SIZE         0x0040   // can range 0 - 1024 depending on data and transfer type
#define  EP3_PACKET_SIZE_LE   0x4000   // this should be Little-Endian version of EP2_PACKET_SIZE
#define  EP4_PACKET_SIZE_LE   0x4000

// HID Descriptor Types
#define DSC_HID                         0x21       // HID Class Descriptor
#define DSC_HID_REPORT           0x22      // HID Report Descriptor

// HID Request Codes
#define GET_REPORT                   0x01   // Code for Get Report
#define GET_IDLE                        0x02   // Code for Get Idle
#define GET_PROTOCOL               0x03   // Code for Get Protocol
#define SET_REPORT                   0x09   // Code for Set Report
#define SET_IDLE                        0x0A   // Code for Set Idle
#define SET_PROTOCOL               0x0B   // Code for Set Protocol

// Define device states
#define  DEV_ATTACHED               0x00  // Device is in Attached State
#define  DEV_POWERED                0x01  // Device is in Powered State
#define  DEV_DEFAULT                 0x02  // Device is in Default State
#define  DEV_ADDRESS                0x03  // Device is in Addressed State
#define  DEV_CONFIGURED          0x04  // Device is in Configured State
#define  DEV_SUSPENDED            0x05  // Device is in Suspended State

// Define bmRequestType bitmaps
#define  IN_DEVICE                      0x00  // Request made to device, direction is IN
#define  OUT_DEVICE                   0x80  // Request made to device, direction is OUT
#define  IN_INTERFACE                0x01  // Request made to interface, direction is IN
#define  OUT_INTERFACE             0x81  // Request made to interface, direction is OUT
#define  IN_ENDPOINT                 0x02  // Request made to endpoint, direction is IN
#define  OUT_ENDPOINT              0x82  // Request made to endpoint, direction is OUT

// Define wValue bitmaps for Standard Feature Selectors
#define  DEVICE_REMOTE_WAKEUP    0x01  // Remote wakeup feature(not used)
#define  ENDPOINT_HALT            0x00  // Endpoint_Halt feature selector

//###########System Definition Dongle Part End##########

//#########System Struct Definiton Dongle Part Start#######
// system usb point 0 data send  --> keyboard data return;
typedef struct _USBP0_DATA 
{
//140B
//9B
    UINT8 out_report[8];
    UINT8 wait_out_report;
//131B
    UINT8 out_report_index;
    UINT8 report_id;
    UINT8 report_type;
    UINT8 get_report_data[64];
    UINT8 set_report_data[64];
}USBP0_DATA;

// system usb point 1 data send --> keyboard
typedef struct _USBP1_DATA
{
//18B
    UINT8 tick;
    UINT8 rf_data_received_len;
    //add
    UINT8 rf_data_media[3];
    //
    UINT8 rf_data_received[8];
    UINT8 in_report[8] ;     
}USBP1_DATA;

//system usb point 2 data send --> mouse
typedef struct _USBP2_DATA
{
    UINT8 in_report[7];
}USBP2_DATA;

//system usb point 2 data send --> mouse, media key
typedef struct _USBP3_DATA
{
// 4B
    UINT8 in_report[3];
    UINT8 in_report_len;
}USBP3_DATA;

// system normal submode	
typedef enum _APPLICATION_NORMAL_SUBMODE
{
    SYSTEM_NORMAL_WORK = 0x00,	   
    SYSTEM_NORMAL_SUSPEND,		
    SYSTEM_WORK_MAX
} SYSTEM_NORMAL_SUBMODE;

//	system test mode
typedef enum _SYTEM_TEST_MODE
{
    TEST_SINGLE_WAVE = 0,						
    TEST_CD,						
    TEST_RF_DATA,					
    TEST_MAX
}SYTEM_TEST_MODE;

//#########System Struct Definiton Dongle Part End##############

//#########System Struct Definiton Shard Part Start####################
// system state defined in data region
typedef struct _APPLICATION_STRUCT_DATA
{
//65B
//8B
    UINT8 system_mode;
    UINT8 normal_sub_mode;
    UINT8 numlock_state;

    UINT8 rf_channel;
    UINT8 rf_pipe_num;
    UINT8 rf_received_len;
    UINT8 rf_address_paged_mouse1;
    UINT8 rf_address_paged_mouse2;
    UINT8 rf_address_paged_keyboard1;
    UINT8 rf_address_paged_keyboard2;
    UINT8 rf_page_fail_wait;

//16B
    UINT8 mouse_data_buffer_received[8];                //data received from mouse
    UINT8 mouse_data_received_key_copy;         //data copy of mouse_data, used for analysis
    UINT8 mouse_data_buffer_uploading[7];              //data will be up to pc for standard mouse
//7B
    UINT8 usb_state;              
    UINT8 usb_protocol_status;
    UINT8* usb_data_pointer;                
    UINT8 usb_data_size;                 
    UINT8 usb_data_send_count;                 
    UINT8 usb_idle_rate;

	UINT8 keycode;
//35B
    TIME_STRUCT time;                     // 6B
    USBP1_DATA usbp1_data;	   	          //used for keyboard upload, 18B
    USBP2_DATA usbp2_data;		          //used for mouse, 7B
    USBP3_DATA usbp3_data;		          //used for mouse & media & &mce &power key upload, 4B
}SYSTEM_STRUCT_DATA;

// system state defined in idata region
/*
typedef struct _APPLICATION_STRUCT_INTERNAL_IDATA
{
    USBP1_DATA usbp1_data;	   	          //used for keyboard upload, 24B
} SYSTEM_STRUCT_IDATA;
*/
// system state defined in xdata region
typedef struct _APPLICATION_STRUCT_XDATA
{
//dongle part, 206B
    USBP0_DATA usbp0_data;		          //used for ep0, 140B
} SYSTEM_STRUCT_XDATA;

//#########System Struct Definiton Shard Part End####################

#define PAGEUP_KEY		   1
#define PAGEDOWN_KEY 	   2
#define B_KEY			   3
#define SHIFT_F5_ESC_KEY   4


//---------------------------------------------------------
//					key code
//---------------------------------------------------------

#define null	0x00

//字母
#define kA 0x04		//A
#define kB 0x05		//B
#define kC 0x06		//C
#define kD 0x07		//D
#define kE 0x08 		//E
#define kF 0x09		//F
#define kG 0x0A		//G
#define kH 0x0B		//H
#define kI 0x0C		//I
#define kJ 0x0D		//J
#define kK 0x0E	    //K
#define kL 0x0F		//L
#define kM 0x10		//M
#define kN 0x11		//N
#define kO 0x12  	//O
#define kP 0x13		//P
#define kQ 0x14 		//Q
#define kR 0x15		//R
#define kS 0x16	    //S
#define kT 0x17		//T
#define kU 0x18		//U
#define kV 0x19		//V
#define kW 0x1A 		//W
#define kX 0x1B		//X
#define kY 0x1C		//Y
#define kZ 0x1D		//Z

//F1 -- F12
#define kF1  0x3A	//F1
#define kF2  0x3B	//F2
#define kF3  0x3C	//F3
#define kF4  0x3D	//F4
#define kF5  0x3E	//F5
#define kF6  0x3F	//F6
#define kF7  0x40	//F7
#define kF8  0x41	//F8
#define kF9  0x42	//F9
#define kF10 0x43	//F10
#define kF11 0x44	//F11
#define kF12 0x45	//F12

//上数字
#define k0 0x27		//) 0
#define k1 0x1E		//! 1
#define k2 0x1F		//@ 2
#define k3 0x20		//# 3
#define k4 0x21		//$ 4
#define k5 0x22		//%5
#define k6 0x23		//^6
#define k7 0x24		//& 7
#define k8 0x25		//* 8
#define k9 0x26		//( 9
#define kMi    0x2D	//—— _
#define kEq    0x2E	//+=

// num pad
#define kNLock 	 0x53			//Num lock
#define kNMulti  0x55			//*
#define kNDivi   0x54			// /
#define kNPlus 	 0x57			//+
#define kNMinus  0x56			//-
#define kN9 	 0x61			//9
#define kN8 	 0x60			//8
#define kN7 	 0x5F   		//7
#define kN6 	 0x5E			//6
#define kN5 	 0x5D			//5
#define kN4 	 0x5C			//4
#define kN3 	 0x5B			//3
#define kN2 	 0x5A			//2
#define kN1 	 0x59			//1
#define kN0 	 0x62			//0
#define kNP  	 0x63			//.
#define kNEnter  0x58			//（小Enter）

//控制
#define kHome 	 0x4A	//Home
#define kIns 	 0x49	//insert
#define kDel	 0x4C	//delete
#define kEnd 	 0x4D	//end
#define kPaUp    0x4B	//page up
#define kPaDo    0x4E	//page down

#define kEsc 	 	0x29	//ESC
#define kCaps 		0x39	//Caps lock
#define kTab        0x2B	//tab
#define kScrol 		0x47	//Scroll lock
#define kPrcsc		0x46	//prcsc sysrq
#define kPause 		0x48	//Pause break
#define kBack 		0x2A	//Back space
#define kEnter 		0x28	//Enter
#define kSpace 		0x2C	//space

#define kLCtrl 		0x01	//左ctrl
#define kLAlt     	0x04    //左Alt
#define kLShift 	0x02	//左Shift
#define kLWin 		0x08	//左windows
#define kRWin 		0x80	//右windows
#define kRShift 	0x20	//右Shift
#define kRAlt 		0x40	//右Alt
#define kRCtrl 		0x10	//右Ctrl
#define kApp 		0x65	//Application

#define kVBar 		0x31	//|"\"
#define kDQuote 	0x34    //" '
#define kColon 		0x33	//: ;
#define kThrow 		0x35	//~ `
#define kLTS 		0x36	//<,
#define kGTS 		0x37	//>.
#define kQM 		0x38	//?/
#define kBR 		0x30	//}]
#define kBL 		0x2F	//{[

#define kUp		0x52		//Up arrow
#define kLeft 	0x50		//Left arrow
#define kRight 	0x4F		//Right arrow
#define kDown 	0x51		//Down arrow

#define kK14	0x89		//international key
#define kKR_L	0x91		//
#define kK107	0x85		//
#define kK45	0x64		//
#define kK42	0x32		//
#define kK131	0x8b 		//
#define kK132	0x8a		//
#define kK133 	0x88		//
#define kK56	0x87		//
#define kKR_R	0x90		//

//keyboard standard place
//R0
#define R0_C0 0x00
#define R0_C1 0x01
#define R0_C2 0x02
#define R0_C3 0x03
#define R0_C4 0x04
#define R0_C5 0x05
#define R0_C6 0x06
#define R0_C7 0x07
#define R0_C8 0x08
#define R0_C9 0x09
#define R0_C10 0x0A
#define R0_C11 0x0B
#define R0_C12 0x0C
#define R0_C13 0x0D
#define R0_C14 0x0E
#define R0_C15 0x0F
#define R0_C16 0x10
#define R0_C17 0x11
#define R0_C18 0x12
//R1
#define R1_C0 0x20
#define R1_C1 0x21
#define R1_C2 0x22
#define R1_C3 0x23
#define R1_C4 0x24
#define R1_C5 0x25
#define R1_C6 0x26
#define R1_C7 0x27
#define R1_C8 0x28
#define R1_C9 0x29
#define R1_C10 0x2A
#define R1_C11 0x2B
#define R1_C12 0x2C
#define R1_C13 0x2D
#define R1_C14 0x2E
#define R1_C15 0x2F
#define R1_C16 0x30
#define R1_C17 0x31
#define R1_C18 0x32
//R2
#define R2_C0 0x40
#define R2_C1 0x41
#define R2_C2 0x42
#define R2_C3 0x43
#define R2_C4 0x44
#define R2_C5 0x45
#define R2_C6 0x46
#define R2_C7 0x47
#define R2_C8 0x48
#define R2_C9 0x49
#define R2_C10 0x4A
#define R2_C11 0x4B
#define R2_C12 0x4C
#define R2_C13 0x4D
#define R2_C14 0x4E
#define R2_C15 0x4F
#define R2_C16 0x50
#define R2_C17 0x51
#define R2_C18 0x52
//R3
#define R3_C0 0x60
#define R3_C1 0x61
#define R3_C2 0x62
#define R3_C3 0x63
#define R3_C4 0x64
#define R3_C5 0x65
#define R3_C6 0x66
#define R3_C7 0x67
#define R3_C8 0x68
#define R3_C9 0x69
#define R3_C10 0x6A
#define R3_C11 0x6B
#define R3_C12 0x6C
#define R3_C13 0x6D
#define R3_C14 0x6E
#define R3_C15 0x6F
#define R3_C16 0x70
#define R3_C17 0x71
#define R3_C18 0x72
//R4
#define R4_C0 0x80
#define R4_C1 0x81
#define R4_C2 0x82
#define R4_C3 0x83
#define R4_C4 0x84
#define R4_C5 0x85
#define R4_C6 0x86
#define R4_C7 0x87
#define R4_C8 0x88
#define R4_C9 0x89
#define R4_C10 0x8A
#define R4_C11 0x8B
#define R4_C12 0x8C
#define R4_C13 0x8D
#define R4_C14 0x8E
#define R4_C15 0x8F
#define R4_C16 0x90
#define R4_C17 0x91
#define R4_C18 0x92
//R5
#define R5_C0 0xA0
#define R5_C1 0xA1
#define R5_C2 0xA2
#define R5_C3 0xA3
#define R5_C4 0xA4
#define R5_C5 0xA5
#define R5_C6 0xA6
#define R5_C7 0xA7
#define R5_C8 0xA8
#define R5_C9 0xA9
#define R5_C10 0xAA
#define R5_C11 0xAB
#define R5_C12 0xAC
#define R5_C13 0xAD
#define R5_C14 0xAE
#define R5_C15 0xAF
#define R5_C16 0xB0
#define R5_C17 0xB1
#define R5_C18 0xB2
//R6
#define R6_C0 0xC0
#define R6_C1 0xC1
#define R6_C2 0xC2
#define R6_C3 0xC3
#define R6_C4 0xC4
#define R6_C5 0xC5
#define R6_C6 0xC6
#define R6_C7 0xC7
#define R6_C8 0xC8
#define R6_C9 0xC9
#define R6_C10 0xCA
#define R6_C11 0xCB
#define R6_C12 0xCC
#define R6_C13 0xCD
#define R6_C14 0xCE
#define R6_C15 0xCF
#define R6_C16 0xD0
#define R6_C17 0xD1
#define R6_C18 0xD2
//R7
#define R7_C0 0xE0
#define R7_C1 0xE1
#define R7_C2 0xE2
#define R7_C3 0xE3
#define R7_C4 0xE4
#define R7_C5 0xE5
#define R7_C6 0xE6
#define R7_C7 0xE7
#define R7_C8 0xE8
#define R7_C9 0xE9
#define R7_C10 0xEA
#define R7_C11 0xEB
#define R7_C12 0xEC
#define R7_C13 0xED
#define R7_C14 0xEE
#define R7_C15 0xEF
#define R7_C16 0xF0
#define R7_C17 0xF1
#define R7_C18 0xF2

// ct key
#define L_Ctrl  0xC0
#define L_Alt   0x69
#define L_Shift 0x2F
#define L_Win	0x30
#define R_Ctrl  0x80
#define R_Alt   0xA9
#define R_Shift 0x4F
#define R_Win 	0x51

//fn key
#define FN_F1 0xC2
#define FN_F2 0xC3
#define FN_F3 0x23
#define FN_F4 0x63
#define FN_F5 0xE0
#define FN_F6 0x66
#define FN_F7 0x27
#define FN_F8 0xC7
#define FN_F9 0xCA
#define FN_F10 0xEA  
#define FN_F11 0x6A
#define FN_F12 0xAA

#define FN_INS 0xCC

#define FN_UP 0x6E
#define FN_DOWN 0xAB
#define FN_LEFT 0xAE
#define FN_RIGHT 0xAC

#define FN_7 0xE5
#define FN_8 0xE6
#define FN_9 0xE7
#define FN_ZERO 0xE8      //*
#define FN_U 0x05         //4
#define FN_I 0x06         //5
#define FN_O 0x07         //6
#define FN_P 0x08         //-
#define FN_J 0x45         //1
#define FN_K 0x46         //2
#define FN_L 0x47        //3
#define FN_Colon 0x48     //+
#define FN_M 0x85          //0
#define FN_GTS 0x87       //.
#define FN_QM  0xA8       // /


// end package
#define END_NULL 0xFF

#endif //_DEFINE_APPLICATION_H_
/***********************************************************
						end file
***********************************************************/
