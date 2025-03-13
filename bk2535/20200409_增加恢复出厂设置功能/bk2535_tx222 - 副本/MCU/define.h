/***********************************************************
FileName	: define.h
Date		: 2011/03/01
Description	: 
Version		: v0.1
Function List: 
----
History: 
<author> <time> <version > <desc>
***********************************************************/

#ifndef _DEFINE_H_
#define _DEFINE_H_

//type redefine
typedef void				VOID, *PVOID;
typedef char				BOOL;
typedef char				BOOLEAN;
typedef unsigned char		UINT8,BYTE, UCHAR, *PUCHAR, uint8;
typedef char				INT8;
typedef short				INT16;
typedef unsigned short		UINT16, uint16;
typedef unsigned long		UINT32, uint32;
typedef long				INT32;

#define VOLATILE        volatile
#define	XDATA			xdata
#define PDATA			pdata
#define IDATA			idata
#define	DATA			data
#define	CODE			code

typedef enum _APPLICATION_MODE
{
    SYSTEM_NORMAL = 0x00,
    SYSTEM_PAGE,
    SYSTEM_CONFIGURE,
    SYSTEM_DEBUG,
    SYSTEM_TEST,
    SYSTEM_MODE_MAX
} SYSTEM_MODE;

typedef union {unsigned int i; unsigned char c[2]; } WORD;

// constant definition
#define FALSE			0
#define TRUE			1

#define SUCCESS			0
#define FAILURE			1

#define	B_0000_0000		0x00
#define	B_0000_0001		0x01
#define	B_0000_0010		0x02
#define	B_0000_0011		0x03
#define	B_0000_0100		0x04
#define	B_0000_0101		0x05
#define	B_0000_0110		0x06
#define	B_0000_0111		0x07
#define	B_0000_1000		0x08
#define	B_0000_1001		0x09
#define	B_0000_1010		0x0A
#define	B_0000_1011		0x0B
#define	B_0000_1100		0x0C
#define	B_0000_1101		0x0D
#define	B_0000_1110		0x0E
#define	B_0000_1111		0x0F
#define	B_0001_0000		0x10
#define	B_0001_0001		0x11
#define	B_0001_0010		0x12
#define	B_0001_0011		0x13
#define	B_0001_0100		0x14
#define	B_0001_0101		0x15
#define	B_0001_0110		0x16
#define	B_0001_0111		0x17
#define	B_0001_1000		0x18
#define	B_0001_1001		0x19
#define	B_0001_1010		0x1A
#define	B_0001_1011		0x1B
#define	B_0001_1100		0x1C
#define	B_0001_1101		0x1D
#define	B_0001_1110		0x1E
#define	B_0001_1111		0x1F
#define	B_0010_0000		0x20
#define	B_0010_0001		0x21
#define	B_0010_0010		0x22
#define	B_0010_0011		0x23
#define	B_0010_0100		0x24
#define	B_0010_0101		0x25
#define	B_0010_0110		0x26
#define	B_0010_0111		0x27
#define	B_0010_1000		0x28
#define	B_0010_1001		0x29
#define	B_0010_1010		0x2A
#define	B_0010_1011		0x2B
#define	B_0010_1100		0x2C
#define	B_0010_1101		0x2D
#define	B_0010_1110		0x2E
#define	B_0010_1111		0x2F
#define	B_0011_0000		0x30
#define	B_0011_0001		0x31
#define	B_0011_0010		0x32
#define	B_0011_0011		0x33
#define	B_0011_0100		0x34
#define	B_0011_0101		0x35
#define	B_0011_0110		0x36
#define	B_0011_0111		0x37
#define	B_0011_1000		0x38
#define	B_0011_1001		0x39
#define	B_0011_1010		0x3A
#define	B_0011_1011		0x3B
#define	B_0011_1100		0x3C
#define	B_0011_1101		0x3D
#define	B_0011_1110		0x3E
#define	B_0011_1111		0x3F
#define	B_0100_0000		0x40
#define	B_0100_0001		0x41
#define	B_0100_0010		0x42
#define	B_0100_0011		0x43
#define	B_0100_0100		0x44
#define	B_0100_0101		0x45
#define	B_0100_0110		0x46
#define	B_0100_0111		0x47
#define	B_0100_1000		0x48
#define	B_0100_1001		0x49
#define	B_0100_1010		0x4A
#define	B_0100_1011		0x4B
#define	B_0100_1100		0x4C
#define	B_0100_1101		0x4D
#define	B_0100_1110		0x4E
#define	B_0100_1111		0x4F
#define	B_0101_0000		0x50
#define	B_0101_0001		0x51
#define	B_0101_0010		0x52
#define	B_0101_0011		0x53
#define	B_0101_0100		0x54
#define	B_0101_0101		0x55
#define	B_0101_0110		0x56
#define	B_0101_0111		0x57
#define	B_0101_1000		0x58
#define	B_0101_1001		0x59
#define	B_0101_1010		0x5A
#define	B_0101_1011		0x5B
#define	B_0101_1100		0x5C
#define	B_0101_1101		0x5D
#define	B_0101_1110		0x5E
#define	B_0101_1111		0x5F
#define	B_0110_0000		0x60
#define	B_0110_0001		0x61
#define	B_0110_0010		0x62
#define	B_0110_0011		0x63
#define	B_0110_0100		0x64
#define	B_0110_0101		0x65
#define	B_0110_0110		0x66
#define	B_0110_0111		0x67
#define	B_0110_1000		0x68
#define	B_0110_1001		0x69
#define	B_0110_1010		0x6A
#define	B_0110_1011		0x6B
#define	B_0110_1100		0x6C
#define	B_0110_1101		0x6D
#define	B_0110_1110		0x6E
#define	B_0110_1111		0x6F
#define	B_0111_0000		0x70
#define	B_0111_0001		0x71
#define	B_0111_0010		0x72
#define	B_0111_0011		0x73
#define	B_0111_0100		0x74
#define	B_0111_0101		0x75
#define	B_0111_0110		0x76
#define	B_0111_0111		0x77
#define	B_0111_1000		0x78
#define	B_0111_1001		0x79
#define	B_0111_1010		0x7A
#define	B_0111_1011		0x7B
#define	B_0111_1100		0x7C
#define	B_0111_1101		0x7D
#define	B_0111_1110		0x7E
#define	B_0111_1111		0x7F
#define	B_1000_0000		0x80
#define	B_1000_0001		0x81
#define	B_1000_0010		0x82
#define	B_1000_0011		0x83
#define	B_1000_0100		0x84
#define	B_1000_0101		0x85
#define	B_1000_0110		0x86
#define	B_1000_0111		0x87
#define	B_1000_1000		0x88
#define	B_1000_1001		0x89
#define	B_1000_1010		0x8A
#define	B_1000_1011		0x8B
#define	B_1000_1100		0x8C
#define	B_1000_1101		0x8D
#define	B_1000_1110		0x8E
#define	B_1000_1111		0x8F
#define	B_1001_0000		0x90
#define	B_1001_0001		0x91
#define	B_1001_0010		0x92
#define	B_1001_0011		0x93
#define	B_1001_0100		0x94
#define	B_1001_0101		0x95
#define	B_1001_0110		0x96
#define	B_1001_0111		0x97
#define	B_1001_1000		0x98
#define	B_1001_1001		0x99
#define	B_1001_1010		0x9A
#define	B_1001_1011		0x9B
#define	B_1001_1100		0x9C
#define	B_1001_1101		0x9D
#define	B_1001_1110		0x9E
#define	B_1001_1111		0x9F
#define	B_1010_0000		0xA0
#define	B_1010_0001		0xA1
#define	B_1010_0010		0xA2
#define	B_1010_0011		0xA3
#define	B_1010_0100		0xA4
#define	B_1010_0101		0xA5
#define	B_1010_0110		0xA6
#define	B_1010_0111		0xA7
#define	B_1010_1000		0xA8
#define	B_1010_1001		0xA9
#define	B_1010_1010		0xAA
#define	B_1010_1011		0xAB
#define	B_1010_1100		0xAC
#define	B_1010_1101		0xAD
#define	B_1010_1110		0xAE
#define	B_1010_1111		0xAF
#define	B_1011_0000		0xB0
#define	B_1011_0001		0xB1
#define	B_1011_0010		0xB2
#define	B_1011_0011		0xB3
#define	B_1011_0100		0xB4
#define	B_1011_0101		0xB5
#define	B_1011_0110		0xB6
#define	B_1011_0111		0xB7
#define	B_1011_1000		0xB8
#define	B_1011_1001		0xB9
#define	B_1011_1010		0xBA
#define	B_1011_1011		0xBB
#define	B_1011_1100		0xBC
#define	B_1011_1101		0xBD
#define	B_1011_1110		0xBE
#define	B_1011_1111		0xBF
#define	B_1100_0000		0xC0
#define	B_1100_0001		0xC1
#define	B_1100_0010		0xC2
#define	B_1100_0011		0xC3
#define	B_1100_0100		0xC4
#define	B_1100_0101		0xC5
#define	B_1100_0110		0xC6
#define	B_1100_0111		0xC7
#define	B_1100_1000		0xC8
#define	B_1100_1001		0xC9
#define	B_1100_1010		0xCA
#define	B_1100_1011		0xCB
#define	B_1100_1100		0xCC
#define	B_1100_1101		0xCD
#define	B_1100_1110		0xCE
#define	B_1100_1111		0xCF
#define	B_1101_0000		0xD0
#define	B_1101_0001		0xD1
#define	B_1101_0010		0xD2
#define	B_1101_0011		0xD3
#define	B_1101_0100		0xD4
#define	B_1101_0101		0xD5
#define	B_1101_0110		0xD6
#define	B_1101_0111		0xD7
#define	B_1101_1000		0xD8
#define	B_1101_1001		0xD9
#define	B_1101_1010		0xDA
#define	B_1101_1011		0xDB
#define	B_1101_1100		0xDC
#define	B_1101_1101		0xDD
#define	B_1101_1110		0xDE
#define	B_1101_1111		0xDF
#define	B_1110_0000		0xE0
#define	B_1110_0001		0xE1
#define	B_1110_0010		0xE2
#define	B_1110_0011		0xE3
#define	B_1110_0100		0xE4
#define	B_1110_0101		0xE5
#define	B_1110_0110		0xE6
#define	B_1110_0111		0xE7
#define	B_1110_1000		0xE8
#define	B_1110_1001		0xE9
#define	B_1110_1010		0xEA
#define	B_1110_1011		0xEB
#define	B_1110_1100		0xEC
#define	B_1110_1101		0xED
#define	B_1110_1110		0xEE
#define	B_1110_1111		0xEF
#define	B_1111_0000		0xF0
#define	B_1111_0001		0xF1
#define	B_1111_0010		0xF2
#define	B_1111_0011		0xF3
#define	B_1111_0100		0xF4
#define	B_1111_0101		0xF5
#define	B_1111_0110		0xF6
#define	B_1111_0111		0xF7
#define	B_1111_1000		0xF8
#define	B_1111_1001		0xF9
#define	B_1111_1010		0xFA
#define	B_1111_1011		0xFB
#define	B_1111_1100		0xFC
#define	B_1111_1101		0xFD
#define	B_1111_1110		0xFE 
#define	B_1111_1111		0xFF


//	pc connect
#define PC_GET_REPORT_1 0xba
#define PC_GET_REPORT_2 0x01

#define PC_SET_REPORT_1 0xba
#define PC_SET_REPORT_2	0x02

#define PC_GET_DATA 	0x01
#define PC_GET_NONDATA 	0x00


// endpoint number
#define  EP_0          0x0
#define  EP_1          0x1
#define  EP_2          0x2
#define  EP_3          0x3
#define  EP_4          0x4
#define  EP_5          0x5
#define  EP_6          0x6
#define  EP_7          0x7


#define  DEVICE     0
#define  INTERFACE  1
#define  ENDPOINT   2

#ifndef  MSB
	#define  MSB 0
#endif

#ifndef  LSB
	#define  LSB 1
#endif

// Define wIndex bitmaps
#define  IN_EP1         0x81      // Index values used by Set and Clear
#define  OUT_EP1        0x01      // commands for Endpoint_Halt
#define  IN_EP2         0x82
#define  OUT_EP2        0x02
#define  IN_EP3         0x83
#define  OUT_EP3        0x03
#define  IN_EP4         0x84
#define  OUT_EP4        0x04
#define  IN_EP5         0x85
#define  OUT_EP5        0x05
#define  IN_EP6         0x86
#define  OUT_EP6        0x06
#define  IN_EP7         0x87
#define  OUT_EP7        0x07

#define EP_IDLE  		0x00
#define EP_TX    		0x11
#define EP_RX    		0x10
#define EP_SET_ADDRESS 	0x21
#define EP_WAIT_STATUS 	0x20
#define EP_STALL 		0xaa




// HID Descriptor Types
#define DSC_HID					0x21  // HID Class Descriptor
#define DSC_HID_REPORT			0x22   // HID Report Descriptor

// Standard Descriptor Types
#define  DSC_DEVICE          	0x01      // Device Descriptor
#define  DSC_CONFIG          	0x02      // Configuration Descriptor
#define  DSC_STRING          	0x03      // String Descriptor
#define  DSC_INTERFACE       	0x04      // Interface Descriptor
#define  DSC_ENDPOINT        	0x05      // Endpoint Descriptor

// Standard Request Codes
#define  GET_STATUS             0x00  // Code for Get Status
#define  CLEAR_FEATURE          0x01  // Code for Clear Feature
#define  SET_FEATURE            0x03  // Code for Set Feature
#define  SET_ADDRESS            0x05  // Code for Set Address
#define  GET_DESCRIPTOR         0x06  // Code for Get Descriptor
#define  SET_DESCRIPTOR         0x07  // Code for Set Descriptor(not used)
#define  GET_CONFIGURATION      0x08  // Code for Get Configuration
#define  SET_CONFIGURATION      0x09  // Code for Set Configuration
#define  GET_INTERFACE          0x0A  // Code for Get Interface
#define  SET_INTERFACE          0x0B  // Code for Set Interface
#define  SYNCH_FRAME            0x0C  // Code for Synch Frame(not used)

// HID Request Codes
#define GET_REPORT              0x01   // Code for Get Report
#define GET_IDLE                0x02   // Code for Get Idle
#define GET_PROTOCOL            0x03   // Code for Get Protocol
#define SET_REPORT              0x09   // Code for Set Report
#define SET_IDLE                0x0A   // Code for Set Idle
#define SET_PROTOCOL            0x0B   // Code for Set Protocol

// Define device states
#define  DEV_ATTACHED           0x00  // Device is in Attached State
#define  DEV_POWERED            0x01  // Device is in Powered State
#define  DEV_DEFAULT            0x02  // Device is in Default State
#define  DEV_ADDRESS            0x03  // Device is in Addressed State
#define  DEV_CONFIGURED         0x04  // Device is in Configured State
#define  DEV_SUSPENDED          0x05  // Device is in Suspended State

// Define bmRequestType bitmaps
#define  IN_DEVICE              0x00  // Request made to device, direction is IN
#define  OUT_DEVICE             0x80  // Request made to device, direction is OUT

#define  IN_INTERFACE           0x01  // Request made to interface, direction is IN
#define  OUT_INTERFACE          0x81  // Request made to interface, direction is OUT

#define  IN_ENDPOINT            0x02  // Request made to endpoint, direction is IN
#define  OUT_ENDPOINT           0x82  // Request made to endpoint, direction is OUT

// Define wValue bitmaps for Standard Feature Selectors
#define  DEVICE_REMOTE_WAKEUP   0x01  // Remote wakeup feature(not used)
#define  ENDPOINT_HALT          0x00  // Endpoint_Halt feature selector

//interrupt status
#define STATUS_RX_DR 	0x40
#define STATUS_TX_DS 	0x20
#define STATUS_MAX_RT 	0x10
#define STATUS_TX_FULL 	0x01

//CD status
#define CD_INT 			0x01

//FIFO_STATUS
#define FIFO_STATUS_TX_REUSE	0x40
#define FIFO_STATUS_TX_FULL		0x20
#define FIFO_STATUS_TX_EMPTY 	0x10
#define FIFO_STATUS_RX_FULL 	0x02
#define FIFO_STATUS_RX_EMPTY 	0x01



#define FLUSH_TX 	{BK2401_CMD = 0xA0;}
#define FLUSH_RX 	{BK2401_CMD = 0x80;}
#define REUSE_TX_PL {BK2401_CMD = 0x10;}
#define NOP   		{BK2401_CMD = 0x00;}

#define ENABLE_RF_INTR  EX5 = 1
#define DISABLE_RF_INTR EX5 = 0

#define	ENABLE_INTERRUPT	EA=1
#define	DISABLE_INTERRUPT	EA=0

#define ENABLE_RF_INT	AIE |= 0x8
#define DISABLE_RF_INT  AIE &= ~0x8


//	mcu idle sleep
#if MCU_TYPE & MCU_OLD
#define MCU_IDLE     		PCON = 0x01 
#define MCU_IDLE_32K 		PCON = 0x05 
#define MCU_SLEEP    		PCON = 0x03

#define MCU_IDLE_OSCXMHZ  	PCON |= 0x01 
#define MCU_IDLE_OSC32KHZ 	PCON = 0x05 
#define MCU_IDLE_RC32KHZ  	PCON = 0x03
#else
#define MCU_IDLE     		PCON2 = 0x01 
#define MCU_IDLE_32K 		PCON2 = 0x05 
#define MCU_SLEEP    		PCON2 = 0x03

#define MCU_IDLE_OSCXMHZ  	PCON2 |= 0x01 
#define MCU_IDLE_OSC32KHZ 	PCON2 = 0x05 
#define MCU_IDLE_RC32KHZ  	PCON2 = 0x03
#endif


#define MCU_TIMER_STOP TR0 = 0 ; ET0 = 0; SPC_FNC_TEMP = SPC_FNC; SPC_FNC = 0x00
#define MCU_TIMER_STAT SPC_FNC = SPC_FNC_TEMP; TR0 = 1 ; ET0 = 1

//	MDU
#define MDU_ENABLE SPC_FNC |= 0x01 
#define MDU_DISABLE SPC_FNC &= ~0x01 

//	DES
#define DES_ENABLE SPC_FNC |= 0x02 
#define DES_DISABLE SPC_FNC &= ~0x02 

//	I2C
#define I2C_ENABLE SPC_FNC |= 0x04 
#define I2C_DISABLE SPC_FNC &= ~0x04 

//	SPI
#define SPI_ENABLE SPC_FNC |= 0x08 
#define SPI_DISABLE SPC_FNC &= ~0x08 

//	PWM
#define PWM_ENABLE SPC_FNC |= 0x10 
#define PWM_DISABLE SPC_FNC &= ~0x10 

//	UART
#define UART_ENABLE SPC_FNC |= 0x20 
#define UART_DISABLE SPC_FNC &= ~0x20 

//	TIMER
#define TIMER_ENABLE SPC_FNC |= 0x40 
#define TIMER_DISABLE SPC_FNC &= ~0x40 

//	ADC
#define ADC_ENABLE SPC_FNC |= 0x80 
#define ADC_DISABLE SPC_FNC &= ~0x80 




//		mcu clk
//
#define	CKCON_CLK_16MHZ		0x00
#define	CKCON_CLK_8MHZ		0x40
#define	CKCON_CLK_4MHZ		0x80
#define	CKCON_CLK_2MHZ		0xc0

#define	CKCON_TIMER2CLK_1_12	0x00
#define	CKCON_TIMER2CLK_1_4		0x20
#define	CKCON_TIMER1CLK_1_12	0x00
#define	CKCON_TIMER1CLK_1_4		0x10
#define	CKCON_TIMER0CLK_1_12	0x00
#define	CKCON_TIMER0CLK_1_4		0x08



#endif
/***********************************************************
						end file
***********************************************************/