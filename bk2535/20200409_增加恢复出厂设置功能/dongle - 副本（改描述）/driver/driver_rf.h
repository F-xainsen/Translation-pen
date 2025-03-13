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
/*! \file driver_rf.h
    \brief The driver headfile of rf module.
*/

#ifndef _DRIVER_RF_H_
#define _DRIVER_RF_H_

/*
P0.2	IRQ	
P0.3	MISO	RF- BK2423
P0.4	MOSI	RF- BK2423
P0.5	SCK	RF- BK2423
P0.6	CSN	RF- BK2423
P0.7	CE	RF- BK2423
*/
sbit RF_SPI_CE =  P0^7;
sbit RF_SPI_CS =  P0^6;
sbit RF_SPI_CLK =  P0^5;
sbit RF_SPI_MOSI = P0^4;
sbit RF_SPI_MISO = P0^3;
sbit RF_SPI_IRQ = P0^2;

#define SYNC_TIMES    2
        
#define RF_ADDRESS_LEN 				            5
#define RF_ADDRESS_FIRST_BYTE                   0xBC

#define ENABLE_RF_WAKEUP                        P0_WUEN |= B_0000_0100
#define DISABLE_RF_WAKEUP                       P0_WUEN &= ~B_0000_0100

#define RF_CHIP_ENABLE                          RF_SPI_CE = 0x01
#define RF_CHIP_DISABLE                         RF_SPI_CE = 0x00

//2433
#define MAX_PACKET_LEN                          32// max value is 32

#define RF_OUT_POWER_LOW                        0
#define RF_OUT_POWER_NORMAL                     1  

#define RF_POWER_MODE_DOWN                      0
#define RF_POWER_MODE_UP                        1

#define RF_MODE_SINGLE                          0
#define RF_MODE_NORMAL                          1

#define POWER_VALUE_5DBM                        5
#define POWER_VALUE_0DBM                        4
#define POWER_VALUE_MINUS_5DBM                  3
#define POWER_VALUE_MINUS_10DBM                 2
#define POWER_VALUE_MINUS_15DBM                 1
#define POWER_VALUE_MINUS_20DBM                 0

#define RF_POWER_DOWN                           driver_rf_spi_power_mode(RF_POWER_MODE_DOWN)
#define RF_POWER_UP                             driver_rf_spi_power_mode(RF_POWER_MODE_UP)

// Interrupt status
#define RF_CMD_NOP   		                   
#define RF_CMD_FLUSH_TX 	                    driver_rf_spi_flush_tx()
#define RF_CMD_FLUSH_RX 	                    driver_rf_spi_flush_rx()
#define RF_CMD_CLEAR_STATUS_RX 	                driver_rf_spi_write_register(WRITE_REG|STATUS, STATUS_RX_DR)
#define RF_CMD_CLEAR_STATUS_TX 	                driver_rf_spi_write_register(WRITE_REG|STATUS, STATUS_TX_DS)
#define RF_CMD_CLEAR_STATUS_MAX                 driver_rf_spi_write_register(WRITE_REG|STATUS, STATUS_MAX_RT)
#define RF_CMD_CLEAR_STATUS_ALL                 driver_rf_spi_write_register(WRITE_REG|STATUS, STATUS_RX_TX_MAX)

//2423
//************************FSK COMMAND and REGISTER****************************************//
// SPI(BK2421) commands
#define READ_REG        0x00  // Define read command to register
#define WRITE_REG       0x20  // Define write command to register
#define RD_RX_PLOAD     0x61  // Define RX payload register address
#define WR_TX_PLOAD     0xA0  // Define TX payload register address
#define FLUSH_TX        0xE1  // Define flush TX register command
#define FLUSH_RX        0xE2  // Define flush RX register command
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command
#define W_TX_PAYLOAD_NOACK_CMD	0xb0
#define W_ACK_PAYLOAD_CMD	0xa8
#define ACTIVATE_CMD		0x50
#define R_RX_PL_WID_CMD		0x60
#define NOP             0xFF  // Define No Operation, might be used to read status register

// SPI(BK2421) registers(addresses)
#define CONFIG          0x00  // 'Config' register address
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address
#define SETUP_AW        0x03  // 'Setup address width' register address
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address
#define RF_CH           0x05  // 'RF channel' register address
#define RF_SETUP        0x06  // 'RF setup' register address
#define STATUS          0x07  // 'Status' register address
#define OBSERVE_TX      0x08  // 'Observe TX' register address
#define CD              0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
#define TX_ADDR         0x10  // 'TX address' register address
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address
#define DYNPD   	    0x1C
#define RF_FEATURE      0x1D
/*
1D	FEATURE			R/W	Feature Register  
	Reserved	7:3	0	R/W	Only '00000' allowed  
	EN_DPL	2	0	R/W	Enables Dynamic Payload Length  
	EN_ACK_PAY	1	0	R/W	Enables Payload with ACK  
	EN_DYN_ACK	0	0	R/W	Enables the W_TX_PAYLOAD_NOACK command  
*/
#define PAYLOAD_WIDTH   0x1f  // 'payload length of 256 bytes modes register address

//interrupt status
#define STATUS_RX_TX_MAX        0x70
#define STATUS_RX_DR            0x40
#define STATUS_TX_DS            0x20
#define STATUS_MAX_RT           0x10

#define STATUS_TX_FULL          0x01

//FIFO_STATUS
#define FIFO_STATUS_TX_REUSE    0x40
#define FIFO_STATUS_TX_FULL     0x20
#define FIFO_STATUS_TX_EMPTY    0x10

#define FIFO_STATUS_RX_FULL     0x02
#define FIFO_STATUS_RX_EMPTY    0x01


UINT8 driver_rf_spi_rw(UINT8 value);                                    
UINT8 driver_rf_spi_read_register(UINT8 reg);                               
void driver_rf_spi_write_register(UINT8 reg, UINT8 value);                 
void driver_rf_spi_read_buffer(UINT8 reg,  UINT8 *pBuf,  UINT8 length);     
void driver_rf_spi_write_buffer(UINT8 reg, UINT8 *pBuf, UINT8 length);    
void driver_rf_spi_flush_tx(void);
void driver_rf_spi_flush_rx(void);
void driver_rf_spi_set_mode_tx(void);
void driver_rf_spi_set_mode_rx(void);
void driver_rf_spi_power_mode(UINT8 mode);
void driver_rf_ouput_power_value_set(UINT8 power);
void driver_rf_spi_switch_bank(char bank);   
void driver_rf_spi_write_bank1_reg(UINT8 reg, UINT32 value);
void driver_rf_initial(void);
BOOL driver_rf_data_send(UINT8 len);
void driver_rf_receive_packet(void);
void driver_rf_channel_switch(void);
//void driver_rf_page_write_address(void);

#if 1//def MODE_TEST
void driver_rf_mode_set_single_wave(UINT8 mode);
void driver_rf_spi_set_mode_tx();
#endif

#endif //_DRIVER_RF_H_

