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

#ifndef _DEFINE_REGISTER_H_
#define _DEFINE_REGISTER_H_

#include<ABSACC.H>

#define  PCHAR (unsigned char volatile xdata *)

#define FRAM_NO_0  XBYTE[0x0808] 
#define FRAM_NO_1  XBYTE[0x0809] 
                                
#define USBINT0   XBYTE[0x080a]
/* software should clear the interrupt bit by set it to 1 after dealing with the interrupt.
7. ctl_rec: data received on control port (endpoint 0) 
6. ctl_send: data send on control port (endpoint 0) 
5. rx_rdy £ºdata received on endpoint 1-7 
4. tx_rdy £ºdata send on endpoint 1-7 
3. usb_reset £º USB Reset interrupt 
2. usb_sus £ºUSB suspend interrupt
1. usb_res£ºUSB Resume interrupt¡£ 
0. usb_sof£º USB Start Of Frame interrupt 
When ctl_rec, rx_rdy or tx_rdy triggered£¬need to query EP_STATUS register for detail information.
*/
#define USBINT1   XBYTE[0x080b]
/* software should clear the interrupt bit by set it to 1 after dealing with the interrupt.
7. bad_token: unsupported token received 
6. crc16_err : the package received crc16 check error 
5. overtime £ºtimeout interrupt(no data received after OUT token or no ACK received after IN token) 
4. pid_err £ºendpoint1-7 transfer PID error interrupt
*/
#define USB_EN0   XBYTE[0x080c]
/*
7. ctl_rec_en : data received on control endpoint 0 interrupt enable bit 
6. ctl_send_en : data sent on control endpoint 0 interrupt enable bit 
5. rx_rdy_en £ºdata received on endpoint 1-7 interrupt enable bit 
4. tx_rdy_en £ºdata sent on endpoint 1-7 interrupt enable bit 
3. usb_reset_en: USB Reset interrupt enable bit 
2. usb_sus_en £ºUSB suspend interrupt enable bit 
1. usb_res_en £ºUSB Resume interrupt enable bit 
0. usb_sof_en £º USB Start Of Frame interrupt enable bit
*/
#define USB_EN1   XBYTE[0x080d]
/*
7. bad_token_en: unsupported token received interrupt enable bit 
6. crc16_err_en : the package received crc16 check error interrupt enable bit 
5. overtime_en £ºtimeout interrupt enable bit 
4. pid_err_en £ºendpoint1-7 transfer PID error interrupt enable bit
*/
#ifdef BK5100_FEATURE
#define EP_STATUS_IN XBYTE[0x080e]
#define EP_STATUS_OUT XBYTE[0x080f]
#else
#define EP_STATUS XBYTE[0x080e]
#endif
/*set by hardware and cleared by software
7. EP7 : indicate rx_rdy or tx_rdy is triggered by endpoint 7 
6. EP6 : indicate rx_rdy or tx_rdy is triggered by endpoint 6 
5. EP5£ºindicate rx_rdy or tx_rdy is triggered by endpoint 5 
4. EP4£ºindicate rx_rdy or tx_rdy is triggered by endpoint 4 
3. EP3£ºindicate rx_rdy or tx_rdy is triggered by endpoint 3 
2. EP2£ºindicate rx_rdy or tx_rdy is triggered by endpoint 2 
1. EP1£ºindicate rx_rdy or tx_rdy is triggered by endpoint 1 
0. Sudat£ºindicate that 8 bytes set up package arrived, otherwise not setup token
*/                                
                                
#define CFG_EP0_1     XBYTE[0x0810]
/*
7. Dir, port direction 1: IN (BK2433 send out data)£» 0: OUT (the PC send out data)¡£ 
6. ep0_en£¬endpoint 0 enable When ep_rdy[0] =0 and ep0_en=0, usb no respond to external now.
1-0. addr[9:8] £ºThe higher 2 bits ([9:8]) address of endpoint0. The low 8 bits address is stored in CFG_EP0_0.
Note: The dir bit of CFG_EP0_1 is set or cleared by software except that it is cleared by hardware when SETUP token coming. 
The direction is forced to OUT to access 8 bytes setup request in this condition. The setup request has the highest priority.
*/
#define CFG_EP0_0     XBYTE[0x0811]
                                 
#define CFG_EP1_1     XBYTE[0x0812]
/*
7. Dir: 1: IN
          0: OUT 
6-5. Mode: 
          0 -- Control Transfer 
          1 -- Bulk Transfer 
          2 -- ISO Transfer 
          3 -- Interrupt Transfer 
4-2. Size £º 
          0¡ª endpoint not available 
          1¡ª 16 bytes buffer size 
          2¡ª32 bytes buffer size 
          3¡ª64 bytes buffer size 
          4¡ª128 bytes buffer size 
          5¡ª256 bytes buffer size 
          6¡ª512 bytes buffer size 
          7¡ªendpoint not available 
1-0. Addr[9:8] £ºThe higher 2 bits ([9:8]) address of endpoint n.
*/
#define CFG_EP1_0     XBYTE[0x0813]
                         
#define CFG_EP2_1     XBYTE[0x0814]
#define CFG_EP2_0     XBYTE[0x0815]
                      
#define CFG_EP3_1     XBYTE[0x0816]
#define CFG_EP3_0     XBYTE[0x0817]
                      
#define CFG_EP4_1     XBYTE[0x0818]
#define CFG_EP4_0     XBYTE[0x0819]
                      
#define CFG_EP5_1     XBYTE[0x081a]
#define CFG_EP5_0     XBYTE[0x081b]
                      
#define CFG_EP6_1     XBYTE[0x081c]
#define CFG_EP6_0     XBYTE[0x081d]
                      
#define CFG_EP7_1     XBYTE[0x081e]
#define CFG_EP7_0     XBYTE[0x081f]
                      
#define EP_HALT       XBYTE[0x0820]
/*
epn_halt: the suspend flag of endpoint n. 1 indicates the endpoint has been suspended and this endpoint is not available now. This endpoint will send back STALL when IN/OUT token received to indicate it is not available now. (epn_halt can only be
read/written by software except ep0_halt) ep0_halt can be cleared by hardware. According to USB protocol, ep0_halt is cleared by haredware when setup token received to avoid that device can not receive control information.
*/
#define EP_RDY        XBYTE[0x0821]
/*
Epn_rdy (n=1-7): endpoint n is ready for transferring USB data now.\ Cleared by hardware and set by software. 
Note: Ep0_rdy is not same with Epn_rdy. It will be forced to 1 by hardware when setup token coming to receive 8 bytes setup request. 
(setup has the highest priority for USB protocol) When Epn_rdy=0, device will send back NACK pakage for PC's IN/OUT request to indicate not ready now.
*/
#define FUNCT_ADR     XBYTE[0x0822]
                                
#define  CNT0         XBYTE[0x0823]
#define  CNT1         XBYTE[0x0824]
#define  CNT2         XBYTE[0x0825]
#define  CNT3         XBYTE[0x0826]
#define  CNT4         XBYTE[0x0827]
#define  CNT5         XBYTE[0x0828]
#define  CNT6         XBYTE[0x0829]
#define  CNT7         XBYTE[0x082a]
#define  CNT0_HBIT    XBYTE[0x082b]
#define  CNT1_HBIT    XBYTE[0x082c]
#define  CNT2_HBIT    XBYTE[0x082d]
#define  CNT3_HBIT    XBYTE[0x082e]
#define  CNT4_HBIT    XBYTE[0x082f]
#define  CNT5_HBIT    XBYTE[0x0830]
#define  CNT6_HBIT    XBYTE[0x0831]
#define  CNT7_HBIT    XBYTE[0x0832]
#define  EP_ADDR_MSB  XBYTE[0x0840]


#define  USB_PWR_CN   XBYTE[0x0841]
/*
7 Pu_en: PULL UP enable D+£¨dp£©pull up enable in chip. When it is disabled, device disconnect with outside circuit. 
6 DN: indicate D+ logic level£¨can used to debug£©. £¨read only£© 
5 DP : indicate D- logic level£¨can used to debug£©. £¨read only£© 
2 Usb_rst: USB module will reset when write 1 into it. (Exclude control register) 
1 USB_sus£º USB module will enter low-power mode when write 1 into it. The USB protocol engineer is stopped and no response to outside. It is used as suspend state usually in USB protocol.£¨can R/W by software£©
0 remote_wakeup£ºaccording USB protocol, the device with remote wakeup function can send wake up signal to host.£¨R/W£©When it is set to 1, USB force D+ and D- into K state, and release it when clear it.
*/

// added on 2011.5.7 to fix usb set_report fail
#define CONST_CFG_EP0_1 0xc2 //0x200
#define CONST_CFG_EP1_1 0xee //0x240
#define CONST_CFG_EP1_2 0xee //0x280
#define CONST_CFG_EP1_3 0xee //0x2c0

#define  EP0_ADDR_DEF  XBYTE[0x0200]
#define  EP0_ADDR_DEF_1  XBYTE[0x0201]
#define  EP0_ADDR_DEF_2  XBYTE[0x0202]
#define  EP0_ADDR_DEF_3  XBYTE[0x0203]
#define  EP0_ADDR_DEF_4  XBYTE[0x0204]
#define  EP0_ADDR_DEF_5  XBYTE[0x0205]
#define  EP0_ADDR_DEF_6  XBYTE[0x0206]
#define  EP0_ADDR_DEF_7  XBYTE[0x0207]

#define  EP1_ADDR_DEF  XBYTE[0x0240]
#define  EP2_ADDR_DEF  XBYTE[0x0280]
#define  EP3_ADDR_DEF  XBYTE[0x02c0]

/*  UINT8 Registers  */
sfr P0     = 0x80;
sfr P1     = 0x90;
sfr P2     = 0xA0;
sfr P3     = 0xB0;
sfr P4     = 0xC0;
sfr PSW    = 0xD0;
sfr ACC    = 0xE0;
sfr B      = 0xF0;
sfr SP     = 0x81;
sfr DPL    = 0x82;
sfr DPH    = 0x83;
sfr PCON   = 0x87; // 0x87: 7-SMOD 6-EUSB 5-CMD_RST 4-Latch_en 3-Deep_sleep 2-OSC32K_sel 1-RC32k_sel 0-IDLE
sfr TCON   = 0x88;
sfr TMOD   = 0x89;
sfr TL0    = 0x8A;
sfr TL1    = 0x8B;
sfr TH0    = 0x8C;
sfr TH1    = 0x8D;
sfr IE     = 0xA8;
sfr IP     = 0xA9;
sfr SCON0  = 0x98;
sfr SBUF0  = 0x99;
sfr DPL1   = 0x84;
sfr DPH1   = 0x85;
sfr DPS    = 0x86;
sfr CKCON  = 0x8E;

sfr EXIF   = 0x91;  
sfr MPAGE  = 0x92;  
sfr T2CON  = 0xC8;
sfr RCAP2L = 0xCA;
sfr RCAP2H = 0xCB;
sfr TL2    = 0xCC;
sfr TH2    = 0xCD;
sfr EICON  = 0x93;  
sfr EIE    = 0xE8;
sfr EIP    = 0xE9;

//  self defined sfr
sfr CLK_EN_CFG = 0x8F;

sfr P0_IOSEL=0xaa;
sfr P0_OPDR=0xba;

sfr P1_IOSEL=0xab;
sfr P1_OPDR=0xbb;

sfr P2_IOSEL=0xac;
sfr P2_OPDR=0xbc;

sfr P3_IOSEL=0xad;
sfr P3_OPDR=0xbd;

sfr P4_IOSEL=0xae;
sfr P4_OPDR=0xbe;

sfr P0_PU = 0x9a;
sfr P1_PU = 0x9b;
sfr P2_PU = 0x9c;
sfr P3_PU = 0x9d;
sfr P4_PU = 0x9e;

sfr P0_PD = 0xb1;
sfr P1_PD = 0xb2;
sfr P2_PD = 0xb3;
sfr P3_PD = 0xb4;
sfr P4_PD = 0xb5;

sfr DS_WUEN = 0xc3;

sfr P0_WUEN = 0xc4;
sfr P1_WUEN = 0xc5;
sfr P2_WUEN = 0xc6;
sfr P3_WUEN = 0xc7;
sfr P4_WUEN = 0xc9;


// port second function enable reg
sfr PALT=0xe7;
sfr P_EXP=0xe6;

// watch dog 
sfr WDCTL=0x97;

// EXINT_MOD
sfr EXINT_MOD = 0xea;
/*
IT9 : when IT9 = 1, int9_n(DES) edge sensitive; when IT9 = 0£¬level sensitive. bit7
IT8 : when IT8 = 1, int8(ADC) edge sensitive; when IT8 = 0£¬level sensitive. 
IT7 : when IT7 = 1, int7_n(PWM) edge sensitive; when IT7 = 0£¬level sensitive. 
IT6 : not available now.
IT5 : when IT5 = 1, int5_n(RF) edge sensitive; when IT5 = 0£¬level sensitive.
IT4 : when IT4 = 1, int4(USB1.1) edge sensitive; when IT4 = 0£¬level sensitive. 
IT3 : when IT3 = 1, int3_n(I2C) edge sensitive; when IT3 = 0£¬level sensitive. 
IT2 : when IT2 = 1, int2(SPI) edge sensitive; when IT2 = 0£¬level sensitive. bit0
*/
// random no generate
sfr RNG_DAT = 0xeb;
sfr RNG_CTL = 0xec;

// mtp 
sfr MTP_CTL = 0xed;
sfr MTP_ADR = 0xee;
sfr MTP_DAT = 0xef;

// ADC
sfr ADCDL = 0x94;
sfr ADCDH = 0x95;
sfr ADCTL = 0x96;

// PWM
sfr PWM0C0 = 0xa1;
sfr PWM0C1 = 0xa2;
sfr PWM0C2 = 0xa3;
sfr PWM1C0 = 0xa4;
sfr PWM1C1 = 0xa5;
sfr PWM1C2 = 0xa6;
sfr PWMICTL= 0xa7;

// TDES
sfr DES_CTL  = 0xd1;
sfr DES_INT  = 0xd2;
sfr DES_KEY1 = 0xd3;
sfr DES_KEY2 = 0xd4;
sfr DES_KEY3 = 0xd5;
sfr DES_IN   = 0xd6;
sfr DES_OUT  = 0xd7;

//  MDU
sfr MD0 = 0xf1;
sfr MD1 = 0xf2;
sfr MD2 = 0xf3;
sfr MD3 = 0xf4;
sfr MD4 = 0xf5;
sfr MD5 = 0xf6;
sfr MDCTL = 0xf7;

// SPI0
sfr SPI0_CN  = 0xf8;
sfr SPI0_CFG = 0xf9;
sfr SPI0_CKR = 0xfa;
sfr SPI0_DAT = 0xfb;

// SMB 
sfr SMB_DAT = 0xb6; 
sfr SMB_CF  = 0xb7; 
sfr SMB_CN  = 0xb8; 
sfr SMB_TMCTL = 0xb9;

/*  BIT Registers  */
/*  PSW */
sbit CY    = PSW^7;
sbit AC    = PSW^6;
sbit F0    = PSW^5;
sbit RS1   = PSW^4;
sbit RS0   = PSW^3;
sbit OV    = PSW^2;
sbit FL    = PSW^1;
sbit P     = PSW^0;

/*  TCON  */
sbit TF1   = TCON^7;
sbit TR1   = TCON^6;
sbit TF0   = TCON^5;
sbit TR0   = TCON^4;
sbit IE1   = TCON^3;
sbit IT1   = TCON^2;
sbit IE0   = TCON^1;
sbit IT0   = TCON^0;

/*  IE  */
sbit EA    = IE^7;
sbit ES1   = IE^6;
sbit ET2   = IE^5;
sbit ES0   = IE^4;
sbit ET1   = IE^3;
sbit EX1   = IE^2;
sbit ET0   = IE^1;
sbit EX0   = IE^0;

/*  P1  */
sbit INT5  = P1^7;
sbit INT4  = P1^6;
sbit INT3  = P1^5;
sbit INT2  = P1^4;
sbit TXD1  = P1^3;
sbit RXD1  = P1^2;
sbit T2EX  = P1^1;
sbit T2    = P1^0;

/*  P3  */
sbit RD    = P3^7;
sbit WR    = P3^6;
sbit T1    = P3^5;
sbit T0    = P3^4;
sbit INT1  = P3^3;
sbit INT0  = P3^2;
sbit TXD0  = P3^1;
sbit RXD0  = P3^0;

/*  SCON0  */
sbit SM0   = SCON0^7; /* alternative SM0_FE_0 */
sbit SM1   = SCON0^6; /* alternative SM1_0 */
sbit SM2   = SCON0^5; /* alternative SM2_0 */
sbit REN   = SCON0^4; /* alternative REN_0 */
sbit TB8   = SCON0^3; /* alternative TB8_0 */
sbit RB8   = SCON0^2; /* alternative RB8_0 */
sbit TI    = SCON0^1; /* alternative TI_0  */
sbit RI    = SCON0^0; /* alternative RI_0  */

/*  T2CON  */
sbit TF2    = T2CON^7;
sbit EXF2   = T2CON^6;
sbit RCLK   = T2CON^5;
sbit TCLK   = T2CON^4;
sbit EXEN2  = T2CON^3;
sbit TR2    = T2CON^2;
sbit C_T2   = T2CON^1;
sbit CP_RL2 = T2CON^0;

/*  EIE  
0 disables external interrupt x (intx_n) 
1 enable interrupts generated by x(intx_n)
*/
sbit EX9   = EIE^7;         //des interrupt
sbit EX8   = EIE^6;         //adc interrupt
sbit EX7   = EIE^5;         //pwm interrupt
sbit EX6   = EIE^4;         //not avaliable
sbit EX5   = EIE^3;         //rf interrupt
sbit EX4   = EIE^2;         //usb
sbit EX3   = EIE^1;         //i2c
sbit EX2   = EIE^0;         //spi

// SMB_CN
sbit MASTER   = SMB_CN^7;
sbit TXMODE       = SMB_CN^6;
sbit STA       = SMB_CN^5;
sbit STO = SMB_CN^4;
sbit ACKRQ  = SMB_CN^3;
sbit ARBLOST   = SMB_CN^2;
sbit ACK   = SMB_CN^1;
sbit SI   = SMB_CN^0;

// ana_cfg  sfr
sfr ANA_CFG0 = 0xd8;
sfr ANA_CFG1 = 0xd9;
sfr ANA_CFG2 = 0xda;
sfr ANA_CFG3 = 0xdb;
sfr ANA_CFG4 = 0xdc;
sfr ANA_CFG5 = 0xdd;
sfr ANA_CFG6 = 0xde;
sfr ANA_CFG7 = 0xdf;

//sfr rf_address_dongle1 = 0xfe;
//sfr rf_address_dongle2 = 0xff;

#define CBYTE ((unsigned char volatile code  *) 0)
#define ID0 CBYTE[0x1ff3]
#define ID1 CBYTE[0x1ff3+1]
#define ID2 CBYTE[0x1ff3+2]
#define ID3 CBYTE[0x1ff3+3]
#define ID4 CBYTE[0x1ff3+4]   
#endif


