#ifndef _BK2x33_MCU_REG_H_
#define _BK2x33_MCU_REG_H_

#define SPC_FNC   CLK_EN_CFG   // 外设的时钟使能
#define DPL0      DPL
#define DPH0      DPH
#define SCON      SCON0
#define SBUF      SBUF0

/*  UINT8 Registers  */
sfr P0     = 0x80;
sfr SP     = 0x81;
sfr DPL    = 0x82;
sfr DPH    = 0x83;
sfr DPL1   = 0x84;
sfr DPH1   = 0x85;
sfr DPS    = 0x86;

sfr PCON   = 0x87;
sfr TCON   = 0x88;
sfr TMOD   = 0x89;
sfr TL0    = 0x8A;
sfr TL1    = 0x8B;
sfr TH0    = 0x8C;
sfr TH1    = 0x8D;
sfr CKCON  = 0x8E;
/*
CLK_EN_CFG    7   6   5   4   3   2   1   0
0x8F              adc_en   timer_en   uart_en   pwm_en   spi_en    i2c_en    des_en   mdu_en
*/
sfr CLK_EN_CFG = 0x8f;

sfr P1     = 0x90;

sfr EXIF   = 0x91;
sfr MPAGE  = 0x92;
sfr EICON  = 0x93;
// ADC
sfr ADCDL = 0x94;
sfr ADCDH = 0x95;
sfr ADCTL = 0x96;
// watch dog
sfr WDCTL  = 0x97;
sfr SCON0  = 0x98;
sfr SBUF0  = 0x99;
sfr P0_PU  = 0x9a;
sfr P1_PU  = 0x9b;
sfr P2_PU  = 0x9c;
sfr P3_PU  = 0x9d;
sfr P4_PU  = 0x9e;
sfr P2     = 0xA0;

// PWM
sfr PWM0C0 = 0xa1;
sfr PWM0C1 = 0xa2;
sfr PWM0C2 = 0xa3;
sfr PWM1C0 = 0xa4;
sfr PWM1C1 = 0xa5;
sfr PWM1C2 = 0xa6;
sfr PWMICTL= 0xa7;


sfr IE     = 0xA8;
sfr IP     = 0xA9;
sfr P0_IOSEL=0xaa;
sfr P1_IOSEL=0xab;
sfr P2_IOSEL=0xac;
sfr P3_IOSEL=0xad;
sfr P4_IOSEL=0xae;

sfr P3     = 0xB0;
sfr P0_PD = 0xb1;
sfr P1_PD = 0xb2;
sfr P2_PD = 0xb3;
sfr P3_PD = 0xb4;
sfr P4_PD = 0xb5;
// SMB
sfr SMB_DAT = 0xb6;
sfr SMB_CF  = 0xb7;
sfr SMB_CN  = 0xb8;
sfr SMB_TMCTL = 0xb9;
sfr P0_OPDR = 0xba;
sfr P1_OPDR = 0xbb;
sfr P2_OPDR = 0xbc;
sfr P3_OPDR = 0xbd;
sfr P4_OPDR = 0xbe;

sfr P4      = 0xC0;
sfr DS_WUEN = 0xc3;
sfr P0_WUEN = 0xc4;
sfr P1_WUEN = 0xc5;
sfr P2_WUEN = 0xc6;
sfr P3_WUEN = 0xc7;
sfr T2CON   = 0xC8;
sfr P4_WUEN = 0xc9;

sfr RCAP2L = 0xCA;
sfr RCAP2H = 0xCB;
sfr TL2    = 0xCC;
sfr TH2    = 0xCD;

sfr PSW    = 0xD0;
// TDES
sfr DES_CTL  = 0xd1;
sfr DES_INT  = 0xd2;
sfr DES_KEY1 = 0xd3;
sfr DES_KEY2 = 0xd4;
sfr DES_KEY3 = 0xd5;
sfr DES_IN   = 0xd6;
sfr DES_OUT  = 0xd7;

#if  MCU_TYPE == BK2533
// ana_cfg  sfr
sfr ANA_CFG0 = 0xd8;
sfr ANA_CFG1 = 0xd9;
sfr ANA_CFG2 = 0xda;
sfr ANA_CFG3 = 0xdb;
sfr ANA_CFG4 = 0xdc;
sfr ANA_CFG5 = 0xdd;
sfr ANA_CFG6 = 0xde;
sfr ANA_CFG7 = 0xdf;
#endif

sfr ACC    = 0xE0;
// port second function enable reg
#if  MCU_TYPE == BK2533
// NVR Flash
sfr  FLASH_KEY = 0xe6;
sfr  FLASH_CTL = 0xed;
sfr  FLASH_ADR = 0xee;
sfr  FLASH_DAT = 0xef;

sfr  nvr_wp0     = 0xb6;
sfr  nvr_wp1     = 0xb7;
sfr  nvr_wf      = 0xb8;
sfr  nvr_wf_part = 0xb9;
#endif

sfr P_EXP  = 0xe6;
sfr PALT   = 0xe7;
sfr EIE    = 0xE8;
sfr EIP    = 0xE9;
// EXINT_MOD
sfr EXINT_MOD = 0xea;

// random no generate
sfr RNG_DAT = 0xeb;
sfr RNG_CTL = 0xec;

#if  MCU_TYPE == BK2433
sfr MTP_CTL = 0xed;
sfr MTP_ADR = 0xee;
sfr MTP_DAT = 0xef;
#endif

sfr B      = 0xF0;
//  MDU
sfr MD0 = 0xf1;
sfr MD1 = 0xf2;
sfr MD2 = 0xf3;
sfr MD3 = 0xf4;
sfr MD4 = 0xf5;
sfr MD5 = 0xf6;
sfr MDCTL = 0xf7;

//sfr MDCON0 = 0xee;
//sfr MDCON1 = 0xef;

// SPI0
sfr SPI0_CN  = 0xf8;
sfr SPI0_CFG = 0xf9;
sfr SPI0_CKR = 0xfa;
sfr SPI0_DAT = 0xfb;





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

/*  EIE  */
sbit EX9   = EIE^7;
sbit EX8   = EIE^6;
sbit EX7   = EIE^5;
sbit EX6   = EIE^4;
sbit EUSB  = EIE^4;
sbit EX5   = EIE^3;
sbit EX4   = EIE^2;
sbit EX3   = EIE^1;
sbit EX2   = EIE^0;


// SMB_CN
sbit MASTER   = SMB_CN^7;
sbit TXMODE   = SMB_CN^6;
sbit STA      = SMB_CN^5;
sbit STO      = SMB_CN^4;
sbit ACKRQ    = SMB_CN^3;
sbit ARBLOST  = SMB_CN^2;
sbit ACK      = SMB_CN^1;
sbit SI       = SMB_CN^0;


sbit P07  = P0^7;
sbit P06  = P0^6;
sbit P05  = P0^5;
sbit P04  = P0^4;
sbit P03  = P0^3;
sbit P02  = P0^2;
sbit P01  = P0^1;
sbit P00  = P0^0;

sbit P17  = P1^7;
sbit P16  = P1^6;
sbit P15  = P1^5;
sbit P14  = P1^4;
sbit P13  = P1^3;
sbit P12  = P1^2;
sbit P11  = P1^1;
sbit P10  = P1^0;

sbit P27  = P2^7;
sbit P26  = P2^6;
sbit P25  = P2^5;
sbit P24  = P2^4;
sbit P23  = P2^3;
sbit P22  = P2^2;
sbit P21  = P2^1;
sbit P20  = P2^0;

sbit P37  = P3^7;
sbit P36  = P3^6;
sbit P35  = P3^5;
sbit P34  = P3^4;
sbit P33  = P3^3;
sbit P32  = P3^2;
sbit P31  = P3^1;
sbit P30  = P3^0;

sbit P47  = P4^7;
sbit P46  = P4^6;
sbit P45  = P4^5;
sbit P44  = P4^4;
sbit P43  = P4^3;
sbit P42  = P4^2;
sbit P41  = P4^1;
sbit P40  = P4^0;

#endif
