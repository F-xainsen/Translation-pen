#ifndef _MAIN_H_
#define _MAIN_H_

//	mcu definition
//
#define	SYS_CLOCK			16000000ul
//#define	SYS_CLOCK		8000000ul

#if SYS_CLOCK==16000000
#define	CKCON_CLK_CFG			CKCON_CLK_16MHZ
#elif SYS_CLOCK==8000000
#define	CKCON_CLK_CFG			CKCON_CLK_8MHZ
#elif SYS_CLOCK==4000000
#define	CKCON_CLK_CFG			CKCON_CLK_4MHZ
#elif SYS_CLOCK==2000000
#define	CKCON_CLK_CFG			CKCON_CLK_2MHZ
#endif


#define us2tcnt_16m_div12(us)  (65536 - (us) * 16UL/12)	//12分频
#define us2tcnt2_16m_div12(us) ((us) * 16UL/12)			//12分频

#define us2tcnt_16m_div4(us)   (65536 - (us) * 16UL/4)	//4分频
#define us2tcnt2_16m_div4(us)  ((us) * 16UL/4)			//4分频

#define us2tcnt_8m_div12(us)   (65536 - (us) * 8UL/12)	//12分频
#define us2tcnt2_8m_div12(us)  ((us) * 8UL/12)			//12分频

#define us2tcnt_8m_div4(us)    (65536 - (us) * 8UL/4)	//4分频
#define us2tcnt2_8m_div4(us)   ((us) * 8UL/4)			//4分频

#define us2tcnt_4m_div12(us)   (65536 - (us) * 4UL/12)	//12分频
#define us2tcnt2_4m_div12(us)  ((us) * 4UL/12)			//12分频

#define us2tcnt_4m_div4(us)    (65536 - (us) * 4UL/4)	//4分频
#define us2tcnt2_4m_div4(us)   ((us) * 4UL/4)			//4分频

#define us2tcnt_2m_div12(us)   (65536 - (us) * 16UL/12)	//12分频
#define us2tcnt2_2m_div12(us)  ((us) * 16UL/12)			//12分频

#define us2tcnt_2m_div4(us)    (65536 - (us) * 2UL/4)	//4分频
#define us2tcnt2_2m_div4(us)   ((us) * 2UL/4)			//4分频


#define us2tcnt_32k_div12(us)  (65536 - (us) * 32768UL/12000000)	//12分频
#define us2tcnt2_32k_div12(us) ((us) * 32768UL/12000000)			//12分频

#define us2tcnt_32k_div4(us)   (65536 - (us) * 32768UL/4000000)		//4分频
#define us2tcnt2_32k_div4(us)  ((us) * 32768UL/4000000)				//4分频


//#define TIMER_CLK_DIV   4
#define TIMER_CLK_DIV   12

#if TIMER_CLK_DIV == 12
#if SYS_CLOCK==16000000ul
#define us2tcnt(us)             us2tcnt_16m_div12(us)
#define us2tcnt2(us) 			us2tcnt2_16m_div12(us)
#elif SYS_CLOCK==8000000ul
#define us2tcnt(us)             us2tcnt_8m_div12(us)
#define us2tcnt2(us) 			us2tcnt2_8m_div12(us)
#elif SYS_CLOCK==4000000ul
#define us2tcnt(us)             us2tcnt_4m_div12(us)
#define us2tcnt2(us) 			us2tcnt2_4m_div12(us)
#elif SYS_CLOCK==2000000ul
#define us2tcnt(us)             us2tcnt_2m_div12(us)
#define us2tcnt2(us) 			us2tcnt2_2m_div12(us)
#endif
#define us2tcnt_32k(us)   		us2tcnt_32k_div12(us)
#define us2tcnt2_32k(us)   		us2tcnt2_32k_div12(us)
#elif TIMER_CLK_DIV == 4
#if SYS_CLOCK==16000000ul
#define us2tcnt(us)             us2tcnt_16m_div4(us)
#define us2tcnt2(us) 			us2tcnt2_16m_div4(us)
#elif SYS_CLOCK==8000000ul
#define us2tcnt(us)             us2tcnt_8m_div4(us)
#define us2tcnt2(us) 			us2tcnt2_8m_div4(us)
#elif SYS_CLOCK==4000000ul
#define us2tcnt(us)             us2tcnt_4m_div4(us)
#define us2tcnt2(us) 			us2tcnt2_4m_div4(us)
#elif SYS_CLOCK==2000000ul
#define us2tcnt(us)             us2tcnt_2m_div4(us)
#define us2tcnt2(us) 			us2tcnt2_2m_div4(us)
#endif
#define us2tcnt_32k(us)   		us2tcnt_32k_div4(us)
#define us2tcnt2_32k(us)   		us2tcnt2_32k_div4(us)
#endif


#if SYS_CLOCK==16000000ul
#define delay_us(us)	__delay_us(us)
#elif SYS_CLOCK==8000000ul
#define delay_us(us)	__delay_us((us)/2)
#elif SYS_CLOCK==4000000ul
#define delay_us(us)	__delay_us((us)/4)
#elif SYS_CLOCK==2000000ul
#define delay_us(us)	__delay_us((us)/8)
#else
#define delay_us(us)	__delay_us(us)
#endif

// 主循环周期
#define TICK_PERIOD 	4000					//us
//#define TICK_PERIOD 	8000					//us
//#define TICK_PERIOD 	10000					//us
//#define TICK_PERIOD 	20000					//us
#define ms2tick(ms) 	(((ms)*1ul)/(TICK_PERIOD/1000))
#define s2tick(s) 		(((s)*1000ul)/(TICK_PERIOD/1000))
#define TICK_PERIOD1 	1000					//us

//-------------------------------------------------
//io inital
//
#define	P0_IOSEL_CFG	B_1111_1100
#define	P0_OPDR_CFG		B_0000_0000
#define	P0_PU_CFG		B_1111_1100
#define	P0_PD_CFG		B_0000_0000
#define	P0_WUKEUP		B_0000_0000
#define	P0_DAT			B_1111_1100

#define	P1_IOSEL_CFG	B_0000_1011	  // 1 = in  0= out 
#define	P1_OPDR_CFG		B_0000_0000
#define	P1_PU_CFG		B_0000_0010
#define	P1_PD_CFG		B_0000_0000
#define	P1_WUKEUP		B_0000_0000	  
#define	P1_DAT			B_1111_1111

#define	P2_IOSEL_CFG	B_1000_0000
#define	P2_OPDR_CFG		B_0000_0000
#define	P2_PU_CFG		B_0000_0000	 //B_0001_1000  有外部上啦，关闭内部上啦
#define	P2_PD_CFG		B_0000_0000
#define	P2_WUKEUP		B_0000_0000
#define	P2_DAT			B_1111_1110

#define	P3_IOSEL_CFG	B_0000_0010
#define	P3_OPDR_CFG		B_0000_0000
#define	P3_PU_CFG		B_0000_0000
#define	P3_PD_CFG		B_0000_0000
#define	P3_WUKEUP		B_0000_0000
#define	P3_DAT			B_1111_0010


#define	P4_IOSEL_CFG	B_0000_0000
#define	P4_OPDR_CFG		B_0000_0000
#define	P4_PU_CFG		B_0000_0000
#define	P4_PD_CFG		B_0000_0000
#define	P4_WUKEUP		B_0000_0000
#define	P4_DAT			B_1111_1111

//-------------------------------------------------
//sbit POWER_CHECK		= P0^0;//O
sbit IO_EN_LEDB			= P0^0;//O
//sbit CHARGE_DET			= P0^1;//O
sbit IO_EN_LEDG			= P0^1;//O
//sbit I2C_SDA			= P0^2;//O
//sbit I2C_CLK			= P0^3;//O
sbit MOSI				= P0^4;//I	 //PULL HIGH
sbit MISO				= P0^5;//I	 //PULL HIGH
sbit SCK				= P0^6;//I	 //PULL HIGH
sbit CS					= P0^7;//I	 //PULL HIGH

//-------------------------------------------------
//sbit MCU_INT			= P1^0;//I	 //PULL HIGH
sbit IO_CIN				= P1^0;//I	 //PULL HIGH
//sbit POWER_SWITCH		= P1^1;//I	 //PULL HIGH 	//这个是不用了的，暂时未屏蔽
sbit IO_BUTTON			= P1^1;//I	 //PULL HIGH
//sbit button				= P1^2;//I	 //PULL HIGH  扳机按键
sbit IO_CLRAE			= P1^3;//I	 //PULL HIGH
//sbit ROW4				= P1^4;//I	 //PULL HIGH
//sbit ROW5				= P1^5;//I	 //PULL HIGH
//sbit ROW6				= P1^6;//I	 //PULL HIGH
//sbit ROW7				= P1^7;//I	 //PULL HIGH

//-------------------------------------------------
//sbit ADC_EN				= P2^0;//O
sbit MCU_DAT2			= P2^1;//O	 ///////debug
//sbit LED0				= P2^2;//O
sbit MCU_DAT0			= P2^3;//O
sbit MCU_DAT1			= P2^4;//O
//sbit LED3				= P2^5;//O
//sbit IR_SEND			= P2^6;//O
sbit IO_CFULL			= P2^7;//I	 //PULL HIGH	

//-------------------------------------------------
//sbit ADC_BAT			= P3^0;//O
sbit IO_EN_LEDR			= P3^0;//O
//sbit LED_ZX				= P3^1;//O	 //准心灯
sbit ADC_EN				= P3^2;//I	 //
sbit IO_EN_LEDZX		= P3^3;//I	 //准心灯
//sbit LED4				= P3^4;//O
//sbit VOUT_EN			= P3^5;//O
//sbit LVDEN			= P3^6;//O
//sbit LVD				= P3^7;//I

//-------------------------------------------------
//sbit Reserved			= P4^0;//O
//sbit RANK11			= P4^1;//I	 //PULL HIGH

//sbit Reserved			= P4^4;//O
//sbit Reserved			= P4^5;//O
//sbit Reserved			= P4^6;//O
//sbit Reserved			= P4^7;//O

//-------------------------------------------------
#define RF_TEST_PIN0 	P47//PORT2PIN(P0, 0)
#define RF_TEST_PIN1 	P46//PORT2PIN(P0, 3)



#define POWER_KEY_PIN	P00
#define POWER_KEY		(!POWER_KEY_PIN)

//#define CHRG_STA_PIN	CHARGE_DET

//#define __power_on()	POWER_SWITCH = 1
//#define __power_down()	POWER_SWITCH = 0


//////////////////////////////////////////////////////
#define LED_PIN_0 PORT2PIN(P3, 7)
#define LED_PIN_1 PORT2PIN(P3, 6)


#define LED0_ON()  LED_PIN_0=0 ;P36=0 		//debug
#define LED0_OFF() LED_PIN_0=1 ;P36=1
#define LED0_XOR() LED_PIN_0^=1

#define LED1_ON()  LED_PIN_1=0 ;P37=0
#define LED1_OFF() LED_PIN_1=1 ;P37=1
#define LED1_XOR() LED_PIN_1^=1

#define system_normal_mode 	0
#define system_pair_mode 	1
#define system_test_mode 	2
#define system_sleep_mode 	3
extern UINT8 system_mode;
extern BYTE JumpFreq_Buf[4];
extern void load_id_addr(void);
extern void save_id_addr(void);


#endif

