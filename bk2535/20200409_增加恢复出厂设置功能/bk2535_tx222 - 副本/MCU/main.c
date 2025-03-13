#include "includes.h"


// wifi 信道中心频率
// 1 -- 2412Mhz
// 2 -- 2417Mhz
// 3 -- 2422Mhz
// 4 -- 2427Mhz
// 5 -- 2432Mhz
// 6 -- 2437Mhz
// 7 -- 2442Mhz
// 8 -- 2447Mhz
// 9 -- 2452Mhz
// 10 -- 2457Mhz
// 11 -- 2462Mhz
// 12 -- 2467Mhz
// 13 -- 2472Mhz

#define RX_CHANNEL	52	  //9信道
#define TX_CHANNEL	27	  //4信道


UINT8 system_mode;

void system_mode_normal(void);

//void system_mode_pair(void);

//void system_mode_sleep(void);

void system_mode_test(void);
/*
#define pwm0_on()  PWM0C2 = 0X80
#define pwm0_off() PWM0C2 &= ~0X80
#define pwm1_on()  PWM1C2 = 0X80
#define pwm1_off() PWM1C2 &= ~0X80

#define pwm0_set_prescal(prescal) PWM0C2 = (PWM0C2 & ~0x70) | ((prescal & 7) << 4)
#define pwm1_set_prescal(prescal) PWM1C2 = (PWM1C2 & ~0x70) | ((prescal & 7) << 4)

#define pwm0_set_period(period) PWM0C0 = period & 0xff, PWM0C1 = (PWM0C1 & ~0x3)|((period >> 8) & 3)
#define pwm1_set_period(period) PWM1C0 = period & 0xff, PWM1C1 = (PWM1C1 & ~0x3)|((period >> 8) & 3)

#define pwm0_set_duty(duty) PWM0C1 = (PWM0C1 & 0x3) | (duty << 2), PWM0C2 = (PWM0C2 & 0xf0) | ((duty >> 6) & 0xf)
#define pwm1_set_duty(duty) PWM1C1 = (PWM1C1 & 0x3) | (duty << 2), PWM1C2 = (PWM1C2 & 0xf0) | ((duty >> 6) & 0xf)

#define pwm0_get_period() (((unsigned short)(PWM0C1 & 3) << 8) | PWM0C0)
#define pwm1_get_period() (((unsigned short)(PWM1C1 & 3) << 8) | PWM1C0)

#define pwm0_get_duty() (PWM0C1 >> 2) | ((PWM0C2 & 0xf) << 6)
#define pwm1_get_duty() (PWM1C1 >> 2) | ((PWM1C2 & 0xf) << 6)
*/


/*---------------------------------------------------------------------------------------------------------*/
void check_battery_voltage1(void);
void check_battery_voltage2(void);
void process_led(void);
void process_green_mode(void);
void init_ram(void);
void init_all_peripheral(void);

/*---------------------------------------------------------------------------------------------------------*/
// (x/2)/2.3*1024
//#define BAT_VOL(x)  	((unsigned short)((x)*1024/2/2.3))	  // x 检测电压。 2433
//#define BAT_VOL(x)  	((unsigned short)((x)*1024/2/2.4))	  // x 检测电压	  2535	   //分压电阻 100k/100k
#define BAT_VOL(x)  	((unsigned short)((x)*1024*68/(100+68)/2.4))	  // x 检测电压	  2535 //分压电阻 100k/68k

#define VOLTAGE_0_L	  	BAT_VOL(4.15)
#define VOLTAGE_0_H   	BAT_VOL(4.20)

#define VOLTAGE_1_L	  	BAT_VOL(3.80)
#define VOLTAGE_1_H   	BAT_VOL(3.85)

#define VOLTAGE_2_L	  	BAT_VOL(3.50)
#define VOLTAGE_2_H   	BAT_VOL(3.55)
                
#define VOLTAGE_3_L	  	BAT_VOL(3.30)
#define VOLTAGE_3_H   	BAT_VOL(3.35)
                
#define VOLTAGE_4_L	  	BAT_VOL(3.20)
#define VOLTAGE_4_H   	BAT_VOL(3.25)


#define BAT_CT 4
unsigned char  pair_mode=1; 
unsigned char xdata	ADEnCont;

unsigned char bat_level;
unsigned char bat_ct, bat_level_just;

unsigned char tran_inv,channel_count;

extern XDATA short GYRO_OFFSET[3];

void check_battery_voltage(void)
{
	if(lvd_adc_dat > VOLTAGE_0_L) {
		if (bat_level_just != 0) {
			bat_level_just = 0;
			bat_ct = 0;
		}
		bat_ct++;
		if (bat_ct >= BAT_CT) {
			bat_ct = BAT_CT;
			bat_level = 0;
		}
	}else if(lvd_adc_dat > VOLTAGE_1_L) {
		if (bat_level_just != 1) {
			bat_level_just = 1;
			bat_ct = 0;
		}
		bat_ct++;
		if (bat_ct >= BAT_CT) {
			bat_ct = BAT_CT;
			bat_level = 1;
		}

	}else if(lvd_adc_dat > VOLTAGE_2_L) {
		if (bat_level_just != 2) {
			bat_level_just = 2;
			bat_ct = 0;
		}
		bat_ct++;
		if (bat_ct >= BAT_CT) {
			bat_ct = BAT_CT;
			bat_level = 2;
		}
	}else if(lvd_adc_dat > VOLTAGE_3_L) {
		if (bat_level_just != 3) {
			bat_level_just = 3;
			bat_ct = 0;
		}
		bat_ct++;
		if (bat_ct >= BAT_CT) {
			bat_ct = BAT_CT;
			bat_level = 3;
		}
	}else if(lvd_adc_dat > VOLTAGE_4_L) {
		if (bat_level_just != 4) {
			bat_level_just = 4;
			bat_ct = 0;
		}
		bat_ct++;
		if (bat_ct >= BAT_CT) {
			bat_ct = BAT_CT;
			bat_level = 4;
		}
	}else {
		if (bat_level_just != 5) {
			bat_level_just = 5;
			bat_ct = 0;
		}
		bat_ct++;
		if (bat_ct >= BAT_CT) {
			bat_ct = BAT_CT;
			bat_level = 5;
		}
	}
}


/*
#define LIMITED_VOLTAGE_L	  	BAT_VOL(2.00)		//两节电池
#define LIMITED_VOLTAGE_H   	BAT_VOL(2.05)

#define LVD_CT 4

void check_battery_voltage1(void)
{
	if(low_power_flag) {
//		if(!LVD)
		if(lvd_adc_dat > LIMITED_VOLTAGE_H)
		{
			if(low_power_ct < LVD_CT)
				low_power_ct++;
		} else {
			low_power_ct = 0;
		}

		if(low_power_ct >= LVD_CT) { 
			low_power_flag = 0;				//退出低电模式
			low_power_ct = 0;
			DEG(("\nclr lowpower"));
		}
	} else {
//		if(LVD)
		if(lvd_adc_dat < LIMITED_VOLTAGE_L)
		{
			if(low_power_ct < LVD_CT)
				low_power_ct++;
		} else {
			low_power_ct = 0;
		}

		if(low_power_ct >= LVD_CT) {
			low_power_flag = 1;				//进入低电模式
			low_power_ct = 0;
			DEG(("\nset lowpower"));
		}
	}
}


//强制关机电压
#define LIMITED_VOLTAGE_L2	  	BAT_VOL(3.00)			//锂电
#define LIMITED_VOLTAGE_H2     	BAT_VOL(3.05)
#define LVD_CT2 4
void check_battery_voltage2(void)
{
	if(low_power_flag2) {
		if(lvd_adc_dat > LIMITED_VOLTAGE_H2) {
			if(low_power_ct2 < LVD_CT2)
				low_power_ct2++;
		} else {
			low_power_ct2 = 0;
		}

		if(low_power_ct2 >= LVD_CT2) { 
			low_power_flag2 = 0;				//退出低电关机
			low_power_ct2 = 0;
			DEG(("\nclr lowpower"));
		}
	} else {
		if(lvd_adc_dat < LIMITED_VOLTAGE_L2) {
			if(low_power_ct2 < LVD_CT2)
				low_power_ct2++;
		} else {
			low_power_ct2 = 0;
		}

		if(low_power_ct2 >= LVD_CT2) {
			low_power_flag2 = 1;				//进入低电关机模式
			low_power_ct2 = 0;
			DEG(("\nset lowpower"));
		}
	}
}
*/

//-----------------------------------------------------------
//void check_battery_voltage2(void)
//{
//	if(low_power_flag) {
//		if(!LVD)
////		if(system.adc.adc_dat > LIMITED_VOLTAGE)
//		{
//			if(low_power_ct < LVD_CT)
//				low_power_ct++;
//		} else {
//			low_power_ct = 0;
//		}

//		if(low_power_ct >= LVD_CT) {
//			low_power_flag = 0;				//退出低电模式
//			low_power_ct = 0;
//			DEG(("\nclr lowpower"));
//		}
//	} else {
//		if(LVD)
////		if(system.adc.adc_dat < LIMITED_VOLTAGE)
//		{
//			if(low_power_ct < LVD_CT)
//				low_power_ct++;
//		} else {
//			low_power_ct = 0;
//		}

//		if(low_power_ct >= LVD_CT) {
//			low_power_flag = 1;				//进入低电模式
//			low_power_ct = 0;
//			DEG(("\nset lowpower"));
//		}
//	}
//}

//#define CHRG_CT 4
//unsigned char bat_chrg_ct;
//bit bat_chrg_flag;
//void check_battery_chrg(void)
//{
//	if(bat_chrg_flag) {
//		if(CHRG_STA_PIN) {					//正常状态
//			if(bat_chrg_ct < CHRG_CT)
//				bat_chrg_ct++;
//		} else {
//			bat_chrg_ct = 0;
//		}
//
//		if(bat_chrg_ct >= CHRG_CT) {
//			bat_chrg_flag = 0;				//退出充电模式
//			bat_chrg_ct = 0;
//			DEG(("clr bat chrg\r\n"));
//		}
//	} else {
//		if(!CHRG_STA_PIN)	{				//充电状态
//			if(bat_chrg_ct < CHRG_CT)
//				bat_chrg_ct++;
//		} else {
//			bat_chrg_ct = 0;
//		}
//
//		if(bat_chrg_ct >= CHRG_CT) {
//			bat_chrg_flag = 1;				//进入充电模式
//			bat_chrg_ct = 0;
//			DEG(("into bat chrg\r\n"));
//		}
//	}
//}

//-----------------------------------------------------------

void do_led(unsigned char led)
{
	if (led & 1) {
		LED0_ON();
	}else {
		LED0_OFF();
	}
	if (led & 2) {
		LED1_ON();
	}else {
		LED1_OFF();
	}
}
#define led_off() do_led(0)



void led_proc(void)
{
	if (bat_level < 4) {		//电池电压正常, 3.3v
		if (fc_mode) {
			if(maincount & 0x20) {			//600ms
				do_led(1);
			}else {
				do_led(0);
			}
		}else {
			if (led_flash_ct) {
				led_flash_ct--;
				if(led_flash_ct & 0x10) {
					do_led(0);
				}else {
					do_led(1);
				}
			}else {
				do_led(1);
			}
		}
	}else {
		if (fc_mode) {
			if(maincount & 0x20) {			//600ms
				do_led(2);
			}else {
				do_led(0);
			}
		}else {
			if (led_flash_ct) {
				led_flash_ct--;
				if(led_flash_ct & 0x10) {
					do_led(0);
				}else {
					do_led(2);
				}
			}else {
				do_led(2);
			}
		}		
	}
}

//-----------------------------------------------------------
//void process_green_mode(void)
//{
//	sleep_ct++;
//	if (!sleep_mode) {	
//		if(fc_mode) {
//		}else if(search_mode) {
//			if(sleep_ct >= s2tick(30)) {		//30s
//				sleep_mode = 1;
//				DEG(("\nsleep_mode=1  "));
//			}
//		} else {		
//			if(sleep_ct >= s2tick(30)) {		//30s
//				if(!rf_diable_flag) {
//					//rf_diable_flag = 1;
//					if(sleep_ct == s2tick(30)) {		//30s
//						DEG(("\nStandby"));
//					}
//				}
//	
//				if(sleep_ct >= s2tick(5*60)) {	//5 min
//				//if(sleep_ct >= s2tick(2*60)) {	//2 min				
//				//if(sleep_ct >= s2tick(30)) {	//30s
//					sleep_mode = 1;
//					DEG(("\nsleep_mode=1  "));
//				}
//			}
//		}
//	}
//
//	if(sleep_mode) {
//		DISABLE_INTERRUPT ;
//		BK2401_CE=0;
//		FLUSH_TX;
//		FLUSH_RX;
//		rf_clrint();
//		AIF &= ~0x08;
//		AIE  &= ~0x08;
//		PowerDown_RF();
//		ET0 = 0;
//		ET1 = 0;
//		T2CON = 0x00;
//		TR0 = 0;
//		TR1 = 0;
//		DISABLE_INTERRUPT ;
//
//		//PALT &= ~0x80;
//		//PALT = 0;
//		SPC_FNC = 0 ;		//all clock off
//
//		DEG(("\r\nsleep "));
//
//		//P0 &= ~0x0c;			//eeprom output 0
//		P0IN_EN = B_1111_1111;	//1为输入，输出时为0
//		P0OUT_EN= B_1111_1111;	//1为输入，输出时为0
//		P0_PU    = B_1111_1111;
//		P0       = B_1111_1111;
//		_nop_();
//		P1IN_EN = B_1111_1110;
//		P0OUT_EN = B_1111_1110;
//		P1_PU    = B_1111_1110;
//		P1       = B_1111_1110;
//		_nop_();
//		P2IN_EN = B_0110_0001;
//		P0OUT_EN = B_0110_0001;
//		P2_PU    = B_0110_0001;
//		P2       = B_1110_0011;	
//		_nop_();
//		P3IN_EN = B_1111_1111;
//		P0OUT_EN = B_1111_1111;
//		P3_PU    = B_1111_1111;	
//		P3       = B_1111_1111;
//		_nop_();
//		P4IN_EN = B_1111_1111;
//		P0OUT_EN = B_1111_1111;
//		P4_PU    = B_1111_1111;
//		P4       = B_1111_1111;	
//		_nop_();
//
//		P0_WUEN = 0x00;		
//		P1_WUEN = 0x00;
//		P2_WUEN = 0x00;		//home  
//		P3_WUEN = 0x00;
//		P4_WUEN = 0x00;     //
//
//		////PCON = MCU_SLEEP;
//		//MCU_IDLE_RC32KHZ;
//		
//		//EXSLEEP |= 0x02;   //sleep2 en 
////		EXSLEEP |= 0x01;   //sleep1 en		
//		PCON2 |= 0x03;   // idle + sleep
//		_nop_();
//		_nop_();
//		_nop_();
//		_nop_();
//
//		DEG(("wakeup\r\n"));
//
//		init_all_peripheral();
//	}
//}

/*---------------------------------------------------------------------------------------------------------*/
void init_ram(void)
{
//	unsigned char i;
	fc_mode = 0;
	search_mode = 1;
	rf_diable_flag = 0;
	sleep_mode = 0;
//	search_mode = 0;

	low_power_flag = 0x00;
	low_power_flag2 = 0;
	//bat_chrg_flag = 0;
	fc_mode_ct = 0x00;
	sleep_ct = 0x00;
	rf_channel = 0x00;
	low_power_ct = 0x00;
	low_power_ct2 = 0;
	//bat_chrg_ct = 0;

	kb_led_sta = 0;

	memset(&system, 0, sizeof(system));

	rf_skip_ct = 0;
	rf_freq_id = 0;
}

/*---------------------------------------------------------------------------------------------------------*/
void poweron_init_ram(void)
{
	power_on_flag = 0xFF;   
}




/*---------------------------------------------------------------------------------------------------------*/
void init_all_peripheral(void)
{	
	mcu_clk_inital();  // 16M
//	delay_ms(20);			//20ms
	
//INIT IO
	mcu_io_inital();

	DISABLE_INTERRUPT ; 
	timer0_inital();
//	timer1_inital();
//	timer2_inital();

//	UartOpen();
	DEG(("bk2535 tx...\r\n"));



	//init data
	init_ram();

//	mcu_read_flash(0);	 //flash有bug，要先读一次。
	load_id_addr();

//  RX0_Address[0] = 0;
//	RX0_Address[1] = ID0;
//	RX0_Address[2] = ID1;
//	RX0_Address[3] = ID2;
//	RX0_Address[4] = PROJECT_ID;

  	RX0_Address[0] = 0;
	RX0_Address[1] = 1;
	RX0_Address[2] = 2;
	RX0_Address[3] = 3;
	RX0_Address[4] = 100;
	Read_sleeptime(); 		//上电要读一次FLASH		得到之前的休眠值  // 增加滚码记忆的读取
	// 先赋值给到对码通道，之后读取flash  如果有滚码，就会进入到正常工作模式

//	rf_channel = RX_CHANNEL;

//	DEG(("netrxid=%bx %bx %bx %bx %bx -- %bx\r\n",RX0_Address[0],RX0_Address[1],RX0_Address[2],RX0_Address[3],RX0_Address[4], rf_channel));
		
	//rf
	DEG(("init rf\r\n"));
	BK2433_RF_Initial();				//RF初始化
    CE_LOW();
	PowerUp_RF();
	rf_clrint();
	rf_set_rx_addr();
	 
	BK2401_RETR       = 0x10; 	//  自动重发4次，每次delay500us
	BK2401_SETUP      = RF_BAND|RF_5dbm|RF_LNA_GAIN_HIGH;	//250kbps, 5dbm
//	BK2401_RFCH = rf_channel;  // 通道为 40
//	BK2401_RFCH = T_JumpFreqTab[rf_freq_id]; 
//	SwitchToRxMode();
	SwitchToTxMode();
	CE_HIGH();


//INIT PWM
	/*
	PWM_ENABLE;
	PWMICTL = 0X00;
	PWM0C0 = 210;						// = 16 000 000÷38 000
	PWM0C1 = (70<<2)&0XFF; 			//50%
	PWM0C2 = (0<<7) +(1<<4) + (70>>6);	//PWM0 disable,prescale=2,
	
	PWM1C0 = 210;						// = 16 000 000÷38 000
	PWM1C1 = (70<<2)&0XFF; 			//50%
	PWM1C2 = (0<<7) +(1<<4) + (70>>6);	//PWM1 disable,prescale=2,	
	*/

//	INIT ADC
	//adc_inital();
	
	search_mode = 1;

	DEG(("init ok\r\n"));
#ifdef TIMER0_INT
	ET0 = 1;
#endif
//	ES0 = 1;
	EA =1;
//	TR1 = 1;					//enable motor proc	
}


void test_rf(void)
{
//	extern void set_tx_power(unsigned char gpa, unsigned char hq, unsigned char paldo, bit pcsel);
	DEG(("Set RF singewave\r\n"));
    BK2401_CE         = 0X00;
	//开发板实测
//	ANA_CTRL11 = 0xff;	  //-11dbm
//	ANA_CTRL11 = 0x9f;	  //-13dbm
//	ANA_CTRL11 = 0xa1;	  //-36dbm
//	ANA_CTRL11 = 0x80;	  //-36dbm
//	set_tx_power(0x1f, 3, 1, 3);
	SwitchToTxMode();
	//SwitchToRxMode();
	BK2401_CE=0;
	BK2401_RFCH = 44;
	BK2401_CE=1;
	//PowerUp_RF();
	RF_Set_Mode(0);
    BK2401_CE         = 0X01;

	while(1) {
		delay_ms(500);
//		P37 ^= 1;
	}
}




#define ID1_ADDR        0x10

void save_id_addr(void)
{
//	_24cxx_write_buf(ID1_ADDR, 3, buf);

	mcu_erase_flash(0);
//	mcu_write_flash(ID1_ADDR+0, RX0_Address[0]);
	mcu_write_flash(ID1_ADDR+1, RX0_Address[1]);
	mcu_write_flash(ID1_ADDR+2, RX0_Address[2]);
	mcu_write_flash(ID1_ADDR+3, RX0_Address[3]);
//	mcu_write_flash(ID1_ADDR+4, RX0_Address[4]);
}
void load_id_addr(void)
{
//	_24cxx_read_buf(ID1_ADDR, 3, buf);

	
//	RX0_Address[0] = mcu_read_flash(ID1_ADDR+0);
	RX0_Address[1] = mcu_read_flash(ID1_ADDR+1);
	RX0_Address[2] = mcu_read_flash(ID1_ADDR+2);
	RX0_Address[3] = mcu_read_flash(ID1_ADDR+3);
//	RX0_Address[4] = mcu_read_flash(ID1_ADDR+4);

	RX0_Address[0] = 0;	
	RX0_Address[4] = PROJECT_ID;
}





void power_down(void)
{
	EA = 0;
	printf("do power down\r\n"); 	
	PowerDown_RF();		   	//rf powerdown
	CE_LOW();
	led_off();
//	__power_down();	

	while(1);
}

//void Print_BK2535_RFReg(void)
//{
//    printf("Print BK2535 RF Reg Start :\n");
//    delay_ms(10);											
//    printf("BK2401_CONFIG : 0x%bx\n",BK2401_CONFIG);	     delay_ms(10);
//    printf("BK2401_ENAA : 0x%bx\n",BK2401_ENAA);       delay_ms(10);
//    printf("BK2401_ENRX : 0x%bx\n",BK2401_ENRX);      delay_ms(10);
//    printf("BK2401_AW : 0x%bx\n",BK2401_AW);	       delay_ms(10);
//    printf("BK2401_RETR : 0x%bx\n",BK2401_RETR);	      delay_ms(10);
//    printf("BK2401_RFCH : 0x%bx\n",BK2401_RFCH);	    delay_ms(10);
//    printf("BK2401_SETUP : 0x%bx\n",BK2401_SETUP);	     delay_ms(10);
//
//    printf("BK2401_R0_ADDR : 0x0%bx",BK2401_R0_ADDR_0);	       delay_ms(10);
//    printf("0%bx",BK2401_R0_ADDR_1);					    delay_ms(10);
//    printf("0%bx",BK2401_R0_ADDR_2);					     delay_ms(10);
//    printf("0%bx",BK2401_R0_ADDR_3);					     delay_ms(10);
//    printf("%bx\n",BK2401_R0_ADDR_4);					     delay_ms(10);
//    
//    printf("BK2401_R1_ADDR: 0x0%bx",BK2401_R1_ADDR_0);	       delay_ms(10);
//    printf("0%bx",BK2401_R1_ADDR_1);					       delay_ms(10);
//    printf("0%bx",BK2401_R1_ADDR_2);					      delay_ms(10);
//    printf("0%bx",BK2401_R1_ADDR_3);						    delay_ms(10);
//    printf("%bx\n",BK2401_R1_ADDR_4);						     delay_ms(10);
//    
//    printf("BK2401_R2_ADDR: 0x%bx\n",BK2401_R2_ADDR);		       delay_ms(10);
//    printf("BK2401_R3_ADDR : 0x%bx\n",BK2401_R3_ADDR);		      delay_ms(10);
//    printf("BK2401_R4_ADDR : 0x%bx\n",BK2401_R4_ADDR);		     delay_ms(10);
//    printf("BK2401_R5_ADDR : 0x%bx\n",BK2401_R5_ADDR);		      delay_ms(10);
//
//    printf("BK2401_TX_ADDR : 0x0%bx",BK2401_TX_ADDR_0);    delay_ms(10);
//    printf("0%bx",BK2401_TX_ADDR_1);						    delay_ms(10);
//    printf("0%bx",BK2401_TX_ADDR_2);					       delay_ms(10);
//    printf("0%bx",BK2401_TX_ADDR_3);					      delay_ms(10);
//    printf("%bx\n",BK2401_TX_ADDR_4);					      delay_ms(10);
//    
//    printf("BK2401_R0_PW : 0x%bx\n",BK2401_R0_PW);		       delay_ms(10);
//    printf("BK2401_R1_PW : 0x%bx\n",BK2401_R1_PW);		       delay_ms(10);
//    printf("BK2401_R2_PW : 0x%bx\n",BK2401_R2_PW);		      delay_ms(10);
//    printf("BK2401_R3_PW : 0x%bx\n",BK2401_R3_PW);		       delay_ms(10);
//    printf("BK2401_R4_PW : 0x%bx\n",BK2401_R4_PW);		       delay_ms(10);
//    printf("BK2401_R5_PW : 0x%bx\n",BK2401_R5_PW);		      delay_ms(10);
//    
//    printf("BK2401_DYNPD : 0x%bx\n",BK2401_DYNPD);		      delay_ms(10);
//    printf("BK2401_FEATURE : 0x0%bx\n",BK2401_FEATURE);		    delay_ms(10);
//    
//    printf("BK2401_CFG_0C_0 : 0x0%bx",BK2401_CFG_0C_0);	       delay_ms(10);
//    printf("%bx",BK2401_CFG_0C_1);						      delay_ms(10);
//    printf("%bx",BK2401_CFG_0C_2);						      delay_ms(10);
//    printf("0%bx\n",BK2401_CFG_0C_3);					       delay_ms(10);
//    
//    printf("BK2401_CFG_0D_0 : 0x%bx",BK2401_CFG_0D_0);	      delay_ms(10);
//    printf("%bx",BK2401_CFG_0D_1);						       delay_ms(10);
//    printf("%bx",BK2401_CFG_0D_2);						       delay_ms(10);
//    printf("0%bx\n",BK2401_CFG_0D_3);					       delay_ms(10);
//    
//    printf("BK2401_RAMP_TAB_0 : 0x%bx ",BK2401_RAMP_TAB_0);	      delay_ms(10);
//    printf("%bx ",BK2401_RAMP_TAB_1);						      delay_ms(10);
//    printf("0%bx ",BK2401_RAMP_TAB_2);						     delay_ms(10);
//    printf("0%bx ",BK2401_RAMP_TAB_3);						     delay_ms(10);
//    printf("%bx ",BK2401_RAMP_TAB_4);						     delay_ms(10);
//    printf("%bx ",BK2401_RAMP_TAB_5);						     delay_ms(10);
//    printf("%bx ",BK2401_RAMP_TAB_6);						      delay_ms(10);
//    printf("%bx ",BK2401_RAMP_TAB_7);						     delay_ms(10);
//    printf("%bx ",BK2401_RAMP_TAB_8);						    delay_ms(10);
//    printf("%bx ",BK2401_RAMP_TAB_9);						    delay_ms(10);
//    printf("%bx\n",BK2401_RAMP_TAB_A);					      delay_ms(10);
//
//    printf("Print BK2535 RF Reg End;\n");					    delay_ms(10);
//    
//}

uint8	FIFO_data[32];
uint8	send_len;

void main(void)
{
	unsigned char R_rr;	
//	BYTE	status;
	poweron_init_ram();  //  不知道干嘛用 
	init_all_peripheral();  //所有资源的初始化 
//	printf("power on ok\r\n");
	power_on_flag = 0x00;
//	test_rf();
	//--------------------------------------
	adc_set_ref(ADC_REF_VOL, 0);
	{
		extern void application_adc_cel(unsigned char pin);
		application_adc_cel(1);  // 读取 p35  的ad值 

	}
	//ADCFunction();
	//--------------------------------------	
	printf("system init ok\r\n");
	up_power000=1;
	delay_us(20000);
	delay_us(20000);	
//	BMI160_Init();
	delay_us(20000);
	delay_us(20000);	
	for(R_rr=0;R_rr<3;R_rr++)
	    GYRO_OFFSET[R_rr]=0;
	TxCmdDataFlag = 0;
	ET0 = 1;
//	P37  ^=1;
	//system_mode = system_pair_mode;			//上电对码模式	直接到测试模式，是可以的
	//system_mode = system_normal_mode;
	#if 1
	while(1)
	{
		switch(system_mode)
		{
			case system_normal_mode:
				system_mode_normal();
				break;
			case system_pair_mode:
				system_mode_pair();
				break;
			case system_sleep_mode:
				system_mode_sleep();
				break;
			case system_test_mode:
				system_mode_test();
				break;
		}
	}
	#endif
}
void system_mode_normal(void)
{
	while(system_mode == system_normal_mode)
	//if(system_mode == system_normal_mode)
	{
		//Check_1ms_fun();			//这是状态的判断和各种显示
		//Normal_to_sleep();			//这里是做静止判断休眠的
		if(ADEnCont >= 200)		  //400MS调用一次AD函数
		{
			ADEnCont = 0;
			ADC_EN = 1 ;
			ADCFunction();		//OK
			ADC_EN = 0;			// 这里的en脚，休眠附带了2MA电流
		}
		if(SendData >= 8)				  
		{					
			SendData = 0;				  //8MS调用一次
			if(LowpowerFlag == 0)
				ADEnCont++;
			//if(LowpowerFlag == 0)
			{
				if(ChangeRXFlag)
				{
					ChangeRXFlag = 0;
					SwitchToTxMode();
				}
		//			process_rf_communication();							 //这里只是判断状态，发射数据不放这里 4MS判断一次
				if(LowpowerFlag == 0)	   //改成只有在低电量的时候才会停止通信，充电的时候也能通信
				{
					key_proc();			//OK
					mouse_proc();		//OK
		    		gyroscope_MOSE(); 	//电流就是调用一次陀螺仪后就下不来了		   //这个算法计算结束，大概需要2mS以上
				
				//if(system_mode == system_sleep_mode)
				//	break;
				/********************************************************/
					CE_HIGH();
					RF_Data_Send();				//这是发数据函数
					CE_LOW();
				}
				 pair_mode=0;
				if(pair_mode)
				{  
				    pair_mode=0;
					RF_id_set();	
				}
				if(tran_inv >= 1)				 //6*8ms 跳频
				{
					tran_inv=0;
					//P37  ^=1;						
					rf_set_channel(JumpFreq_Buf[channel_count++]);
					if(channel_count>3)
						channel_count=0;
				}
			}				
		}
	}
} //while(1)

