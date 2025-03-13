/***********************************************************
FileName	:
Date		: 2011/03/01
Description	:
Version		: v0.1
Function List:
----
History:
<author> <time> <version > <desc>
***********************************************************/

#include "includes.h"
UINT8 XDATA Time_100MS_cont = 0;

bit Time_1ms_flag = 0;		// 1 = 10MS计时到了


void timer0_inital(void)
{
	SPC_FNC |= 0x40 ;
	ET0 = 0;
	TR0 = 0;
	TF0 = 0;
	TMOD &= 0xf0;
	TMOD |= 0x01;						//using Timer0 as tick base
	TL0 = (us2tcnt(TICK_PERIOD) & 0x00FF);
	TH0 = (us2tcnt(TICK_PERIOD) >> 8);
//	ET0 = 1;							//enable Timer0 int
	TR0 = 1;
}

//-------------------------------------------------------
void timer1_inital(void)
{
	SPC_FNC |= 0x40 ;
	ET1 = 0;
	TR1 = 0;
	TF1 = 0;
	TMOD &= 0x0f;
	TMOD |= 0x10;						//using Timer0 as tick base
	TL1 = (us2tcnt(500) & 0x00FF);
	TH1 = (us2tcnt(500) >> 8);
//	ET1 = 1;							//enable Timer1 int
	TR1 = 1;
}

//-------------------------------------------------------
void timer2_inital(void)
{
	SPC_FNC |= 0x40 ;
//	ET2 = 0;
	TR2 = 0;
	TF2 = 0;
	T2CON  = 0x00;
	RCAP2H = (us2tcnt(5) >> 8);
	RCAP2L = (us2tcnt(5) & 0x00FF);	
	TL2 = (us2tcnt(2000) & 0x00FF);
	TH2 = (us2tcnt(2000) >> 8);
//	ET2 = 1;							//enable Timer2 int
//	TR2 = 1;
}




void timer0_isr (void) interrupt 1 //using 2       1ms  进一次中断
{
	TR0 = 0;
	TF0 = 0;
	TL0 = (us2tcnt(TICK_PERIOD1) & 0x00FF);
	TH0 = (us2tcnt(TICK_PERIOD1) >> 8);
	TR0 = 1;
	timer0_flag = 1;
	if(system_mode == system_pair_mode)
	{
		Pair_to_sleep++;
		if(Pair_to_sleep > 200)
			Pair_to_sleep = 200;
	}

	Time_1ms_flag = 1;
	Check_1ms_fun();
	normal_to_sleep_int = 1;
	Normal_to_sleep();			//这里是做静止判断休眠的
	SendData++;			//发射数据的时间计数器
	if(SendData> 200)
		SendData = 200;
	Time_100MS_cont++;
	if(Time_100MS_cont >= 100)
	{
		Time_100MS_cont = 0;		//100ms 的时基
//		if(system_mode == system_normal_mode)
//			normal_to_sleep_int = 1;
		if(system_mode == system_pair_mode)
			Pair_to_sleep_int  = 1;
	}
}

void timer1_isr(void) interrupt 3 using 2
{
	TR1 = 0;	//STOP
	TF1 = 0;	//CLEAR FLAG
	TL1 = (us2tcnt(500) & 0x00FF);
	TH1 = (us2tcnt(500) >> 8);
	TR1 = 1;	//RUN
}

void timer2_isr(void) interrupt 5 using 3
{
	TF2 = 0;
	ET2 = 0;	//diable Interrupt
	TR2 = 0;		
}

/*

（2+ div） * ( 1+count) / 32   ms

*/
//8ms div = 2, count = 63
//rtc 32khz
//void rtc_set(unsigned char div, unsigned short count)
//{
//	RTC_TIMER_CTL = div & 0x03;
//	RTC_COUNT_H = count/0x100;
//	RTC_COUNT_L = count%0x100;     	//rtc timer num
//	RTC_TIMER_CTL |= 0x04;       	//open rtc
//	EX6 = 1;
//	EA = 1;
//}
//#define us2rct(us) ((us) * 16UL/1000)	// 32khz/2
#define us2rct(us) ((us) * 8UL/1000)	// 32khz/2
void rtc_init(void)
{
	RTC_TIMER_CTL = 0;
	RTC_COUNT_H = (us2rct(8400)-1) >> 8;
	RTC_COUNT_L = (us2rct(8400)-1) & 0xff;     	//rtc timer num
	RTC_TIMER_CTL |= 0x04;       	//open rtc
	EX6 = 1;
//	EA = 1;
	EXSLEEP = 2;
//	PCON2 |= 0x03;  //slect RC32K  
}
void rtc_int() interrupt 12
{
//	EX6 = 0;
//	RTC_TIMER_CTL |= 0x10;       //write 1 to clear int_flag
//	EXSLEEP = 0;
//	AIF &= 0xef;
//	EX6 = 1;
	
	AIF &= ~0x10;
	RTC_TIMER_CTL |= 0x10;       //write 1 to clear int_flag
//	RTC_TIMER_CTL = 0;
//	RTC_COUNT_H = us2rct(TICK_PERIOD) >> 8;
//	RTC_COUNT_L = us2rct(TICK_PERIOD) & 0xff;     	//rtc timer num
	RTC_TIMER_CTL |= 0x04;       //open rtc

	timer0_flag = 1;
}

