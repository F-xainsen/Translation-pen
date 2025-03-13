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

#ifndef _TIMER_H_
#define _TIMER_H_
extern bit Time_1ms_flag;		// 1 = 10MS计时到了

extern void timer0_inital(void);
extern void timer1_inital(void);
extern void timer2_inital(void);
extern void WaitUs(UINT32 us);

extern void rtc_init(void);

#endif

/***********************************************************
						end file
***********************************************************/