#ifndef _sleep_H_
#define _sleep_H_

extern bit Mouse_move_flag;		//1 = 空鼠移动
extern bit normal_to_sleep_int;
extern INT32 XDATA narmal_sleep_cont;	  //用来休眠计数
extern void Normal_to_sleep(void);
extern void system_mode_sleep(void);

#endif
