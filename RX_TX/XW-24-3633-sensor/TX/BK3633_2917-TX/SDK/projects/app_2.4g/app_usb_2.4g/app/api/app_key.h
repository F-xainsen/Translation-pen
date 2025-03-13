#ifndef _APP_KEY_H__
#define _APP_KEY_H__

#define	SHORT_CNT			          2
#define	LONG_CNT			          50
#define	LONG_REPEAT_CNT		      300


#define LED_R							    	0x10
#define LED_W										0x11
#define MOTOR										0x04

#define ROW1  									0x12
#define ROW2                    0x13
#define ROW3                    0x33
#define COL1                    0x06
#define COL2                    0x05

#define PAGE_UP									0x01
#define PAGE_DOWN               0x02
#define LASER_KEY               0x04
#define DIG_LASER_KEY           0x08
#define ENTER_KEY               0x10
#define TAB_KEY                 0x20
                               
#define DOWN	0                 
#define UP		1

#define ROW_NUM 	3	
#define COL_NUM 	2	

#define POS(row, col) (row * COL_NUM + col)

// >优先级比&高，先执行判断，再&就出错，加小括号即可

// 判断指定行列按钮状态-是否抬起
#define IS_KEY_UP(row, col)			((states & (1L << POS(row, col))) > 0)
// 判断指定行列按钮状态-是否按下
#define IS_KEY_DOWN(row, col)		((states & (1L << POS(row, col))) == 0)
// 获取之前的按钮状态
#define KEY_STATE(row, col)		  (((states & (1L << POS(row, col))) == 0) ? 0 : 1)

// 将指定行列按钮状态-设置为抬起1
#define SET_KEY_UP(row, col)		states |= (1L << POS(row, col))
// 将指定行列按钮状态-设置为按下0
#define SET_KEY_DOWN(row, col)	states &=~(1L << POS(row, col))

void MK_scan();
void app_key_scan();
#endif