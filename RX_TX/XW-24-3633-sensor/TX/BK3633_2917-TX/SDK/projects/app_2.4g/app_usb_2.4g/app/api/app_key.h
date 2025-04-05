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

// >���ȼ���&�ߣ���ִ���жϣ���&�ͳ�����С���ż���

// �ж�ָ�����а�ť״̬-�Ƿ�̧��
#define IS_KEY_UP(row, col)			((states & (1L << POS(row, col))) > 0)
// �ж�ָ�����а�ť״̬-�Ƿ���
#define IS_KEY_DOWN(row, col)		((states & (1L << POS(row, col))) == 0)
// ��ȡ֮ǰ�İ�ť״̬
#define KEY_STATE(row, col)		  (((states & (1L << POS(row, col))) == 0) ? 0 : 1)

// ��ָ�����а�ť״̬-����Ϊ̧��1
#define SET_KEY_UP(row, col)		states |= (1L << POS(row, col))
// ��ָ�����а�ť״̬-����Ϊ����0
#define SET_KEY_DOWN(row, col)	states &=~(1L << POS(row, col))

void MK_scan();
void app_key_scan();
#endif