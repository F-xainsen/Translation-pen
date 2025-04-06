#ifndef _APP_KEY_H__
#define _APP_KEY_H__

#include "app.h" 

#define ROW1 0x12
#define ROW2 0x13
#define ROW3 0x33
#define COL1 0x06
#define COL2 0x05

#define KEY_NONE  			0xFF
#define KEY_PAGE_UP		 	0x01
#define KEY_PAGE_DOWN	 	0x02
#define KEY_LIGHT		 	0x04
#define KEY_DIGITAL		 	0x08
#define KEY_FUNC_TOP	 	0x10
#define KEY_FUNC_BOTTOM		0x20

#define KEY_LONG_PRESS		20	    // 3秒触发
#define KEY_SHORT_PRESS		1	    // 100 ms 触发短按
#define KEY_DOUBLE_PRESS	1000	// 1秒内按下

#define IS_KEY_PAGE_UP_PRESS		 (keys_press & KEY_PAGE_UP)
#define IS_KEY_PAGE_DOWN_PRESS		 (keys_press & KEY_PAGE_DOWN)
#define IS_KEY_LIGHT_PRESS			 (keys_press & KEY_LIGHT)
#define IS_KEY_DIGITAL_PRESS		 (keys_press & KEY_DIGITAL)
#define IS_KEY_FUNC_TOP_PRESS		 (keys_press & KEY_FUNC_TOP)
#define IS_KEY_FUNC_BOTTOM_PRESS	 (keys_press & KEY_FUNC_BOTTOM)

#define KEY_PAGE_UP_PRESS		 	 (keys_press |= KEY_PAGE_UP)
#define KEY_PAGE_DOWN_PRESS		 	 (keys_press |= KEY_PAGE_DOWN)
#define KEY_LIGHT_PRESS			 	 (keys_press |= KEY_LIGHT)
#define KEY_DIGITAL_PRESS		 	 (keys_press |= KEY_DIGITAL)
#define KEY_FUNC_TOP_PRESS		 	 (keys_press |= KEY_FUNC_TOP)
#define KEY_FUNC_BOTTOM_PRESS	 	 (keys_press |= KEY_FUNC_BOTTOM)

#define KEY_PAGE_UP_RELEASE		 	 (keys_press &= ~KEY_PAGE_UP)
#define KEY_PAGE_DOWN_RELEASE		 (keys_press &= ~KEY_PAGE_DOWN)
#define KEY_LIGHT_RELEASE			 (keys_press &= ~KEY_LIGHT)
#define KEY_DIGITAL_RELEASE		 	 (keys_press &= ~KEY_DIGITAL)
#define KEY_FUNC_TOP_RELEASE		 (keys_press &= ~KEY_FUNC_TOP)
#define KEY_FUNC_BOTTOM_RELEASE	 	 (keys_press &= ~KEY_FUNC_BOTTOM)

extern uint8_t key_loop_flag;
extern uint8_t key_loop_ct;

extern uint16_t key_double_ct;
extern uint8_t key_last;

extern uint8_t keys_press;
extern uint8_t keys_count[6];

void key_loop(void);

#endif
