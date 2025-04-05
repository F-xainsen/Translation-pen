#include "app_key.h"
#include "gpio.h"
#include "user_handle.h"
#include "flash.h"

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

uint8_t key_loop_flag = 0;
uint8_t key_loop_ct = 0;

uint16_t key_double_ct = 0;
uint8_t key_last = KEY_NONE;

uint8_t keys_press = 0;
uint8_t keys_count[6] = {0};

uint8_t key_press_count = 0;

extern SYSTEM_STRUCT_DATA system_data;
extern uint8_t mouse_flag;
extern uint8_t mouse_en;
// 短按
void key_short_press_handler(uint8_t key){
	key_press_count++;
	gpio_set(0x11, 0);
	switch (key)
	{
		case KEY_PAGE_UP:
			break;
		case KEY_PAGE_DOWN:
			break;
		case KEY_LIGHT:
			gpio_set(0x32, 0);
			break;
		case KEY_DIGITAL:
	        if(key_last == key) send_action(FUNC_SWITCH_ACTION);
		
			mouse_en = 1;
			send_action(DIGITAL_ACTION);
			break;
		case KEY_FUNC_TOP:
			send_action(FUNCTOP_ACTION);
			mouse_en = 1;
			break;
		case KEY_FUNC_BOTTOM:
		    send_action(FUNCBOTTOM_ACTION);
			break;
		default:
			break;
	}
	key_last = key;
	key_double_ct = 0;
}

// 长按
void key_long_press_handler(uint8_t key){
	switch (key)
	{
		case KEY_PAGE_UP:
			if(key_press_count == 2){
				if(keys_count[KEY_PAGE_DOWN] >= KEY_LONG_PRESS){
					 system_data.system_mode = SYSTEM_PAGE;
				}
			}
			else send_action(PAGEUPLONG_ACTION);
			break;
		case KEY_PAGE_DOWN:
			if(key_press_count == 2){
				if (keys_count[KEY_PAGE_UP] >= KEY_LONG_PRESS)
				{
					system_data.system_mode = SYSTEM_PAGE;
				}
            }
			else send_action(PAGEDOWNLONG_ACTION);
			break;
		case KEY_LIGHT:
			break;
		case KEY_DIGITAL:
			break;
		case KEY_FUNC_TOP:
			break;
		case KEY_FUNC_BOTTOM:
			send_action(FUNCBOTTOMLONG_ACTION);
			break;
		default:
			break;
	}
}

// 长按释放
void key_long_release_handler(uint8_t key){
	if(key_press_count)key_press_count--;
	gpio_set(0x11, 1);
	switch (key)
	{
		case KEY_PAGE_UP:
			break;
		case KEY_PAGE_DOWN:
			break;
		case KEY_LIGHT:
			gpio_set(0x32, 1);
			break;
		case KEY_DIGITAL:
			send_action(CLEAR_ACTION);
			mouse_en = 0;
			break;
		case KEY_FUNC_TOP:
			send_action(CLEAR_ACTION);
			mouse_en = 0;
			break;
		case KEY_FUNC_BOTTOM:
			send_action(CLEAR_ACTION);
			break;
		default:
			break;
	}
}

// 短按释放
void key_short_release_handler(uint8_t key){
	if(key_press_count)key_press_count--;
	gpio_set(0x11, 1);
	switch (key)
	{
		case KEY_PAGE_UP:
			send_action(PAGEUP_ACTION);
			break;
		case KEY_PAGE_DOWN:
			send_action(PAGEDOWN_ACTION);
			break;
		case KEY_LIGHT:
			gpio_set(0x32, 1);
			break;
		case KEY_DIGITAL:
			send_action(CLEAR_ACTION);
			mouse_en = 0;
			break;
		case KEY_FUNC_TOP:
			send_action(CLEAR_ACTION);
			mouse_en = 0;
			break;
		case KEY_FUNC_BOTTOM:
			break;
		default:
			break;
	}
}



void key_press(uint8_t key){
	if(keys_count[key] == KEY_SHORT_PRESS) key_short_press_handler(key);
	if(keys_count[key] == KEY_LONG_PRESS) key_long_press_handler(key);
	if(keys_count[key] <= KEY_LONG_PRESS) keys_count[key]++;
}

void key_release(uint8_t key){
	if(keys_count[key] >= KEY_SHORT_PRESS && keys_count[key] <= KEY_LONG_PRESS) key_short_release_handler(key);
	if(keys_count[key] > KEY_LONG_PRESS) key_long_release_handler(key);
    keys_count[key] = 0;
}

void key_count(){
	IS_KEY_PAGE_UP_PRESS?key_press(KEY_PAGE_UP):key_release(KEY_PAGE_UP);
	IS_KEY_PAGE_DOWN_PRESS?key_press(KEY_PAGE_DOWN):key_release(KEY_PAGE_DOWN);
	IS_KEY_LIGHT_PRESS?key_press(KEY_LIGHT):key_release(KEY_LIGHT);
	IS_KEY_DIGITAL_PRESS?key_press(KEY_DIGITAL):key_release(KEY_DIGITAL);
	IS_KEY_FUNC_TOP_PRESS?key_press(KEY_FUNC_TOP):key_release(KEY_FUNC_TOP);
	IS_KEY_FUNC_BOTTOM_PRESS?key_press(KEY_FUNC_BOTTOM):key_release(KEY_FUNC_BOTTOM);
}

void key_loop(){
	gpio_set(ROW1,0);
	gpio_set(ROW2,1);
	gpio_set(ROW3,1);
	!gpio_get_input(COL1)?KEY_PAGE_UP_PRESS:KEY_PAGE_UP_RELEASE;
	!gpio_get_input(COL2)?KEY_PAGE_DOWN_PRESS:KEY_PAGE_DOWN_RELEASE;

	gpio_set(ROW1,1);
	gpio_set(ROW2,0);
	gpio_set(ROW3,1);
	!gpio_get_input(COL1)?KEY_LIGHT_PRESS:KEY_LIGHT_RELEASE;
	!gpio_get_input(COL2)?KEY_DIGITAL_PRESS:KEY_DIGITAL_RELEASE;

	gpio_set(ROW1,1);
	gpio_set(ROW2,1);
	gpio_set(ROW3,0);
	!gpio_get_input(COL2)?KEY_FUNC_TOP_PRESS:KEY_FUNC_TOP_RELEASE;
	!gpio_get_input(COL1)?KEY_FUNC_BOTTOM_PRESS:KEY_FUNC_BOTTOM_RELEASE;

	key_count();
}
