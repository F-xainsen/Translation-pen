#include "app_key.h"
#include "app.h" 
#include "gpio.h"
#include "driver_rf.h"
uint8_t key_values[] = {
    0x01, // POS(0, 0) -> PAGE_UP
    0x02, // POS(0, 1) -> PAGE_DOWN
    0x04, // POS(1, 0) -> LASER_KEY
    0x08, // POS(1, 1) -> DIG_LASER_KEY
    0x10, // POS(2, 0) -> ENTER_KEY
    0x20  // POS(2, 1) -> TAB_KEY
};

// 按键计时器数组（毫秒）
uint32_t key_timers[ROW_NUM][COL_NUM];

//uint32_t get_timer_ms() {
//    static uint32_t timer = 0;
//    timer++;
//    return timer;
//}
//uint32_t update_timer_ms() {
//    static uint32_t timer = 0;
//    return timer;
//}

// 定义按键状态
typedef enum {
    KEY_UP,    // 按键抬起
    KEY_DOWN,  // 按键按下
    KEY_LONG_PRESS // 按键长按
} KeyState;

void ROW_OUT(uint8_t row){
	uint8_t row1, row2, row3;
	row1 = (row == 0) ? 0 : 1;
	row2 = (row == 1) ? 0 : 1;
	row3 = (row == 2) ? 0 : 1;
	gpio_set(ROW1,row1);
	gpio_set(ROW2,row2);
	gpio_set(ROW3,row3);
}

uint8_t COL_IN(uint8_t col){
	
	if(col == 0) return gpio_get_input(COL1);
	if(col == 1) return gpio_get_input(COL2);
	return 0;
}


// 短按处理函数
void on_short_press(uint8_t row, uint8_t col) {
    uint8_t key_value = key_values[POS(row, col)];
    uart_printf("Short press: Key 0x%02X (Row %d, Col %d)\r\n", key_value, row + 1, col + 1);
    // send_data(key_value, false); // 发送短按数据
}

// 长按处理函数
void on_long_press(uint8_t row, uint8_t col) {
    uint8_t key_value = key_values[POS(row, col)];
    uart_printf("Long press: Key 0x%02X (Row %d, Col %d)\n", key_value, row + 1, col + 1);
    // send_data(key_value, true); // 发送长按数据
}

// 按键按下处理函数
void MK_on_keydown(uint8_t row, uint8_t col) {
    uart_printf("第 %d 行 %d 列按钮Down!\r\n", (int)row + 1, (int)col + 1);
    gpio_set(LED_R, 0);
}

// 按键抬起处理函数
void MK_on_keyup(uint8_t row, uint8_t col) {
    uart_printf("第 %d 行 %d 列按钮Up!\r\n", (int)row + 1, (int)col + 1);
    gpio_set(LED_R, 1);
}

extern uint8_t rf_fifo_data[MAX_PACKET_LEN];
uint16_t states = 0xFFFF;
void MK_scan(){
	static uint32_t timer = 0x00;
	static uint16_t key_count_shake = 0;
	uint8_t row = 0, col = 0;
	uint8_t key_value = 0;
	// 外循环，把第row行拉低，其他拉高
	for(row = 0; row < ROW_NUM; row++){	// 0,1,2
		ROW_OUT(row);
		
		// 内循环，读取第col列电平信号，存到states对应位置
		for(col = 0; col < COL_NUM; col++){ // 0,1
				if(COL_IN(col) != KEY_STATE(row, col)){ // 电平发生了变化
					// 判断第row行col列按钮状态(抬起1高电平，按下0低电平)
					if(COL_IN(col)){ // 抬起
						SET_KEY_UP(row, col); 
						MK_on_keyup(row,col);
						key_timers[row][col] = 0;
						timer = 0;
					}else {					 // 按下
						timer++;
						SET_KEY_DOWN(row, col);
						MK_on_keydown(row, col);
						key_value = key_values[POS(row, col)];
						rf_fifo_data[0] = key_value;
						//uart_printf("Key pressed: 0x%02X   rf_fifo_data:0x%02X\r\n", key_value, rf_fifo_data[0]);
						if (!COL_IN(col)) {
							  uart_printf("timer:%d",timer);
								if (timer >= 1000) { // 长按阈值设为
										uart_printf("Long press: Key 0x%02X (Row %d, Col %d)\r\n", key_values[POS(row, col)], row + 1, col + 1);
								} else {
										uart_printf("Short press: Key 0x%02X (Row %d, Col %d)\r\n", key_values[POS(row, col)], row + 1, col + 1);
								}
						}
					}
				}
			
		}
	}
}


void row_scan(uint8_t row)
{
	static uint32_t timer = 0;
	uint8_t col = 0;
	uint8_t key_value = 0;
	ROW_OUT(row);
	for(col = 0; col<2; col++)
	if(!COL_IN(col)){
		key_value = key_values[POS(row, col)];
		if (timer >= 1000) { // 长按阈值设为
				uart_printf("Long press: Key 0x%02X (Row %d, Col %d)\r\n", key_values[POS(row, col)], row + 1, col + 1);
		} else {
				uart_printf("Short press: Key 0x%02X (Row %d, Col %d)\r\n", key_values[POS(row, col)], row + 1, col + 1);
		}
		MK_on_keydown(row,col);
		timer++;
	}else{
		gpio_set(LED_R,1);
		timer = 0;
	}
}


int key_state[3][2] = {0};

void app_key_scan(){
		uint8_t row = 0, col = 0;
	
		for(row = 0; row <3; row++){
			ROW_OUT(row);
			for(col = 0; col <2; col++){
				if(!COL_IN(col)){
					key_state[row][col]++;
					//uart_printf("key_state[row][col]:%d\r\n",key_state[row][col]);
					if (key_state[row][col] == 500) {
							uart_printf("按键[%d,%d] 短按\n", row+1, col+1);
					}else{
							if (key_state[row][col] == 8000) {
								uart_printf("按键[%d,%d] 长按\n", row+1, col+1);
								//uart_printf("key_state[row][col]:%d\r\n",key_state[row][col]);
							}
					} 
				}else{
					if (key_state[row][col] > 0 && key_state[row][col] < 500) {
							uart_printf("按键[%d,%d] 误触\n", row+1, col+1);
					}
					
					key_state[row][col] = 0; // 重置状态
					//uart_printf("123");
				}
			}
			gpio_set(ROW1,1);
			gpio_set(ROW2,1);
			gpio_set(ROW2,1);
			
			
			
		}
}

