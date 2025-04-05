#ifndef _USER_HANLDLE_H_ //this file's unique symbol checking section
#define _USER_HANLDLE_H_

#include <stdbool.h>          // standard boolean definitions
#include <stdint.h>           // standard integer functions
#include "rwip_config.h"


#define PAGEUP_ACTION           0
#define PAGEUPLONG_ACTION       1
#define PAGEDOWN_ACTION         2
#define PAGEDOWNLONG_ACTION     3
#define DIGITAL_ACTION          4
#define DIGITALLONG_ACTION      5
#define FUNCTOP_ACTION          6
#define FUNCTOPLONG_ACTION      7
#define FUNCBOTTOM_ACTION       8
#define FUNCBOTTOMLONG_ACTION   9

#define MOUSE_ACTION            0x0A
#define FUNC_SWITCH_ACTION      0x0B
#define BAT_SET_ACTION          0x0C
#define STATUS_REPORT_ACTION    0x0D
#define DOWNLINK_ACTION         0x0E

#define TEST_ACTION             0x0F

#define CLEAR_ACTION            0xAC
#define JUMP_FREQ_ACTION        0xCC
#define REPAIR_ACTION           0xF4


void user_fn24_main(void);

extern uint8_t first_pair;
extern uint8_t pair_step_timeout; // 200ms reset pairstep
extern uint8_t pairStep;

// =================================================================
// 跳频
// =================================================================

#define MAX_FREQ_SIZE 3

extern uint8_t ch_chan;
extern uint8_t channel_table[MAX_FREQ_SIZE];
extern uint8_t ch_cur_index;

extern uint8_t vibrationMinus;
extern uint16_t vibrationDuration;
extern uint16_t vibrationBootTime;

extern uint8_t low_power_flag;

extern uint8_t single_monitor_flag;
// 屏幕解析度
extern uint16_t resolution_Width;
extern uint16_t resolution_Height;

extern int16_t current_mouse_x;
extern int16_t current_mouse_y;

// 定时上报状态
extern uint8_t  status_report_flag;
extern uint16_t status_report_ct;

// 定时上报电池状态
extern uint8_t  bat_report_flag;
extern uint16_t bat_report_ct;

void jump_frequency(void);

void send_action(uint8_t action);
void mouse_send(uint8_t pos,int16_t x,int16_t y);

void receive_action(void);

void bat_check(void);

#endif

