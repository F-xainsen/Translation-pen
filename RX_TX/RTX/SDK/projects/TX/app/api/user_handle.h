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

#define CLEAR_ACTION            0xAC


void user_fn24_main(void);

extern uint8_t first_pair;
extern uint8_t pair_step_timeout; // 200ms reset pairstep
extern uint8_t pairStep;

// =================================================================
// 跳频
// =================================================================

#define MAX_FREQ_SIZE 5

extern uint8_t ch_chan;
extern uint8_t channel_table[MAX_FREQ_SIZE];
extern uint8_t ch_cur_index;

extern uint8_t vibrationMinus;
extern uint16_t vibrationDuration;
extern uint16_t vibrationBootTime;

extern uint8_t low_power_flag;

void jump_frequency(void);

void send_action(uint8_t action);
void mouse_send(int16_t x,int16_t y);

void receive_action(void);

#endif

