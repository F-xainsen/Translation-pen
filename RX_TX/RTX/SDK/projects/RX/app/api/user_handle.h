#ifndef _USER_HANLDLE_H_ // this file's unique symbol checking section
#define _USER_HANLDLE_H_

#include <stdbool.h> // standard boolean definitions
#include <stdint.h>  // standard integer functions
#include "rwip_config.h"

void user_fn24_main(void);

void status_report(uint8_t key1,uint8_t key2,uint16_t adc_value, uint8_t pair_status);
void key_send(uint8_t keyfunc,uint8_t keycode);
void mouse_send(uint16_t x, uint16_t y);

void EP3_callback(void *_arr, int len);

extern uint8_t first_pair;
extern uint16_t pair_timeout;
extern uint8_t pair_step_timeout; // 200ms reset pairstep
extern uint8_t pairStep;

// =================================================================
// 跳频
// =================================================================

#define MAX_FREQ_SIZE 5

extern uint8_t ch_chan;
extern uint8_t channel_table[MAX_FREQ_SIZE];
extern uint8_t ch_cur_index;

void jump_frequency(void);

#endif
