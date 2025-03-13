#ifndef _pairfunc_H_
#define _pairfunc_H_

extern bit Pair_to_sleep_int;
extern bit Ctrl_rx_pair_flag;		//1 = 发送数据给RX，控制RX进行对码
extern UINT8 XDATA Pair_to_sleep;
extern UINT8 XDATA Normal_to_pair_cont;

extern void Normal_to_pair(void);
extern void rf_data_read(BYTE bytes);
extern void system_mode_pair(void);
#endif