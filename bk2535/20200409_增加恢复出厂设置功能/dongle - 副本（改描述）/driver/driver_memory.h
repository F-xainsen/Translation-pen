/****************************************************************************
**
** Copyright (C) 2011 Beken Corporation, All rights reserved.
**
** Contact: Beken Corporation (www.beken.com)
**
** Author:  river
**
** History: 2012/03/07 
**
** Version: 1.0
**
****************************************************************************/

#ifndef _DRIVER_MEMORY_H_
#define _DRIVER_MEMORY_H_

/****************************************************************************
**
** Copyright (C) 2011 Beken Corporation, All rights reserved.
**
** Contact: Beken Corporation (www.beken.com)
**
** Author:  river
**
** History: 2012/03/07 
**
** Version: 1.0
**
****************************************************************************/

#include "headfile\includes.h"

/***************************************************************
*   DATA region   128B = 32B(r0-r3) + 16B(128b) +80B(stack)
***************************************************************/
//66b
extern DATA SYSTEM_STRUCT_DATA system_data;
extern DATA UINT8 usb_endpoint_state[4];

//shared by dongle/mouse/part
extern VOLATILE bit flag_page_flag;
extern VOLATILE bit flag_zs_ctrl;
extern VOLATILE bit flag_zx_ctrl;
extern VOLATILE bit flag_tx_charge;
extern VOLATILE bit flag_mouse_ctrl;
extern VOLATILE bit flag_mouse_update;                //mouse data received and need to analysis again
extern VOLATILE bit flag_keyboard_update;           //keyboard data received and need to analysis again

extern VOLATILE bit flag_rf_bk2423;
extern VOLATILE bit flag_rf_send_end;
extern VOLATILE bit flag_page_need_ack;
extern VOLATILE bit flag_rf_paged_mouse;
extern VOLATILE bit flag_rf_paged_keyboard;
extern VOLATILE bit flag_rf_old_mouse;
extern VOLATILE bit flag_rf_old_keyboard;
extern VOLATILE bit flag_rf_driver_received;
extern VOLATILE bit flag_rf_link_ok;
extern VOLATILE bit flag_rf_link_status;
extern VOLATILE bit flag_grose_status;
extern VOLATILE bit flag_rf_packet_received;
extern VOLATILE bit flag_rf_next_channel;
extern VOLATILE bit flag_rf_led_received;

extern VOLATILE bit flag_media_uploading;            //media data need upload to pc
extern VOLATILE bit flag_mce_uploading;               //mce data need upload to pc		
extern VOLATILE bit flag_power_uploading;            //power data need upload to pc		

extern VOLATILE bit flag_usb_report_state;
extern VOLATILE bit flag_usb_send_wait_state;
extern VOLATILE bit flag_usb_remote_wakeup;
extern VOLATILE bit flag_usb_suspend;

extern VOLATILE bit flag_usb_config_set;

extern VOLATILE bit flag_usb_endpoint0_ready_report;
extern VOLATILE bit flag_usb_endpoint0_accept;
extern VOLATILE bit flag_usb_endpoint1_uploading;
extern VOLATILE bit flag_usb_endpoint2_uploading;
extern VOLATILE bit flag_usb_endpoint3_uploading;
extern VOLATILE bit flag_usb_endpoint1_ready_uploading;
extern VOLATILE bit flag_usb_endpoint2_ready_uploading;
extern VOLATILE bit flag_usb_endpoint3_ready_uploading;
extern VOLATILE bit flag_usb_endpoint1_null_data;
//extern VOLATILE bit flag_send_state;

extern VOLATILE bit flag_usb_idle;
extern VOLATILE bit flag_usb_get_hid_report;
extern VOLATILE bit flag_usb_get_protocol;

extern VOLATILE bit flag_usb_state_normal;

extern VOLATILE bit flag_mouse_swing_left_first;
extern VOLATILE bit flag_mouse_swing_right_first;

extern VOLATILE bit flag_keyboard_standard_pressed;
extern VOLATILE bit flag_keyboard_media_pressed;
extern VOLATILE bit flag_keyboard_mce_pressed;
extern VOLATILE bit ChangeTxMode ;
extern VOLATILE bit TimeFlag ;
extern VOLATILE bit ChangeZSCMD ;
extern VOLATILE bit ChangeZXCMD ;
extern VOLATILE bit flag_keyboard_power_pressed;
extern IDATA UINT8 Tx_Vol_Data ;
extern IDATA UINT8 NoLinkCont;
#ifdef MODE_TEST
extern VOLATILE bit flag_test_power_up;
extern VOLATILE bit flag_test_single_wave;
#endif


sbit key_test = P2^1;         //ESC_F5

/***************************************************************
*   extern IDATA region  128B = address with r0/r1
***************************************************************/
//52+15+8=75B
//extern IDATA SYSTEM_STRUCT_IDATA system_idata;
extern IDATA TIMER timer_test;                        // 4B
extern IDATA TIMER timer_cd_detect;

extern IDATA UINT8 RF_CHANNEL_TABLE[16];
extern IDATA UINT8 rf_address_rx0[RF_ADDRESS_LEN];         //rx0 tx addr
extern IDATA UINT8 rf_address_rx1[RF_ADDRESS_LEN];
extern IDATA UINT8 rf_address_rx2[RF_ADDRESS_LEN];

#ifdef MODE_TEST
extern IDATA UINT8 packet_count;
extern IDATA UINT8 P_count;
extern IDATA UINT8 N_count;
extern IDATA UINT8 P_flag;
extern IDATA UINT8 N_flag;
extern IDATA UINT8 P_status_count;
extern IDATA UINT8 N_status_count;
extern IDATA UINT8 RF_TEST_CHANNEL[3];
extern IDATA UINT8 RF_TEST_ADDRESS[RF_ADDRESS_LEN];
#endif

//usb descriptor
extern IDATA setup_buffer setup;                                                    // 8B

/***************************************************************
*   extern XDATA region 64KB = address with gptr + movx
***************************************************************/
//396+12+30+[19]B
extern XDATA SYSTEM_STRUCT_XDATA system_xdata;
extern XDATA UINT8 rf_fifo_data[32];             //rf fifo
extern XDATA UINT8 rf_fifo_data_mose[4];
extern XDATA UINT8 rf_tx_data[6];             //rf fifo
extern XDATA UINT8 rf_tx_data_len;


#ifdef UART_INTERRUPT
extern XDATA UINT8 uart_data_receive[UART_DATA_LEN];
extern XDATA UINT8 uart_data_send[UART_DATA_LEN];
extern XDATA UINT8 uart_data_temp[2*UART_DATA_LEN];
extern XDATA UINT8 uart_data_len_receive;
extern XDATA UINT8 uart_data_len_send;
extern XDATA UINT8 uart_data_count_send;
extern XDATA INT8 uart_data_need_write_bytes;
extern XDATA UINT8 uart_data_index;
#endif

/***************************************************************
*   extern CODE region 32KB = address with gptr + movc
***************************************************************/
extern CODE UINT8 RF_BANK0_REG[23][2];
extern CODE UINT8 RF_BANK0_TSET_REG[4][2];

extern CODE UINT32 RF_BANK1_REG_0_13[4];
extern CODE UINT32 RF_BANK1_REG_4_2401[1];
extern CODE UINT8 RF_BANK1_REG14[11];

extern CODE UINT8 VALUE_ZERO_PACKET[2];
extern CODE UINT8 VALUE_ONE_PACKET[2];
extern CODE UINT8 VALUE_TWO_PACKET[2];

//usb descriptor
extern CODE struct_usb_device_descriptor hid_device_descriptor; // 19B
extern CODE const struct_hid_configuration_descriptor hid_configuration_descriptor;
extern CODE const struct_hid_keyboard_report_descriptor hid_keyboard_report_descriptor;
extern CODE const struct_hid_mouse_report_descriptor hid_mouse_report_descriptor;
extern CODE const struct_hid_media_report_descriptor hid_media_report_descriptor;

extern CODE UINT8 * usb_string_table[LEN_USB_STRING];            // 3B
extern CODE const UINT8 usb_string_descriptor0[LEN_STRING0];
extern CODE const UINT8 usb_string_descriptor1[LEN_STRING1];
extern CODE const UINT8 usb_string_descriptor2 [LEN_STRING2];

// key table
// mouse special key
extern CODE UINT8 mouse_key_value_special[25][2];
// standard key 
extern CODE UINT8 keyboard_key_value_standard[8][20];
//media & mce key
extern CODE UINT8 media_key_f1_f12[12];
extern CODE UINT8 keyboard_key_value_media[57][3];

// power manage
extern CODE UINT8 keyboard_key_value_power[3][2];

extern UINT8 g_FrqUpdataTimer;
extern UINT8 tx_send_cont;
extern UINT16 Normal_to_pair_cont;
extern UINT16 Not_link_cont;
extern UINT8 JumpFreqCount;

#define MaxJumpFreqNum   4
extern CODE BYTE JumpFreq_Buf[MaxJumpFreqNum];
extern CODE BYTE PUBLICRX0_Address[5];


#endif  //_DRIVER_MEMORY_H_

/***********************************************************
						end file
***********************************************************/
