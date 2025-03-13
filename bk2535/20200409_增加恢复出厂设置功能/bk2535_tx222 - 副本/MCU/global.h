/***********************************************************
FileName	: 
Date		: 2011/03/01
Description	: 
Version		: v0.1
Function List: 
----
History: 
<author> <time> <version > <desc>
***********************************************************/

#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#ifdef DEF_VAR 
#define EXTERN 
#else
#define EXTERN extern
#endif


//#define EEPROM_ADDR		0x50

#define WRITE_CMD		0x00
#define READ_CMD		0x01
#define ACK_FLAG		0x00
#define NAK_FLAG		0x01



extern XDATA unsigned char RX0_Address[];
extern XDATA unsigned char TX_Address[];


/*---------------------------------------------------------------------------------------------------------*/
EXTERN bit sleep_mode,rf_diable_flag,timer0_flag,search_mode;
EXTERN bit eeprom_exist_flag;
EXTERN bit low_power_flag,low_power_flag2, power_on_flag;

EXTERN xdata unsigned char low_power_ct,low_power_ct2;

EXTERN unsigned char xdata  fc_mode;
EXTERN xdata unsigned short fc_mode_ct,sleep_ct;

EXTERN unsigned char  maincount;
EXTERN unsigned short lvd_adc_dat;

//EXTERN unsigned char mode;


EXTERN XDATA unsigned char rf_channel;
//EXTERN XDATA unsigned char rf_channel_dyn;

EXTERN XDATA unsigned char no_response_ct;
EXTERN XDATA unsigned char connect_lost_ct;
EXTERN unsigned char idata max_trans_ct;


EXTERN unsigned char kb_led_sta;

EXTERN unsigned char idata led_flash_ct;


extern unsigned char code T_JumpFreqTab[4];

EXTERN unsigned char xdata rf_skip_ct, rf_freq_id;





//typedef struct	_SYS_STRUCT
//{
//	BYTE keyboard[8];
//	BYTE mouse[4];
//	BYTE multimedia[2];
//
//	struct {
//		BYTE keyboard_data_ready   : 1 ;
//		BYTE mouse_data_ready      : 1 ;
//		BYTE multimedia_data_ready : 1 ;
//		BYTE sysctrl_data_ready    : 1 ;
//	}status;
//	
//	unsigned char mouse_key;
//	short mouse_x, mouse_y, mouse_z;
//	BYTE keyboard_buf[8];
//	BYTE mouse_buf[4];
//	BYTE multimedia_buf[3];
//	BYTE sysctrl_buf[3];
//}SYS_STRUCT;

typedef struct	_SYS_STRUCT
{
	BYTE keyboard[7];   
	BYTE mouse[4];
	BYTE multimedia[2];
}SYS_STRUCT;

EXTERN SYS_STRUCT xdata system;

extern XDATA UINT8 key_array[];
EXTERN bit ghost_key_flag;
EXTERN unsigned short idata func_keys,last_func_keys;


#endif

