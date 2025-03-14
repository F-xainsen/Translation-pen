#ifndef _APP_KEY_H__
#define _APP_KEY_H__
#include "user_config.h"
#include "Define_system.h"
#define SEND_BUFFER_CNT  6
#define SEND_BUFFER_SIZE  8

#define FLAG_KEY_ACTIVE                         0X00010000
#define FLAG_KEY_PAIRED                         0X00000001
#define FLAG_PEER_EN                            0X00000002
#define FLAG_PEER_PUBLIC_ADDR                   0X00000004
#define FLAG_CLASS_PROTOCOL                     0X00000008
#define FLAG_PAIR_RW_INIT_EN                    0X00000010
#define FLAG_CONNECT                            0X00000020
#define BK3633_DEMO_KIT			1

#define	BlueLedPort		0x11


#define KEYBOARD_MAX_COL_SIZE 3
#define KEYBOARD_MAX_ROW_SIZE 5


typedef enum
{
	VOICE_KEY_DOWN = 0,
	GENERAL_KEY_DOWN,
	ALL_KEY_FREE,
	ALL_KEY_FREE_DELAY,
}app_key_type;

extern void Delay_ms(int num);
extern void Delay_us(int num);

extern uint8_t bt_tx_buff[SEND_BUFFER_CNT][SEND_BUFFER_SIZE];
extern uint8_t tx_buff_len[SEND_BUFFER_CNT];
extern uint8_t tx_buff_head,tx_buff_tail,tx_buff_count;
extern uint8_t  stan_key_len,old_key_len;
extern app_key_type app_key_state;
extern unsigned char voice_key_press_status;
extern unsigned char restart_adv_after_stop;
extern unsigned char need_delete_current_adv_activity;

#define	APP_KEYSCAN_DURATION	(1)		// 10ms
#define	APP_LED_DURATION	    (20)		// 10ms

#define	APP_LEDDUTY_DB		((50/APP_KEYSCAN_DURATION)-1)

#define flag_rf_paged               0x00000001
#define flag_dongle_lost            0x00000002
#define flag_rf_send                0x00000004
#define flag_tick_8ms_loop          0x00000008
#define flag_key_released           0x00000010
#define flag_key_pressed            0x00000020
#define flag_key_short              0x00000040
#define flag_key_long               0x00000080
#define flag_status_idle            0x00000100
#define flag_status_sleep           0x00000200
#define flag_rf_send_end            0x00000400
#define flag_rf_last_state          0x00000800
#define flag_rf_need_clear_maxRT    0x00001000
#define flag_rf_hop_16ch            0x00002000
#define flag_wake_up_by_timer       0x00004000
#define flag_power_up               0x00008000
#define flag_low_batt               0x00010000
#define flag_rf_receive_page        0x00020000 // 0 allow to page with dongle when power up
#define flag_rf_send_success        0x00040000
#define	flag_system_powerdown		0x00080000	// dongle unused.

#ifdef __MOUSE__
#define SENSOR_TEST         1

#define MOUSE_STANDARD_KEY_LEFT                 0x01
#define MOUSE_STANDARD_KEY_RIGHT                0x02
#define MOUSE_STANDARD_KEY_MIDDLE               0x04
#define MOUSE_STANDARD_KEY_BACK                 0x08
#define MOUSE_STANDARD_KEY_FORAWARD             0x10
#define MOUSE_STANDARD_KEY_DPI                  0x20
#define MOUSE_STANDARD_KEY_SWING_LEFT           0x40
#define MOUSE_STANDARD_KEY_SWING_RIGHT          0x80


#define B_0100_0000         0x40
#define B_1000_0000         0x80
#define B_1100_0000         0xc0
#define B_1000_0001         0x81
#define B_0000_0000         0x00
#define B_1000_0010         0x82
#define B_1000_0011         0x83
#define B_0111_1111         0x7f

#define KEY_LEFT                0x33
#define KEY_RIGHT               0x31
#define KEY_MIDDLE              0x32
#define KEY_PAIR                0x10
#define WHEEL_B                 0x30
#define WHEEL_A                 0x34
#define LED_24                  0x15
#define LED_BT                  0x14


/*keys scanning***********************************/

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
#define MOUSE_KEY               0x10
#define RETURN_KEY              0x20
                               
#define DOWN	0                 
#define UP		1

#define ROW_NUM 	3	
#define COL_NUM 	2	

#define POS(row, col) (row * COL_NUM + col)

// >优先级比&高，先执行判断，再&就出错，加小括号即可

// 判断指定行列按钮状态-是否抬起
#define IS_KEY_UP(row, col)			((states & (1L << POS(row, col))) > 0)
// 判断指定行列按钮状态-是否按下
#define IS_KEY_DOWN(row, col)		((states & (1L << POS(row, col))) == 0)
// 获取之前的按钮状态
#define KEY_STATE(row, col)		  (((states & (1L << POS(row, col))) == 0) ? 0 : 1)

// 将指定行列按钮状态-设置为抬起1
#define SET_KEY_UP(row, col)		states |= (1L << POS(row, col))
// 将指定行列按钮状态-设置为按下0
#define SET_KEY_DOWN(row, col)	states &=~(1L << POS(row, col))

/************************************/

#define SENSOR_SDIO_PIN 		0x11
#define SENSOR_SCLK_PIN 		0x10
#define SENSOR_MOTSWK_PIN 	    0x12	// note:must check the IO need (pull down & up)wakeup


#define TEST_PIN1               0X02
#define TEST_PIN2               0x03
#define TEST_PIN3               0x04
#define TEST_PIN4               0x05

#define PCBA_SCLK           0x04
#define PCBA_SDIO           0x32
#define PCBA_SACK           0x31

#define ADC_CHANNEL         6

typedef enum
{
    APP_VOLTAGE_1V5=0Xba,
    APP_VOLTAGE_1V4=0Xac,
    APP_VOLTAGE_1V3=0Xa1,
    APP_VOLTAGE_1V2=0X96,
    APP_VOLTAGE_1V1=0X8b,
    APP_VOLTAGE_1V0=0X80,
    APP_VOLTAGE_0V9=0x73,	// 0X70,
    APP_VOLTAGE_0V8=0X60,
    APP_VOLTAGE_0V7=0X50,
    APP_VOLTAGE_0V6=0X40,
} bat_voltage;
#else
#define SCAN_COUT  0x07
#define KEY_STATUS_CHANGE         0X0001
#define KEY_STATUS_ACTIVE         0X0002
#define KEY_STATUS_ERROR          0X0004
#define KEY_STATUS_GHOST          0X0008
#define KEY_STATUS_FN             0X0010
#define KEY_STATUS_NUMLOCK        0X0020
#define KEY_STATUS_PAIR           0X0040
#define KEY_STATUS_ENTER_PINCODE  0X0080
#define KEY_STATUS_ENTER_SLEEP    0X0100
#define KEY_STATUS_WAKE_UP        0X0200
#define KEY_STATUS_RELEASE        0X0400
#define KEY_STATUS_IN_SLEEP       0X0800
#define KEY_STATUS_IN_WAKE_UP     0X1000
#define KEY_STATUS_POWER_ON       0X2000
#define MEDIA_KEY_UPLOAD       0X01
#define POWER_KEY_UPLOAD       0X02
#define STANDARD_KEY_UPLOAD    0X04
#define MEDIA_KEY_SEND         0X10
#define POWER_KEY_SEND         0X20
#define STANDARD_KEY_SEND      0X40
#define KEY_ERROR_TIMEOUT 35000


#define KEY_NULL            0x00
#define KEY_A               0x04        //A
#define KEY_B               0x05        //B
#define KEY_C               0x06        //C
#define KEY_D               0x07        //D
#define KEY_E               0x08        //E
#define KEY_F               0x09          //F
#define KEY_G               0x0A        //G
#define KEY_H               0x0B        //H
#define KEY_I               0x0C        //I
#define KEY_J               0x0D        //J
#define KEY_K               0x0E        //K
#define KEY_L               0x0F        //L
#define KEY_M               0x10        //M
#define KEY_N               0x11        //N
#define KEY_O               0x12        //O
#define KEY_P               0x13        //P
#define KEY_Q               0x14        //Q
#define KEY_R               0x15        //R
#define KEY_S               0x16        //S
#define KEY_T               0x17        //T
#define KEY_U               0x18        //U
#define KEY_V               0x19        //V
#define KEY_W               0x1A        //W
#define KEY_X               0x1B        //X
#define KEY_Y               0x1C        //Y
#define KEY_Z               0x1D        //Z
#define KEY_1               0x1E        //1 !
#define KEY_2               0x1F        //2 @
#define KEY_3               0x20        //3 #
#define KEY_4               0x21
#define KEY_5               0x22
#define KEY_6               0x23
#define KEY_7               0x24
#define KEY_8               0x25
#define KEY_9               0x26
#define KEY_0               0x27
#define KEY_ENTER           0x28
#define KEY_ESC             0x29
#define KEY_BACKSPACE       0x2A
#define KEY_TAB             0x2B
#define KEY_SPACE           0x2C
#define KEY_SUB             0x2D    // - and _
#define KEY_EQUAL           0x2E    // = and +
#define KEY_LEFT_BRACKET    0x2F    // [ and {
#define KEY_RIGHT_BRACKET   0x30    // ] and }
#define KEY_VERTICAL_LINE   0x31    // "\" and |
#define KEY_WAVE            0x32    // ` and ~
#define KEY_SEMICOLON       0x33    // ; and :
#define KEY_QUOTE           0x34    // ' and "
#define KEY_THROW             0x35  // ~ and `
#define KEY_COMMA           0x36    // , and <
#define KEY_DOT             0x37    // . and >
#define KEY_QUESTION        0x38    // / and ?
#define KEY_CAPS_LOCK       0x39
#define KEY_F1              0x3A
#define KEY_F2              0x3B
#define KEY_F3              0x3C
#define KEY_F4              0x3D
#define KEY_F5              0x3E
#define KEY_F6              0x3F
#define KEY_F7              0x40
#define KEY_F8              0x41
#define KEY_F9              0x42
#define KEY_F10             0x43
#define KEY_F11             0x44
#define KEY_F12             0x45
#define KEY_PRT_SCR         0x46
#define KEY_SCOLL_LOCK      0x47
#define KEY_PAUSE           0x48
#define KEY_INS             0x49
#define KEY_HOME            0x4A
#define KEY_PAGEUP          0x4B
#define KEY_DEL             0x4C
#define KEY_END             0x4D
#define KEY_PAGEDOWN        0x4E
#define KEY_RIGHT_ARROW     0x4F
#define KEY_LEFT_ARROW      0x50
#define KEY_DOWN_ARROW      0x51
#define KEY_UP_ARROW        0x52
#define KEY_PAD_NUMLOCK     0x53
#define KEY_PAD_DIV         0x54    // /
#define KEY_PAD_MUL         0x55    // *
#define KEY_PAD_SUB         0x56    // -
#define KEY_PAD_ADD         0x57    // +
#define KEY_PAD_ENTER       0x58
#define KEY_PAD_1           0x59
#define KEY_PAD_2           0x5A
#define KEY_PAD_3           0x5B
#define KEY_PAD_4           0x5C
#define KEY_PAD_5           0x5D
#define KEY_PAD_6           0x5E
#define KEY_PAD_7           0x5F
#define KEY_PAD_8           0x60
#define KEY_PAD_9           0x61
#define KEY_PAD_0           0x62
#define KEY_PAD_DOT         0x63
#define KEY_PRESSED         0x00
#define KEY_RELEASED        0x01
//控制
/*
#define KEY_LCTRL           0x01    //左ctrl
#define KEY_LALT            0x04    //左Alt
#define KEY_LSHFIT          0x02    //左Shift
#define KEY_LWIN            0x08    //左windows
#define KEY_RWIN            0x80    //右windows
#define KEY_RSHIFT          0x20    //右Shift
#define KEY_RALT            0x40    //右Alt
#define KEY_RCTRL           0x10    //右Ctrl
#define KEY_APP             0x65    //Application
*/
#define KEY_LCTRL           0xE0    //左ctrl
#define KEY_LALT            0xE2    //左Alt
#define KEY_LSHFIT          0xE1    //左Shift
#define KEY_LWIN            0xE3    //左windows
#define KEY_RWIN            0xE7    //右windows
#define KEY_RSHIFT          0xE5    //右Shift
#define KEY_RALT            0xE6    //右Alt
#define KEY_RCTRL           0xE4    //右Ctrl
#define KEY_SET             0x65    //Application
#define KEY_K14             0x89        //international key
#define KEY_KR_L            0x91        //
#define KEY_K107            0x85        //
#define KEY_K45             0x64        //
#define KEY_K42             0x32        //
#define KEY_K131            0x8b        //
#define KEY_K132            0x8a        //
#define KEY_K133            0x88        //
#define KEY_K56             0x87        //
#define KEY_KR_R            0x90        //
//SELF DEFINE
#define KEY_H7              0XB0//H7          R0_C15
#define KEY_MAIL            0XB1//mail        R0_C16
#define KEY_H8              0XB2//H8          R0_C18
#define KEY_POWER           0XB3//POWER       R1_C0
#define KEY_H2              0XB4//H2          R1_C9
#define KEY_WWW_FAVORITE    0XB5//www favoriteR1_C17
#define KEY_H9              0XB6//H9          R1_C18
#define KEY_PAIR            0XB7//bind        R2_C0
#define KEY_FN              0XB8//FN          R2_C9
#define KEY_WWW_FORWARD     0XB9//www forward R2_C16
#define KEY_H10             0XBA//H10         R2_C18
#define KEY_SLEEP           0XBB//SLEEP       R3_C0
#define KEY_H1              0XBC//H1          R3_C7
#define KEY_VOLUME_SUB      0XBD//volume-     R3_C15
#define KEY_WWW_STOP        0XBE//www stop    R3_C16
#define KEY_MY_COMPUTER     0XBF//my computer R3_C17
#define KEY_H11             0XC0//H11         R3_C18
#define KEY_H3              0XC1//H3          R4_C9
#define KEY_PLAY_PAUSE      0XC2//play/pause  R4_C14
#define KEY_VOLUME_ADD      0XC3//volume+     R4_C15
#define KEY_WWW_BACK        0XC4//www back    R4_C16
#define KEY_STOP            0XC5//stop        R4_C17
#define KEY_H12             0XC6//H12         R4_C18
#define KEY_WAKE_UP         0XC7//WAKE UP     R5_C0
#define KEY_NEXT_TRACK      0XC8//next track  R5_C15
#define KEY_WWW_REFRESH     0XC9//www refresh R5_C16
#define KEY_CALCULATOR      0XCA//calculator  R5_C17
#define KEY_H13             0XCB//H13         R5_C18
#define KEY_H4              0XCC//H4          R6_C9
#define KEY_PREV_TRACK      0XCD//prev track  R6_C15
#define KEY_MUTE            0XCE//mute        R6_C16
#define KEY_WEB_HOME        0XCF//web home    R6_C17
#define KEY_H14             0XD0//H14         R6_C18
#define KEY_H5              0XD1//H5          R7_C11
#define KEY_H6              0XD2//H6          R7_C12
#define KEY_MEDIA           0XD3//media       R7_C15
#define KEY_WWW_SEARCH      0XD4//www search  R7_C16
#define KEY_BACKLIGHT       0XD5//BACKLIGHT   R7_C18
#define KEY_VOICE			0x3E//自己定义一个语音按键键值
#define END_NULL 0x00

#define KEY_POWER_U			0X7F
#define	KEY_VOL_ADD_U		0X80
#define	KEY_VOL_SUB_U		0X81
#endif

void key_init(void);
void key_state_reset(void);
void key_wakeup_config(void);
void key_wakeup_set(void);
void key_process(void);
app_key_type key_status_check(void);
unsigned char key_press_detect(void);
void start_key_timer(void);
void hid_send_keycode( void );
extern uint8_t detect_key_start;
extern volatile uint8_t uRF_TXTime;
#if	(KEYBOARD)

extern volatile eRC_TIMER_DUTY eTimerDuty;
extern volatile uint8_t uTimer2msCnt;
#endif
#ifdef __MOUSE__
void MK_scan();
extern void MK_on_keydown(uint8_t row, uint8_t col);
extern void MK_on_keyup(uint8_t row, uint8_t col);

void app_check_pair_and_mode_change(void);
void app_mouse_key_check(void);
void app_mouse_key_scan(void);
void app_mouse_wheel_scan(void);
void pair_key_check(void);
void gpio_sleep(void);
void app_timer0_1_int_cb(void);
void app_timer1_2_int_cb(void);
void driver_timer0_reset(void);
void Timer0_1m5_Start(uint8_t index,uint32_t cnt);
void driver_timer1_reset(void);
void driver_delay_set_with_base_tick(TIMER *timer, uint32_t timeout);
uint8_t driver_delay_with_base_tick_expire_check(TIMER *timer);
void Wait_Xms(uint32_t ms);
bool app_sensor_check(void);
#endif
#endif
