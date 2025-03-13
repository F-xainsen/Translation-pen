#ifndef _Function_H_
#define _Function_H_

#define LEDON  	1
#define LEDOFF	0

//#define LEDZX_HIGH  LED_ZS = LEDON	   	//准心灯的指示
//#define LEDZX_LOW  	LED_ZS = LEDOFF
//
//#define LEDZS_HIGH 	LED_ZS = LEDOFF		//指示灯的指示
//#define LEDZS_LOW  	LED_ZS = LEDON

extern bit RXdataOKflag;		//1 = 接收数据成功
extern unsigned char xdata VOLSTATUS;
extern bit LowpowerFlag;  	//1 = 低电量
extern bit ADCFunOK;	//1 = AD转换OK
extern bit UpMouseFlag;	//1 = 拿起空鼠
extern bit CfullFlag;		//1 = 充满
extern bit ChargeFlag;		//1= 正在充电
extern bit ZXDPowerONFlag;		//1 = 开启准心灯，当解码出来后，按键不能控制
extern bit TxCmdDataFlag;		//1 = 回传命令数据
extern bit ChangeRXFlag;		//1 = 改变成RX模式
extern bit LEDZSDFLAG;			//1 = 指示灯亮起
extern bit RF_WRITE_FLASH;
extern bit Pair_mode_check_flag;

extern bit Usb_Ctrl_EN_flag;	// 1 = USB要控制显示，就不做其他的显示
extern bit Usb_led_ctrl_byte1_status;
extern bit Usb_sleeptime_flag;
extern bit RF_Usb_Reset;
extern bit system_reset_flag;

extern unsigned char xdata TxCmdData;	//回传的数据
extern unsigned char xdata RxCmdData;	//回传的数据
extern unsigned char xdata TxCmdDataCnt;	//回传的数据延时再发送
extern unsigned char xdata Time8MSCont;	//8MS的计数器
extern unsigned char xdata SendData;	//发送数据标志位
extern unsigned int xdata UpMouseCont;		//空鼠抬起显示计数器
extern unsigned short xdata ADValue;
extern unsigned int xdata LowPowerDispCont;	
extern unsigned char xdata ChargeDispCont;	
extern unsigned char xdata UpMouseDispCont;
extern unsigned char RFdatabuf[2];
extern unsigned long xdata Usb_Set_Sleep_time;			//代表USb端要控制的休眠时间

extern unsigned int xdata Usb_Ctrl_NoDisp_R_ct;	//从RF收到的数据给到这里
extern unsigned int xdata Usb_Ctrl_Disp_R_ct;		//从RF收到的数据给到这里
extern unsigned int xdata Usb_Ctrl_NoDisp_B_ct;	//从RF收到的数据给到这里
extern unsigned int xdata Usb_Ctrl_Disp_B_ct;		//从RF收到的数据给到这里
extern unsigned int xdata Usb_Ctrl_NoDisp_G_ct;	//从RF收到的数据给到这里
extern unsigned int xdata Usb_Ctrl_Disp_G_ct;		//从RF收到的数据给到这里
extern unsigned int xdata Usb_Ctrl_NoDisp_ZX_ct;	//从RF收到的数据给到这里
extern unsigned int xdata Usb_Ctrl_Disp_ZX_ct;		//从RF收到的数据给到这里

extern void ButtonFun(void);
//extern void LEDDisplay(void);
extern void ADCFunction(void);
extern void ChangeFun(void);
extern void Decode(void);	 //RF数据接收解码处理  读取一次后，就把数据清了
extern void system_mode_pair(void);
extern void Check_1ms_fun(void);
extern void Read_sleeptime(void);
#endif