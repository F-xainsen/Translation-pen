#include "includes.h"

UINT8 XDATA Mouse_X_last = 0;
UINT8 XDATA Mouse_Y_last = 0;
bit Mouse_move_flag = 0;		//1 = 空鼠移动
bit normal_to_sleep_int = 0;
INT32 XDATA narmal_sleep_cont = 0;	  //用来休眠计数


void system_mode_sleep(void)
{
	//休眠处理函数 system_mode = system_sleep_mode
	ADC_EN = 0;
	//LEDZX_LOW;
	//LEDZS_LOW;

	IO_EN_LEDB = 0;
	IO_EN_LEDG = 0;
	IO_EN_LEDR = 0;
	IO_EN_LEDZX = 0;
	Usb_Ctrl_Disp_R_ct = 0;
	Usb_Ctrl_NoDisp_R_ct = 0;
	Usb_Ctrl_Disp_B_ct = 0;
	Usb_Ctrl_NoDisp_B_ct = 0;
	Usb_Ctrl_Disp_G_ct = 0;
	Usb_Ctrl_NoDisp_G_ct = 0;
	Usb_Ctrl_Disp_ZX_ct = 0;
	Usb_Ctrl_NoDisp_ZX_ct = 0;

	CfullFlag = 0;
	Usb_Ctrl_EN_flag = 0;
	narmal_sleep_cont = 0;
	LEDZSDFLAG = 0;
	ZXDPowerONFlag = 0;
	BMI160_Sleep();		//这里应该还要加上陀螺仪的休眠处理
	PowerDown_RF();		//关RF
	ET0 = 0;
	TR0 = 0;
	CLK_EN_CFG = 0;
	P1_WUEN = 0x03;		//P11设置成唤醒引脚
	//P3_WUEN	= 0x04;
//	EA = 1;
	EX6=1;			  //设置GPIO 唤醒
	EXSLEEP = 1;		//supper sleep mode		这个好像是更低功耗的休眠模式
	while(1)
	{
		MCU_IDLE_RC32KHZ;  //设置成32K，和IDLE（= sleep指令）
		if(RF_Usb_Reset == 0)
		{
			if(IO_BUTTON == 0)			// 在USB恢复的状态下，只有充电才能唤醒
				break;
		}
		if(IO_CIN)
			break;
	}
	//唤醒后操作
//	P37 = 1; 
	_nop_();
	_nop_();
	_nop_();
	_nop_();
   	PCON2 &= ~0x02;
	EXSLEEP = 0;		//supper sleep mode		这个好像是更低功耗的休眠模式
	P1_WUEN = 0x00;		//P12设置成唤醒引脚
	P3_WUEN	= 0x00;
	CLK_EN_CFG = 1;
	EX6=0;				//关GPIO唤醒中断
//	TR0 = 1;
	timer0_inital();	//重新初始化定时器
	ET0 = 1;							//enable Timer0 int
//	BMI160_Init();
	up_power000 = 1;
	PowerUp_RF();	 //开RF
	if(Pair_mode_check_flag)
		system_mode = system_pair_mode;		//唤醒后，就进入对码模式
	else
		system_mode = system_normal_mode;	// 要是有滚码记忆，唤醒之后就要进入正常的发射模式
}

void Normal_to_sleep(void)
{
	//在正常工作模式下调用
	//if((system.mouse[1] != Mouse_X_last) || system.mouse[2] != Mouse_Y_last)
//	if((system.mouse[1] != 0) || system.mouse[2] != 0 || system.mouse[0] != 0 || system.mouse[3] != 0 )  
//	{
//		//Mouse_X_last = system.mouse[1];		 //如果空鼠数据产生变化，就要把变化后的值，给记录下来，作为下次判断空鼠是否移动的条件
//		//Mouse_Y_last = system.mouse[2];
//	   	Mouse_move_flag = 1;
//		narmal_sleep_cont = 0;
//	}
//	else
//	{
//		//空鼠不移动，就要计时，用做3分钟不动做，就进入休眠模式
//		Mouse_move_flag = 0; 				//这个标志位用来给计数器计数
//		//narmal_sleep_cont = 0;
//	}
	if(ChargeFlag)
		narmal_sleep_cont = 0;	
	if(normal_to_sleep_int)		   		//现在改成是1ms的定时
	{
		normal_to_sleep_int = 0;	  //1MS的定时，中断拉高
		if(Mouse_move_flag == 0)
		{
			if(Usb_Set_Sleep_time == 0)					//这个时间计数器为0时，表示不需要休眠功能
			{
				narmal_sleep_cont = 0;
				return;
			}	
			narmal_sleep_cont++;
			if(narmal_sleep_cont >= Usb_Set_Sleep_time) //	 Usb_Set_Sleep_time 这寄存器代表的是USB要设置的休眠时间
			{
				narmal_sleep_cont = 0;
				//Ctrl_rx_pair_flag = 1;		//控制RX进入对码状态
				system_mode = system_sleep_mode;		//切换到休眠模式
			//	break;
			}
		}
		else
		{
			narmal_sleep_cont = 0;
		}
	}
}