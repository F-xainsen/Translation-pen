#include "includes.h"

#define BatVoltage4_1	  437//457//874
#define BatVoltage4_0	  427
#define BatVoltage3_9	  416//443//832
#define BatVoltage3_83	  408
#define BatVoltage3_72	  397
#define BatVoltage3_7	  395//428//789
#define BatVoltage3_62	  386
#define BatVoltage3_54	  377
#define BatVoltage3_5	  373//410//746
#define BatVoltage3_2	  340//384//682

#define Flash_addr	0x10

unsigned char ChargeCont;
unsigned char xdata VOLSTATUS;
//unsigned char xdata ChargeStatus;
unsigned int xdata UpMouseCont;		//空鼠抬起显示计数器
//unsigned char xdata LowPowerCnt;	//低电量计数器
bit UpMouseFlag;	//1 = 拿起空鼠
bit ChargeFlag;		//1= 正在充电
bit CfullFlag;		//1 = 充满
bit LowpowerFlag;  	//1 = 低电量
bit ADCFunOK;	//1 = AD转换OK
bit QueryChargeFlag;	//1 = 开启充电状态查询
bit MouseDataFlag =1;		//1 = 开启鼠标数据上传
bit ZXDPowerONFlag;		//1 = 开启准心灯，当解码出来后，按键不能控制
bit TxCmdDataFlag;		//1 = 回传命令数据
bit ChangeRXFlag;		//1 = 改变成RX模式
bit LEDZSDFLAG;			//1 = 指示灯亮起

bit RF_WRITE_FLASH;		// 1 = 要把滚码写到flash中
bit RF_Usb_Reset = 0;
bit system_reset_flag = 0;
bit Pair_mode_check_flag;		// 1 = 要对码，0 = 不要对码
bit Clear_IDflash_flag;

bit Button_Down_flag = 0;

unsigned char RFdatabuf[2];
bit RXdataOKflag;		//1 = 接收数据成功
unsigned char xdata TxCmdData;	//回传的数据
unsigned char xdata RxCmdData;	//回传的数据
unsigned char xdata TxCmdDataCnt;	//回传的数据延时再发送
unsigned char xdata Time8MSCont;	//8MS的计数器
unsigned char xdata SendData;	//发送数据标志位
unsigned char xdata LowPowerCnt;	//低电量计数器
unsigned int xdata LowPowerDispCont;	
unsigned char xdata ChargeDispCont;	
unsigned char xdata UpMouseDispCont;	
unsigned short xdata ADValue;
unsigned char Time_10ms_ct = 0;
unsigned char Charge_disp_ct = 0;
unsigned int xdata Charge_full_ct = 0;
unsigned int  Cfull_disp_ct = 0;
unsigned char Button_Down_ct = 0;
unsigned char xdata CLEAR_DOWN_CT = 0;

unsigned int xdata Usb_Ctrl_NoDisp_R_ct = 0;	//从RF收到的数据给到这里
unsigned int xdata Usb_Ctrl_Disp_R_ct = 0;		//从RF收到的数据给到这里
unsigned int xdata Usb_Ctrl_NoDisp_B_ct = 0;	//从RF收到的数据给到这里
unsigned int xdata Usb_Ctrl_Disp_B_ct = 0;		//从RF收到的数据给到这里
unsigned int xdata Usb_Ctrl_NoDisp_G_ct = 0;	//从RF收到的数据给到这里
unsigned int xdata Usb_Ctrl_Disp_G_ct = 0;		//从RF收到的数据给到这里
unsigned int xdata Usb_Ctrl_NoDisp_ZX_ct = 0;	//从RF收到的数据给到这里
unsigned int xdata Usb_Ctrl_Disp_ZX_ct = 0;		//从RF收到的数据给到这里

unsigned int xdata Usb_Temp_R_ct = 0;
unsigned int xdata Usb_Temp_B_ct = 0;
unsigned int xdata Usb_Temp_G_ct = 0;
unsigned int xdata Usb_Temp_ZX_ct = 0;

//unsigned char xdata read_flash_data;
unsigned long xdata Usb_Set_Sleep_time = 0;			//代表USb端要控制的休眠时间

bit Usb_Change_flag = 0;
bit Usb_sleeptime_flag = 0;   // 1 = USB端重新设置了休眠的时间，这个时间要写入FLASH
bit Usb_Ctrl_EN_flag = 0;	// 1 = USB要控制显示，就不做其他的显示
bit Usb_led_ctrl_byte1_status = 0; // 记录USB控制LED灯的byte1 即有效位
//unsigned char BatStatus;
//**************************测试用的*********************/
bit First_power = 0;
//**************************测试用的*********************/
void ClearRFdata(void);
void ADCFunction(void);
void ADCVOLCmp(unsigned short  voltage)	;
void ButtonFun(void)
{	//按键按下，准心灯亮起，放开则灭
	if(IO_BUTTON == 0)
	{
		if(Button_Down_flag == 0)
			Button_Down_ct++;
		if(Button_Down_ct > 3)
		{
			Button_Down_ct = 0;
			IO_EN_LEDZX = 1;
			Usb_Ctrl_Disp_ZX_ct = 0;
			Button_Down_flag = 1;
		}
	}
	else
	{
		if(Button_Down_flag)
		{
			IO_EN_LEDZX = 0;
			Button_Down_flag = 0;
		}
	    Button_Down_ct = 0;
//		if(Usb_Ctrl_LedZX == 0)
//			IO_EN_LEDZX = 0;	
	} 	
}

//void Button_Disp(void)
//{
//	if(Usb_Ctrl_LedZX)
//	{
//		//IO_EN_LEDZX = 1;
//		return;
//	}
//	if(Button_Down_flag)
//		IO_EN_LEDZX = 1;
//	else
//		IO_EN_LEDZX = 0;
//}

void ChangeFun(void)
{	
	if(IO_CIN)	//充电插入
	{
		if(ChargeFlag == 0)	  	//之前未充电，则切换充电模式
		{
			ChargeCont++;
			if(ChargeCont > 20)		//消抖
			{
				ChargeCont = 0;
				ChargeFlag = 1;
				Time8MSCont = 0;
				LowpowerFlag = 0;	//充电状态下，一定不会有低电量
				IO_EN_LEDR = 0;
				LEDZSDFLAG = 0;	
				IO_EN_LEDZX = 0;
				RF_Usb_Reset = 0;			// USB的恢复初始化标志位。在充电的时候才会拉低
				//Usb_Ctrl_EN_flag = 0;	//测试用的

			}
		}
	}
	else			//无充电
	{
		if(ChargeFlag)
		{
			ChargeCont = 0;
			IO_EN_LEDG = 0;
		}
		ChargeFlag = 0;
		CfullFlag  = 0;
		UpMouseFlag = 0;
		UpMouseCont = 0;
		Cfull_disp_ct = 0;
	}

	if(ChargeFlag)		//在充电的状态下
	{
		if(IO_CFULL == 1)
		{
			Charge_full_ct++;
			if(Charge_full_ct > 2000)
			{
				Charge_full_ct = 0;
				CfullFlag = 1;
			}	
		}
	}
}
void ADCFunction(void)
{
	//ADC采样功能，加上电压比较功能，得出BatStatus的值,以及低电压标志位
	adc_set_ref(ADC_REF_VOL, 0);			//这是AD部分吗？
	{
		extern void application_adc_cel(unsigned char pin);
		application_adc_cel(1);  // 读取 p30  的ad值 	 这里的AD应该是电压的AD值 adCalib
//		printf("ADValue = %d\n", ADValue);

//	   	ADValue = BatVoltage3_2 ;

		ADCVOLCmp(ADValue);

	}
}
void ADCVOLCmp(unsigned short  voltage)			//AD采样的电压比较
{
	if(ChargeFlag == 0)					//在未充电的状态下，检测电池电压等级
	{
		//LowpowerFlag = 0; 
		if(voltage >= BatVoltage4_0)
		{
			VOLSTATUS = 5;
		}
		else if(voltage >= BatVoltage3_83)
		{
			VOLSTATUS = 4;
		}
		else if(voltage >= BatVoltage3_72)
		{
			VOLSTATUS = 3;
		}
		else if(voltage >= BatVoltage3_62)
		{
			VOLSTATUS = 2;
		}
		else if(voltage >= BatVoltage3_54)	
		{
			VOLSTATUS = 1;
			//LowpowerFlag = 1;				//低电量标志位，就要开始显示，清定时计数器

		}
		else if(voltage < BatVoltage3_54)	
		{
			LowpowerFlag = 1;				//低电量标志位，就要开始显示，清定时计数器

		}
		RFdatabuf[0] = VOLSTATUS;
		ADCFunOK = 1;
	}
//	else		 //在充电状态下
//	{
//		if(voltage >= BatVoltage4_1)
//		{
//			CfullFlag = 1;	
//		}	
//	}		
}

void System_to_sleep(void)
{
	RF_CHIP_DISABLE
	RF_POWER_DOWN			  //关RF芯片
	TR0 = 0;
	ET0 = 0;				  //关定时器

}
void Charge_Disp(void)		   //此函数10ms调用一次
{
	if(CfullFlag == 0)		   //在未充满的时候，做闪灯显示
	{
		if(ChargeFlag)		  	
		{
			Charge_disp_ct++;
			if(Charge_disp_ct <= 20)
				IO_EN_LEDG = 1;
			else if(Charge_disp_ct <= 100)
				IO_EN_LEDG = 0;
			if(Charge_disp_ct > 100)
				Charge_disp_ct = 0;
		}
		else
			Charge_disp_ct = 0;	
	}
	else	  	//在充满的时候，要做长亮显示1分钟，关显示，休眠		,还没能验证到
	{
		Cfull_disp_ct++;
		if(Cfull_disp_ct <= 6000)
			IO_EN_LEDG = 1;
		else
		{
			Cfull_disp_ct = 0;
			IO_EN_LEDG = 0;
			system_mode = system_sleep_mode;		//切换到休眠模式
		}	
	}
}
void LowPower_Disp(void)		//低电量显示函数，实现400MS的闪灭	 ,LED R
{
	if(LowpowerFlag)
	{
		LowPowerDispCont++;
		if(LowPowerDispCont < 40)
			IO_EN_LEDR = 1;
		else if(LowPowerDispCont < 440)
			IO_EN_LEDR = 0;
		if(LowPowerDispCont >440)
			LowPowerDispCont = 0;		
	}
}
void Usb_LedCtrl(void)			 	//接收到RF的数据，来控制显示的LED灯
{
	if(Usb_Ctrl_EN_flag == 0)		//当USB控制灯显示时，其他的显示都要退出，直到退出USB显示
		return;						//通过设置显示和熄灭时间，设置成0，就可以控制长亮和长灭
	if(Usb_Ctrl_Disp_R_ct != 0)
		Usb_Temp_R_ct++;
	else
		IO_EN_LEDR = 0;	
	if(Usb_Ctrl_Disp_B_ct != 0)
		Usb_Temp_B_ct++;
	else
		IO_EN_LEDB = 0;
	if(Usb_Ctrl_Disp_G_ct != 0)
		Usb_Temp_G_ct++;
	else
		IO_EN_LEDG = 0;
	if(Button_Down_flag == 0)
	{
		if(Usb_Ctrl_Disp_ZX_ct != 0)
			Usb_Temp_ZX_ct++;
		else
			IO_EN_LEDZX = 0;
	}

	if(Usb_Temp_R_ct < Usb_Ctrl_Disp_R_ct)
		IO_EN_LEDR = 1;
	else if(Usb_Temp_R_ct < (Usb_Ctrl_Disp_R_ct + Usb_Ctrl_NoDisp_R_ct))
		IO_EN_LEDR = 0;
	else
		Usb_Temp_R_ct = 0;

	if(Usb_Temp_B_ct < Usb_Ctrl_Disp_B_ct)
		IO_EN_LEDB = 1;
	else if(Usb_Temp_B_ct < (Usb_Ctrl_Disp_B_ct + Usb_Ctrl_NoDisp_B_ct))
		IO_EN_LEDB = 0;
	else
		Usb_Temp_B_ct = 0;

	if(Usb_Temp_G_ct < Usb_Ctrl_Disp_G_ct)
		IO_EN_LEDG = 1;
	else if(Usb_Temp_G_ct < (Usb_Ctrl_Disp_G_ct + Usb_Ctrl_NoDisp_G_ct))
		IO_EN_LEDG = 0;
	else
		Usb_Temp_G_ct = 0;	

	if(Button_Down_flag)
		return;
	if(Usb_Temp_ZX_ct < Usb_Ctrl_Disp_ZX_ct)
		IO_EN_LEDZX = 1;
	else if(Usb_Temp_ZX_ct < (Usb_Ctrl_Disp_ZX_ct + Usb_Ctrl_NoDisp_ZX_ct))
		IO_EN_LEDZX = 0;
	else
		Usb_Temp_ZX_ct = 0;	

}
void Save_sleeptime(void)	  //掉电记忆，只记忆休眠的时间 
{
	unsigned char xdata i,write_flash_data[10] = 0;
	if(Usb_sleeptime_flag == 0  && RF_WRITE_FLASH == 0 && Clear_IDflash_flag == 0)						   //只有在USB重新设置休眠时间时，才会重新写FLASH
		return;
	//if(Usb_sleeptime_flag)
	{
		Usb_sleeptime_flag = 0;
		write_flash_data[0] = Usb_Set_Sleep_time;		//低8位
		write_flash_data[1] = Usb_Set_Sleep_time >> 8;			//buff是8位的，数据只需要移位处理
		write_flash_data[2] = Usb_Set_Sleep_time >> 16;
	    	write_flash_data[3] = Usb_Set_Sleep_time >> 24;			//数据是long型，4byte，位移拆分
	}
	if(Clear_IDflash_flag)
	{
		Clear_IDflash_flag = 0;
		Pair_mode_check_flag = 1;			// 清除之后，就可以进入对码模式
		system_mode = system_pair_mode;	
	}
	else if(RF_WRITE_FLASH || (Pair_mode_check_flag == 0))
	{
		RF_WRITE_FLASH = 0;
		Pair_mode_check_flag = 0;		// 为了在写休眠时间的记忆时，也把滚码写上
		write_flash_data[4] = 0x33;
		write_flash_data[5] = RX0_Address[0];
		write_flash_data[6] = RX0_Address[1];
		write_flash_data[7] = RX0_Address[2];
		write_flash_data[8] = RX0_Address[3];
		write_flash_data[9] = RX0_Address[4];
		//IO_EN_LEDZX = ~IO_EN_LEDZX;
	}
	mcu_erase_flash(0);	 		//先擦除FLASH，才能写入
	mcu_write_flash(Flash_addr+0, 0x55);
	mcu_write_flash(Flash_addr+1, 0xAA);
	for(i=2;i<12;i++)
	{
		mcu_write_flash(Flash_addr +i,write_flash_data[i-2]);	
	}
	/*
	mcu_write_flash(Flash_addr+2, write_flash_data[0]);
	mcu_write_flash(Flash_addr+3, write_flash_data[1]);
	mcu_write_flash(Flash_addr+4, write_flash_data[2]);
	mcu_write_flash(Flash_addr+5, write_flash_data[3]);
	*/
	//IO_EN_LEDR = ~IO_EN_LEDR;
}

void Read_sleeptime(void)
{
	unsigned char xdata i,read_flash_data[10] = 0;
//	unsigned long xdata temp_long_ct;
	unsigned char xdata Check_read_data[2] = 0;
	Check_read_data[0] = mcu_read_flash(Flash_addr+0);	 	//上电读一次
	Check_read_data[1] = mcu_read_flash(Flash_addr+1);	 	//上电读一次
	for(i=2;i<12;i++)
		read_flash_data[i-2] = mcu_read_flash(Flash_addr + i);		// 一次把数据读完
	/*
	read_flash_data[0] = mcu_read_flash(Flash_addr+2);	 	//上电读一次
	read_flash_data[1] = mcu_read_flash(Flash_addr+3);	 	//上电读一次
	read_flash_data[2] = mcu_read_flash(Flash_addr+4);	 	//上电读一次
	read_flash_data[3] = mcu_read_flash(Flash_addr+5);	 	//上电读一次
	*/
	//读出数据后，做数据处理
	if(Check_read_data[0] == 0x55 && Check_read_data[1] == 0xAA)
	{
		for(i=0;i<3;i++)
		{
			Usb_Set_Sleep_time |= read_flash_data[3-i];
			Usb_Set_Sleep_time <<= 8;	
		}
		Usb_Set_Sleep_time |= read_flash_data[0];
	}
	else
	{
		Usb_Set_Sleep_time =  180000;		//第一次上电，默认为3分钟，180000MS
	}
	if(read_flash_data[4] == 0x33)
	{
		// 表示有记忆滚码，上电就不需要对码
		for(i=0;i<5;i++)
			RX0_Address[i] = read_flash_data[5+i];			// 读取出滚码数据，记录，给到地址
		system_mode = system_normal_mode;			// 有读到滚码，唤醒之后就不要进行正常模式
		Pair_mode_check_flag = 0;
	}
	else
	{
		system_mode = system_pair_mode;				// 如果没读到滚码，就要进入对码模式
		Pair_mode_check_flag = 1;
	}
}
//**************************测试用的*********************/
void Test_temp(void)
{
	if(First_power == 0)
	{
		First_power = 1;
//		Usb_Ctrl_Disp_R_ct = 1000;
//		Usb_Ctrl_NoDisp_R_ct = 0;
//		Usb_Ctrl_Disp_G_ct = 1000;
//		Usb_Ctrl_NoDisp_G_ct = 1500;
//		Usb_Ctrl_Disp_B_ct = 1500;
//		Usb_Ctrl_NoDisp_B_ct = 2000;
//		Usb_Ctrl_Disp_ZX_ct = 2000;
//		Usb_Ctrl_NoDisp_ZX_ct = 0;
//		Usb_Ctrl_EN_flag = 1;
//		Usb_Set_Sleep_time = 40000;
		//LowpowerFlag = 1;	   		//上电如果低电量，会影响时钟
	}
}
//**************************测试用的*********************/

void Clear_IDflash_key(void)			// 用来清除flash 中的滚码记忆功能
{
	// 放在 10ms 的时基调用
	if(IO_CLRAE == 0)
	{
		CLEAR_DOWN_CT++;
		if(CLEAR_DOWN_CT > 4)
		{
			CLEAR_DOWN_CT = 0;
			Clear_IDflash_flag = 1;			// 清除flash 的滚码数据
		}
	}
	else
		CLEAR_DOWN_CT = 0;
}
void Usb_reset_fun(void)
{
	if(RF_Usb_Reset && system_reset_flag)			// 在有复位的标志位下，才会运行 ，只运行一次
	{
		system_reset_flag = 0;
		IO_EN_LEDZX = ~IO_EN_LEDZX;
		Usb_Set_Sleep_time = 180000;		// 复位状态下，休眠时间回到默认值
		Pair_mode_check_flag = 1;			// 清除之后，清除记忆功能
		mcu_erase_flash(0);	 		// 擦除FLASH，把所有的记忆清除
		system_mode = system_sleep_mode;	// 进入休眠模式
	}
	
}
void Check_1ms_fun(void)		//这个是1MS调用一次
{
	if(Time_1ms_flag == 0)
		return;
	Time_1ms_flag = 0;
	//ChangeFun();   		//得到	ChargeFlag 和 CfullFlag 标志位
	if(Usb_led_ctrl_byte1_status == 0)
	{
		if(ChargeFlag)					//判断充电的状态
			Usb_Ctrl_EN_flag = 0;
		else
			Usb_Ctrl_EN_flag = 1;		//没有充电的状态，就显示USB控制的数据
	}
	if(LowpowerFlag == 0)	//显示的优先级为：低电量显示 》usb控制显示 》 充电显示
		Usb_LedCtrl();		//USB给到的数据是ms的单位，就放到这里来调用USB控制的闪灯
	Test_temp();
	//P3IN_EN = P3_IOSEL_CFG ;  // p0  设置为输入上拉状态
	//P3OUT_EN = P3_IOSEL_CFG ;
	Time_10ms_ct++;
	if(Time_10ms_ct >= 10)
	{
		Time_10ms_ct = 0;
		ChangeFun();   		//得到	ChargeFlag 和 CfullFlag 标志位
		Usb_reset_fun();
		Clear_IDflash_key();
		ButtonFun();		//按键按下，准心灯亮起，放开则灭；如果有上位机控制亮起，就要一直长亮，直到按键按下后放开才会灭。
		Save_sleeptime();	//在这里做保存flash数据的
		if(Usb_Ctrl_EN_flag == 0 && LowpowerFlag == 0 )	//如果在USB控制灯的状态下，优先USB的控制，充电显示不能工作	
			Charge_Disp();			//验证OK 充电显示
		LowPower_Disp();		//低电量显示
	}
}
