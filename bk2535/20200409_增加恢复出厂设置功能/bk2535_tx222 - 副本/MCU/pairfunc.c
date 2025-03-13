#include "includes.h"

XDATA UINT8 FIFO_data2[32];
UINT8 XDATA Pair_to_sleep = 0;
bit Pair_to_sleep_int = 0;
UINT8 XDATA Time_3min_cont = 0;
bit Pair_OK_Flag = 0;
bit Ctrl_rx_pair_flag = 0;		//1 = 发送数据给RX，控制RX进行对码
UINT8 XDATA Normal_to_pair_cont = 0;
void rf_data_read(BYTE bytes);

void system_mode_pair(void)	 //对码连接的状态
{
	UINT8 buf[5]={0x11, 0x22, 0x33, 0x44, 0x55},status,len,i;
	RF_CMD_FLUSH_TX;                                                //fifo clear
    RF_CMD_FLUSH_RX;
    RF_POWER_DOWN;
    RF_POWER_UP;
    RF_STATUS_CLEAR_ALL;
	BK2401_RFCH=65;
    memcpy ( BK2401_R0_ADDR, buf, 5 );
	memcpy ( BK2401_TX_ADDR, buf, 5 );
	SwitchToRxMode();
	while(system_mode == system_pair_mode)
	//if(system_mode == system_pair_mode)
	{
		//P00 = 0;
		//IO_EN_LEDZX = 1;
		status = BK2401_STATUS; 
		if(status & STATUS_RX_DR)	
		{
			len = BK2401_RPL_WIDTH; 
			rf_data_read(len);
			RF_CMD_FLUSH_TX;
			RF_CMD_FLUSH_RX;
	        BK2401_STATUS = status;
			if((len==6)&&(FIFO_data2[5]==0xcc))
			{
				RX0_Address[0]=FIFO_data2[0];
				RX0_Address[1]=FIFO_data2[1];
				RX0_Address[2]=FIFO_data2[2];
				RX0_Address[3]=FIFO_data2[3];
				RX0_Address[4]=FIFO_data2[4];
				Pair_OK_Flag = 1;
			}
		}
		if(Pair_to_sleep_int)
		{
			Pair_to_sleep_int = 0;			//100ms的定时进入一次
			Time_3min_cont++;
			if(Time_3min_cont >= 10)
			{
				Time_3min_cont = 0;
				ADC_EN = 1 ;
				ADCFunction();
				ADC_EN = 0 ;
//				if(IO_CIN)
//					Pair_OK_Flag = 1;
//				else
//					system_mode = system_sleep_mode;	   //定时5秒，对码不上，就休眠
			}
		}
		if(Pair_OK_Flag)
		{
			Pair_OK_Flag = 0;
			//P00 = 1;
			memcpy ( BK2401_R0_ADDR, RX0_Address, 5 );
			memcpy ( BK2401_TX_ADDR, RX0_Address, 5 );
			for(i=0;i<5;i++)
			{
				printf("RX0_Address = %d\n", RX0_Address[i]);
			}
			SwitchToTxMode();
			system_mode = system_normal_mode;
			//IO_EN_LEDZX = 0;
		}
	}
}
void rf_data_read(BYTE bytes)
{
    UINT8 i;
    
    memset(FIFO_data2, 0, 32);
    RF_CMD_R_RX_PAYLOAD;
    for (i=0; i<bytes; i++)
		FIFO_data2[i] = BK2401_FIFO; 	  // data
    RF_CMD_NOP;
}

void Normal_to_pair(void)		  //在正常模式下，如果断开连接，要切换成对码模式
{
//	if(ChargeFlag)
//		Normal_to_pair_cont= 0;

	if(Normal_to_pair_cont > 80)
	{
		Normal_to_pair_cont = 0 ;
		system_mode = system_pair_mode;
	}	
}