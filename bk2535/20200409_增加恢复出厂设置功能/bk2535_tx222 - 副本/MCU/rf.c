#include "includes.h"

void RF_Read_data(void)	;
BYTE BK2401_FIFO_data[10];


void rf_isr (void) interrupt 11	 //rf interrupt
{
	AIF &= ~0x08;
}

unsigned	int	R_nusd = 10;
extern unsigned char tran_inv;
void rf_send_data(void)
{
	if(ADCFunOK)
	{
		ADCFunOK = 0;
		RFdatabuf[0] = RFdatabuf[0] << 2;
	}
	if(ChargeFlag)			   //充电的标志，用BIT7表示
	{
		RFdatabuf[0] = RFdatabuf[0] | 0x01;		//有充电的时候，拉高bit0
//		P36 ^= 1;
	}
	if(up_power000==0)
	{
		RFdatabuf[0] = RFdatabuf[0] | 0x02;		
	}
	BK2401_FIFO = maincount	;
		
	BK2401_FIFO = system.mouse[0];		//鼠标数据		
	BK2401_FIFO = system.mouse[1];
	BK2401_FIFO = system.mouse[2];
	BK2401_FIFO = system.mouse[3]; 
	BK2401_FIFO = RFdatabuf[0];

	if((system.mouse[1] != 0) || system.mouse[2] != 0 || system.mouse[0] != 0 || system.mouse[3] != 0 )
		Mouse_move_flag = 1;
	else
		Mouse_move_flag = 0;
			
	system.mouse[0]=0;
	system.mouse[1]=0;
	system.mouse[2]=0;
	system.mouse[3]=0;
//	RFdatabuf[0] = 0;
	/*

	BK2401_FIFO = maincount	;
	
	
	BK2401_FIFO = BK2401_FIFO_data[0];				
	BK2401_FIFO = BK2401_FIFO_data[1];
	BK2401_FIFO = BK2401_FIFO_data[2]; 
	BK2401_FIFO = BK2401_FIFO_data[3];				
	BK2401_FIFO = BK2401_FIFO_data[4];
	BK2401_FIFO = BK2401_FIFO_data[5];
	BK2401_FIFO = BK2401_FIFO_data[6]; 
	
	BK2401_FIFO = BK2401_FIFO_data[7];				
	BK2401_FIFO = BK2401_FIFO_data[8];
	BK2401_FIFO = BK2401_FIFO_data[9];
	BK2401_FIFO = BK2401_FIFO_data[10]; 
	
	BK2401_FIFO = BK2401_FIFO_data[11];				
	BK2401_FIFO = BK2401_FIFO_data[12];	
	*/
}

//-------------------------------------------------------
UINT8 Send_Packet_Ack(UINT8 *buf,UINT8 length)
{
	UINT8 sta,i;
	
    RF_CMD_FLUSH_TX                                                 //fifo clear
    RF_CMD_FLUSH_RX
    RF_POWER_DOWN
    RF_POWER_UP
    RF_STATUS_CLEAR_ALL
  
	delay_us(50);
	
	//BK2401_RETR = 0x14;
	
	RF_CMD_W_TX_PAYLOAD
    for (i=0; i<length; i++)
        BK2401_FIFO = buf[i];
    	
    RF_CMD_NOP  ;     
        
    //sleep and wake up by rf interrupt
    ENABLE_RF_INT;
    MCU_IDLE_OSCXMHZ;
    DISABLE_RF_INT;
	do
	{
		sta = BK2401_STATUS;
	}while((sta & STATUS_RX_TX_MAX) == 0x00);																	//清中断标志						 
	
	if(sta & STATUS_TX_DS)
	{					
		return STATUS_TX_DS;
	}
	else
    if(sta & STATUS_MAX_RT)
	{
		return STATUS_MAX_RT;
	}
	else
	{
			printf(" 2  ");
        return STATUS_MAX_RT;

	}
}




void process_rf_communication(void)
{
	unsigned char sta;

		CE_LOW();
		sta = BK2401_STATUS;
	    //printf(" 333\n");
//		printf("%d",sta);
		while(sta & 0x71)
		{
			if(sta&STATUS_RX_DR) 		 //收到数据
			{
				if(ChangeRXFlag)		//接收模式
				{
					BK2401_CMD = 0x40;	//read fifo command	
					_nop_(); _nop_();
					RxCmdData = BK2401_FIFO;	//只用一个byte来表示命令  这命令只控制灯显示
					BK2401_CMD = 0x00;
					RXdataOKflag = 1;
//					P36  ^=1;
				}
				FLUSH_RX;
				BK2401_STATUS = STATUS_RX_DR;
				/************************************/
				PowerDown_RF();
				PowerUp_RF();						  //从2433上面学到的控制操作
				BK2401_STATUS = 0x70;
				delay_us(200);
				SwitchToTxMode();
				/************************************/
				max_trans_ct = 0;
			} 
			if(sta&STATUS_TX_DS) 					//发射成功
			{
				if(ChangeRXFlag == 0)		//只有在不是接收模式的时候，才会切换模式
				{
					BK2401_STATUS = 0x70;
					FLUSH_TX;
					CE_LOW();
					max_trans_ct = 0;
					PowerDown_RF();
					PowerUp_RF();
					delay_us(200);
					SwitchToRxMode();		//发射转成接收模式											 
					ChangeRXFlag = 1;	
				}
				FLUSH_TX;
				BK2401_STATUS = STATUS_TX_DS;
				max_trans_ct = 0;
				//printf("ok\n");
			}
			if(sta&STATUS_MAX_RT) 
			{
//				P36 ^= 1;
				FLUSH_TX;
				FLUSH_RX;
				BK2401_CONFIG=BK2401_CONFIG&0xfd;  //rf power down
				BK2401_CONFIG=BK2401_CONFIG|0x02;  //rf power up
				BK2401_STATUS = 0x70;
				delay_us(10);
//				FLUSH_TX;
				BK2401_STATUS = STATUS_MAX_RT ;
				
				if(tran_inv<100)
					tran_inv ++;
			//	printf(" 2");
		//		if(max_trans_ct < 100)
		//			max_trans_ct++;
			}
		//	FLUSH_TX;
			sta = BK2401_STATUS;
		//	printf(" 333\n");
		//	printf("%d",sta);
		}
	
//		PowerUp_RF();
	   // set_rf_5byte_address(0);
	//	BK2401_RFCH = rf_channel;
	//	BK2401_RFCH = T_JumpFreqTab[rf_freq_id]; 
//		BK2401_SETUP  = RF_BAND|RF_5dbm|RF_LNA_GAIN_HIGH;	//2mbps/1mbps/250kbps, 5dbm
//	    SwitchToTxMode();

//		CE_LOW();
//		BK2401_CMD = 0x60;	//write fifo command
//		//BK2401_CMD = 0x68;	//write fifo command  no_ack
//		delay_us(10);
//		rf_send_data();  // 把需要发送的数据写入 fifo
//		
//		BK2401_CMD = 0x00;		   //2455 - 130us, 2533 - 120us, 2535 - 80us
//	
//		CE_HIGH();
	
//		delay_ms(1);
	//	rf_clrint();
}

void RF_id_set(void)
{
	BYTE id[5];
	PowerDown_RF();
    id[0] = 0;
    id[1] = 1;
	id[2] = 2;
	id[3] = 3;
	id[4] = 100;
	memcpy(BK2401_R0_ADDR,id,5);
	memcpy(BK2401_R1_ADDR,id,5);
	memcpy(BK2401_TX_ADDR,id,5);  
	PowerUp_RF();
}
void RF_Data_Send(void)
{
	UINT8 sta,i;
	RF_CMD_FLUSH_TX                                                 //fifo clear
    RF_CMD_FLUSH_RX
    RF_POWER_DOWN
    RF_POWER_UP
    RF_STATUS_CLEAR_ALL
	delay_us(50);
	BK2401_RETR = 0x13;
	RF_CMD_W_TX_PAYLOAD	//write fifo command
	delay_us(10);
	rf_send_data();  // 把需要发送的数据写入 fifo
	BK2401_CMD = 0x00;
	do
	{
		sta = BK2401_STATUS;
	}while((sta & STATUS_RX_TX_MAX) == 0x00);		//清中断标志

	//BK2401_CMD = 0x00;		   //2455 - 130us, 2533 - 120us, 2535 - 80us
	if(sta & STATUS_TX_DS)
	{	
		RF_CMD_FLUSH_TX                                                 //fifo clear
        RF_CMD_FLUSH_RX
	    RF_STATUS_CLEAR_ALL;
		SwitchToRxMode();
		delay_us(10);
		i=100;
		while(i--)
		{
			delay_us(20);
			sta = BK2401_STATUS;
			if(sta & STATUS_RX_DR)
			{
				Normal_to_pair_cont = 0;
				RF_Read_data();
//				len = BK2401_RPL_WIDTH;    		
//				if(len==2)
//				{   //DEG(("===%X\n",(uint16)(FIFO_data[0])));
//					RF_CMD_R_RX_PAYLOAD
//					rf_data1 = BK2401_FIFO;
//					rf_data2 = BK2401_FIFO;
//					RF_CMD_NOP;
//					rf_data_cmd = rf_data1 & 0xf0;
//					if(rf_data_cmd == 0xF0)
//					{
//						rf_data = rf_data2 & 0x01;
//						if(rf_data  == 0x01)
//							ZXDPowerONFlag = 1; 
//						else 
//							ZXDPowerONFlag = 0;
//					}
//					rf_data_cmd = rf_data1 & 0x0f;
//					if(rf_data_cmd == 0x0f)
//					{	
//						if(LowpowerFlag)		  //低电量模式下，不操作
//							break;
//						rf_data = rf_data2 & 0x02;						 
//						if(rf_data  == 0x02)
//						{
//						 	if(ChargeFlag)
//							{
//								if(UpMouseFlag == 0)
//								{
//									UpMouseFlag = 1;
//									UpMouseCont = 0;
//									Time8MSCont = 0;
//									LEDZSDFLAG = 0;
//								}
//							}
//							else
//								LEDZSDFLAG = 1;		//点亮指示灯
//						}	
//						else   //收到00数据
//						{
//							if(UpMouseFlag==1 || ChargeFlag==1)		//在抬起显示时，收到命令不操作
//							{
//								break;	
//							}
//							LEDZSDFLAG = 0;
//						}
//					}
//					break;		
//				}	   
			}
			BK2401_STATUS = 0;
			
		}
		RF_CMD_FLUSH_TX;
		RF_CMD_FLUSH_RX;
		SwitchToTxMode();
		//return STATUS_TX_DS;
		BK2401_STATUS = STATUS_TX_DS;
	}
	if(sta & STATUS_MAX_RT)	
	{
		tran_inv = 1;
		BK2401_STATUS = STATUS_MAX_RT;
		if(Pair_mode_check_flag)
		{
			Normal_to_pair_cont++;
			Normal_to_pair();
		}
	}
}

void RF_Read_data(void)		//收到RF的数据 ，在这里做数据的处理，转换成显示，闪灯时间，或者是休眠时间
{
	UINT8 xdata rf_len,i,rf_data_buff[6];

	rf_len = BK2401_RPL_WIDTH;
	RF_CMD_R_RX_PAYLOAD;
	for(i=0;i<rf_len;i++)
		rf_data_buff[i] = BK2401_FIFO;				//读出RF传输的数据
	RF_CMD_NOP;
	if(rf_len == 2)			// 这个是控制写flash的数据
	{
		if(rf_data_buff[0] == 0x56 && rf_data_buff[1] == 0x78)
		{
			RF_WRITE_FLASH = 1;			// 此标志位为PC 控制发射端记忆滚码
		}
	}
	if(rf_len == 3)
	{
			if(rf_data_buff[0] == 0x12 && rf_data_buff[1] == 0x34 && rf_data_buff[2] == 0x88)
			{
				if(!RF_Usb_Reset)
				{
					RF_Usb_Reset = 1;
					system_reset_flag = 1;
				}
			}
	}
	if(rf_len == 4)			 //这个表示是休眠时间的数据
	{
		Usb_Set_Sleep_time = 0;
		Usb_sleeptime_flag = 1;
		narmal_sleep_cont = 0;
		for(i=0;i<3;i++)
		{
			Usb_Set_Sleep_time |= rf_data_buff[i];
			Usb_Set_Sleep_time <<= 8;	
		}
		Usb_Set_Sleep_time |= rf_data_buff[3]; 			//RF的数据要结合起来，给到要设置的休眠时间
//		if(rf_data_buff[0] == 0 && rf_data_buff[1]==0 && rf_data_buff[2] ==0x11 && rf_data_buff[3] == 0xaa)
//			IO_EN_LEDB = ~IO_EN_LEDB;
//		if(Usb_Set_Sleep_time == 0x11aa)
//			IO_EN_LEDG = ~IO_EN_LEDG;
	}
	if(rf_len == 6)				//这里是读到LED控制灯
	{
		//IO_EN_LEDG = ~IO_EN_LEDG;
		if(rf_data_buff[0] == 0)	   	//读到0，就是代表着USB控制放开
		{
			IO_EN_LEDB = 0;
			IO_EN_LEDG = 0;
			IO_EN_LEDR = 0;
			IO_EN_LEDZX = 0;
			//Usb_Ctrl_EN_flag = 0;
			Usb_led_ctrl_byte1_status = 0;
			return;
		}
		else
		{
			Usb_Ctrl_EN_flag = 1;			 //有控制，就一定要显示
			Usb_led_ctrl_byte1_status = 1;
			switch(rf_data_buff[1])
			{
				case 0:
					Usb_Ctrl_Disp_R_ct = rf_data_buff[2];
					Usb_Ctrl_Disp_R_ct <<= 8;
					Usb_Ctrl_Disp_R_ct |= rf_data_buff[3];

					Usb_Ctrl_NoDisp_R_ct = rf_data_buff[4];
					Usb_Ctrl_NoDisp_R_ct <<= 8;
					Usb_Ctrl_NoDisp_R_ct |= rf_data_buff[5];
					break;
				case 1:
					Usb_Ctrl_Disp_G_ct = rf_data_buff[2];
					Usb_Ctrl_Disp_G_ct <<= 8;
					Usb_Ctrl_Disp_G_ct |= rf_data_buff[3];

					Usb_Ctrl_NoDisp_G_ct = rf_data_buff[4];
					Usb_Ctrl_NoDisp_G_ct <<= 8;
					Usb_Ctrl_NoDisp_G_ct |= rf_data_buff[5];
					break;
				case 2:
					Usb_Ctrl_Disp_B_ct = rf_data_buff[2];
					Usb_Ctrl_Disp_B_ct <<= 8;
					Usb_Ctrl_Disp_B_ct |= rf_data_buff[3];

					Usb_Ctrl_NoDisp_B_ct = rf_data_buff[4];
					Usb_Ctrl_NoDisp_B_ct <<= 8;
					Usb_Ctrl_NoDisp_B_ct |= rf_data_buff[5];
					break;
				case 3:
					Usb_Ctrl_Disp_ZX_ct = rf_data_buff[2];
					Usb_Ctrl_Disp_ZX_ct <<= 8;
					Usb_Ctrl_Disp_ZX_ct |= rf_data_buff[3];

					Usb_Ctrl_NoDisp_ZX_ct = rf_data_buff[4];
					Usb_Ctrl_NoDisp_ZX_ct <<= 8;
					Usb_Ctrl_NoDisp_ZX_ct |= rf_data_buff[5];
					break;
			}
		}
	}
}


