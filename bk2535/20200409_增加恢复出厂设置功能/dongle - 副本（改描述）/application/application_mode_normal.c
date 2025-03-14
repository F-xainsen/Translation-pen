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
/*! \file application_mode_normal.c
    \brief The normal mode is the main mode that implements the product function.
*/

#include "headfile\includes.h"


unsigned char timer_rf_next_8ms = 0;
//unsigned char g_HavePageFlag=0;

//=========================================================
void application_normal_mode_initial(void)
{
    system_data.normal_sub_mode = SYSTEM_NORMAL_WORK;

//	DEG(("Into normal mode\n"));

#ifdef WATCHDOG_ENABLE
    CLK_EN_CFG |= 0x01;
    WDCTL = 0x06;       //prescale=128, 16*32768/16000000=128ms
    WDCTL = 0xd1;       //idle open
    WDCTL = 0xa5;       //enable wdt, and clear it
#endif
}

//=========================================================
void application_normal_mode(void)
{ 
	unsigned char i,send_count=0;
    application_normal_mode_initial();
     
    while(system_data.system_mode == SYSTEM_NORMAL)
    {
//		test_pin0 ^=1;
    // DEG(("f0=%bx\n",system_data.normal_sub_mode));
        switch(system_data.normal_sub_mode)
        {
          
            case SYSTEM_NORMAL_WORK:
                 #ifdef WATCHDOG_ENABLE
                    WDCTL = 0xa5;       //enable wdt and clear it
                 #endif
                 
				
                system_data.rf_page_fail_wait++;
                if(system_data.rf_page_fail_wait > 20)
                   system_data.rf_page_fail_wait = 0;
                
                if(flag_usb_state_normal==0)
                {
					if(flag_usb_idle || flag_usb_get_hid_report||flag_usb_config_set)
                    {
                        //USB枚举成功
//                        DEG(("usb ok\n"));
                        flag_usb_state_normal = 1;
						RF_CMD_FLUSH_TX;
    					RF_CMD_FLUSH_RX;

						driver_rf_spi_set_mode_rx();
					}
				}
                application_dongle_rf_data_received_check(); //receive page and data package  //收发都做在这个函数里面
				
                application_dongle_rf_data_received_analysis();
                application_dongle_usb_data_send_to_pc();
               	
             //   if(flag_usb_suspend)
                //    system_data.normal_sub_mode = SYSTEM_NORMAL_SUSPEND;
                break;
                
            case SYSTEM_NORMAL_SUSPEND:                
                #ifdef WATCHDOG_ENABLE
                WDCTL = 0xa5;               //enable wdt, and clear it
                WDCTL = 0xde;               //close wdt in idle mode
                WDCTL = 0xda;       
                #endif
                application_dongle_normal_submode_suspend();                
                #ifdef WATCHDOG_ENABLE
                CLK_EN_CFG |= 0x01;
                WDCTL = 0xd1;       //open wdt with idle enable
                WDCTL = 0xa5;       //enable wdt and clear it
                 #endif
                break;
        }
		if(flag_usb_endpoint0_accept)									  //在这里做USB命令的判断
		{
			flag_usb_endpoint0_accept = 0;
//			if(flag_rf_link_status == 0)
//				return;
			if(COMMAND_HEAD == system_xdata.usbp0_data.set_report_data[0])
			{
				if(system_xdata.usbp0_data.set_report_data[1] == ADCVOL_CHECK_COMMAND)	 //电量数据
				{
					if(system_xdata.usbp0_data.set_report_data[3] == 0x01)	 //查询电量
					{
						flag_keyboard_update=1;		//反馈出码值
						//P1 = P1 ^1;
						switch(Tx_Vol_Data)
						{
							case 0x01:	
								system_data.keycode = 3;
								break;
							case 0x02:
								system_data.keycode = 4;
								break;
							case 0x03:
								system_data.keycode = 5;
								break;
							case 0x04:
								system_data.keycode = 6;
								break;
							case 0x05:
								system_data.keycode = 7;
								break;
						}
					}
				}
				else if(system_xdata.usbp0_data.set_report_data[1] == CHANGE_CHECK_COMMAND)	 //充电数据
				{
					if(system_xdata.usbp0_data.set_report_data[3] == 0x01)		//查询充电
					{
						flag_keyboard_update=1;		//反馈出码值
						if(flag_tx_charge)		// 表示充电
						{
							system_data.keycode = 1;
							//P1 = P1 ^ 0X01;		
						}
						else
						{
						 	system_data.keycode = 2;
						}	
					}	
				}
				else if(system_xdata.usbp0_data.set_report_data[1] == MOUSE_SWICTH_COMMAND)
				{
					if(system_xdata.usbp0_data.set_report_data[3] == 0x01)
						flag_mouse_ctrl = 0;
					else
						flag_mouse_ctrl = 1;	
				}
				else if(system_xdata.usbp0_data.set_report_data[1] == LED_CTRL_COMMAND || system_xdata.usbp0_data.set_report_data[1] == 0)		   //第2位是数据长度，无需判断
				{
					//if(system_xdata.usbp0_data.set_report_data[3] == 1)
					if(flag_rf_link_status)
					{
						//在这里填充下发的数据buff
						//P1 ^=  1;
						rf_tx_data_len = 6;
						tx_send_cont = 6; 	//设置重发的次数
						for(i=0;i<rf_tx_data_len;i++)
						{
							rf_tx_data[i] = system_xdata.usbp0_data.set_report_data[i+1];	  
						}
					}
				}
				else if(system_xdata.usbp0_data.set_report_data[1] == SLEEPTIME_CTRL_COMMAND)
				{
					if(flag_rf_link_status)
					{
						rf_tx_data_len = 4;
						tx_send_cont = 6; 	//设置重发的次数
						//P1 ^=  1;
						for(i=0;i<rf_tx_data_len;i++)
						{
							rf_tx_data[i] = system_xdata.usbp0_data.set_report_data[i+3];	  
						}
					}	
				}
				else if(system_xdata.usbp0_data.set_report_data[1] == CHECK_LINK_COMMAND)
				{
					if(system_xdata.usbp0_data.set_report_data[3] == 0x01)	 //查询链接的状态
					{
						flag_keyboard_update=1;		//反馈出码值
						if(flag_rf_link_status)		//已连接
							system_data.keycode = 8;
						else
							system_data.keycode = 9;	 //未连接
					}
				}
				else if(system_xdata.usbp0_data.set_report_data[1] == WRITE_FLASH_COMMAND)
				{
					if(system_xdata.usbp0_data.set_report_data[3] == 0x01)	 //表示要让发射端记忆当前的滚码
					{
						// 等会再这里添加数据要回传的数�
						rf_tx_data_len = 2;
						tx_send_cont = 6; 	//设置重发的次数	
						rf_tx_data[0] = 0x56;
						rf_tx_data[1] = 0x78;
						//P1 ^=  1;
					}
				}
				else if(system_xdata.usbp0_data.set_report_data[1]  == RESET_ALL_COMMAND)
				{
					if(system_xdata.usbp0_data.set_report_data[3] == 0x01)
					{
						// 恢复出厂设置的传输
						rf_tx_data_len = 3;
						tx_send_cont = 6; 	//设置重发的次数	
						rf_tx_data[0] = 0x12;
						rf_tx_data[1] = 0x34;
						rf_tx_data[2] = 0x88;
						P1 ^=  1;
					}
				}
				else if(system_xdata.usbp0_data.set_report_data[1]  == READ_GROSE_COMMAND)
				{
					if(system_xdata.usbp0_data.set_report_data[3] == 0x01)
					{
						// 要反馈数据给到PC 端
						flag_keyboard_update=1;		//反馈出码值
						if(flag_grose_status)
							system_data.keycode = 10;	// 1 表示正常，反馈Q
						else
							system_data.keycode = 11;	// 0 表示不正常，反馈T
					}
				}
			}
		}	   
    }
}


//=========================================================
void application_dongle_rf_data_received_check(void)
{
#if 1
    UINT8 status,i,j, len,updatatimer=150,tx_status_data= 0;
	static UINT8 rf_addr_flag=0,rf_last_status=0xff;
	UINT32 cur_keytemp;
//	flag_page_flag = 1;
    if(flag_page_flag==0)
		updatatimer=25;
	else
        updatatimer=150;
	if(g_FrqUpdataTimer>updatatimer)  //32*2ms	一共64MS
    {
        g_FrqUpdataTimer=0;
		if(flag_page_flag)
		{
            JumpFrequency(1);
//			DEG(("skip ok\n"));
//			test_pin0 =1;
			
		}
		else
		{
            if(JumpFreqCount==(MaxJumpFreqNum-1))
        	{
        	    //set page address
                //rf_pipe_set_public();
				//driver_rf_spi_write_register(WRITE_REG|RF_CH,25);
//				DEG(("skip frq\n"));
				JumpFreqCount=0xff;
				application_page_mode();
				rf_addr_flag=0;
				JumpFrequency(1);
        	}
			else
			{
			    if(rf_addr_flag==0)
			    {
			        rf_addr_flag=1;
                	driver_rf_spi_write_buffer((WRITE_REG|RX_ADDR_P0), rf_address_rx0, RF_ADDRESS_LEN);
                	driver_rf_spi_write_buffer((WRITE_REG|TX_ADDR), rf_address_rx0, RF_ADDRESS_LEN);
                	driver_rf_spi_write_buffer((WRITE_REG|RX_ADDR_P1), rf_address_rx0, RF_ADDRESS_LEN);
				   // DEG(("%bx,%bx,%bx,%bx,%bx\n",rf_address_rx0[0],rf_address_rx0[1],rf_address_rx0[2],rf_address_rx0[3],rf_address_rx0[4]));
			    }
				JumpFrequency(1);
//				DEG(("nor frq\n"));
			}
		}
    }
    else if(g_FrqUpdataTimer == updatatimer-1)
    {
    //    rf_addr_flag=1;
        driver_rf_spi_write_buffer((WRITE_REG|RX_ADDR_P0), rf_address_rx0, RF_ADDRESS_LEN);
        driver_rf_spi_write_buffer((WRITE_REG|TX_ADDR), rf_address_rx0, RF_ADDRESS_LEN);
        driver_rf_spi_write_buffer((WRITE_REG|RX_ADDR_P1), rf_address_rx0, RF_ADDRESS_LEN);
    }
	else  
	{ 
        driver_rf_receive_packet();					//把这里的单收函数，改成收发一起的
		if(flag_rf_driver_received)
        {	
            flag_rf_driver_received = 0;
			g_FrqUpdataTimer=0;
			/***************************************/
			driver_delay_us(500);
			driver_rf_spi_set_mode_tx();
			driver_delay_us(1);
			RF_CMD_FLUSH_TX;                                                //fifo clear
			RF_CMD_FLUSH_RX;
			RF_CMD_CLEAR_STATUS_ALL;
			//有一个数据重发5次的操作
			//在这里添加要回传的数据给到TX端
			len = 1;
			if(tx_send_cont > 0)
				tx_send_cont--;
			if(tx_send_cont != 0)
			{
//				P1 ^=  1;
				len = rf_tx_data_len;
			}
			else
			{
				for(i=0;i<6;i++)
					rf_tx_data[i] = 0;
			}						
			driver_rf_spi_write_buffer(WR_TX_PLOAD, rf_tx_data, len);
			
			do
        	{
            	status = driver_rf_spi_read_register(STATUS);
        	}while((status & STATUS_RX_TX_MAX) == 0x00);
			driver_rf_spi_set_mode_rx();
			/***************************************/          
			flag_page_flag=1;

			if(flag_usb_state_normal==0)
				return;
			if(system_data.rf_received_len==6)
			{
				tx_status_data = rf_fifo_data[5];
				if(tx_status_data & 0x01 == 1)				   //检测充电
				{
			   		flag_tx_charge = 1;
				}
				else
					flag_tx_charge = 0;
				tx_status_data = tx_status_data >>1;
				if(tx_status_data & 0x01 == 1)
					flag_grose_status = 1;
				else
					flag_grose_status = 0;
				
				Tx_Vol_Data = tx_status_data >> 1;
				Tx_Vol_Data = Tx_Vol_Data & 0x07;			//这里是电量寄存器
					
			}
			if((system_data.rf_received_len==2)&&((rf_fifo_data[0]&0xf0)==0x00))
			{
//	             DEG(("f0=%x,f1=%x\n",(UINT16)rf_fifo_data[0],(UINT16)rf_fifo_data[1]));
                if(rf_last_status!=(rf_fifo_data[0]&0x0f))//RF数据有更新才发送USB数据
                {
			  
                    rf_last_status=rf_fifo_data[0];
					flag_keyboard_update=1;
					//system_data.keycode=rf_fifo_data[1];
					if(rf_fifo_data[1])
//                        flag_page_flag=1;
					switch(rf_fifo_data[1])
					{
					   case 0x00:
					   	 system_data.keycode=0;
					   	 break;
                       case 0x01:
					   	 system_data.keycode=1;     //PU
					   	 break;
					   case 0x02:
					     system_data.keycode=2; 	//PD
					   	 break;
					   case 0x04:
					   	 system_data.keycode=3;     //B
					   	 break;
					   case 0x08:
					   	 system_data.keycode=4;     //esc/
					   	 break;
					   case 0x10:
					   	 system_data.keycode=5;
					   	 break;	
					  case 0x05:
//					  	DEG(("recece Page\n"));	
					   	 system_data.keycode=1;
					   	 break;
					}
				//	DEG(("key\n"));
                }
			}
			else if((system_data.rf_received_len==6)&&((rf_fifo_data[1]!=0x00)||(rf_fifo_data[2]!=0x00)||(rf_fifo_data[3]!=0x00)||(rf_fifo_data[4]!=0x00))
				||((rf_fifo_data[1]!=rf_fifo_data_mose[0])||(rf_fifo_data[2]!=rf_fifo_data_mose[1])||(rf_fifo_data[3]!=rf_fifo_data_mose[2])||(rf_fifo_data[4]!=rf_fifo_data_mose[3])))
			{
			   	rf_fifo_data_mose[0]=rf_fifo_data[1];
			   	rf_fifo_data_mose[1]=rf_fifo_data[2];
			   	rf_fifo_data_mose[2]=rf_fifo_data[3];
			   	rf_fifo_data_mose[3]=rf_fifo_data[4];			
				flag_mouse_update=1;
			}

			//鼠标数据
		}
	}
#endif		
}

//=========================================================
void application_dongle_rf_data_received_analysis(void)
{
    static UINT8 key_type=0,shift_esc_flag=0;
	
    //mouse
   
    
    // keyboard: key pressed timeout, auto release
    if(flag_usb_endpoint1_null_data)
    {
        flag_usb_endpoint1_null_data = 0;
		if(flag_keyboard_media_pressed)
		{
            flag_keyboard_update = 1;
            system_data.keycode=0;
			key_type=KEYBOARD_DATA_MEDIA;
		}
		else
		if(flag_keyboard_standard_pressed)
		{
			flag_keyboard_update = 1;
		    system_data.keycode=0;
            key_type=KEYBOARD_DATA_STANDARD;
		}
    }
    //flag_usb_endpoint2_uploading = 1;
	//flag_mouse_update=1;
	//flag_keyboard_update=1;
  // system_data.keycode=1;
    if(flag_keyboard_update)				                                       
    {
        flag_keyboard_update = 0;
		if(system_data.keycode==0)
		{
            if(key_type==KEYBOARD_DATA_MEDIA)  
            {
                system_data.usbp3_data.in_report[0] = 0x03;
				system_data.usbp3_data.in_report[1] = 0x00;
				system_data.usbp3_data.in_report[2] = 0x00;
				flag_usb_endpoint3_uploading = 1;
				flag_keyboard_media_pressed = 0;
				system_data.usbp3_data.in_report_len = 3;
            }
			else
			if(key_type==KEYBOARD_DATA_STANDARD) 
			{
                driver_buffer_set(system_data.usbp1_data.in_report, 0, 8);
				flag_usb_endpoint1_uploading = 1;
				flag_keyboard_standard_pressed = 0;
			}
		}
		else
		{
		    driver_buffer_set(system_data.usbp1_data.in_report, 0, 8);
			driver_buffer_set(system_data.usbp3_data.in_report, 0, 3);
            switch(system_data.keycode)
           {
                case 1:
					system_data.usbp1_data.in_report[2]=kX;
					flag_usb_endpoint1_uploading = 1;
					flag_keyboard_standard_pressed = 1;
					key_type=KEYBOARD_DATA_STANDARD;
					break;
				case 2:
					system_data.usbp1_data.in_report[2]=kZ;
					flag_usb_endpoint1_uploading = 1;
					flag_keyboard_standard_pressed = 1;
					key_type=KEYBOARD_DATA_STANDARD;
					break;
				case 3:
					system_data.usbp1_data.in_report[2]=kS;
					flag_usb_endpoint1_uploading = 1;
					flag_keyboard_standard_pressed = 1;
					key_type=KEYBOARD_DATA_STANDARD;
					break;
				case 4:
					system_data.usbp1_data.in_report[2]=kU;
					flag_usb_endpoint1_uploading = 1;
					flag_keyboard_standard_pressed = 1;
					key_type=KEYBOARD_DATA_STANDARD;
					break;
				case 5:
					system_data.usbp1_data.in_report[2]=kV;
					flag_usb_endpoint1_uploading = 1;
					flag_keyboard_standard_pressed = 1;
					key_type=KEYBOARD_DATA_STANDARD;
					break;
				case 6:
					system_data.usbp1_data.in_report[2]=kW;
					flag_usb_endpoint1_uploading = 1;
					flag_keyboard_standard_pressed = 1;
					key_type=KEYBOARD_DATA_STANDARD;
					break;
				case 7:
					system_data.usbp1_data.in_report[2]=kY;
					flag_usb_endpoint1_uploading = 1;
					flag_keyboard_standard_pressed = 1;
					key_type=KEYBOARD_DATA_STANDARD;
					break;
				case 8:
					system_data.usbp1_data.in_report[2]=kO;
					flag_usb_endpoint1_uploading = 1;
					flag_keyboard_standard_pressed = 1;
					key_type=KEYBOARD_DATA_STANDARD;
					break;
				case 9:
					system_data.usbp1_data.in_report[2]=kP;
					flag_usb_endpoint1_uploading = 1;
					flag_keyboard_standard_pressed = 1;
					key_type=KEYBOARD_DATA_STANDARD;
					break;
				case 10:
					system_data.usbp1_data.in_report[2]=kQ;
					flag_usb_endpoint1_uploading = 1;
					flag_keyboard_standard_pressed = 1;
					key_type=KEYBOARD_DATA_STANDARD;
					break;
				case 11:
					system_data.usbp1_data.in_report[2]=kT;
					flag_usb_endpoint1_uploading = 1;
					flag_keyboard_standard_pressed = 1;
					key_type=KEYBOARD_DATA_STANDARD;
					break;
//				case SHIFT_F5_ESC_KEY:
//					if(shift_esc_flag&0x01)
//					{
//                        system_data.usbp1_data.in_report[2]=kEsc;
//					}
//					else
//					{
//                        system_data.usbp1_data.in_report[0]=kLShift;
//						system_data.usbp1_data.in_report[2]=kF5;
//					}
//					shift_esc_flag++;
//					flag_usb_endpoint1_uploading = 1;
//					flag_keyboard_standard_pressed = 1;
//					key_type=KEYBOARD_DATA_STANDARD;
//					break;
				default:
					break;
            }
		}
    }
	if(flag_mouse_update)
	{
	    //rf_fifo_data[0]=0;
		//rf_fifo_data[1]=1;
		//rf_fifo_data[2]=0;
		//rf_fifo_data[3]=0;
		flag_mouse_update=0;
		if(flag_mouse_ctrl == 1)						   //这是禁止鼠标上传数据
			return;
		flag_usb_endpoint2_uploading = 1;
		rf_fifo_data[1] = rf_fifo_data[1] & 0x01;
		system_data.mouse_data_buffer_uploading[0]=0x01;
		system_data.mouse_data_buffer_uploading[1]=rf_fifo_data[1];
		system_data.mouse_data_buffer_uploading[2]=rf_fifo_data[2];
		system_data.mouse_data_buffer_uploading[3]=rf_fifo_data[3];
		system_data.mouse_data_buffer_uploading[4]=rf_fifo_data[4];
		
	}
		
}


//=========================================================
void application_dongle_usb_data_send_to_pc(void)
{
	
    if(flag_usb_endpoint1_uploading && flag_usb_endpoint1_ready_uploading)
    {
        driver_usb_fifo_write(EP_1, 8, (UINT8 *)system_data.usbp1_data.in_report);
        driver_buffer_set(system_data.usbp1_data.in_report, 0, 8);
        flag_usb_endpoint1_uploading = 0;
        flag_usb_endpoint1_ready_uploading = 0;
        driver_usb_set_current_endpoint(EP_1);
    }

    if(flag_usb_endpoint2_uploading && flag_usb_endpoint2_ready_uploading)
    {
        if((system_data.usb_protocol_status == 0) && flag_usb_get_protocol && flag_usb_idle) //MAC BOOT
        {
            driver_usb_fifo_write(EP_2, 2, (UINT8 *)system_data.mouse_data_buffer_uploading);//system_data.usbp2_data.in_report);
        }
        else
        {
        #ifdef MOUSE_SWING
            //system_data.usbp2_data.in_report[6] = system_data.mouse_data_buffer_uploading[MOUSE_INDEX_SWING];
            driver_usb_fifo_write(EP_2, 5, (UINT8 *)system_data.mouse_data_buffer_uploading);//system_data.usbp2_data.in_report);
        #else
            driver_usb_fifo_write(EP_2, 5, (UINT8 *)system_data.mouse_data_buffer_uploading);//system_data.usbp2_data.in_report);
        #endif
        }

        driver_buffer_set(system_data.mouse_data_buffer_uploading, 0, 7);
        driver_buffer_set(&system_data.usbp2_data.in_report, 0, 7);
        flag_usb_endpoint2_uploading = 0;
        flag_usb_endpoint2_ready_uploading = 0;
        driver_usb_set_current_endpoint(EP_2);
    }

    if(flag_usb_endpoint3_uploading && flag_usb_endpoint3_ready_uploading)
    {
        flag_media_uploading = 0;
    #ifdef KEYBOARD_MCE
        flag_mce_uploading = 0;
    #endif
        flag_power_uploading = 0;
        driver_usb_fifo_write(EP_3, system_data.usbp3_data.in_report_len, (UINT8 *)system_data.usbp3_data.in_report);
        driver_buffer_set(system_data.usbp3_data.in_report, 0, 3);
		//driver_buffer_set(system_data.usbp1_data.rf_data_media, 0, 3);
        flag_usb_endpoint3_uploading = 0;
        flag_usb_endpoint3_ready_uploading = 0;
        driver_usb_set_current_endpoint(EP_3);
    }
}


//=========================================================
void application_dongle_normal_submode_suspend(void)
{
    UINT8 temp_clk;

    USB_EN0 |= 0x02;            //usb_res_en:  USB Resume interrupt enable bit
    USB_PWR_CN |= 0x02;    //USB_sus： USB module will enter low-power mode when write 1 into it. The USB protocol engine is stopped and no response to outside. It is used as suspend state usually in USB protocol.(can R/W by software）

    temp_clk = CLK_EN_CFG;
    CLK_EN_CFG = 0x40;        //timer enable
    PALT = 0;                        //gpio, no second function
    PCON &= ~0x40;             //close 48M_en

    while(1)
    {
        RF_CHIP_DISABLE;//RF_POWER_DOWN;
        TF0 = 0;
        TR0 = 0; 
        TL0 = (65050 & 0x00FF);
        TH0 = (65050 >> 8);			
        TR0 = 1; 	
        MCU_IDLE_OSC32KHZ;
        wait();

        RF_CHIP_ENABLE;//RF_POWER_UP;
        TF0 = 0;
        TR0 = 0; 
        TL0 = (65500 & 0x00FF);
        TH0 = (65500 >> 8);			
        TR0 = 1; 	
        MCU_IDLE_OSC32KHZ;


        driver_rf_receive_packet();
        if(flag_rf_driver_received)
        {	
            flag_rf_driver_received = 0;
            PCON |= 0x40;                           //48M_en 
            PCON &= ~0x06;                        //no 32khz
            CLK_EN_CFG = temp_clk;           //resume clk before

            if(flag_usb_remote_wakeup)
            {
                while (CFG_EP1_1 != CONST_CFG_EP1_1)  //CFG_EP1_1 [0x0812], CFG_EP1_0 [0x0813] (endpoint 1 configure register)
                    USB_PWR_CN = 0x80;           //PULL UP enable, D+(dp) pull up enable in chip. When it is disabled, device disconnect with outside circuit.
                driver_delay_us(5000);
                USB_PWR_CN = 0x84;           //Usb_rst: USB module will reset when write 1 into it. (Exclude control register)

                USB_PWR_CN |= 0x01;         //wakeup: according USB protocol, the device with remote wakeup function can send wake up signal to host.(R/W）When it is set to 1, USB force D+ and D- into K state, and release it when clear it.
                driver_delay_us(2500);
                USB_PWR_CN &= ~0x01;

                flag_usb_suspend = 0;
                system_data.normal_sub_mode = SYSTEM_NORMAL_WORK;	
            }
            else
            {				
                USB_PWR_CN |= 0x02;         //USB_sus： USB module will enter low-power mode when write 1 into it. The USB protocol engineer is stopped and no response to outside. It is used as suspend state usually in USB protocol.(can R/W by software）
                CLK_EN_CFG = 0x40;             //timer enable
                PCON &= ~0x40;                  //close 48M_en    				
            }	
        }

        if(!flag_usb_suspend)
        {
            flag_usb_endpoint1_uploading = 1;
            PCON |= 0x40;   
            PCON &= ~0x06; 
            CLK_EN_CFG = temp_clk;

            while (CFG_EP1_1 != CONST_CFG_EP1_1)
                USB_PWR_CN = 0x80;
            USB_PWR_CN = 0x84;
            system_data.normal_sub_mode = SYSTEM_NORMAL_WORK;
            break;
        }	
    }
}


/***********************************************************
						end file
***********************************************************/

