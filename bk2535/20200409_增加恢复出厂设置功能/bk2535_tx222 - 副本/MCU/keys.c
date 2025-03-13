#include "includes.h"
#include "char.h"
#include "multimedia.h"
#include "utils.h"
#include <math.h>


XDATA UINT8 key_array[KEY_COL_NUM];

XDATA UINT8 up_power000=0,up_power_cun=0;
extern BYTE BK2401_FIFO_data[10];


CODE UINT8 keyboard_table_07[KEY_COL_NUM][8]=		//for 07 PAGE
{
	{char_Esc,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},

	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},

	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
};

//-----------------------------------------------------------
CODE UINT16 keyboard_table_0c[KEY_COL_NUM][8]=		//for multimedia,0c page
{
	{0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},
	{AC_Home,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},
	{0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},
	{0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},
	{0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},

	{0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},
	{0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},
	{0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},
	{0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},
	{0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},

	{0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},
	{0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},
	{0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},
	{0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},
	{0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000},
};

#ifndef gyroscope_en
CODE UINT16 angle_Tab1[46] =	//4gmax  8bit  1=31.25mg 
	{
	 8192,        //90
     8187,        //88°
     8172,        //86°
     8147,        //84°
     8113,        //82°
     8067,        //80°
     8013,        //78°
     7949,        //76°
     
     7875,     //74°
	 7791, 	   //72°
	 7698, 	   //70°
	 7596, 	   //68°
	 7483, 	   //66°
	 7363, 	   //64°
	 7233, 	   //62°
	 7094, 	   //60°
	 
	 6947, 	   //58°		 
	 6791, 	   //56°
	 6627, 	   //54°
	 6455, 	   //52
	 6275, 	   //50°
	 6087, 	   //48°
	 5893, 	   //46°
	 5691, 	   //44°
	 
	 5481, 		//42°	  
	 5266, 		//40°
	 5044, 		//38°
	 4815, 		//36°
	 4581, 		//34°
	 4341, 		//32°
	 4096, 		//30°
	 3946, 		//28°
	 
	 3591, 		//26°	  
	 3332, 		//24°
	 3069, 		//22°
	 2802, 		//20°
	 2531, 		//18°
	 2258,	    //16°
	 1982,		//14°
	 1703,	    //12°
	 
	 1422,		//10°	  
	 1140,		//8°
	 856,		//6°
	 572,		//4°
	 286,		//2°
	 0    		//0°
	};

XDATA short GYRO_OFFSET[3];
XDATA char DPI_DATA=0;
XDATA UINT8 KET_DALY=0;

#endif


void read_keys_pin(void) 
{
	memset(key_array, 0, sizeof(key_array));

   	key_array[0]  |= P00 ? 0 : 0x01;		//esc
   	key_array[1]  |= P01 ? 0 : 0x01;		//ac_home

//   	key_array[14] |= P02 ? 0 : 0x20;	//mouse left key

	key_array[14] |= P11 ? 0 : 0x10;		//mouse x
	key_array[14] |= P03 ? 0 : 0x80;		//mouse y
}


unsigned char is_keydown(void)
{
	unsigned char i, res=0;
	for(i=0; i<LENGTH(key_array); i++) {
		res |= key_array[i];
	}
	return res;
}

void wait_all_key_release(void)
{
	while(1) {
		delay_ms(10);
	 	read_keys_pin();
		if (!is_keydown())
			break;
	}
}



void mouse_proc(void)
{
	memset(system.mouse, 0, sizeof(system.mouse));
	
	if (KEY_LMOUSE) {
		system.mouse[0] |= 0x01; 	
	}
	if (KEY_RMOUSE) {
		system.mouse[0] |= 0x02; 	
	}
//	if (KEY_MMOUSE) {
//		system.mouse[0] |= 0x04; 	
//	}
	
	if (key_array[14] & 0x40) {		//x
		system.mouse[0]  |= 0x01;
	}
	if (key_array[14] & 0x80) {		//y
		system.mouse[2] = 5;
	}
}

//-----------------------------------------------------------
void key_proc(void)
{
	unsigned char i,j, key_ct;

	memset(system.keyboard, 0, sizeof(system.keyboard));
	memset(system.multimedia, 0, sizeof(system.multimedia));

	last_func_keys = func_keys;
	func_keys = 0x00;
	
	read_keys_pin();
//	printf("key - %bx, %bx\r\n", key_array[0], key_array[1]);
	ghost_key_flag = 0;
	for(i=0;i<LENGTH(key_array)-1;i++) {
		if(calc_1_nr(key_array[i]) > 1) {	
			for(j=i+1;j<LENGTH(key_array)-1;j++) {
				if(calc_1_nr(key_array[j]) > 1) {
					if (calc_1_nr(key_array[i]|key_array[j]) < (calc_1_nr(key_array[i]) + calc_1_nr(key_array[j]))) {
						ghost_key_flag = 1;	
						return;
					}
				}
			}
		}
	}

	//-----------------------------------------------------------
	key_ct = 1;
	for(i=0;i<KEY_COL_NUM;i++) {
		unsigned char tmp = key_array[i];
		if(key_array[i]) {
			for(j=0;j<8;j++) {
				if(tmp & 0x01) {
					system.multimedia[0] = (system.multimedia[0] & 0xf0) | (keyboard_table_0c[i][j]>>8) & 0x0f;
					system.multimedia[1] = keyboard_table_0c[i][j] & 0xff;
					system.keyboard[key_ct++] = keyboard_table_07[i][j];
//					key_pressed_flag = 1;
					if(key_ct>6) {
						goto decode_keys_end;
					}
				}
				tmp >>= 1;
			}
		}
	}
decode_keys_end:;
	if (KEY_POWER) {
		system.multimedia[0] |= 0x10;	
	}
	if (KEY_SLEEP) {
		system.multimedia[0] |= 0x20;	
	}
	if (KEY_WAKEUP) {
		system.multimedia[0] |= 0x40;	
	}

	if (KEY_LCTRL) {
		system.keyboard[0] |= 0x01;	
	}
	if (KEY_LSHIFT) {
		system.keyboard[0] |= 0x02;	
	}
	if (KEY_LALT) {
		system.keyboard[0] |= 0x04;	
	}
	if (KEY_LWIN) {
		system.keyboard[0] |= 0x08;	
	}
	if (KEY_RCTRL) {
		system.keyboard[0] |= 0x10;	
	}
	if (KEY_RSHIFT) {
		system.keyboard[0] |= 0x20;	
	}
	if (KEY_RALT) {
		system.keyboard[0] |= 0x40;	
	}
	if (KEY_RWIN) {
		system.keyboard[0] |= 0x80;	
	}


//	if(KEY_VOLP)				//vol+
//		func_keys |= 0x04;
//	if(KEY_VOLM)				//vol-
//		func_keys |= 0x08;
	
//	if(KEY_OK) {				//OK
//		func_keys |= 0x20;
//	}
		
//	if((func_keys==0x22 ) && (last_func_keys != 0x22))// && search_mode)
//	{
//		printf("fc_mode^=1\r\n");
//
//		fc_mode ^= 1;
//		sleep_ct = 0;
//		fc_mode_ct = 0;
//		last_func_keys = func_keys;	//very important,need update key status
//		if(fc_mode) {
//			while(!TF0);
//			TF0 = 0;
//			TL0 = us2tcnt(7800)%256;
//			TH0 = us2tcnt(7800)/256;		//7.8ms
//
//			CE_LOW();
//			rf_clrint();
//			set_rf_constant_address();
//			BK2401_RFCH   = 80;
//			BK2401_STATUS = 0x70;
//			SwitchToRxMode();
//			FLUSH_RX;
//			CE_HIGH();
//		}else {
//			CE_LOW();
//			set_rf_5byte_address(0);
//			BK2401_RFCH   = rf_channel;
//			SwitchToTxMode();
//		}
//	}
}
#ifndef gyroscope_en

UINT8 BMI160_Init(void)
{
   UINT8 id;
  
    SMBus_rece(1,0,&id); 
	if (id!=0xd1)
	{
		return 0;
	}
	
	SMBus_tran(0x40,0x29);
	SMBus_tran(0x41,0x05);
	SMBus_tran(0x42,0x29);
	SMBus_tran(0x43,0x00);
	
	SMBus_tran(0x7e,0x11);
	SMBus_tran(0x7e,0x15);
	SMBus_rece(1,3,&id);
	if(id==0x14)
		return 1;
	else
	return 0;
}
UINT8 BMI160_Sleep(void)
{
//	SMBus_tran(0x03,0x20);
	SMBus_tran(0x7e,0x10);	  	//这里两个命令就是控制加速器和陀螺仪的休眠
	SMBus_tran(0x7e,0x14);
}

void gyroscope_MOSE(void)
{

	XDATA UINT8 i,BMI160_data_iic[14];
	XDATA short acc_FIFO[3],gyro_FIFO[3];
	XDATA UINT16 BMI160_data;
//	XDATA INT32 BMI160_data000;
	XDATA  char a[2];
	
	//P2IN_EN &= ~0x18;        //output
    //P2OUT_EN &= ~0x18;        //output
	//SCL_O=1;
	//SDA_O=1;
	
	if(up_power000)
   	{
		if(BMI160_Init()==1)
		{
			up_power000=0;	
		}
		return;
   	}
  	else
  	{
  	  
		SMBus_rece(12, 0X0C,BMI160_data_iic);
  	}
	BMI160_data=BMI160_data_iic[1];
	BMI160_data=(BMI160_data<<8)+BMI160_data_iic[0];
	gyro_FIFO[0]=BMI160_data;
	
	BMI160_data=BMI160_data_iic[3];
	BMI160_data=(BMI160_data<<8)+BMI160_data_iic[2];
	gyro_FIFO[1]=BMI160_data;
	BMI160_data=BMI160_data_iic[5];
	BMI160_data=(BMI160_data<<8)+BMI160_data_iic[4];
	gyro_FIFO[2]=BMI160_data;

	BMI160_data=BMI160_data_iic[7];
	BMI160_data=(BMI160_data<<8)+BMI160_data_iic[6];
	acc_FIFO[0]=BMI160_data;
	BMI160_data=BMI160_data_iic[9];
	BMI160_data=(BMI160_data<<8)+BMI160_data_iic[8];
	acc_FIFO[1]=BMI160_data;
	BMI160_data=BMI160_data_iic[11];
	BMI160_data=(BMI160_data<<8)+BMI160_data_iic[10];
	acc_FIFO[2]=BMI160_data;
//    gyro_FIFO[0]=0;
//    gyro_FIFO[1]=0;
//	gyro_FIFO[2]=0;
//	acc_FIFO[0]=0;
//	acc_FIFO[1]=0;
//	acc_FIFO[2]=0x2000;
        //printf("%5d,%5d,%5d,%5d,%5d,%5d\r\n", gyro_FIFO[0],gyro_FIFO[1],gyro_FIFO[2],acc_FIFO[0],acc_FIFO[1],acc_FIFO[2]);
	gyro_read_defult(gyro_FIFO);
		//printf("%5d,%5d,%5d\r\n", gyro_FIFO[0],gyro_FIFO[1],gyro_FIFO[2]);
	Kalman_filter(acc_FIFO,gyro_FIFO);
		//printf("%5d,%5d,%5d\r\n", gyro_FIFO[0],gyro_FIFO[1],gyro_FIFO[2]);
		
		
	for(i=0;i<2;i++)
	{
		tolerance_sub(&gyro_FIFO[i]);
		gyro_div(&gyro_FIFO[i],DPI_DATA);
	} 
		
		//printf("%5bd,%5bd,%5d,%5d\r\n", a[0],a[1],gyro_FIFO[0],gyro_FIFO[1]);
	touch_filter_sub(gyro_FIFO);   //一阶低通滤波
	a[0]=gyro_FIFO[0];
	a[1]=gyro_FIFO[1];
		
	if(a[0]||a[1])
	{
		a[1]=-a[1];
		//printf("%5d,%5d  00\r\n", a[0],a[1]);
		//system.mouse[1]=0xff;//a[1];x
		//system.mouse[2]=0;//y
		
		system.mouse[1]=a[1];//a[1];x
		system.mouse[2]=a[0];//y
		/*
		bt_tx_buff[tx_buff_head][0]=0xa1;
		bt_tx_buff[tx_buff_head][1]=0x02;
		bt_tx_buff[tx_buff_head][2]= MOUSE_KKEY;
		bt_tx_buff[tx_buff_head][3]=a[1];
		bt_tx_buff[tx_buff_head][4]=a[0];
	    bt_tx_buff[tx_buff_head][5]=0;
		bt_tx_buff[tx_buff_head][5]=0;
		tx_buff_len[tx_buff_head]=7;
		tx_buff_head++;
		tx_buff_count++;
		tx_buff_head=tx_buff_head%SEND_BUFFER_CNT;
		*/
	}
}


void gyro_read_defult(short *FIFO)
{

 static XDATA short gyro_defult[3]={0,0,0};
 static XDATA UINT8  Len;
 XDATA UINT8 i=0;
 XDATA short a;

   while(i<3)
   {
    a = FIFO[i]-gyro_defult[i];
    i++;
	a =abs(a);
    if(a>0x001a)
    {
     Len =0;
     gyro_defult[0] = FIFO[0];
     gyro_defult[1] = FIFO[1];
     gyro_defult[2] = FIFO[2];
     break;                                    //跳出循环
    }
   }
   Len++;
   if(Len>0x80)
   {
	Len =0;
    for(i=0;i<3;i++)
      {
       GYRO_OFFSET[i]=(gyro_defult[i]+FIFO[i])>>1;
       FIFO[i] =0;
	   //gy_off_set=1;
      }
   }
   else for(i=0; i<3; i++)
        {
         a=FIFO[i];
         FIFO[i] = FIFO[i]-GYRO_OFFSET[i];
		 //a1=abs(FIFO[i]);
		//if(FIFO[i]<5)
		// FIFO[i]=0;
		//FIFO[i]=FIFO[i]/0x1a;
		 if((FIFO[i]^a)&0x8000)  //方向改变
	 	  {
	 	   
	 	   if(abs(a)>0x0400)
	 	 	{
	 	 	 if(a&0x8000)
			 	FIFO[i] =0x8000;
			 else FIFO[i] =0x7FFF;
	 	 	}
	 	  }
		
        }
}

void Kalman_filter( short *acc, short *gyro)
{
 static XDATA short  anglex,pre_accx[6],pre_accz[6],pre_gyrox[6],pre_gyroy[6],pre_gyroz[6];
 //bit flag=0;
 XDATA UINT8 i;
 XDATA short M,anglex_acc;
 XDATA UINT16 Rx;
 
  for(i=0;i<5;i++)
  	{
  	 pre_accx[i]=pre_accx[i+1];
	 pre_accz[i]=pre_accz[i+1];
	 pre_gyrox[i]=pre_gyrox[i+1];
	 pre_gyroy[i]=pre_gyroy[i+1];
	 pre_gyroz[i]=pre_gyroz[i+1];
  	}
  pre_accx[5] =acc[0];
  pre_accz[5] =acc[2];
  pre_gyrox[5]=gyro[0];
  pre_gyroy[5]=gyro[1];
  pre_gyroz[5]=gyro[2];
 //printf("%5d,%5d,%5d\r\n", pre_gyrox[5],pre_gyroy[5],pre_gyroz[5]);
  Kalman_four_buff_filter(pre_accx);
  pre_accx[5] =Kalman_filter_arg(pre_accx[4],pre_accx[5],30);
  Kalman_four_buff_filter(pre_accz);
  pre_accz[5] =Kalman_filter_arg(pre_accz[4],pre_accz[5],30);
  Kalman_four_buff_filter(pre_gyrox);
  Kalman_four_buff_filter(pre_gyroy);
  Kalman_four_buff_filter(pre_gyroz);
  //printf("%5d,%5d,%5d\r\n", pre_gyrox[5],pre_gyroy[5],pre_gyroz[5]);
  M =Kalman_axis_limit(pre_gyroy[5],anglex);

 Rx =abs(pre_gyrox[5])>>5;
 Rx +=abs(pre_gyroy[5])>>5;
 Rx+=abs(pre_gyroz[5])>>5;
 
 if(Rx/256)
	Rx =0xff;
 else
 	Rx &=0x00ff;
 Rx=Rx*Rx;

 i =Kalman_filter_Hk(Rx);
  
 anglex_acc =Kalman_filter_get_angle(pre_accx[5],pre_accz[5],M);//<<8; //x
 anglex_acc*=256;
 //printf("%5d,%5d,%5d,%5d\r\n", pre_accx[5],pre_accz[5],M,pre_gyroy[5]);
 anglex =Kalman_filter_arg(M,anglex_acc,i);
 //printf("%5d,%5d,%5d\r\n", pre_accx[5],pre_accz[5],anglex);


 angle_line_arg(gyro,anglex);
 //printf("%5d,%5d\r\n", gyro[0],gyro[1]);
}

void Kalman_four_buff_filter(short *acc)
{
 XDATA INT32 sum;
 sum =acc[0];
 sum +=acc[1];
 sum +=acc[2];
 sum +=acc[3];
 sum +=acc[4];
 sum +=acc[5];
 //sum +=acc[6];
 //sum +=acc[7];
 sum =sum/6;
 //acc[5] =((short *)&sum);
 acc[5] =sum;
}
short Kalman_filter_arg(short gy_ang,short acc_ang,UINT8 Hk)
{
  XDATA UINT8 flag1;
 
 acc_ang -=gy_ang; //acc的矫正量，用一个系数判断矫正量的可信度    //Yk-(Xk-1 +Uk)
 if(acc_ang&0x8000)
 	{
 	 acc_ang =-acc_ang;
     flag1 =1;
 	}
 else flag1 =0;

 //acc_ang =mul8_16(Hk,acc_ang);//表示八位乘以16位
   acc_ang=acc_ang/(256/Hk);
 if(flag1)
 	acc_ang =-acc_ang;

 return gy_ang +acc_ang;   //求出角度值 这里乘法是无符号乘法
}
short Kalman_axis_limit(short gy,short angle)
{
/*-------------------------------------------------------------- 
 2000°=0x80*8ms  则陀螺仪实际移动角度=2000°/(0x80*125)*x=x*1/8
 而angle关系为0x40=90°  1°=0x40/90°	带入得x*1/8*0x40/90° 
 x/11.25为对应角度值
 --------------------------------------------------------------*/
 
 gy =gy/11;
 return angle+gy;   //最佳估计值 X(k-1) +Uk 
}
UINT8 Kalman_filter_Hk(UINT16 r)
{
 static XDATA UINT8 anglex_cov=10;
 static XDATA UINT8 pre_Hk=0xff;
 XDATA UINT8 Hk;

 anglex_cov =anglex_cov+0x05; 
 r +=anglex_cov;
 Hk =anglex_cov/arg_bit16_to_bit8(r);			  //Hk
 
 anglex_cov =arg_bit16_to_bit8((256-Hk)*anglex_cov);
 if(Hk>pre_Hk)   //Hk能直接取最小值，但要线性增大
 	{
 	 pre_Hk +=12;
	 Hk =pre_Hk;
 	}
 else pre_Hk=Hk;

  if(Hk<40)
	 Hk=2;
  else Hk=40;
  
 return Hk;
}
UINT8 arg_bit16_to_bit8(UINT16 buff)
{
  cyp val;
 
  val.INT_data =buff;
  if(val.CHAR_data[1]&0x80)
	 val.CHAR_data[0]++;
  return val.CHAR_data[0];
}

char Kalman_filter_get_angle(short X,short Y,short limit)
{
 XDATA UINT8 flag1,flag2,flag3;
 XDATA char i;
 XDATA short P;
 XDATA short trig_x[2];	
 XDATA short A;
 
 trig_x[0] =limit;
 i =angle_get_arg(trig_x);
if(trig_x[1]<5793)
// if(trig_x[1]<11586)   //45°5793
     flag3 =0;
 else
   flag3 =1;
 flag1 =0;
 flag2 =0;
 if(Y&0x8000)
  {
   flag2=1;
   Y =-Y;
  }
 //当分量值大于0x20要做反向补偿，否则不能实现正态分布

 if(X&0x8000)
 {
 	  X=-X;
	  if(X<8192)//0xe0~0x00  -8192
	  {
		flag1 =1;
	  }
	 else if(X<16384)//0xc0~0xe0  -16384
	  {
		flag1 =1;
		flag2 =~flag2;
		X=0x4000-X;
	  }
 }
 else
 {
	 if(X>0x2000)//0x20~0x40
	  {
		flag2 =~flag2;
	    X =0x4000-X;
	  }
 }
 if(!flag3)
  A=X;
 else A=Y;
 

  for(i=0;i<46;i++)
   {
    if(A>=angle_Tab1[i])  //余弦    夹角为与重力线夹角
      break;
   }
 if(flag3)	  //求出角度   0~0x40  90°范围
  i =45-i;   //0~45 46个数

 A =i*64;

// i=div16_8(A,45);
   i=A/45;

 if(flag1)
 	i =0x80-i;  //0x40~0x80  90°<A<180°
 	P=i;
 if(flag2)
 	P =-P;
    i=P;
 //bk3231_printf( "par0000 %d,%d,%d,%d,%d,%d\r\n",flag1,flag2,flag3,i,limit,P);
 return i;
 
}

UINT8 angle_get_arg(short  *angle)
{
 XDATA UINT8 i,flag=0;
 XDATA UINT16 A;
 if(angle[0]&0x8000)  //这里都是按与重力线夹角计算
 	{
 	 angle[0] =-angle[0];
	 if(angle[0]>0x4000)
	 	{
	 	 angle[0] =0x8000-angle[0];
		 flag |=0x04;
	 	}
	 else flag |=0x08;
 	}
 else 
 	{
 	 if(angle[0]>0x4000)
	 	{
	 	 angle[0] =0x8000-angle[0];
		 flag |=0x02;
	 	}
	 else flag |=0x01;
 	}
 //angle[0]=0x1000;

 A =arg_bit16_to_bit8(angle[0])*45;
 A =A>>6;
 i=A;
 angle[0]=angle_Tab1[45-i]; //sin@
 angle[1]=angle_Tab1[i]; //cos@
 //printf("%5d,%5d,%5d    11\r\n", angle[0],angle[1],A);
 return flag;
}
UINT16 mul13_16 (unsigned int NUM1, unsigned int NUM2)
{ 
   UINT16 buf;
   
   MD1 = (NUM1 & 0xff00) >> 8;
   MD0 =  NUM1 & 0x00ff;
	 
   MD5 = (NUM2 & 0xff00) >> 8;
   MD4 =  NUM2 & 0x00ff;	 
	 
   MDCTL = 0x40;
	 printf("%5bx\r\n", MDCTL);
   //while ( !(MDCTL & 0x20) )  	 // wait done
   	{
		printf("%5bx\r\n", MDCTL);
   }
   
   buf =MD3<<8;
   buf |=MD2;
   buf <<=4;
   buf |=MD1>>4;
   if(MD1&0x08)
     buf ++;
   return buf;
 }

void angle_line_arg(short*gyro,short angx)
{   //仰角计算Z轴 到坐标平面的投影 Z*cos@
 
 XDATA short trig_x[2];
 UINT8 m, flag=0;
 XDATA INT32 B_word1,B_word2,B_word3,B_word4,B_word5;
 
 trig_x[0] =angx;
 m =angle_get_arg(trig_x);

 m=_swap_(m);
 
 m &=0xf0;                    //x象限高四位 y低四位
// printf("%5d,%5d,%5d,%5d,%5x\r\n", trig_x[0],trig_x[1],gyro[0],gyro[1],flag);

 flag ^=0x08;      //X = xsinA+z sin(90+A)=xsinA+z cosA 


// printf("%5bx\r\n", flag);
// printf("%5x\r\n", m);                 
 if(m&0x10)                   //用flag 低四位表示 公式中的正负号
    flag ^=0x00;   //1象限
 if(m&0x20)
 	flag ^=0x06;   //2象限
 if(m&0x40)
 	flag ^=0x0f;   //3象限
 if(m&0x80)
 	flag ^=0x09;   //4象限
 if(gyro[2]&0x8000)	
 	{
 	 flag ^=0x05;
	 gyro[2] =-gyro[2];
    }
 if(gyro[0]&0x8000)
 	{
 	 flag ^=0x0a;
	 gyro[0] =-gyro[0];
 	}

   B_word4 = trig_x[0];
   B_word5 = gyro[2];
   B_word1=B_word4*B_word5;
   //p=mul13_16(trig_x[0],gyro[2]);
   B_word1 = B_word1/4096;
  
  
 if(flag&0x01) B_word1 =-B_word1;

 B_word4 = trig_x[1];
 B_word5 = gyro[0];
 
 B_word2 =B_word4*B_word5;
 B_word2 =B_word2/4096;
 if(flag&0x02) B_word2 =-B_word2;
 B_word3=B_word1+B_word2;
 // p=B_word3;
  //printf("%5d,%5d,%5d,%5d,%5d,%bx\r\n",gyro[0],gyro[2],trig_x[0],trig_x[1],p,flag);
 if(B_word3&0x80000000)
 {
    B_word3= -B_word3;
	if(B_word3>0x3f00)
		 gyro[1]=0xc100;
	else
	 gyro[1] =B_word1+B_word2;
 }
 else
 {
    if(B_word3>0x3F00)
		 gyro[1]=0x3F00;
	else
	 gyro[1] =B_word1+B_word2;
	
 }

//x

 //B_word1=trig_x[1]*gyro[2]/4096;
 B_word4=trig_x[1];
 B_word5 =gyro[2];
// B_word1=(int)trig_x[1]*(int)gyro[2];
 B_word1=B_word5*B_word4;
 B_word1 =B_word1/4096;
 if(flag&0x04) B_word1 =-B_word1;

 B_word4=trig_x[0];
 B_word5 =gyro[0];
// B_word2=(int)trig_x[0]*(int)gyro[0];
 B_word2=B_word5*B_word4;
 B_word2=  B_word2/4096;
 if(flag&0x08) B_word2 =-B_word2;

 B_word3=B_word1+B_word2;
 
 if(B_word3&0x80000000)
 {
    B_word3= -B_word3;
	if(B_word3>0x3f00)
		 gyro[0]=0xc100;
	else
	 gyro[0] =B_word1+B_word2;
 }
 else
 {
    if(B_word3>0x3F00)
		 gyro[0]=0x3F00;
	else
	 gyro[0] =B_word1+B_word2;
 }
// gyro[0] =B_word1+B_word2;//y

//printf("%5d,%5d,%5d,%5d,%5bx\r\n",gyro[0],gyro[1],trig_x[0],trig_x[1],flag);

}

//-------------------------------------------------------
//Func  : _swap_
//Desc	: 高低字节交换
//Input	:
//Output: 
//Return: 
//Author: Owen  
//Date	: 2013/07/05
//-------------------------------------------------------
UINT8 _swap_(UINT8 a)
{
  XDATA UINT8 ACC0;
  ACC0=a;
  ACC0 <<=4;
  ACC0=ACC0+(a>>4);
  return ACC0;
}
void tolerance_sub(short *p)
{
   XDATA short a;
    a = abs(*p);
    if(a>0x10)
         a -=0x10;
    else a=0;
    if(*p & 0x8000)
         *p = (~a)+1;
    else *p = a;
}
void gyro_div(short*p, char dpi)
{

 XDATA char i;
 XDATA short m;
 m=*p>>1;
 if(*p&0x8000)
 {
 	*p = -*p;
	*p <<=1;
	*p = -*p;	
 }
 else
 *p <<=1;          //默认放大1.5倍
 if(dpi>0)
  {
   if(dpi>5)
     dpi=5;
   for(i=0;i<dpi;i++)
     *p +=m;
  }
 else if(dpi&0x80)
  {
   dpi =-dpi;
   if(dpi>4)
     dpi=4;
   for(i=0;i<dpi;i++)
     {
	  if(i>1)
		 *p =*p>>1;
	  else
	   *p=*p-m;
	 }
  }
}

void touch_filter_sub(short *FIFO)
{
 static XDATA short gyro_pre_FIFO[2];
 static XDATA UINT8 gyro_coe[2],gyro_ct[2],gy_use_x,gy_use_y;
 XDATA UINT8 i;

 if(FIFO[0]||FIFO[1])
 	{
		gy_use_x =first_order_filter(&FIFO[0],&gyro_pre_FIFO[0],gy_use_x,&gyro_coe[0],&gyro_ct[0]);
		gy_use_y =first_order_filter(&FIFO[1],&gyro_pre_FIFO[1],gy_use_y,&gyro_coe[1],&gyro_ct[1]);
 	//gy_use_z =first_order_filter(&FIFO[2],&gyro_pre_FIFO[2],gy_use_x,&gyro_coe[2],&gyro_ct[2]);
 	}
 else
 	{
 	 i=0;
     do{
	 	gyro_pre_FIFO[i] =0;
		gyro_ct[i] =0;
		gyro_coe[i] =80;//FITITER_COE_Min;
		i++;
	 	}while(i<2);
 	}

}
UINT8 first_order_filter(short *now, short *old,UINT8 flag, UINT8 *coe, UINT8 *count)
{
 XDATA UINT8 m;
 XDATA short data0;

  if(*now ==0)   //本次为0或者前后两次异号
 	{
     *count =0;
	 *coe =80;//FITITER_COE_Min;
	 *old =0;
	 return 1;
 	}
 data0=*now-*old;
 m=1;
if(data0 & 0x8000) m =0;
 if(m==flag)
   {
	*count +=data0/1024;
	*count +=1;
    if(*count>8)
      {
       *count=0;
       *coe +=8;
       if(*coe>180)//FITITER_COE_Max)
        *coe =180;//FITITER_COE_Max;
      }
   }
  else {
        *count=0;                 //方向改变 计数器清0，系数取最小
        *coe =80;//FITITER_COE_Min;
       }
 // printf("%5bd, 22\r\n",*coe);

  data0=data0/256;
 *old +=*coe * data0;
 *now =*old/256;
 //printf("%5d, 22\r\n",*now);
  if(m) return  1;
  else  return  0; 
}


#endif

//-------------------------------------------------------
//Func  : I2C总线发送
//Desc	: 
//Input	:
//Output: 
//Return: 
//Author: Owen  
//Date	: 2013/07/29
//-------------------------------------------------------
bit SMBus_tran(BYTE p, BYTE dat)
{
	P2IN_EN |= 0x18;  //input
	P2OUT_EN |= 0x18;
	if(!SDA_I || !SCL_I)
 		return FAILURE;
	P2IN_EN &= ~0x18;        //output
	P2OUT_EN &= ~0x18;
	SDA_O =0;
	if(I2C_TX(0xd0)==FALSE)
	return FAILURE;
    if(I2C_TX(p)==FALSE)
	return FAILURE;
	if(I2C_TX(dat)==FALSE)
	return FAILURE;	
	P2IN_EN &= ~0x18;        //output
	P2OUT_EN &= ~0x18;       //output
	SCL_O =0;
	delay_us(2);
	SDA_O =0;
	delay_us(2);
	SCL_O =1;
	delay_us(2);
	SDA_O =1;
	return SUCCESS;

}

//-------------------------------------------------------
//Func  : I2C_TX
//Desc	: I2C单BYTE发送
//Input	:
//Output: 
//Return: 
//Author: Owen  
//Date	: 2013/05/13
//-------------------------------------------------------
bit I2C_TX(BYTE a)
{

	BYTE i=8;

	P2IN_EN &= ~0x18;        //output
	P2OUT_EN &= ~0x18;        //output
	do{
		SCL_O =0;
		delay_us(2);
		if(a&0x80)        
			SDA_O=1;
		else   
			SDA_O=0;
		SCL_O =1;
		P2IN_EN |= 0x08;  //input
		P2OUT_EN |= 0x08;
		while(!SCL_I);
			P2IN_EN &= ~0x08;  //output
			P2OUT_EN &= ~0x08;
		a <<=1;
	}while(--i);
	SCL_O =0;
	delay_us(2);
	SDA_O=1;
	delay_us(2);
	SCL_O =1;
	P2IN_EN |= 0x18;  //input
	P2OUT_EN |= 0x18;
	delay_us(1);
	if(SDA_I)
		return FALSE;
	else 
		return TRUE;
}


//-------------------------------------------------------
//Func  : I2C总线接收
//Desc	: 
//Input	:
//Output: 
//Return: 
//Author: Owen  
//Date	: 2013/07/29
//-------------------------------------------------------
bit SMBus_rece(BYTE a, BYTE addr,BYTE *p)
{
 	BYTE i=0;
   
 	P2IN_EN |= 0x18;  //input
	P2OUT_EN |= 0x18;  //input
 	if(!SDA_I ||!SCL_I)
 		return FAILURE;
 	P2IN_EN &= ~0x18;        //output
	P2OUT_EN &= ~0x18;         //output
 	SDA_O =0;
 	if(I2C_TX(0xd0)==FALSE)
    	return FAILURE;
	if(I2C_TX(addr)==FALSE)
    	return FAILURE;
	P2IN_EN &= ~0x18;        //output
	P2OUT_EN &= ~0x18;        //output
 	SCL_O =0;
 	delay_us(2);
 	SDA_O =0;
 	delay_us(2);
 	SCL_O =1;
 	delay_us(2);
 	SDA_O =1;
	delay_us(2);
	
	P2IN_EN |= 0x18;  //input
	P2OUT_EN |= 0x18;  //input
 	if(!SDA_I ||!SCL_I)
 		return FAILURE;
 	P2IN_EN &= ~0x18;        //output
	P2OUT_EN &= ~0x18;         //output
 	SDA_O =0;
 	if(I2C_TX(0xd1)==FALSE)
    	return FAILURE;
 	for(i=0;i<a;i++)
 	{
     	p[i]=I2C_RX();
	 	if(i ==(a-1))
	 	{
	 	 	SDA_O =1;   //NACK
	 	 	delay_us(2);
         	SCL_O =1;
	 	}
	 	else{
	 	 	SDA_O =0;
		 	delay_us(2);
         	SCL_O =1;     //ACK
	 	}
 	}
 	P2IN_EN &= ~0x18;        //output
	P2OUT_EN &= ~0x18;        //output
 	SCL_O =0;
 	delay_us(2);
 	SDA_O =0;
 	delay_us(2);
 	SCL_O =1;
 	delay_us(2);
 	SDA_O =1;
 	return SUCCESS;
}

//-------------------------------------------------------
//Func  : I2C_RX
//Desc	: I2C单BYTE接收
//Input	:
//Output: 
//Return: 
//Author: Owen  
//Date	: 2013/05/13
//-------------------------------------------------------
UINT8 I2C_RX(void)
{
	BYTE i,ACC11;
	P2IN_EN &= ~0x18;        //output
	P2OUT_EN &= ~0x18;         //output
	SCL_O =0;
	delay_us(1);
	SDA_O =1;
	P2IN_EN |= 0x10;  //input
	P2OUT_EN |= 0x10;
	i =8;
	do{
		ACC11 <<=1;
		SCL_O =1;
		P2IN_EN |= 0x08;  //input
		P2OUT_EN |= 0x08;  //input
		while(!SCL_I);
		P2IN_EN &= ~0x08;  //output
		P2OUT_EN &= ~0x08;  //output
		if(SDA_I) 
			ACC11 |=0x01;
		else 
			ACC11 &=0xfe;
		SCL_O =0;
	}while(--i);
	P2IN_EN &= ~0x18;        //output
	P2OUT_EN &= ~0x18;        //output
	return ACC11;
}
/*
//-------------------------------------------------------
//Func  : TOUCH_TRAN
//Desc	: DATA发射
//Input	:
//Output: 
//Return: 
//Author: Owen  
//Date	: 2013/05/9
//-------------------------------------------------------
void TOUCH_TRAN_IIC(void)
{
	BYTE i,array[3];
	i = 0;
	do
	{
		array[2] = system.touchpad_mode_set[1];
      	array[1] = system.touchpad_mode_set[0];
	  	array[0] = TP_ADDR;
		gyr_being = SMBus_tran(array,3);
		i++;
	}while((gyr_being == FAILURE)&&(i<5));

	#ifdef DEBUG
    if(gyr_being == SUCCESS)
         printf("TP write OK!\n");
    else printf("TP is err!\n");
    #endif

}
*/

