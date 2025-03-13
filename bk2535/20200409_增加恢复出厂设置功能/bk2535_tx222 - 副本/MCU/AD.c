/***********************************************************
Copyright (C), 1998-2011, Esense Technology Co., Ltd.
FileName	:
Author		: joe
Date		: 2011/03/01
Description	:
Version		: v0.1
Function List:
----
History:
<author> <time> <version > <desc>
joe  2011/03/01  0.1  build this moudle
***********************************************************/

#include "includes.h"

unsigned short 	adCalib;


/*
void adc_init(void)
{
  	CLK_EN_CFG |= 0x80;   //open adc clk	
	EA=1;
  	EX8=1;
}

void adc_mode_set(unsigned char mode, unsigned char pin)
{
  switch(mode) {
    case 0:
      ADC_CTL &= 0x39;  // power down adc
      break;
    case 1:  // single mode
      ADC_CTL = 0x40|((pin&0x7)<<3)|0x6;  //  01 xxx 110   ----- mode1  PIN   i/o enable  int enable
	  ADC_CTL2 =0x60; // add filter; add delay
      ADC_DATA_H =0x90;   //[7]adc_wate timer double,[6:4] div clk	
 //   ADC_CTL2 |= 0x02; 
 //   ADC_RATE = 0x0f;
      break;      
    case 2: // software mode
      ADC_CTL = 0x80|((pin&0x7)<<3)|0x6; //10  xxx  1 1 0  -----mode2  PIN  i/o enable int enable +  data ready
	  ADC_CTL2 = 0x60; // add delay
      ADC_DATA_H = 0x90;   //[7]adc_wate timer double // [6:4] div clk
//    ADC_CTL2 |= 0x02;
      ADC_RATE = 0x0f;
      break;
    case 3: // continuous mode
      ADC_CTL = 0xc0|((pin&0x7)<<3)|0x6; //11  xxx  1 1 0  -----mode2  PIN  i/o enable int enable +  data ready
	  ADC_CTL2 = 0x60; 
      ADC_DATA_H = 0x90;   //[7]adc_wate timer double // [6:4]  not div clk
      ADC_CTL2 |= 0x02;
      ADC_RATE = 0x0f;   // F=clk/2*(adc_ctl[4:0],adc_rate[7:0])   961 byte/s
      break;
    case 4: // continuous mode, filter enable
      ADC_CTL = 0xc0|((pin&0x7)<<3)|0x6; //11 xxx  1 1 0  -----mode2  PIN  i/o enable int enable +  data ready
	  ADC_CTL2 = 0x60; 
      ADC_DATA_H = 0x10;   //[7]adc_wate timer double // [6:4]  not div clk
      ADC_CTL2 |= 0x82; // filter enable, 4 sample average
      ADC_RATE = 0x0f;   // F=clk/2*(adc_ctl[4:0],adc_rate[7:0])   961 byte/s
      break;
    default:
      break;
  }
}



void adc_int () interrupt 14
{
//	EX8=0; 
//	adc_h_t = ADC_DATA_H;
//	AIF &= 0xBF; 
//	adc_finish = 1; 
//	EX8=1;

//    adc_dat =(adc_datah & 0x0f);
//    adc_dat =((adc_dat<<8)|adc_datal);
//    //PRINT("%d\r\n",tmp);
    AIF &= ~0x40;
}
*/



/*
unsigned char adc_get_chvalue(unsigned char ch)
{
	ADC_ENABLE;
	//AIE |= 0x40;          		// enable ADC interrupt	

	// 采样速率最大 29k
	// (n+1)*1000
	//ADCDH = 7<<2;          	//转换速度, 8k 采样
	//ADCDH = 24<<2;          //转换速度, 25k 采样

    adc_ctl = 0x00;
    adc_ctl |= (ch<<3); 	//set channel 
    adc_ctl |= (1<<1); 	//channel enable
	//ADCTL |= (1<<2); 	//int enable

	adc_ctl |= 0x42;
	delay_us(50);
	ch = ((UINT16)(adc_datah & 0x03)<<6) | (adc_datal>>2); 
	adc_ctl |= 0x42;
	delay_us(50);
	ch = ((UINT16)(adc_datah & 0x03)<<6) | (adc_datal>>2);

	//ADCTL &= ~(1<<1);  	//关闭IO第二功能
	adc_ctl &= ~(3<<1);  	//关闭IO第二功能
	adc_ctl = 0;
    AIF &= ~0x40;
	AIE &= ~0x40;      	// disable ADC interrupt
	ADC_DISABLE;
	
	return ch;
}
*/

unsigned short adc_get(unsigned char ch)
{
//	unsigned short adc;
	short adc;
	ADC_ENABLE;
	//AIE |= 0x40;          		// enable ADC interrupt	

	delay_us(50);
	//adc_datah = 0x90;	//40us
	adc_datah = 0x10;	//20us
	adc_rate = 200;		//40k
	adc_ctl2 = 0x40;

    adc_ctl = 0x00;
    adc_ctl |= (ch<<3); //set channel 
    adc_ctl |= (1<<1); 	//channel enable
	//ADCTL |= (1<<2); 	//int enable

//	P35 = 0;
	adc_ctl |= 0x42;
//	while((adc_ctl & 1));
	delay_us(50);
	adc = ((UINT16)(adc_datah & 0x03)<<8) | (adc_datal); 
//	P35 = 0;
	adc_ctl |= 0x42;
//	while((adc_ctl & 1));
	delay_us(50);
	adc = ((UINT16)(adc_datah & 0x03)<<8) | (adc_datal);

	//ADCTL &= ~(1<<1);  	//关闭IO第二功能
	adc_ctl &= ~(3<<1);  	//关闭IO第二功能
	adc_ctl = 0;
    AIF &= ~0x40;
	AIE &= ~0x40;      	// disable ADC interrupt
	ADC_DISABLE;
	
	adc -= adCalib;
	if (adc > 1023)
		adc = 1023;
	if (adc < 0)
		adc = 0;
	return adc;
}




#define __ADCEL_TIMES		5
#define ADCEL_TIMES			(1<<__ADCEL_TIMES)
//AD计算公式： AD * 1.2 = V * 512
void application_adc_cel(unsigned char pin)
{
	unsigned short  adcV;
//	unsigned long	sum;
	unsigned short	sum;	   //不超过100次  adc值在512左右
	unsigned char   i;

	P3IN_EN  |= 1<<pin ;
	P3OUT_EN |= 1<<pin ;
//	P3_OPDR  = P3_OPDR_CFG ;
	P3_PU    &= ~(1<<pin);
//	P3_PD    = P3_PD_CFG ;
//	P3_WUEN  = P3_WUKEUP;
//	P3       = P3_DAT ;

	adc_set_ref(ADC_REF_VOL, 1);
	i = 0;
	sum = 0;
	adCalib = 0;
//	for (i=0; i<100; i++) {
	for (i=0; i<ADCEL_TIMES; i++) {
		adcV = adc_get(pin);
		sum  += adcV;
		ADValue = adcV;	
//		printf("pin3.%bd, adc value = 0x%x\r\n", pin, adcV);
	}

//	adCalib =  sum/ADCEL_TIMES;
	adCalib =  sum >> __ADCEL_TIMES;
	adCalib -= 0x200;
//	printf("adCalib = 0x%x\r\n", adCalib);

	adc_set_ref(ADC_REF_VOL, 0);
//	P3_PU    |= (1<<pin);
}



