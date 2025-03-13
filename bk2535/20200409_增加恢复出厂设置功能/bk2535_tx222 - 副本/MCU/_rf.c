#include "includes.h"




// rx0 tx  addr
unsigned char xdata RX0_Address[5];//={0x15,0x29,0x23,0xC6,0x00};   
//XDATA UINT8 TX_Address[]={0x10,0x76,0x14,0xCD,0x16};

XDATA UINT8 rf_address_rx2[5]={0xC3,0x00,22,32,42};

unsigned char code PUBLIC_Address[5] = {PROJECT_ID, ~PROJECT_ID, PROJECT_ID, ~PROJECT_ID, PROJECT_ID};
BYTE JumpFreq_Buf[4]={25,78,65,43};

//      	RF
//latest config txt,//please optimize code here.
CODE UINT32 Bank1_Reg0_8[9] = 		
{
#if RF_BAND==RF_250kbps   // 目前用的为250kbps
	//250k
	0x9C23C053,		   		
	0x80FC3DC8,
	0x00009A40,
	0xC086E419,
	0x49DCEDB4,
	0x81780780,
	0x0002FEFF,
	0x262808E0,	//0x262808E1, //	0x262808E2,	
	0x1111130C,
#elif RF_BAND==RF_1Mbps
	//1M
	0x53C0239C,
	0x80FC3DC8,
	0x00009A40,
	0x8086E419,
	0x49DCEDB4,
	0x81780480,
	0x0002FEFF,
	0x262848A0,	//0x262848A1,
	0x1111130C,
#elif RF_BAND==RF_2Mbps
	//2M
	0x53C0239C,
	0x80FC3DC8,
	0x00009A40,
	0x8086E419,
	0x49DCEDB4,
	0x81780780,
	0x0002FEFF,
	0x262808E0,	//0x262808E1,
	0x1111130C,
#endif
};

// 0 - 1.2v, 1 - vdd/2
void adc_set_ref(bit ref, bit cel)
{
	unsigned long reg7 = little2big32(Bank1_Reg0_8[7]);

	if (cel) 	//bit25
		reg7 |= 0x02000000;
	else 
		reg7 &= ~0x02000000;
		

	if (ref)  //bit24
		reg7 |= 0x01000000;
	else
		reg7 &= ~0x01000000;

	W_BANK1_ANALOG_REG2(7, reg7);
}



/*
// bank1 reg C value
CODE UINT8 Bank1_RegC[]={0x80, 0x01, 0x33, 0x10};
// bank1 reg D value
CODE UINT8 Bank1_RegD[]={0x94, 0xB7, 0x80, 0x01};  //bk2535  pll 130us  兼容2433
CODE UINT8 Bank1_Ramp_Table[11] = 
{
    0x1A, 0x28, 0x92, 0x26, 0xCA, 0xC2, 0xB4, 0x8D, 0xEB, 0xFC, 0x0F
};


//CODE UINT8 Bank1_RegC[]={0x80, 0x03, 0x33, 0x10};
//CODE UINT8 Bank1_RegD[]={0x14, 0xB7, 0x80, 0x01};
CODE UINT8 Bank1_Ramp_Table[11] = 
{
   0x41, 0x20, 0x08, 0x04, 0x81, 0x20, 0xCF, 0xF7, 0xFE, 0xFF, 0xFF //41 20 08 04 81 20 CF F7 FE FF FF
};
*/

////CODE UINT8 Bank1_RegD[]={0x14, 0xB4, 0x80, 0x01};		//40us
////CODE UINT8 Bank1_RegD[]={0x94, 0xB4, 0x80, 0x01};		//50us
////CODE UINT8 Bank1_RegD[]={0x14, 0xB5, 0x80, 0x01};		//60us
////CODE UINT8 Bank1_RegD[]={0x94, 0xB5, 0x80, 0x01};		//70us
////CODE UINT8 Bank1_RegD[]={0x14, 0xB6, 0x80, 0x01};		//80us    bk2535  pll default
////CODE UINT8 Bank1_RegD[]={0x94, 0xB6, 0x80, 0x01};		//100us
////CODE UINT8 Bank1_RegD[]={0x14, 0xB7, 0x80, 0x01};		//120us   bk2535  pll 兼容2533
////CODE UINT8 Bank1_RegD[]={0x94, 0xB7, 0x80, 0x01};  		//130us   bk2535  pll 兼容2433

CODE UINT8 Bank1_RegC[]={0x00, 0x12, 0x73, 0x00};
CODE UINT8 Bank1_RegD[]={0xb6, 0xB7, 0x80, 0x00};			//bk2535  pll 120us	 兼容2533
CODE UINT8 Bank1_Ramp_Table[11] = 
{
   0x41, 0x20, 0x08, 0x04, 0x81, 0x20, 0xCF, 0xF7, 0xFE, 0xFF, 0xFF //41 20 08 04 81 20 CF F7 FE FF FF
};


//-------------------------------------------------------
//Function:
//Desc	:
//Input	:
//Output:
//Return:
//Others:
//Date	: 2011/03/01
//-------------------------------------------------------
void BK2433_RF_Initial(void)
{
	UINT8 i;
	
    for(i=0; i<=8; i++)
        W_BANK1_ANALOG_REG1(i, Bank1_Reg0_8[i]);
			
	W_BANK1_DIGITAL_REG(BK2401_CFG_0C_ARRAY,Bank1_RegC,4);
	W_BANK1_DIGITAL_REG(BK2401_CFG_0D_ARRAY,Bank1_RegD,4);
	W_BANK1_DIGITAL_REG(BK2401_RAMP_TAB_ARRAY,Bank1_Ramp_Table,11);

	TX_FREQ_OFFSET[0] = 0x00;
    TX_FREQ_OFFSET[1] = 0x00;
    TX_FREQ_OFFSET[2] = 0x00;
    TX_FREQ_OFFSET[3] = 0x96;
    RX_FREQ_OFFSET[0] = 0x00;
    RX_FREQ_OFFSET[1] = 0x00;
    RX_FREQ_OFFSET[2] = 0x00;
    RX_FREQ_OFFSET[3] = 0xA0;

	MOD_DELAY = 0x0A;
    PLL_SDM = 0x00;    
    FM_GAIN_H = 0x00;
    FM_GAIN_L = 0xB0;
    FM_KMOD_SET_H = 0x01;
    FM_KMOD_SET_L = 0x00;
    MOD_COEFFICIENT_H = 0x09;
    MOD_COEFFICIENT_L = 0xAA;
    ANA_CTRL10 = 0x82;
	ANA_CTRL11 = 0xff;
    ANA_CTRL12 = 0x07;
    ANA_CTRL13 = 0x81;
	//------------------------------------------------  同 bk2423 寄存器，初始化bank0
	BK2401_CONFIG     = 0x7f;//0x3f;  // 开启 中断触发 。 接收模式 
	
	BK2401_ENAA       = 0X3f; // 所有通道使能自动应答
   	//BK2401_ENAA       = 0X01; // 所有通道使能自动应答
	BK2401_ENRX       = 0X01;  //  使能通道0
	
	BK2401_AW         = 0x03;  // 地址宽度为5 byte
	BK2401_RETR       = 0x14; 	// delay 500us ，自动重发4次
    BK2401_RFCH       = 40;    // 设置 初始通道
	
	BK2401_SETUP      = RF_BAND|RF_5dbm|RF_LNA_GAIN_HIGH;	//2mbps/1mbps/250kbps, 5dbm
	  					// 250kbps /  5db 
    //RX0_Address[0] = 0;
	memcpy(BK2401_R0_ADDR,rf_address_rx2,5);
	memcpy(BK2401_R1_ADDR,RX0_Address,5);  
 //   RX0_Address[4] = ~RX0_Address[4];
//	memcpy(BK2401_R1_ADDR,RX0_Address,5);	
  //  RX0_Address[4] = ~RX0_Address[4];
    BK2401_R2_ADDR    = 0X12;
    BK2401_R3_ADDR    = 0X13;   
    BK2401_R4_ADDR    = 0X14;
    BK2401_R5_ADDR    = 0X15;
 	memcpy(BK2401_TX_ADDR,RX0_Address,5);  

	
    BK2401_R0_PW      = 0X10;	   //32 byte  change 16 byte 
    BK2401_R1_PW      = 0X20;
    BK2401_R2_PW      = 0X20;
    BK2401_R3_PW      = 0X20;
    BK2401_R4_PW      = 0X20;
    BK2401_R5_PW      = 0X20;
	
    BK2401_DYNPD      = 0X3f;
	
    BK2401_FEATURE    = 0X07;
	//------------------------------------------------    	
	PowerDown_RF();
    PowerUp_RF();
    PowerDown_RF();
    BK2401_CE         = 0X00;
    BK2401_CE         = 0X01;

	//SwitchToRxMode();	
//	AIE |= 0x08;          		// enable rf interrupt
}



void W_BANK1_ANALOG_REG2(UINT8 addr, UINT32 value)
{
    while(!(BK2401_SCTRL&0x80));

    BK2401_SDATA[3] = value>>24;
    BK2401_SDATA[2] = value>>16;
    BK2401_SDATA[1] = value>>8;
    BK2401_SDATA[0] = value>>0;

    BK2401_SCTRL  = addr;
}

void W_BANK1_ANALOG_REG1(UINT8 addr, UINT32 value)
{
    while(!(BK2401_SCTRL&0x80));

    BK2401_SDATA[0] = value>>24;
    BK2401_SDATA[1] = value>>16;
    BK2401_SDATA[2] = value>>8;
    BK2401_SDATA[3] = value>>0;

    BK2401_SCTRL  = addr;
}

//-------------------------------------------------------
//Function:
//Desc	:
//Input	:
//Output:
//Return:
//Others:
//Date	: 2011/03/01
//-------------------------------------------------------
void W_BANK1_ANALOG_REG(UINT8 addr, UINT8* pvalue)
{
    while(!(BK2401_SCTRL&0x80));

    memcpy(BK2401_SDATA,pvalue,4);

    BK2401_SCTRL  = addr;
}

//-------------------------------------------------------
//Function:
//Desc	:
//Input	:
//Output:
//Return:
//Others:
//Date	: 2011/03/01
//-------------------------------------------------------
void W_BANK1_DIGITAL_REG(UINT8 *paddr, UINT8* pvalue,int len)
{
    UINT8 i;
    for(i=0;i<len;i++)
        paddr[i]=pvalue[i];
}



//-------------------------------------------------------
//Function:
//Input	:
//Output:
//Return:
//Others:
//Date	: 2011/03/01
//-------------------------------------------------------
void RF_Set_Mode(BYTE mode)
{
//	UINT32 reg_val = 0;
	UINT8 WriteArr[4];
	UINT8 j;
	
	for ( j = 0; j < 4; j ++ )
	{    
		WriteArr[j] =  ( Bank1_Reg0_8[2] >> ( 8 * ( 3 - j ) ) ) & 0xff; 
	}

	if(0 == mode)		// Single Wave
	{
		WriteArr[2] &= ~0x80;
		WriteArr[3] &= 0x80;
		WriteArr[3] |= 0x40;
		W_BANK1_ANALOG_REG(2,WriteArr);
		ANA_CTRL13 |= 0x20;
	}
	else
	{
		WriteArr[2] |= 0x80;
		WriteArr[3] &= 0x80;
		WriteArr[3] |= 0x40;
		W_BANK1_ANALOG_REG(2,WriteArr);
		ANA_CTRL13 &= ~0x20;
	}
	BK2401_CE = 0X00;
	BK2401_CE = 0X01;
}


//-------------------------------------------------------
//Function:
//Desc	:
//Input	:
//Output:
//Return:
//Others:
//Date	: 2011/03/01
//-------------------------------------------------------
void SwitchToRxMode(void)
{
    FLUSH_RX;
    FLUSH_TX;
	RFX2401C_RX();
    BK2401_CE=0;
    BK2401_CONFIG=BK2401_CONFIG|0x01;//set bit 1
    BK2401_CE=1;
}

//-------------------------------------------------------
//Function:
//Desc	:
//Input	:
//Output:
//Return:
//Others:
//Date	: 2011/03/01
//-------------------------------------------------------
void SwitchToTxMode(void)
{
    FLUSH_TX;
    FLUSH_RX;
	RFX2401C_TX();
    BK2401_CE=0;
    BK2401_CONFIG=BK2401_CONFIG&0xfe;//clear bit 1
    BK2401_CE=1;
}


//-------------------------------------------------------
//void SetChannel(void)
//{
//    BK2401_CE=0;
//	BK2401_RFCH = hooping[rf_channel];
//    BK2401_CE=1;
//}


void rf_set_channel(unsigned char ch)
{
    BK2401_CE=0;
	BK2401_RFCH = ch;
    BK2401_CE=1;
}

void rf_set_baud(unsigned char baud)
{
    BK2401_CE=0;
	BK2401_SETUP  = (BK2401_SETUP & ~0x28) | baud;
    BK2401_CE=1;	
}
void rf_set_dbm(unsigned char dbm)
{
    BK2401_CE=0;
	BK2401_SETUP  = (BK2401_SETUP & ~6) | dbm;
    BK2401_CE=1;	
}
void rf_set_retr(unsigned char retr)
{
    BK2401_CE=0;
	BK2401_RETR   = (BK2401_RETR & ~0xf) | retr;
    BK2401_CE=1;	
}




//-------------------------------------------------------
//Function:
//Desc	:
//Input	:
//Output:
//Return:
//Others:
//Date	: 2011/03/01
//-------------------------------------------------------
//void PowerUp_RF(void)
//{
//    BK2401_CONFIG=BK2401_CONFIG|0x02;
//}

//-------------------------------------------------------
//Function:
//Desc	:
//Input	:
//Output:
//Return:
//Others:
//Date	: 2011/03/01
//-------------------------------------------------------
//void PowerDown_RF(void)
//{
//    BK2401_CONFIG=BK2401_CONFIG&0xfd;
//}


//-----------------------------------------------------------
void set_rf_constant_address(void)
{
//	UINT8 Address_constant[5];
//	Address_constant[0] = CONSTANT_NETID;
//	Address_constant[1] = ~CONSTANT_NETID;
//	Address_constant[2] = CONSTANT_NETID;
//	Address_constant[3] = ~CONSTANT_NETID;
//	Address_constant[4] = CONSTANT_NETID;
	memcpy(BK2401_R0_ADDR,PUBLIC_Address,5);
 	memcpy(BK2401_TX_ADDR,PUBLIC_Address,5);
	BK2401_AW = 0x03;	   //3//3b ADDRESS
}

//-----------------------------------------------------------
//void rf_set_tx_addr(void)
//{
//	memcpy(BK2401_R0_ADDR,TX_Address,5);
// 	memcpy(BK2401_TX_ADDR,TX_Address,5);
//	BK2401_AW = 0x03;	   //5B ADDRESS
//}
void rf_set_rx_addr(void)
{
	memcpy(BK2401_R0_ADDR,RX0_Address,5);
 	memcpy(BK2401_TX_ADDR,RX0_Address,5);
	BK2401_AW = 0x03;	   //5B ADDRESS
}

void set_rf_5byte_address(unsigned char addr0)
{
	RX0_Address[0] = addr0;
	memcpy(BK2401_R0_ADDR,RX0_Address,5);
 	memcpy(BK2401_TX_ADDR,RX0_Address,5);
	BK2401_AW = 0x03;	   // 地址长度为 5byte
}


void rf_clrint(void)
{
	unsigned char sta;
	sta = BK2401_STATUS;
	while(sta & 0x71) 
	{
		if(sta&STATUS_MAX_RT) 
		{
			FLUSH_TX;
//			FLUSH_RX;
//			BK2401_CONFIG=BK2401_CONFIG&0xfd;  //rf power down
//			BK2401_CONFIG=BK2401_CONFIG|0x02;  //rf power up
			BK2401_STATUS = STATUS_MAX_RT;
//			delay_us(10);
		}

		if(sta&STATUS_RX_DR) 
		{
			FLUSH_RX;
			BK2401_STATUS = STATUS_RX_DR;
		}

		if(sta&STATUS_TX_DS) 
		{
			BK2401_STATUS = STATUS_TX_DS ;
		}
		FLUSH_TX;
		sta = BK2401_STATUS;
	}
}

// gpa: 	0 -- 0x1f
// hq:  	0, 1, 3
// paldo:	0, 3
// pcsel:	0, 1
// 数字越大，功率越大, 初始化默认 5dbm
//              gpa        hq        paldo         pcsel
//  5dbm        1f		   3         1             3
//  3dbm        1f		   3         1             0
//  0dbm        1f		   1         1             0
// -3dbm        10		   1         1             0
// -6dbm         b		   1         1             0
// -9dbm         7		   1         1             0
// -12dbm        4		   1         1             0
// -18dbm        3		   1         1             0
// -23dbm        1		   1         1             0
// -27dbm        6		   0         0             0
// -30dbm        4		   0         0             0
// -33dbm        3		   0         0             0
// -36dbm        1		   0         0             0
// -54dbm        0		   0         0             0
void set_tx_power(unsigned char gpa, unsigned char hq, unsigned char paldo, bit pcsel)
{
	ANA_CTRL11 = (ANA_CTRL11 & ~0x1f) | (gpa&0x1f);
	ANA_CTRL11 = (ANA_CTRL11 & ~0x60) | ((hq&3)<<5);

	if (pcsel)		  	//bit16
		W_BANK1_ANALOG_REG2(4, 0xB4EDDC49);	//0x49DCEDB4
	else 
		W_BANK1_ANALOG_REG2(4, 0xB4ECDC49);

	//bit23:22
#if RF_BAND==RF_250kbps
	//W_BANK1_ANALOG_REG2(3, 0x19E486C0)	//0xC086E419
	W_BANK1_ANALOG_REG2(3, 0x192486C0|((paldo&3)<<22));	
#else
	//W_BANK1_ANALOG_REG2(3, 0x19E48680)	//0x8086E419
	W_BANK1_ANALOG_REG2(3, 0x19248680|((paldo&3)<<22));	
#endif
}

