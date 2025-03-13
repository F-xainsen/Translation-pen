/***********************************************************
FileName	:
Date		: 2011/03/01
Description	:
Version		: v0.1
Function List:
----
History:
<author> <time> <version > <desc>
***********************************************************/

#include "includes.h"

//-------------------------------------------------------
//Function:
//Desc	:
//Input	:
//Output:
//Return:
//Others:
//Date	: 2011/03/01
//-------------------------------------------------------
void mcu_clk_inital(void)
{
	PCON2 = 0 ;	
    SPC_FNC = 0 ;
    //CKCON = CKCON_CLK_CFG|CKCON_TIMER2CLK_1_4|CKCON_TIMER1CLK_1_4|CKCON_TIMER0CLK_1_4 ;		
	CKCON = CKCON_CLK_CFG|CKCON_TIMER2CLK_1_12|CKCON_TIMER1CLK_1_12|CKCON_TIMER0CLK_1_12 ;
	//CKCON = CKCON_CLK_CFG;
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
void mcu_io_inital(void)
{	
	P0IN_EN = P0_IOSEL_CFG ;  // p0  设置为输入上拉状态
	P0OUT_EN = P0_IOSEL_CFG ;
	P0_OPDR = P0_OPDR_CFG ;
	P0_PU = P0_PU_CFG ;
	P0_PD = P0_PD_CFG ;
	P0_WUEN = P0_WUKEUP;
	P0 = P0_DAT ;

	P1IN_EN = P1_IOSEL_CFG;  // p1  设置为输出 高电平 
	P1OUT_EN = P1_IOSEL_CFG;
	P1_OPDR = P1_OPDR_CFG ;
	P1_PU = P1_PU_CFG ;
	P1_PD = P1_PD_CFG ;
	P1_WUEN = P1_WUKEUP;
	P1 = P1_DAT ;

	P2IN_EN = P2_IOSEL_CFG ; // p2  设置为输出 高电平 
	P2OUT_EN = P2_IOSEL_CFG;
	P2_OPDR = P2_OPDR_CFG;
	P2_PU = P2_PU_CFG ;
	P2_PD = P2_PD_CFG ;
	P2_WUEN = P2_WUKEUP;
	P2 = P2_DAT ;

	P3IN_EN = P3_IOSEL_CFG ;   // P30 设置为 输入状态 
	P3OUT_EN = P3_IOSEL_CFG ;  // 其他io口设置为 输出 (p31-p33 为输出低，p34-p37 为输出高)
	P3_OPDR = P3_OPDR_CFG ;
	P3_PU = P3_PU_CFG ;
	P3_PD = P3_PD_CFG ;
	P3_WUEN = P3_WUKEUP;
	P3 = P3_DAT ;
	
	P4IN_EN = P4_IOSEL_CFG ; // p4  设置为输出 高电平 
	P4OUT_EN = P4_IOSEL_CFG ;
	P4_OPDR = P4_OPDR_CFG ;
	P4_PU = P4_PU_CFG ;
	P4_PD = P4_PD_CFG ;
	P4_WUEN = P4_WUKEUP;
	P4 = P4_DAT;	
}





/***********************************************************
						end file
***********************************************************/
