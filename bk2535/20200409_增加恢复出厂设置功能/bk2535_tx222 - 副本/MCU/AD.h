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

#ifndef _AD_H_
#define _AD_H_
extern unsigned short 	adCalib;
//extern unsigned short  adcV;
extern void adc_inital(void);
//extern void adc_get_value(void);
//extern unsigned char adc_get_chvalue(unsigned char ch);
extern unsigned short adc_get(unsigned char ch);

#endif

/***********************************************************
						end file
***********************************************************/