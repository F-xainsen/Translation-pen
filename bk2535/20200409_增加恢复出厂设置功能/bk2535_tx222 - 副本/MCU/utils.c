#include "includes.h"


// ??è????úêy
unsigned char get_rand(void)
{
	unsigned char dat,temp=0;
	RNG_CTRL = 0x80;
	while (!(RNG_CTRL & 0x80));
	delay_us(10);
	dat = RNG_DATA;
	return dat;
}


//#define TX_PIN P05
//#define TX_PIN P33
#define TX_PIN P21
//void init_tx_io(void)
//{
//	TX_PIN = 1;	
//	P1IN_EN  = P1IN_EN  | 0x02 ;
//	P1OUT_EN = P1OUT_EN | 0x02 ;
////	set_io_out(P2, 1);	//tx
//}


//char tx_putchar (char dat)			//38400
char putchar (char dat)			//38400
{
	unsigned char i,_dat = dat;
	EA =  0;
#if SYS_CLOCK==16000000ul
	TX_PIN =0;
	__delay_us(19);				//26us
	for (i=0; i<8; i++) {
		if (dat & 1)
			TX_PIN = 1; 
		else
			TX_PIN = 0; 
		dat>>=1;
		__delay_us(18);
	}
	TX_PIN =1;
	__delay_us(18);
#elif SYS_CLOCK==8000000ul
	TX_PIN =0;
	__delay_us(8);				//26us
	_nop_(); _nop_();
	for (i=0; i<8; i++) {
		if (dat & 1)
			TX_PIN = 1; 
		else
			TX_PIN = 0; 
		dat>>=1;
		__delay_us(7);
	}
	_nop_(); _nop_();
	TX_PIN =1;
	__delay_us(8);
#elif SYS_CLOCK==4000000ul
	TX_PIN =0;
	__delay_us(3);				//26us
	for (i=0; i<8; i++) {
		if (dat & 1) {
			TX_PIN = 1; 
		}else {
			TX_PIN = 0; 
			_nop_();
			_nop_();
			_nop_();
			_nop_();
		}
		dat>>=1;
		__delay_us(2);
	}
	_nop_();
	TX_PIN =1;
	__delay_us(2);
#elif SYS_CLOCK==2000000ul
	TX_PIN =0;
	for(j=1;j--;){_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();}				//26us
	for (i=8; i--;) {
		//ACC  
		if (_dat & 1)
			TX_PIN = 1; 
		else {
			TX_PIN = 0; 
			_nop_(); _nop_(); _nop_();  _nop_();
		}
		_dat>>=1;
		for(j=1;j--;){_nop_();_nop_();_nop_();_nop_();}
	}
	_nop_();_nop_();
	TX_PIN =1;
	for(j=1;j--;){_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();}
#endif		
	EA = 1;
	return dat;
}

/*
//-----------------------------------------------------------
unsigned int crc16(unsigned char *ptr,unsigned char len)        // ptr 为数据指针，len 为数据长度
{
	unsigned int i;
	unsigned int crc=0xffff;
	while(len--)
	{
		for(i=1; i <= 0x80; i<<=1)
		{
			if((crc&0x8000)!=0)
			{
				crc <<= 1;
				crc ^= 0x8005;
			}
			else
				crc <<= 1;
			if((*ptr&i) != 0)
				crc ^= 0x8005;
		}
		ptr++;
	}
	crc ^= 0xffff;	//取反
	crc &= 0xffff;	//只要低16bit
	return(crc);
}

unsigned int crc16_int(unsigned char *ptr,unsigned char len)        // ptr 为数据指针，len 为数据长度
{
	unsigned int i;
	unsigned int crc=0xffff;
	while(len--)
	{
		for(i=1; i <= 0x80; i<<=1)
		{
			if((crc&0x8000)!=0)
			{
				crc <<= 1;
				crc ^= 0x8005;
			}
			else
				crc <<= 1;
			if((*ptr&i) != 0)
				crc ^= 0x8005;
		}
		ptr++;
	}
	crc ^= 0xffff;	//取反
	crc &= 0xffff;	//只要低16bit
	return(crc);
}
*/
/*
// pov (0--up 1--right-up 2--right 3--right-down 4--down 5--left-down 6--left 7--left-up)
unsigned char pin2pov(unsigned char pin)  //up down left right
{
	code unsigned char PovTable[] =  {
	    		//	; R L D U  
		0x0f,	//	; 0 0 0 0  
		0x00,   //  ; 0 0 0 1  
		0x04,   //  ; 0 0 1 0  
		0x0F,   //  ; 0 0 1 1  
		0x06,   //  ; 0 1 0 0  
		0x07,   //  ; 0 1 0 1 *
		0x05,   //  ; 0 1 1 0  
		0x0F,   //  ; 0 1 1 1  
		0x02,   //  ; 1 0 0 0  
		0x01,   //  ; 1 0 0 1  
		0x03,   //  ; 1 0 1 0 *
		0x0f,   //  ; 1 0 1 1  
		0x0f,   //  ; 1 1 0 0  
		0x0f,   //  ; 1 1 0 1  
		0x0f,   //  ; 1 1 1 0  
		0x0f    //  ; 1 1 1 1 *
	};
	return PovTable[pin]; 
}
*/
////计算一个byte中bit为1的个数
unsigned char calc_1_nr(unsigned char dat)
{
	unsigned char code _table[] = {0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4};
	return _table[dat&0xf]+_table[dat>>4];
}
//
//计算一个byte中第一个为1的位置
unsigned char calc_1_ps(unsigned short dat)
{
	unsigned char code _table[] = {0,1,2,1,3,1,2,1,4,1,2,1,3,1,2,1};
	if (dat & 0xff) {
		if(dat & 0xf) {
			return _table[dat&0xf];		
		}else {
			return _table[dat>>4]+4;		
		}
	}else if (dat & 0xff00) {
		dat >>= 8;
		if(dat & 0xf) {
			return _table[dat&0xf] + 8;		
		}else {
			return _table[dat>>4]+4 + 8;		
		}
	}

	return 0;
}




////////////////////////////////////////////////////////////////
//#define MS_DLY	1420		//bk2433, bk2533
//#define MS_DLY	960				//bk2535
#define MS_DLY	965				//bk2535
void delay_ms(unsigned int ms)
{
    unsigned int i, j;
    
    for(i=0; i<ms; i++) {
#if SYS_CLOCK==16000000ul
        for(j=0; j<MS_DLY; j++) {		//2535
            _nop_();
        }
#elif SYS_CLOCK==8000000ul
        for(j=0; j<MS_DLY/2; j++) {
            _nop_();
        }
#elif SYS_CLOCK==4000000ul
        for(j=0; j<MS_DLY/4; j++) {
            _nop_();
        }
#elif SYS_CLOCK==2000000ul
        for(j=0; j<MS_DLY/8; j++) {
            _nop_();
        }
#endif
    }
}


////////////////////////////////////////////////////////////////
// 100 --> 100us	//bk2433, bk2533
// 100 --> 128us	//bk2535
void __delay_us(unsigned int us)
{
    unsigned int i;
    for(i=0; i<us; i++) {
    	_nop_();
    }
}

/*
void WaitUs(UINT32 us)	
{
	UINT16 	i;
	while (us)
	{
		for (i = 0; i < 1; ++i);
		us--;
	}
}
*/


