#ifndef __UTILS_H__
#define __UTILS_H__

typedef signed char s8;
typedef signed int  s16;
typedef signed long s32;
typedef unsigned char u8;
typedef unsigned int  u16;
typedef unsigned long u32;

typedef double fp32;

#define LENGTH(array) (sizeof(array)/sizeof(array[0]))

#define set_bit(dat, bno) ((dat)|=1<<(bno))
#define clr_bit(dat, bno) ((dat)&=~(1<<(bno)))
#define xor_bit(dat, bno) ((dat)^=1<<(bno))

#define _set_bit(dat, bits) ((dat)|=(bits))
#define _clr_bit(dat, bits) ((dat)&=~(bits))
#define _xor_bit(dat, bits) ((dat)^=(bits))


#define big2little16(A) ((((unsigned short)(A) & 0xff00) >> 8) | (((unsigned short)(A) & 0x00ff) << 8))
#define big2little32(A) ((((unsigned long)(A) & 0xff000000) >> 24) | (((unsigned long)(A) & 0x00ff0000) >> 8) \
                        | (((unsigned long)(A) & 0x0000ff00) << 8) | (((unsigned long)(A) & 0x000000ff) << 24))
#define little2big16(A) big2little16(A)
#define little2big32(A) big2little32(A)


extern void WaitUs(u32 us);

extern void __delay_us(unsigned int us);
extern void delay_ms(unsigned int ms);
//#if SYS_CLOCK==16000000ul
//#define delay_us(us)	__delay_us(us)
//#elif SYS_CLOCK==8000000ul
//#define delay_us(us)	__delay_us((us)/2)
//#elif SYS_CLOCK==4000000ul
//#define delay_us(us)	__delay_us((us)/4)
//#elif SYS_CLOCK==2000000ul
//#define delay_us(us)	__delay_us((us)/8)
//#else
//#define delay_us(us)	__delay_us(us)
//#endif

unsigned char get_rand(void);
extern unsigned int crc16(unsigned char *ptr,unsigned char len);
extern unsigned int crc16_int(unsigned char *ptr,unsigned char len) ;
extern unsigned char calc_1_nr(unsigned char dat);
extern unsigned char calc_1_ps(unsigned short dat);

// pov (0--up 1--right-up 2--right 3--right-down 4--down 5--left-down 6--left 7--left-up)
//extern unsigned char pin2pov(unsigned char pin);  //up down left right


#define __STRCAT(s0,s1)   s0##s1
#define STRCAT(s0,s1)   __STRCAT(s0,s1)



#define PORT2PIN(port, pin) STRCAT(port,pin) //PORT2PIN(P0,1) -> P01

#define PORT2IOSEL(port) STRCAT(port,_IOSEL)   	//PORT2IOSEL(P0) -> P0_IOSEL
#define PORT2DIR(port) STRCAT(port,_IOSEL)		//PORT2DIR(P0) -> P0_IOSEL
#define PORT2OPDR(port) STRCAT(port,_OPDR)		//PORT2OPDR(P0) -> P0_OPDR
#define PORT2PU(port) STRCAT(port,_PU)			//PORT2PU(P0) -> P0_PU
#define PORT2PD(port) STRCAT(port,_PD)			//PORT2PD(P0) -> P0_PD
#define PORT2WUEN(port) STRCAT(port,_WUEN)		//PORT2WUEN(P0) -> P0_WUEN


#define set_io_in(port, pin) (PORT2DIR(port) |= 1<<(pin))		//输入 set_io_in(P0, 1)
#define set_io_out(port, pin) (PORT2DIR(port) &= ~(1<<(pin)))	//输出 set_io_out(P0, 1)


#define set_pull_up(port, pin)   (PORT2PU(port) |= 1<<(pin))	//输入上拉 set_io_up(P0, 1)
#define set_pull_down(port, pin) (PORT2PU(port) |= 1<<(pin))	 //输入下拉 set_io_down(P0, 1)
#define set_io_od(port, pin)   	 (PORT2OPDR(port) |= 1<<(pin))	 //od输出 set_no_pull(P0, 1)
#define set_io_wuen(port, pin)   (PORT2WUEN(port) |= 1<<(pin))	 //唤醒使能 set_no_pull(P0, 1)

#define clr_pull_up(port, pin)   (PORT2PU(port) &= ~(1<<(pin)))	//输入上拉 set_io_up(P0, 1)
#define clr_pull_down(port, pin) (PORT2PU(port) &= ~(1<<(pin)))	 //输入下拉 set_io_down(P0, 1)
#define clr_io_od(port, pin)   	 (PORT2OPDR(port) &= ~(1<<(pin)))	 //od输出 set_no_pull(P0, 1)
#define clr_io_wuen(port, pin)   (PORT2WUEN(port) &= ~(1<<(pin)))	 //唤醒使能 set_no_pull(P0, 1)

#define set_io(port, pin)   (PORT2PIN(port,pin) = 1)
#define clr_io(port, pin)   (PORT2PIN(port,pin) = 0)
#define is_io_set(port, pin)   (PORT2PIN(port,pin))
#define is_io_clr(port, pin)   (!(PORT2PIN(port,pin)))




#endif
