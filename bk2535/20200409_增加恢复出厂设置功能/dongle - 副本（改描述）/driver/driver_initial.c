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
/*! \file driver_initial.c
    \brief The driver interface of initialization module.
*/

#include "headfile\includes.h"

/*! \fn void driver_clock_initial(void)
    \brief Initialize the clock module to system, dongle-16mhz.

    \param void
    \return void
*/
void driver_clock_initial(void)
{
    ANA_CFG0 = 0xE0;        // used for clock switch close rc16
    PCON = 0;               // open 16mhz
    driver_delay_us(300);   // 1ms
    driver_gpio_initial();
    RF_POWER_UP;
    // 20130316 make usb signal clear,ronghui
    //ANA_CFG3 = 0x40;        // used for clock switch 
    ANA_CFG6 = 0x94;        // open  48M PLL, for 48M analog regs does't have reset,  
    driver_delay_us(200);
    ANA_CFG6 = 0x90;  
    CLK_EN_CFG = 0;
    CKCON = CKCON_CLK_16MHZ | CKCON_TIMER2CLK_1_4 | CKCON_TIMER1CLK_1_4 | CKCON_TIMER0CLK_1_4;	
    driver_delay_us(100);
    PCON = 0x40;            // usb enable, the 48MHz clock will exist when EUSB=1
    EIP |= 0x04;            // usb interrupt, External interrupt 4 priority control, high
}

/*! \fn void driver_gpio_initial(void)
    \brief Initialize the gpio setting of system, including input/output direction, internal pull-up/pull-down, internal open-drain, wake up enable.

    \param void
    \return void
*/
void driver_gpio_initial(void)
{
    P0_IOSEL = P0_IOSEL_CFG_DONGLE;
    P0_OPDR = P0_OPDR_CFG_DONGLE;
    P0_PU = P0_PU_CFG_DONGLE;
    P0_PD = P0_PD_CFG_DONGLE;
    P0_WUEN = P0_WAKEUP_CFG_DONGLE;

    P1_IOSEL = P1_IOSEL_CFG_DONGLE;
    P1_OPDR = P1_OPDR_CFG_DONGLE;
    P1_PU = P1_PU_CFG_DONGLE;
    P1_PD = P1_PD_CFG_DONGLE;
    P1_WUEN = P1_WAKEUP_CFG_DONGLE;

    P2_IOSEL = P2_IOSEL_CFG_DONGLE;
    P2_OPDR = P2_OPDR_CFG_DONGLE;
    P2_PU = P2_PU_CFG_DONGLE;
    P2_PD = P2_PD_CFG_DONGLE;
    P2_WUEN = P2_WAKEUP_CFG_DONGLE;

    P3_IOSEL = P3_IOSEL_CFG_DONGLE;
    P3_OPDR = P3_OPDR_CFG_DONGLE;
    P3_PU = P3_PU_CFG_DONGLE;
    P3_PD = P3_PD_CFG_DONGLE;
    P3_WUEN = P3_WAKEUP_CFG_DONGLE;
}

/*! \fn void driver_usb_initial(void)
    \brief Initialize the usb module, which is used as a hid input device.

    \param void
    \return void
*/
void driver_usb_initial(void)					 
{ 
    system_data.usb_state = DEV_DEFAULT;

//set to normal state and wait to no setup token
    USB_PWR_CN &= ~0x02;            //no suspend
    while (CFG_EP0_1 != CONST_CFG_EP0_1)       //wait to no setup token
        CFG_EP0_1 = CONST_CFG_EP0_1;

    driver_delay_us(300);
//usb fifo configure
    EP_ADDR_MSB = 0x00; //It decides the port address locates above 1K space.

    CFG_EP0_1 = CONST_CFG_EP0_1;   // 0x1 1 0000 00;  dir = bk2433 out ;  ep0_en = 1 ; 
    CFG_EP0_0 = 0x00;   // addr = 0x00  // 0x200 -> 512

    CFG_EP1_1 = CONST_CFG_EP1_1;   // 0x 1 11 011 00;  dir = in ;  mod = INT ; ep_len = 64
    CFG_EP1_0 = 0x40;   // addr = 0x40  // 0x240 -> 512+64

    CFG_EP2_1 = CONST_CFG_EP1_2;   // 0x 1 11 011 00;  dir = in ; mod = INT ; ep_len = 64
    CFG_EP2_0 = 0x80;   // addr = 0x80  // 0x280

    CFG_EP3_1 = CONST_CFG_EP1_3;   // 0x 1 11 011 00;  dir = in ; mod = INT ; ep_len = 64
    CFG_EP3_0 = 0xc0;   // addr = 0xC0  // 0x2C0

//only open interrupt needed
    USB_EN0 = 0xfe;
    USB_EN1 = 0x00;
    EXINT_MOD &= ~0x04;  // int4(usb_int)  level detected

//usb enable physically
    USB_PWR_CN |= 0x80;   // enable pull_up on D+

    EX4 = 1;                        // enable usb interrupt
}

/*! \fn void driver_buffer_set(UINT8 *ptr, UINT8 value, UINT8 len)
    \brief Set buffer to the value with len bytes.

    \param ptr - pointer of buffer
    \param value -the value of buffer
    \param len -the length of buffer
    \return void
*/
void driver_buffer_set(UINT8 *ptr, UINT8 value, UINT8 len)
{
    //if(len)
    //{
        while(len)
        {
            *ptr++ = value;
            len--;
        }
   // }
}

/*! \fn void driver_buffer_copy(UINT8 *dest, UINT8 *src, UINT8 len)
    \brief Copy buffer \a src to \a dest with len bytes.

    \param dest - pointer of destination buffer
    \param src -pointer of source buffer
    \param len -the length of buffer
    \return void
*/
void driver_buffer_copy(UINT8 *dest, UINT8 *src, UINT8 len)
{   
    //if(len)
    //{
        while(len)
        {
            *dest++ = *src++;
            len--;
        }
    //}
}

/***********************************************************
						end file
***********************************************************/
