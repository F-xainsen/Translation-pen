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
/*! \file driver_timer.c
    \brief The driver interface of timer module.
*/

#include "headfile\includes.h"

/*! \fn void driver_timer0_initial(void)
    \brief Initialize timer0 for system basic tick, dongle-2ms.

    \param void
    \return void
*/
void driver_timer0_initial(void)
{
    CLK_EN_CFG |= 0x40;
    ET0 = 0; 	
    TR0 = 0; 
    TF0 = 0;
    TMOD &= 0xf0; 	
    TMOD |= 0x01;						
    
    TL0 = (TIMER0_2MS_CNT_16MHZ & 0x00FF);
    TH0 = (TIMER0_2MS_CNT_16MHZ >> 8);	

    ET0 = 1;							
    TR0 = 1;					
}

/*! \fn void driver_timer0_reset(void)
    \brief Reset timer0 for system basic tick.

    \param void
    \return void
*/
void driver_timer0_reset(void)
{
    TR0 = 0; 
    TF0 = 0;
    
    TL0 = (TIMER0_2MS_CNT_16MHZ & 0x00FF);
    TH0 = (TIMER0_2MS_CNT_16MHZ >> 8);

    TR0 = 1;     
}

/*! \fn void driver_delay_us(UINT16 xus)
    \brief Delay for a short time by for loop.

    \param xus - the count of for loop
    \return void
*/
void driver_delay_us(UINT16 xus)
{
   UINT8 i;
    while(xus)
    {   
       for(i=0;i<6;i++)
        wait();
       /*
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        wait();
        */
        xus--;
    }
}

/***********************************************************
						end file
***********************************************************/
