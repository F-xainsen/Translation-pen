/***********************************************************
FileName	: includes.h
***********************************************************/

#ifndef _INCLUDES_H_
#define _INCLUDES_H_

//command
//System Macro Definition for Module
//#define BOARD_VERSION_RAM
//#define BK2533

//#define DEBUG
//#define DEBUG_UART

//#define MODE_NORMAL
//#define MODE_PAGE


#define DEBUG
//#define TIMER0_INT


#define NO_SENSOR

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <intrins.h>
#include <ABSACC.H>
#include <stdarg.h>

#include "bk_mcu_rf.h"

#include "define.h"

#include "utils.h"
#include "Uart.h"
#include "timer.h"
#include "rf.h"
#include "MCUInital.h"
#include "AD.h"
#include "nvr.h"


#include "keys.h"
#include "main.h"

#include "global.h"
#include "function.h"
#include "pairfunc.h"
#include "sleep.h"

#endif

