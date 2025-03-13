#ifndef __BK_MCU_RF_H__
#define __BK_MCU_RF_H__

#include <absacc.h>

#define BK2433 1
#define BK2533 2
#define BK2535 4


#define MCU_FLASH  (BK2533 || BK2535)
#define MCU_OTP    (BK2433)

#define MCU_OLD    (BK2433 || BK2533)

#ifndef MCU_TYPE
#define MCU_TYPE  BK2535
#endif


#if MCU_TYPE & MCU_OLD
#include "bk2x33_reg.h"
#else
#include "bk2535_reg.h"
#endif

#include "bk_mcu_rf_xreg.h"


#define ID0 CBYTE[0x7FF7]
#define ID1 CBYTE[0x7FF7+1]
#define ID2 CBYTE[0x7FF7+2]
#define ID3 CBYTE[0x7FF7+3]
#define ID4 CBYTE[0x7FF7+4]



#endif
