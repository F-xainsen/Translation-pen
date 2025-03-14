/**
 ****************************************************************************************
 *
 * @file user_config.h
 *
 * @brief Configuration of the BT function
 *
 * Copyright (C) Beken 2019
 *
 ****************************************************************************************
 */
#ifndef USER_CONFIG_H_
#define USER_CONFIG_H_
#include "uart2.h"
#include "uart.h"

#define ITEST_ENABLE               1
#define RF_FREQ_OFFSET_CALI        0

#define VIRTUAL_UART_H4TL          1
#define UART_PRINTF_ENABLE         1///串口打印开关
#define DEBUG_HW                   0///硬件调试
#define DEBUG_HW_DIGITAL           0///数字硬件调试
#define GPIO_DBG_MSG               0///DEBUG信息可以通过DEBUG_MSG函数输出到GPIO
#define DEBUG_RF_REG               0///RF调试，为1可以通过串口读写RF寄存器
#define LDO_MODE                   0///LDO工作模式
#define LDO_MODE_IN_SLEEP          1

#define OPTIMIZE_POWER_CONSUMPTION      0

#if(OPTIMIZE_POWER_CONSUMPTION)
#undef LDO_MODE
#define LDO_MODE                     0
#undef LDO_MODE_IN_SLEEP
#define LDO_MODE_IN_SLEEP            0

#define OPTIMIZE_POWER_CONSUMPTION_TEMP 0
#if(OPTIMIZE_POWER_CONSUMPTION_TEMP)
#define CONFIG_RC32K_CALIBRATION     1
#define ADC_DRIVER                   1
#endif// OPTIMIZE_POWER_CONSUMPTION_TEMP
#endif // OPTIMIZE_POWER_CONSUMPTION


//DRIVER CONFIG
#define UART0_DRIVER                1
#define UART2_DRIVER                1
#define GPIO_DRIVER                    0
#define ADC_DRIVER                    1

#define I2C_DRIVER                    0
#define PWM_DRIVER                    0
#define USB_DRIVER                  0 
#define SPI_DRIVER                  0 
#define AON_RTC_DRIVER              0
#define I2S_DRIVER                    1

#define ADC_CALIB                    0//////У׼ADC��Ҫ��оƬһ���ȶ��Ĺ���ѹ��Ȼ����ADC�ο���ѹ

#define ENABLE_PHY_2M_LE_CODE       0 ////2M֧��2M�շ�   

/******************************************************/

#define uart_printf              uart0_printf

/// Default Device Name
#define APP_DFLT_DEVICE_NAME            ("BK3633_BLE789")
#define APP_DFLT_DEVICE_NAME_LEN        (sizeof(APP_DFLT_DEVICE_NAME))

#define APP_SCNRSP_DATA         "\x09\xFF\x00\x60\x42\x4B\x2D\x42\x4C\x45"
#define APP_SCNRSP_DATA_LEN     (10)


#define APP_DFLT_ADV_0XFF            ("BK3633_01234567890_01234567890_01234567890_01234567890_01234567890_01234567890_01234567890_01234567890_01234567890_01234567890")
#define APP_DFLT_ADV_0XFF_LEN        (sizeof(APP_DFLT_ADV_0XFF))



/// Advertising channel map - 37, 38, 39
#define APP_ADV_CHMAP           (0x07)
/// Advertising minimum interval - 40ms (64*0.625ms)
#define APP_ADV_INT_MIN         (160)
/// Advertising maximum interval - 40ms (64*0.625ms)
#define APP_ADV_INT_MAX         (160)
/// Fast advertising interval
#define APP_ADV_FAST_INT        (32)


//��С���Ӽ��
#define BLE_UAPDATA_MIN_INTVALUE        60
//������Ӽ�� 
#define BLE_UAPDATA_MAX_INTVALUE        80
//����Latency
#define BLE_UAPDATA_LATENCY                0
//���ӳ�ʱ
#define BLE_UAPDATA_TIMEOUT                600

// If you want to enable MULTI BOND, make sure CFG_BLE_SMP, BLE_APP_SEC are enabled
#define MULTI_BOND              0

#ifndef CONFIG_RC32K_CALIBRATION
#define CONFIG_RC32K_CALIBRATION     0
#endif


//#define __APP_ATTR_ARM __attribute__((section("APP_BLE_FIQ")))
#define __APP_ATTR_ARM 
#endif // USER_CONFIG_H_


