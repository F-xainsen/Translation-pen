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
#define _BK3633_
#define VIRTUAL_UART_H4TL          1
#define UART_PRINTF_ENABLE         1///串口打印开关
#define DEBUG_HW                   0///硬件调试
#define DEBUG_HW_DIGITAL           0///数字硬件调试
#define GPIO_DBG_MSG               0///DEBUG信息可以通过DEBUG_MSG函数输出到GPIO
#define DEBUG_RF_REG               0///RF调试，为1可以通过串口读写RF寄存器
#define LDO_MODE                   0///LDO工作模式
#define LDO_MODE_IN_SLEEP          1

//DRIVER CONFIG
#define UART0_DRIVER                1
#define UART2_DRIVER                1
#define GPIO_DRIVER                    0
#define ADC_DRIVER                    0
#define I2C_DRIVER                    0
#define PWM_DRIVER                    1
#define USB_DRIVER                  1
#define SPI_DRIVER                  0 
#define AON_RTC_DRIVER              1
#define TIMER0_DRIVER                0

#if(USB_DRIVER)
#define uart_printf              uart2_printf  ///UART2 LOG打印
#else
#define uart_printf              uart0_printf
#endif
/// Default Device Name
#define APP_DFLT_DEVICE_NAME            ("BK3633_BLE")
#define APP_DFLT_DEVICE_NAME_LEN        (sizeof(APP_DFLT_DEVICE_NAME))

#define APP_SCNRSP_DATA         "\x09\xFF\x00\x60\x42\x4B\x2D\x42\x4C\x45"
#define APP_SCNRSP_DATA_LEN     (10)

/// Advertising channel map - 37, 38, 39
#define APP_ADV_CHMAP           (0x07)
/// Advertising minimum interval - 100ms (160*0.625ms)
#define APP_ADV_INT_MIN         (160 )
/// Advertising maximum interval - 100ms (160*0.625ms)
#define APP_ADV_INT_MAX         (160) 
/// Fast advertising interval
#define APP_ADV_FAST_INT        (32)

/// Advertising channel map - 37, 38, 39
#define APP_MESH_ADV_CHMAP           (0x07)
/// Advertising minimum interval - 50ms (80*0.625ms)
#define APP_MESH_ADV_INT_MIN         (80 )
/// Advertising maximum interval - 50ms (80*0.625ms)
#define APP_MESH_ADV_INT_MAX         (80)
/// Fast advertising interval
#define APP_MESH_ADV_FAST_INT        (32)

//最小连接间隔
#define BLE_UAPDATA_MIN_INTVALUE        20
//最大连接间隔 
#define BLE_UAPDATA_MAX_INTVALUE        40
//连接Latency
#define BLE_UAPDATA_LATENCY                0
//连接超时
#define BLE_UAPDATA_TIMEOUT                600

#define BLE_SCAN_PHY_CODED 0

#define APP_DIR_READ_NOTIFY                    0


#endif // USER_CONFIG_H_


