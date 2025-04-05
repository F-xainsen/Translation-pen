/**
 ****************************************************************************************
 *
 * @file arch_main.c
 *
 * @brief Main loop of the application.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */
 //#define __AES_TEST__
#define __USB_TEST__

#include <stdlib.h>    // standard lib functions
#include <stddef.h>    // standard definitions
#include <stdint.h>    // standard integer definition
#include <stdbool.h>   // boolean definition
#include <string.h>   // boolean definition
#include "intc.h"      // Interrupt initialization
#include "uart.h"      // UART initialization
#include "uart2.h"      // UART2 initialization
#include "flash.h"     // Flash initialization
#include "app.h"       // application functions
#include "reg_access.h"
#include "boot.h"
#include "dbg.h"
#include "icu.h"
#include "user_config.h"
#include "gpio.h"
#include "icu.h"
#include "wdt.h"
#include "spi.h"
#include "adc.h"
#include "uart2.h"
#include "aon_rtc.h"
#include "rf.h"
#ifdef __AES_TEST__
#include "aes.h"
#endif
#if(USB_DRIVER)
#include "usb.h"
#include "driver_icu.h"
#endif
#include "Application_mode.h"

#include "user_handle.h"


extern void  xvr_reg_initial_24(void);
uint8_t uart_rx_en;

void platform_reset(uint32_t error)
{

    uart_printf("reset error = %x\r\n", error);
    // Disable interrupts
    GLOBAL_INT_STOP();

    cpu_reset();

}
#ifdef __AES_TEST__
extern uint8_t aes_ok;
extern uint8_t encrypted_data[16];

uint8_t value[16]={0x00, 0x01 ,0x02 ,0x03 ,0x04 ,0x05, 0x06 ,0x07 ,0x08,
    0x09 ,0x0A ,0x0B, 0x0C ,0x0D, 0x0E ,0x0F};
uint8_t key[16]={0xED, 0x8D, 0x9C, 0x50, 0xD1, 0x55, 0xAA, 0x1E, 0x4D, 0x8F,
    0xF5, 0x81, 0x13, 0x32, 0x4F, 0x04, };
uint8_t encrypted_data[16]={0x37, 0x37, 0x79, 0x52, 0x26, 0x2E, 0x35, 0x08,
    0xAA, 0x12, 0x32, 0x2D, 0x99, 0xB3, 0xEC, 0x36 };

#endif
#if(USB_DRIVER)

extern void DelayNops_usb(volatile unsigned long nops);
void usb_mod_enable(int en_dis){
    if(en_dis){//usb modal enable
        setf_SYS_Reg0x3_usb_pwd;
        DelayNops_usb(500);
        clrf_SYS_Reg0x3_usb_pwd;
    }
    else{
        setf_SYS_Reg0x3_usb_pwd;
    }
}

void usb_mod_ie_enable(int en_dis){
    if(en_dis){
        ICU_INT_ENABLE_SET(ICU_INT_ENABLE_IRQ_USB_MASK);
    }else{
        ICU_INT_ENABLE_CLEAR(ICU_INT_ENABLE_IRQ_USB_MASK);
    }
}

#endif

int main(void)
{
    icu_init();
    //wdt_disable();
    intc_init();
    
    #if(UART_PRINTF_ENABLE)
    #if(!USB_DRIVER)
    uart_init(115200);
    #endif
    uart2_init(115200);//
    #endif
    uart_printf("main start~~~~~\r\n");

    gpio_init();
    flash_init();
  //  xvr_reg_initial_24();
  //  gpio_set_neg(0x04);

    xvr_reg_initial();
    #ifdef __USB_TEST__
    mcu_clk_switch(MCU_CLK_64M);
    #else
    mcu_clk_switch(MCU_CLK_16M);
    #endif   
    #if(AON_RTC_DRIVER)
    aon_rtc_init();
    #endif
    #if(SPI_DRIVER)
    spi_init(0,0,0);
    #endif


    #if(ADC_DRIVER)
    adc_init(1,1);
    #endif
    #if(USB_DRIVER)
    usb_init(usb_mod_enable,usb_mod_ie_enable);
    AplUsb_SetRxCbk(3, EP3_callback);
    #endif

    gpio_cb_register(app_gpio_sleep);
    GLOBAL_INT_START();
 
    uart_printf("main start~~~~~\r\n");

	user_fn24_main();
}






