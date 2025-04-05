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
// #define __USB_TEST__
#include <stdlib.h>    // standard lib functions
#include <stddef.h>    // standard definitions
#include <stdint.h>    // standard integer definition
#include <stdbool.h>   // boolean definition
#include <string.h>   // boolean definition
#include <math.h>
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
#include "app_gyroscope.h"
#ifdef __AES_TEST__
#include "aes.h"
#endif
#if(USB_DRIVER)
#include "usb.h"
#include "driver_icu.h"
#endif
#include "Application_mode.h"

#include "user_handle.h"

#include "i2c.h"
#include "gpio.h"

extern void  xvr_reg_initial_24(void);
uint8_t uart_rx_en;

void platform_reset(uint32_t error)
{

    uart_printf("reset error = %x\r\n", error);
    // Disable interrupts
    GLOBAL_INT_STOP();

    cpu_reset();

}


int main(void)
{
    icu_init();
    //wdt_disable();
    intc_init();
    
    #if(UART_PRINTF_ENABLE)
    #if(!USB_DRIVER)
    uart_init(115200);
    #endif
    // uart2_init(115200);//
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
    mcu_clk_switch(MCU_CLK_64M);
    // mcu_clk_switch(MCU_CLK_16M);
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
    #endif

    gpio_cb_register(app_gpio_sleep);
    GLOBAL_INT_START();
		
    uart_printf("main start~~~~~\r\n");
		user_fn24_main();
}
