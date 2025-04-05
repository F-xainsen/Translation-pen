#include "app.h"
#include "icu.h"
#include "gpio.h"
#include "wdt.h"
#include "driver_timer.h"

#include "user_handle.h"

#include "usb_hid_keys.h"

extern SYSTEM_STRUCT_DATA system_data;
extern uint32 RF_flag;

APP_MOUSE_T mouse_val;
volatile uint8_t uRF_TXTime;
volatile uint8_t uCH_ChCt = 0;
#if    (KEYBOARD)
volatile eRC_TIMER_DUTY eTimerDuty;
volatile uint8_t uTimer2msCnt;
#endif

void app_mouse_wheel_scan(void)
{
    static uint8_t old_pinLevel = 0;
    static uint8_t wheel_tog = 0 ;
    uint8_t pinLevel = 0;

    if(gpio_get_input(WHEEL_B))
        pinLevel |= B_0100_0000;
    if(gpio_get_input(WHEEL_A))
        pinLevel |= B_1000_0000;

    if(pinLevel == B_0100_0000)
    {
        if(old_pinLevel == B_1100_0000)
            wheel_tog = B_1000_0001 ;                 // 2
        else if(old_pinLevel == B_0000_0000)
            wheel_tog = B_1000_0010 ;                 // 4
    }
    else if(pinLevel == B_1000_0000)
    {
        if(old_pinLevel == B_1100_0000)
            wheel_tog = B_1000_0011;                // 3
        else if(old_pinLevel == B_0000_0000)
            wheel_tog = B_1000_0000 ;               // 1
    }
    else
    {
        if(wheel_tog & B_1000_0000)
        {
            wheel_tog &= B_0111_1111;

            if(((wheel_tog == 0x00) && (pinLevel==B_1100_0000))||((wheel_tog == 0x01) && (pinLevel==B_0000_0000)))
                system_data.wheel++;
            else if(((wheel_tog == 0x02) && (pinLevel==B_1100_0000))||((wheel_tog == 0x03) && (pinLevel==B_0000_0000)))
                system_data.wheel--;
        }
    }
    old_pinLevel = pinLevel;
}


void app_mouse_key_check(void)
{
    if(RF_flag & flag_key_short)
    {
        RF_flag &= ~flag_key_short ;
        // 现在sensor的部分不加
        // if((system_data.key_valid & MOUSE_STANDARD_KEY_DPI) && !flag_dongle_lost)                                                //cpi switch
        //driver_sensor_dpi_switch();
    }
}

void app_gpio_sleep(void)
{
    REG_GPIO_WUATOD_STATUS = 0xffffffff ;
    SYS_REG0X10_INT_EN |= (0x01 << POS_SYS_REG0X10_INT_EN_GPIO);

}
/*! \fn void driver_timer0_reset(void)
    \brief Reset timer0 for system basic tick.

    \param void
    \return void
*/
void driver_timer0_reset(void)
{
    #if    (!KEYBOARD)
    Timer0_Stop(1);                // must used.
    Timer0_1m5_Start(1,1500000); // 1.5ms
    system_data.time.tick_2ms = 0;
    RF_flag &= ~flag_tick_8ms_loop;
    #endif

}

void driver_timer1_reset(void)
{
    #if    (KEYBOARD)
    Timer1_Stop(1);                // must used.
    Delay_us(50);                 // must used. 10 err, 50 ok, 100 ok,150 ok, 1500=880us.
    eTimerDuty = cRC_TIMER_2ms;
    Timer1_ReStart(1, 2000);     // 2ms
    system_data.time.tick_4ms = 0;
    RF_flag &= ~flag_tick_8ms_loop;
    #endif
}

/*! \fn void driver_delay_set_with_base_tick(TIMER *timer, UINT8 timeout)
    \brief Set a delay \a timeout*system_tick ms with \a timer.

    \param timer - the timer used for delay
    \param timeout - delay timeout*system_tick ms
    \return void
*/
void driver_delay_set_with_base_tick(TIMER *timer, uint32_t timeout)
{
//    ET0 = 0;
    timer->value_expired = system_data.time.tick_system + timeout;
//    ET0 = 1;
}

/*! \fn BOOL driver_delay_with_base_tick_expire_check(TIMER *timer)
    \brief Check the delay set by \a driver_delay_set_with_base_tick is whether expired.

    \param timer - the timer used for delay
    \return TRUE - expired, FALSE - otherwise
*/
uint8_t driver_delay_with_base_tick_expire_check(TIMER *timer)
{
    uint8_t flag_expired;

 //   ET0 = 0;
    flag_expired = system_data.time.tick_system >= timer->value_expired;
 //   ET0 = 1;

    return flag_expired;
}
/*
    ms*8ms
*/
void Wait_Xms(uint32_t ms)
{
    TIMER Delay;
    driver_delay_set_with_base_tick(&Delay, ms);
    while (!driver_delay_with_base_tick_expire_check(&Delay));
}

extern uint8_t tab_switch_en;
extern uint16_t tab_switch_ct;

void app_timer0_1_int_cb(void)
{
    #if (!KEYBOARD)
    uRF_TXTime++;
    system_data.time.tick_2ms++;
    system_data.time.tick_lbd_check++;
    if(system_data.time.tick_2ms >= 4) // 8ms
    {
        system_data.time.tick_2ms = 0;
        system_data.time.tick_system++;
        RF_flag |= flag_tick_8ms_loop ;
    }
    #endif
    RF_flag |= flag_wake_up_by_timer;

    // if(++uCH_ChCt >= 200 && system_data.system_mode == SYSTEM_NORMAL){
    //     uCH_ChCt = 0;
    //     ch_chan = 1;
    // }

    
    if(system_data.system_mode == SYSTEM_PAGE){
        if(pairStep){
            pair_step_timeout++;
            if(pair_step_timeout >= 200){   // 400ms reset pairstep
                pair_step_timeout = 0;   
                pairStep = 0;
            }
        }
        // 开机时定时跳出配对模式
        if(!first_pair && pair_timeout){
            pair_timeout--;
            if(pair_timeout == 0) system_data.system_mode = SYSTEM_NORMAL;
        }
    }

    if(tab_switch_en){
        if(++tab_switch_ct >= 500){
            key_send(KEY_MOD_LALT, KEY_TAB);
            key_send(KEY_MOD_LALT, 0);
            tab_switch_ct = 0;
        }
    }

}

void app_timer1_2_int_cb(void)
{
    #if (KEYBOARD)
    //  LED_Toggle();
    if(eTimerDuty == cRC_TIMER_2ms)
    {
        uTimer2msCnt++;
        if(uTimer2msCnt > 1)
        {
            uTimer2msCnt -= 2;
            bTimerInc = 1;
        }
    }
    else
    {
        bTimerInc = 1;
        if(eTimerDuty == cRC_TIMER_8ms)
        {
            uRF_TXTime++;
            system_data.time.u32KeyPressTime++;
            system_data.time.tick_4ms++;
            system_data.time.systemBindTick++;
            system_data.time.tick_lbd_check++;
        }
    }

    if(bTimerInc)
    {
        uRF_TXTime++;
        system_data.time.u32KeyPressTime++;
        system_data.time.tick_4ms++;
        system_data.time.systemBindTick++;
        system_data.time.tick_lbd_check++;
        if(system_data.time.tick_4ms > 1) // 8ms
        {
            system_data.time.tick_4ms -= 2;
            system_data.time.tick_system++;

            app_val.power_on_cnt++;
            RF_flag |= flag_tick_8ms_loop;
        }
        if(RF_flag & flag_release_delay)
        {
            system_data.time.releaseTick++;
            if(system_data.time.releaseTick > 10)
            {
                RF_flag &= ~flag_release_delay;
                RF_flag |= flag_release_send;
            }
        }
        RF_flag |= flag_power_on_timer ;
    }
    eTimerDuty = cRC_TIMER_4ms;
    #endif
}
