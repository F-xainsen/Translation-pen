#include "user_handle.h"
#include "app.h"
#include "application_mode.h"
#include "application_mode_PCBA.h"
#include "application_mode_test.h"
#include "bk3633_reglist.h"
#include "driver_sensor.h"
#include "driver_timer.h"
#include "icu.h"
#include <string.h>

#include "flash.h"
#include "gpio.h"

#include <string.h>

#include "driver_usb.h"
#include "usb.h"

#include "app_key.h"
#include "app_gyroscope.h"
extern SYSTEM_STRUCT_DATA system_data;
extern void f24Init(void);
extern void fReadRFAddr(void);
extern void xvr_reg_initial_24(void);

// =================================================================
// 跳频
extern volatile uint8_t uCH_ChCt;
uint8_t channel_table[MAX_FREQ_SIZE] = {26, 51, 115, 75, 36}; // vlaue of 0 ~ 127

uint8_t ch_cur_index = 0;
uint8_t ch_chan      = 0;
// =================================================================

// 重传
uint8_t retransmit[10] = {0};
uint8_t retransmit_size = 0;
uint8_t retransmit_count = 0;

// 配对
uint8_t first_pair = 0;
uint8_t pair_step_timeout = 0; 
uint8_t pairStep  = 0;

uint8_t RXAddr[6] = {0};

// 振动
uint8_t vibrationMinus = 0;
uint16_t vibrationDuration = 0;
uint16_t vibrationBootTime = 0;

uint8_t low_power_flag = 0;

uint8_t mouse_flag = 0;
uint8_t mouse_en = 0;
int16_t mouse_X = 0;
int16_t mouse_Y = 0;


extern volatile uint8_t uart_rx_done;
extern volatile uint32_t uart_rx_index;
extern uint8_t uart_rx_buf[UART_FIFO_MAX_COUNT];

extern uint8_t rf_fifo_data[MAX_PACKET_LEN];

extern uint32_t RF_flag;

void user_application_initial(void)
{
    // SENSOR_TYPE eType;
    // eType = system_data.sensor_val.type;
    // memset(&system_data, 0, sizeof(SYSTEM_STRUCT_DATA));
    // system_data.sensor_val.type = eType;
    fReadRFAddr();
    f24Init();

    Timer_Initial(0, 1, 15);  // T0,1M
    Timer_Initial(1, 0, 0);   // T1,32k
    Timer0_Start(1, 1000000); // 2ms
                              // 注册IO回调
    timer_cb_register(0, 1, app_timer0_1_int_cb);

    //  WHEEL_WAKEUP_EX_INTERRUPT_FALLING_EDGE;
    SYS_REG0X10_INT_EN |= POS_SYS_REG0X10_INT_EN_TIMER0 | POS_SYS_REG0X10_INT_EN_GPIO;
}

void user_rf_receive_set(void)
{
    uart_printf("user_rf_receive_set\r\n");

    TRX_SETUP_RETR = 0x08; // 250us,redo 8times
    FLUSH_RX;
    FLUSH_TX;
    TRX_IRQ_STATUS = 0x70;

    flash_read_data(RXAddr, 0x7d210,6);

    P0_Address[0] = RXAddr[0];
    P0_Address[1] = RXAddr[1];
    P0_Address[2] = RXAddr[2];
    P0_Address[3] = RXAddr[3];
    P0_Address[4] = RXAddr[4];
    P0_Address[5] = RXAddr[5];
    memcpy_2461(&TRX_RX_ADDR_P0_0, P0_Address, RF_ADDRESS_LEN);
    memcpy_2461(&TRX_TX_ADDR_0, P0_Address, RF_ADDRESS_LEN);

    uart_printf("send to RXAddr=0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n", RXAddr[0], RXAddr[1], RXAddr[2], RXAddr[3], RXAddr[4], RXAddr[5]);
    
    driver_rf_ouput_power_value_set(POWER_VALUE_5DBM); 

    SwitchToTxMode();
    Delay_ms(1);
}

void user_rf_receive_package(void)
{
    uint8_t status;
    uint8_t fifo_status;
    uint8_t len;
    status = TRX_IRQ_STATUS;

    //   uart_printf("s=%x\r\n",status);
    // read register STATUS's value
    if (status & STATUS_RX_DR) // if receive data ready (RX_DR) interrupt
    {
        // do
        // {
        len = TRX_RX_RPL_WIDTH;
        if (len <= MAX_PACKET_LEN)
        {
            R_RX_PAYLOAD(rf_fifo_data, len);       // read receive payload from RX_FIFO buffer
            if (!(RF_flag & flag_rf_receive_page)) //&& ((rf_fifo_data[0] == DATATYPE_PAGE_START) || (rf_fifo_data[0] == DATATYPE_PAGE_END_MOUSE))
            {
                RF_flag |= flag_rf_receive_page;
                uRXDataLens = len;
            }
        }
        else
        {
            RF_CMD_FLUSH_RX;
            // break;
        }
        fifo_status = TRX_IRQ_STATUS; // read register FIFO_STATUS's value
                                      // }
                                      // while((fifo_status & STATUS_RX_EMPTY) != STATUS_RX_EMPTY);  // check the rx fifo is empty         // while not empty
        TRX_IRQ_STATUS = fifo_status;
    }
}

void user_application_normal_mode(void)
{
    uart_printf("user_application_normal_mode \r\n");

    user_rf_receive_set();

    SwitchToRxMode();
    Delay_ms(1);
    while (system_data.system_mode == SYSTEM_NORMAL)
    {

        user_rf_receive_package();
        if (RF_flag & flag_rf_receive_page)
        {
            RF_flag &= ~flag_rf_receive_page;
        }
        // RF 数据接收
        if (uRXDataLens)
        {
            receive_action();
            uRXDataLens = 0;
        }

        // UART 数据接收
        else if (uart_rx_done)
        {

            if (uart_rx_index > MAX_PACKET_LEN)
            {
                uart_rx_index = 0;
                uart_rx_done  = 0;
                continue;
            }
            for (uint8_t i = 0; i < uart_rx_index; i++)
            {
                rf_fifo_data[i] = uart_rx_buf[i];
            }

            SwitchToTxMode();
            Delay_ms(1);

            if (uart_rx_index == 3 && uart_rx_buf[0] == 0xFA)
            {
                ch_chan = 1;
            }

            if (driver_rf_data_send_ACK(uart_rx_index)) // receive dongle send end package to mouse
            {
                if (uart_rx_index == 1 && uart_rx_buf[0] == 0xF4)
                {
                    system_data.system_mode = SYSTEM_PAGE;
                }
            }else{
                retransmit_count = 3;
                retransmit_size = uart_rx_index;
                memcpy(retransmit,uart_rx_buf,uart_rx_index);
            }

            SwitchToRxMode();
            Delay_ms(1);

            uart_rx_done  = 0;
            uart_rx_index = 0;
        }

        if (ch_chan)
        {
            ch_chan = 0;

            rf_fifo_data[0] = 0xCC;
            SwitchToTxMode();
            Delay_ms(1);

            driver_rf_data_send_ACK(1);

            SwitchToRxMode();
            jump_frequency();
        
            Delay_ms(1);
        }

        if(retransmit_count != 0){
            retransmit_count--;
            memcpy(rf_fifo_data,retransmit,retransmit_size);
            SwitchToTxMode();
            
            if (driver_rf_data_send_ACK(retransmit_size))
            {
                retransmit_count = 0;
            }

            SwitchToRxMode();
        }

        if (key_loop_flag)
        {
            key_loop();
            key_loop_flag = 0;
            key_loop_ct   = 0;
        }
				
				if(mouse_flag && mouse_en){
					mouse();
					mouse_send(mouse_X,mouse_Y);
					mouse_flag = 0;
				}
    }
}

void user_rf_receive_page_set(void)
{
    TRX_SETUP_RETR = 0x08; // 250us,redo 8times
    FLUSH_RX;
    FLUSH_TX;
    TRX_IRQ_STATUS = 0x70;

    driver_rf_ouput_power_value_set(POWER_VALUE_5DBM); 

    SwitchToRxMode();
    Delay_ms(1);
}

#define PAIR_HEADER 0xAA
#define PAIR_FLAG   0xBB

extern volatile uint16_t pair_led_ct;
extern volatile uint16_t led_on_boot;

void user_application_page_mode(void)
{
    uart_printf("enter page mode\r\n");

    uint8_t selfAddr[6] = {0};

    flash_read_data(selfAddr, 0x7e000, 6);
    uart_printf("selfAddr=0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n", selfAddr[0], selfAddr[1], selfAddr[2], selfAddr[3], selfAddr[4], selfAddr[5]);

    memcpy_2461(&TRX_RX_ADDR_P0_0, rf_address_tx_pub, RF_ADDRESS_LEN);
    memcpy_2461(&TRX_TX_ADDR_0, rf_address_tx_pub, RF_ADDRESS_LEN);

    TRX_CE    = 0;
    TRX_RF_CH = 0x80 + 100;
    TRX_CE    = 1;

    // user_rf_receive_set();	// TX mode

    while (system_data.system_mode == SYSTEM_PAGE)
    {
        FLUSH_RX;
        FLUSH_TX;
        if (pairStep == 0)
        {
            rf_fifo_data[0] = PAIR_HEADER;
            rf_fifo_data[1] = PAIR_FLAG;
            rf_fifo_data[2] = selfAddr[0];
            rf_fifo_data[3] = selfAddr[1];
            rf_fifo_data[4] = selfAddr[2];
            rf_fifo_data[5] = selfAddr[3];
            rf_fifo_data[6] = selfAddr[4];
            rf_fifo_data[7] = selfAddr[5];

            SwitchToTxMode();
            Delay_ms(10);

            if (driver_rf_data_send_ACK(8)) // send to receiver
            {
                uart_printf("[success] send pair addr\r\n");
                SwitchToRxMode();
                Delay_ms(50); // 15 = 15ms;(4);                // 4ms
                RF_flag &= ~flag_rf_receive_page;
                driver_rf_receive_package();
                if (RF_flag & flag_rf_receive_page)
                {
                    if ((uRXDataLens == 8) && (rf_fifo_data[0] == PAIR_HEADER) && (rf_fifo_data[1] == PAIR_FLAG))
                    {
                        uart_printf("[success] receive pair addr\r\n");
                        RXAddr[0] = rf_fifo_data[2];
                        RXAddr[1] = rf_fifo_data[3];
                        RXAddr[2] = rf_fifo_data[4];
                        RXAddr[3] = rf_fifo_data[5];
                        RXAddr[4] = rf_fifo_data[6];
                        RXAddr[5] = rf_fifo_data[7];

                        pairStep = 1;
                    }
                    else
                    {
                        uart_printf("[fail] invalid pair addr\r\n");
                    }
                }
                else
                {
                    uart_printf("[fail] receive pair addr\r\n");
                }
            }
            else
            {
                // uart_printf("[fail] send pair addr\r\n");
            }
        }
        else
        {
            rf_fifo_data[0]  = PAIR_HEADER;
            rf_fifo_data[1]  = PAIR_FLAG;
            rf_fifo_data[2]  = selfAddr[0];
            rf_fifo_data[3]  = selfAddr[1];
            rf_fifo_data[4]  = selfAddr[2];
            rf_fifo_data[5]  = selfAddr[3];
            rf_fifo_data[6]  = selfAddr[4];
            rf_fifo_data[7]  = selfAddr[5];
            rf_fifo_data[8]  = RXAddr[0];
            rf_fifo_data[9]  = RXAddr[1];
            rf_fifo_data[10] = RXAddr[2];
            rf_fifo_data[11] = RXAddr[3];
            rf_fifo_data[12] = RXAddr[4];
            rf_fifo_data[13] = RXAddr[5];

            SwitchToTxMode();
            Delay_ms(15);
            if (driver_rf_data_send_ACK(14)) // receive dongle send end package to mouse
            {
                uart_printf("[success] send pair addr + remote addr\r\n");
                SwitchToRxMode();
                Delay_ms(20); // 15 = 15ms;(4);                // 4ms
                RF_flag &= ~flag_rf_receive_page;
                driver_rf_receive_package();
                if (RF_flag & flag_rf_receive_page)
                {
                    uart_printf("[success] receive pair addr + remote addr\r\n");
                    if (
                        (uRXDataLens == 14) &&
                        (rf_fifo_data[0] == PAIR_HEADER) &&
                        (rf_fifo_data[1] == PAIR_FLAG) &&
                        (rf_fifo_data[2] == RXAddr[0]) &&
                        (rf_fifo_data[3] == RXAddr[1]) &&
                        (rf_fifo_data[4] == RXAddr[2]) &&
                        (rf_fifo_data[5] == RXAddr[3]) &&
                        (rf_fifo_data[6] == RXAddr[4]) &&
                        (rf_fifo_data[7] == RXAddr[5]) &&

                        (rf_fifo_data[8] == selfAddr[0]) &&
                        (rf_fifo_data[9] == selfAddr[1]) &&
                        (rf_fifo_data[10] == selfAddr[2]) &&
                        (rf_fifo_data[11] == selfAddr[3]) &&
                        (rf_fifo_data[12] == selfAddr[4]) &&
                        (rf_fifo_data[13] == selfAddr[5]))
                    {

                        RF_flag |= flag_rf_paged;
                        uart_printf("[success] pair ok\r\n");
                        break;
                    }
                    else
                    {
                        uart_printf("page confirm resp err\r\n");
                    }
                }
                else
                {
                    uart_printf("no rx page resp\r\n");
                }
            }
            else
            {
                uart_printf("page confirm tx err\r\n");
            }
        }

        Delay_ms(100);
    }
    
    system_data.system_mode = SYSTEM_NORMAL;
    if ((RF_flag & flag_rf_paged))
    {
        uart_printf("RXAddr=0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n", RXAddr[0], RXAddr[1], RXAddr[2], RXAddr[3], RXAddr[4], RXAddr[5]);
        Delay_ms(100);
        flash_init();
        // 持久化地址
        flash_write_some_data(RXAddr, 0x7d210, 8);
    }

    
    P0_Address[0] = RXAddr[0];
    P0_Address[1] = RXAddr[1];
    P0_Address[2] = RXAddr[2];
    P0_Address[3] = RXAddr[3];
    P0_Address[4] = RXAddr[4];
    P0_Address[5] = RXAddr[5];
    memcpy_2461(&TRX_RX_ADDR_P0_0, P0_Address, RF_ADDRESS_LEN);
    memcpy_2461(&TRX_TX_ADDR_0, P0_Address, RF_ADDRESS_LEN);

    f24GetFreqConfigValue();
    SwitchToTxMode();
    driver_rf_ouput_power_value_set(cRFPWR);
    system_data.system_mode = SYSTEM_NORMAL;
    RF_CMD_FLUSH_TX;
    RF_CMD_FLUSH_RX;
    uart_printf("exiting page mode!!!\r\n");
    jump_frequency();

    pair_led_ct = 0;
    gpio_set(0x11,1);
}

void user_fn24_main(void)
{
    xvr_reg_initial_24();

    user_application_initial();


    gpio_config(0x04,OUTPUT,PULL_NONE); // 马达
    gpio_set(0x04, 0);

    gpio_config(ROW1, OUTPUT, PULL_NONE);
    gpio_config(ROW2, OUTPUT, PULL_NONE);
    gpio_config(ROW3, OUTPUT, PULL_NONE);
    gpio_config(COL1, INPUT, PULL_HIGH);  //PULL_LOW   PULL_HIGH
    gpio_config(COL2, INPUT, PULL_HIGH);  
    gpio_set(ROW1,1);
    gpio_set(ROW2,1);
    gpio_set(ROW3,1);

    gpio_config(0x10,OUTPUT,PULL_NONE); // 红灯
    gpio_config(0x11,OUTPUT,PULL_NONE); // 白灯
    gpio_set(0x10,1);
    gpio_set(0x11,1);
    led_on_boot = 1000;

    gpio_config(0x14,INPUT,PULL_HIGH); // 充电插入检测
    gpio_config(0x15,INPUT,PULL_HIGH); // 充电满电检测

    gpio_config(0x32, OUTPUT, PULL_NONE); // 激光灯
    gpio_set(0x32, 1);
		mouse_init();

    vibrationBootTime = 1000;

    if (first_pair)
    {
        system_data.system_mode = SYSTEM_PAGE;
    }else{
        system_data.system_mode = SYSTEM_NORMAL;
    }
    
    while (1)
    {
			mouse_init();
			mouse();
//        if (system_data.system_mode == SYSTEM_NORMAL)
//        {
//            user_application_normal_mode();
//        }
//        else if (system_data.system_mode == SYSTEM_PAGE)
//        {
//            user_application_page_mode();
//        }
    }
}

// =================================================================
// 跳频
// =================================================================

void jump_frequency(void)
{
    // uart_printf("channel %d will be change \r\n", TRX_RF_CH);

    ch_cur_index++;
    ch_cur_index %= MAX_FREQ_SIZE;

    uCH_ChCt  = 0;
    TRX_CE    = 0;
    TRX_RF_CH = 0x80 + channel_table[ch_cur_index];
    TRX_CE    = 1;

    // uart_printf("channel change to %d \r\n", TRX_RF_CH);
}


// =================================================================
// 发送接收
// =================================================================

void send_action(uint8_t action)
{
    rf_fifo_data[0] = action;

    SwitchToTxMode();
    Delay_ms(1);

    driver_rf_data_send_ACK(1);

    SwitchToRxMode();
    Delay_ms(1);
}

extern volatile uint16_t vibration_sec;
extern volatile uint8_t  vibration_min;
void receive_action(){
    if(uRXDataLens != 10) return;
    switch (rf_fifo_data[0])
    {
    case 0x05: // 振动设置
                
        vibrationBootTime = 1000;    // 开启时振动 和 关闭时振动

        vibration_sec = 0;
        vibration_min = 0;

        if(rf_fifo_data[1] == 0x00){
            gpio_set(0x04, 0);
            vibrationMinus = 0;
            vibrationDuration = 0;
            return;
        }
        vibrationMinus = rf_fifo_data[2]; // 振动间隔 5 - 180 min
        vibrationDuration = rf_fifo_data[3]; // 持续时长 1 - 10 sec
        vibrationDuration *= 1000;

        break;

    case 0x06:
        if(rf_fifo_data[1] == 0x01){
            // open air mouse

        }else{
            // close air mouse

        }
        break;
        
    default:
        break;
    }
}

void mouse_send(int16_t x,int16_t y){
    rf_fifo_data[0] = 0x0A;
    rf_fifo_data[1] = x >> 8;
    rf_fifo_data[2] = x & 0xFF;
    rf_fifo_data[3] = y >> 8;
    rf_fifo_data[4] = y & 0xFF;
    
    SwitchToTxMode();

    driver_rf_data_send_ACK(5);
    SwitchToRxMode();
}


// =================================================================
// ADC
// =================================================================
