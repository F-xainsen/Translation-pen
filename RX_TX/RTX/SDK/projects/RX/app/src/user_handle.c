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
#include "usb_hid_keys.h"


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

uint8_t first_pair = 0;
uint16_t pair_timeout = 1000; // 2 seconds change to normal mode
uint8_t pair_step_timeout = 0; 
uint8_t pairStep  = 0;

void user_rf_rx_handler(void);

// usb 传输缓存
uint8_t tx_send_flag       = 0;
uint8_t tx_send_buffer[10] = {0};

// 重传
uint8_t retransmit[10] = {0};
uint8_t retransmit_size = 0;
uint8_t retransmit_count = 0;

extern uint8_t rf_fifo_data[MAX_PACKET_LEN];

extern uint32_t RF_flag;

enum AIR_MOUSE_KEYS
{
    KeyPageUp,
    KeyPageUpLongPress,
    KeyPageDown,
    KeyPageDownLongPress,
    KeyDigitalPointer,
    KeyDigitalPointerLongPress,
    KeyFuncKeyTop,
    KeyFuncKeyTopLongPress,
    KeyFuncKeyBottom,
    KeyFuncKeyBottomLongPress,
};

uint8_t AIR_MOUSE_KEY_FLAGS = 0;
#define AIR_MOUSE_FULLSCREEN_FLAG 0x80

uint8_t tab_switch_en = 0;
uint16_t tab_switch_ct = 0;

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
    TRX_SETUP_RETR = 0x08; // 250us,redo 8times
    FLUSH_RX;
    FLUSH_TX;
    TRX_IRQ_STATUS = 0x70;

    uint8_t selfAddr[6] = {0};
    flash_read_data(selfAddr, 0x7e000,6);

    P0_Address[0] = selfAddr[0];
    P0_Address[1] = selfAddr[1];
    P0_Address[2] = selfAddr[2];
    P0_Address[3] = selfAddr[3];
    P0_Address[4] = selfAddr[4];
    P0_Address[5] = selfAddr[5];
    memcpy_2461(&TRX_RX_ADDR_P0_0, P0_Address, RF_ADDRESS_LEN);
    memcpy_2461(&TRX_TX_ADDR_0, P0_Address, RF_ADDRESS_LEN);

    uart_printf("receive to 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n", selfAddr[0], selfAddr[1], selfAddr[2], selfAddr[3], selfAddr[4], selfAddr[5]);


    driver_rf_ouput_power_value_set(POWER_VALUE_5DBM);

    SwitchToRxMode();
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
                {
                    RF_flag |= flag_rf_receive_page;
                    uRXDataLens = len;
                }
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

void user_rf_receive_handle(void)
{
    user_rf_receive_package();
    if (RF_flag & flag_rf_receive_page)
    {
        RF_flag &= ~flag_rf_receive_page;
    }
}

extern volatile uint8_t uart2_rx_done;
extern volatile uint32_t uart2_rx_index;
extern uint8_t uart2_rx_buf[UART_FIFO_MAX_COUNT];

void user_application_normal_mode(void)
{
    user_rf_receive_set();
    while (system_data.system_mode == SYSTEM_NORMAL)
    {
        user_rf_receive_handle();
        if (uRXDataLens)
        {
            user_rf_rx_handler();

            uRXDataLens = 0;
        }

        else if(retransmit_count != 0){
            retransmit_count--;
            memcpy(rf_fifo_data,retransmit,retransmit_size);
            SwitchToTxMode();
            // Delay_ms(1);
            
            if (driver_rf_data_send_ACK(retransmit_size))
            {
                retransmit_count = 0;
            }

            SwitchToRxMode();
            // Delay_ms(1);
        }

        // 串口手动发送
        else if (uart2_rx_done)
        {

            if (uart2_rx_index > MAX_PACKET_LEN)
            {
                uart2_rx_index = 0;
                uart2_rx_done  = 0;
                continue;
            }
            for (uint8_t i = 0; i < uart2_rx_index; i++)
            {
                rf_fifo_data[i] = uart2_rx_buf[i];
            }

            SwitchToTxMode();
            // Delay_ms(1);

            if (driver_rf_data_send_ACK(uart2_rx_index)){
            }
            else {
                memcpy(retransmit,uart2_rx_buf,uart2_rx_index);
                retransmit_size = uart2_rx_index;
                retransmit_count = 3;
            }

            SwitchToRxMode();
            // Delay_ms(1);

            uart2_rx_done  = 0;
            uart2_rx_index = 0;
        }

        // 发送处理
        else if (tx_send_flag)
        {

            for (uint8_t i = 0; i < 10; i++)
            {
                rf_fifo_data[i] = tx_send_buffer[i];
            }

            SwitchToTxMode();
            Delay_ms(1);

            
            if (driver_rf_data_send_ACK(10))
            {
            }else{
                retransmit_count = 3;
                retransmit_size = 10;
                memcpy(retransmit,tx_send_buffer,10);
            }

            SwitchToRxMode();
            Delay_ms(1);

            tx_send_flag = 0;
        }

        // 主动跳频
        if (ch_chan)
        {
            ch_chan = 0;

            jump_frequency();
        }
    }
}

void user_rf_receive_page_set(void)
{
    TRX_SETUP_RETR = 0x08; // 250us,redo 8times
    FLUSH_RX;
    FLUSH_TX;
    TRX_IRQ_STATUS = 0x70;

    driver_rf_ouput_power_value_set(POWER_VALUE_5DBM); //

    SwitchToRxMode();
    Delay_ms(1);
}

#define PAIR_HEADER 0xAA
#define PAIR_FLAG   0xBB

void user_application_page_mode(void)
{
    uart_printf("enter page mode\r\n");
    uint8_t TXAddr[6] = {0};

    uint8_t selfAddr[6] = {0};
    flash_read_data(selfAddr, 0x7e000,6);
	uart_printf("selfAddr=0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n", selfAddr[0], selfAddr[1], selfAddr[2], selfAddr[3], selfAddr[4], selfAddr[5]);

    TRX_CE    = 0;
    TRX_RF_CH = 0x80 + 100;
    TRX_CE    = 1;
    // user_rf_receive_page_set(); // Rx Mode

    memcpy_2461(&TRX_RX_ADDR_P0_0, rf_address_tx_pub, RF_ADDRESS_LEN);
    memcpy_2461(&TRX_TX_ADDR_0, rf_address_tx_pub, RF_ADDRESS_LEN);

    while (system_data.system_mode == SYSTEM_PAGE)
    {
        FLUSH_RX;
        FLUSH_TX;
        if (pairStep == 0)
        {

            SwitchToRxMode();
            Delay_ms(20);

            RF_flag &= ~flag_rf_receive_page;
            driver_rf_receive_package();

            if (RF_flag & flag_rf_receive_page)
            {
                uart_printf("[success] receive pair addr\r\n");
                if (
                    (uRXDataLens == 8) &&
                    (rf_fifo_data[0] == PAIR_HEADER) &&
                    (rf_fifo_data[1] == PAIR_FLAG))
                {
                    TXAddr[0] = rf_fifo_data[2];
                    TXAddr[1] = rf_fifo_data[3];
                    TXAddr[2] = rf_fifo_data[4];
                    TXAddr[3] = rf_fifo_data[5];
                    TXAddr[4] = rf_fifo_data[6];
                    TXAddr[5] = rf_fifo_data[7];

                    rf_fifo_data[0] = PAIR_HEADER;
                    rf_fifo_data[1] = PAIR_FLAG;
                    rf_fifo_data[2] = selfAddr[0];
                    rf_fifo_data[3] = selfAddr[1];
                    rf_fifo_data[4] = selfAddr[2];
                    rf_fifo_data[5] = selfAddr[3];
                    rf_fifo_data[6] = selfAddr[4];
                    rf_fifo_data[7] = selfAddr[5];

                    SwitchToTxMode();
                    Delay_ms(15);
                    if (driver_rf_data_send_ACK(8))
                    {
                        uart_printf("[success] send pair addr\r\n");
                        pairStep = 1;
                    }
                    else
                    {
                        uart_printf("[fail] send pair addr\r\n");
                    }
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
            SwitchToRxMode();
            Delay_ms(10);
            RF_flag &= ~flag_rf_receive_page;
            driver_rf_receive_package();

            if (RF_flag & flag_rf_receive_page)
            {
                uart_printf("[success] receive pair addr + remote addr\r\n");
                if (
                    (uRXDataLens == 14) &&
                    (rf_fifo_data[0] == PAIR_HEADER) &&
                    (rf_fifo_data[1] == PAIR_FLAG) &&
                    (rf_fifo_data[2] == TXAddr[0]) &&
                    (rf_fifo_data[3] == TXAddr[1]) &&
                    (rf_fifo_data[4] == TXAddr[2]) &&
                    (rf_fifo_data[5] == TXAddr[3]) &&
                    (rf_fifo_data[6] == TXAddr[4]) &&
                    (rf_fifo_data[7] == TXAddr[5]) &&
                    (rf_fifo_data[8] == selfAddr[0]) &&
                    (rf_fifo_data[9] == selfAddr[1]) &&
                    (rf_fifo_data[10] == selfAddr[2]) &&
                    (rf_fifo_data[11] == selfAddr[3]) &&
                    (rf_fifo_data[12] == selfAddr[4]) &&
                    (rf_fifo_data[13] == selfAddr[5]))
                {
                    rf_fifo_data[0]  = PAIR_HEADER;
                    rf_fifo_data[1]  = PAIR_FLAG;
                    rf_fifo_data[2]  = selfAddr[0];
                    rf_fifo_data[3]  = selfAddr[1];
                    rf_fifo_data[4]  = selfAddr[2];
                    rf_fifo_data[5]  = selfAddr[3];
                    rf_fifo_data[6]  = selfAddr[4];
                    rf_fifo_data[7]  = selfAddr[5];
                    rf_fifo_data[8]  = TXAddr[0];
                    rf_fifo_data[9]  = TXAddr[1];
                    rf_fifo_data[10] = TXAddr[2];
                    rf_fifo_data[11] = TXAddr[3];
                    rf_fifo_data[12] = TXAddr[4];
                    rf_fifo_data[13] = TXAddr[5];

                    SwitchToTxMode();
                    Delay_ms(15);
                    if (driver_rf_data_send_ACK(14))
                    {
                        uart_printf("[success] send pair addr + remote addr\r\n");
                        RF_flag |= flag_rf_paged;
                        uart_printf("[success] pair ok\r\n");
                        break;
                    }
                }
            }
        }
    }
    
    system_data.system_mode = SYSTEM_NORMAL;


    if ((RF_flag & flag_rf_paged))
    {
        uart_printf("TXAddr=0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n", TXAddr[0], TXAddr[1], TXAddr[2], TXAddr[3], TXAddr[4], TXAddr[5]);
        Delay_ms(100);
        
        flash_init();
        flash_write_some_data(TXAddr, 0x7d210, 6);
    }

    P0_Address[0] = selfAddr[0];
    P0_Address[1] = selfAddr[1];
    P0_Address[2] = selfAddr[2];
    P0_Address[3] = selfAddr[3];
    P0_Address[4] = selfAddr[4];
    P0_Address[5] = selfAddr[5];
    memcpy_2461(&TRX_RX_ADDR_P0_0, P0_Address, RF_ADDRESS_LEN);
    memcpy_2461(&TRX_TX_ADDR_0, P0_Address, RF_ADDRESS_LEN);

    f24GetFreqConfigValue();
    SwitchToRxMode();
    driver_rf_ouput_power_value_set(cRFPWR);
    RF_CMD_FLUSH_TX;
    RF_CMD_FLUSH_RX;
    uart_printf("[success] exiting page mode\r\n");
    jump_frequency();
}

void user_fn24_main(void)
{
    xvr_reg_initial_24();

    user_application_initial();

    system_data.system_mode = SYSTEM_PAGE;

    while (1)
    {
        if (system_data.system_mode == SYSTEM_NORMAL)
        {
            user_application_normal_mode();
        }
        else if (system_data.system_mode == SYSTEM_PAGE)
        {
            user_application_page_mode();
        }
    }
}

uint16_t bat_adc = 0x0F00;
void user_rf_rx_handler()
{
    switch (rf_fifo_data[0])
    {
    case KeyPageUp:
        key_send(0x00, KEY_PAGEUP);
        key_send(0x00, 0x00);
        break;
    case KeyPageUpLongPress:
        if (AIR_MOUSE_KEY_FLAGS & AIR_MOUSE_FULLSCREEN_FLAG)
        {
            key_send(0x00, KEY_ESC);
            AIR_MOUSE_KEY_FLAGS &= ~AIR_MOUSE_FULLSCREEN_FLAG;
            key_send(0x00, 0x00);
        }
        else
        {
            key_send(KEY_MOD_LSHIFT, KEY_F5);
            AIR_MOUSE_KEY_FLAGS |= AIR_MOUSE_FULLSCREEN_FLAG;
            key_send(0x00, 0x00);
        }
        break;
    case KeyPageDown:
        key_send(0x00, KEY_PAGEDOWN);
        key_send(0x00, 0x00);
        break;
    case KeyPageDownLongPress:
        key_send(0x00, KEY_B);
        key_send(0x00, 0x00);
        break;
    case KeyDigitalPointer:
        status_report(0x02, 0x00, bat_adc, 0x09);
        break;
    case KeyDigitalPointerLongPress:
        break;
    case KeyFuncKeyTop:
        status_report(0x06, 0x00, bat_adc, 0x09);
        break;
    case KeyFuncKeyTopLongPress:
        break;
    case KeyFuncKeyBottom:
        status_report(0x03, 0x00, bat_adc, 0x09);
        break;
    case KeyFuncKeyBottomLongPress:
        key_send(KEY_MOD_LALT, KEY_TAB);
        key_send(KEY_MOD_LALT, 0);
        tab_switch_en = 1;
        break;
    case 0x0A: // 鼠标数据发送
        if(uRXDataLens != 5) break;
        mouse_send(((rf_fifo_data[1] << 8) + rf_fifo_data[2]),((rf_fifo_data[3] << 8) + rf_fifo_data[4]));
				uart_printf("rec ok!");
        break;
    case 0x0B: // 功能切换
        status_report(0x07, 0x00, bat_adc, 0x09);
        break;
    case 0x0C: // 电量设置
        if(uRXDataLens != 3) break;
        bat_adc = (rf_fifo_data[1] << 8) + rf_fifo_data[2];
        break;
    case 0x0F:
        for (uint8_t a = 0; a < uRXDataLens; a++)
        {
            uart_printf("%x", rf_fifo_data[a]);
        }
        break;
    case 0xAC: // 状态清除
        tab_switch_en = 0;
        tab_switch_ct = 0;
        status_report(0x00, 0x00, bat_adc, 0x09);
        key_send(0x00, 0x00);
        break;
    case 0xCC: // 跳频
        jump_frequency();
        break;
    case 0xF4:
        system_data.system_mode = SYSTEM_PAGE;
        break;
    case 0xFA:
        if (uRXDataLens == 3)
        {
            ch_chan = 1;
        }
        break;
    default:
        break;
    }
}

// ====================================================================================
// MOUSE RELATIVE REPORT
// ====================================================================================

void status_report(uint8_t key1, uint8_t key2, uint16_t adc_value, uint8_t pair_status)
{
    uint8_t arr[10]; // 正常发送 0x01, 0x00, 0x00, 0x2a, 0x0f, 0x00, 0x09, 0x00, 0x00, 0x00
    arr[0] = 0x01;

    // 按键同步位
    // 0x01: DeviceButton.LaserPointer
    // 0x02: DeviceButton.DigitalPointer
    // 0x03: DeviceButton.FuncKeyBottom
    // 0x04: DeviceButton.PageUp
    // 0x05: DeviceButton.PageDown
    // 0x06: DeviceButton.FuncKeyTop
    // 0x07: DeviceButton.DigitalPointerDoubleClick
    // 0x08: DeviceButton.Record
    arr[1] = key1;
    arr[2] = key2;

    // 电池 bat
    // 是否充电 = bat & 0x0001
    // int minVoltage = 3300;
    // int warinigVoltage = 3500;
    // int fullVoltage = 4000;
    // int percent = (((int)bat - warinigVoltage) * 75 / (fullVoltage - warinigVoltage)) + 25;              // (bat - 3500) * 0.15 + 25
    // if (bat < warinigVoltage) percent = ((int)bat - minVoltage) * 25 / (warinigVoltage - minVoltage);    // (bat - 3300) * 0.125
    // percent > 75 4格满电
    // percent > 50 3格电
    // percent > 25 2格电
    // percent > 0  1格电
    // percent = 0  0格电
    arr[3] = adc_value & 0xFF;
    arr[4] = adc_value >> 8;

    // 型号
    arr[5] = 0x00;

    // 匹配状态 ?
    arr[6] = pair_status;

    arr[7] = 0x00;
    arr[8] = 0x00;
    arr[9] = 0x00;

    AplUsb_StartTx(4, arr, 10);
    Delay_ms(50);
}

void key_send(uint8_t keyfunc, uint8_t keycode)
{
    uint8_t arr[9];
    arr[0] = 0x03;
    arr[1] = keyfunc; // key code id E0 - E7
    arr[2] = 0x00;    // key code id E0 - E7
    arr[3] = keycode; // key code id 00 - FF
    arr[4] = 0x00;    // key code id 00 - FF
    arr[5] = 0x00;    // key code id 00 - FF
    arr[6] = 0x00;    // key code id 00 - FF
    arr[7] = 0x00;    // key code id 00 - FF
    arr[8] = 0x00;    // key code id 00 - FF

    AplUsb_StartTx(1, arr, 9);
    Delay_ms(200);
}

void mouse_send(uint16_t x, uint16_t y)
{
    uint8_t arr[6];
    arr[0] = 0x01; // 0x01 时是鼠标
    arr[1] = 0x00;

    arr[2] = x & 0xFF; // 鼠标模式 x轴 范围 0 - 32767
    arr[3] = x >> 8;   // 鼠标模式 x轴 头8位

    arr[4] = y & 0xFF; // 鼠标模式 y轴 范围 0 - 32767
    arr[5] = y >> 8;   // 鼠标模式 y轴 头8位
    AplUsb_StartTx(1, arr, 6);
    Delay_ms(100);
}

void EP3_callback(void *_arr, int len)
{
    uint8_t *arr = (uint8_t *)_arr;
    uint8_t vibrationCountdown;
    uint8_t vibrationDuration;
    if (len != 10)
        return;
    switch (arr[1])
    {
    case 0x03: // key change
        // TODO: reset all key code
        // only clear key can be change in desktop application
        if (arr[8] == 0x05 && arr[9] == 0x4c)
        {
            // TODO: change clear key long press to CTRL+ALT+DELETE
        }

        break;
    case 0x04: // start pair
        // TODO: disconnect -> resend status -> complete pair
        system_data.system_mode = SYSTEM_PAGE;
        break;
    case 0x05: // vibration mode
        tx_send_buffer[0] = 0x05;
        if (!(arr[3] & 1))
        {
            // TODO: close vibration
            tx_send_buffer[1] = 0x00;
            tx_send_flag      = 1;
            break;
        }
        // TODO: open vibration
        vibrationCountdown = arr[4]; // minutes  5 - 180 minutes
        vibrationDuration  = arr[6]; // seconds 1 - 10 seconds

        tx_send_buffer[1] = 0x01;
        tx_send_buffer[2] = vibrationCountdown;
        tx_send_buffer[3] = vibrationDuration;
        tx_send_flag      = 1;
        break;
    default:
        // other settings
        tx_send_buffer[0] = 0x06;
        if (arr[1] & 0x40)
        {
            // open air mouse
            tx_send_buffer[1] = 0x01;
        }
        else
        {
            // close air mouse
            tx_send_buffer[1] = 0x00;
        }

        if (arr[1] & 0x20)
        {
            // open single monitor
            tx_send_buffer[1] |= 0x02;
        }
        else
        {
            // close single monitor
            tx_send_buffer[1] |= 0x02;
        }

        uint8_t cursor_speed = arr[1] & 0x0f; // cursor speed value of 1 - 10 means 10% - 100%
        tx_send_buffer[1] |= cursor_speed << 4;

        // uint16_t resolutionWidth = (arr[7] << 8) +  arr[6];
        tx_send_buffer[2] = arr[7];
        tx_send_buffer[3] = arr[6];

        // uint16_t resolutionHeight = (arr[9] << 8) + arr[8];
        tx_send_buffer[4] = arr[9];
        tx_send_buffer[5] = arr[8];

        // uint16_t mouseX = (arr[3] << 8) +  arr[2];
        tx_send_buffer[6] = arr[3];
        tx_send_buffer[7] = arr[2];
        // uint16_t mouseY = (arr[5] << 8) +  arr[4];
        tx_send_buffer[8] = arr[5];
        tx_send_buffer[9] = arr[4];

        tx_send_flag = 1;
        break;
    }
}

// =================================================================
// 跳频
// =================================================================

void jump_frequency()
{
    // uart_printf("channel %d will be change \r\n", TRX_RF_CH);

    ch_cur_index++;
    ch_cur_index %= MAX_FREQ_SIZE;
    
    uCH_ChCt = 0;
    TRX_CE    = 0;
    TRX_RF_CH = 0x80 + channel_table[ch_cur_index];
    TRX_CE    = 1;

    // uart_printf("channel change to %d \r\n", TRX_RF_CH);
}
