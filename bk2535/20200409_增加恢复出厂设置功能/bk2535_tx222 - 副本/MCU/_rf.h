#ifndef __RF_H_
#define __RF_H_


#define PROJECT_ID		100


//#define __RFX2401C_RX P23
//#define __RFX2401C_TX P24

#define RFX2401C_TX() //__RFX2401C_RX=0, __RFX2401C_TX=1
#define RFX2401C_RX() //__RFX2401C_TX=0, __RFX2401C_RX=1
#define RFX2401C_SHUTDOWN() //__RFX2401C_RX=0, __RFX2401C_TX=0


#define RF_2Mbps      (1<<3)
#define RF_1Mbps      (0<<3)
#define RF_250kbps    (4<<3)

#define RF_N10dbm     (0<<1)  //-10 
#define RF_N5dbm      (1<<1)  //-5 
#define RF_0dbm       (2<<1)  //0 
#define RF_5dbm       (3<<1)  //5

#define RF_LNA_GAIN_HIGH    1
#define RF_LNA_GAIN_LOW     0


//#define RF_BAND       RF_1Mbps  
#define RF_BAND       RF_250kbps


#define RF_MODE_SINGLE    0
#define RF_MODE_NORMAL    1

#define RF_POWER_UP         {BK2401_CONFIG |= 0x02;}
#define RF_POWER_DOWN       {BK2401_CONFIG &= 0xfd;}

#define RF_CHIP_ENABLE      {BK2401_CE = 0x01;}
#define RF_CHIP_DISABLE     {BK2401_CE = 0x00;}

#define CE_LOW()            BK2401_CE = 0/*; TEST_PIN1 = 0*/
#define CE_HIGH()           BK2401_CE = 1/*; TEST_PIN1 = 1*/


#define CMD_NOP                0x00
#define CMD_FLUSH_RX           0x80
#define CMD_FLUSH_TX           0xA0
#define CMD_REUSE_TX_PL        0x10
#define CMD_R_RX_PAYLOAD       0x40
#define CMD_W_TX_PAYLOAD       0x60
#define CMD_W_ACK_PAYLOAD      0x68              //+PIPE, see W_ACK_PAYLOAD
#define CMD_W_TX_PAYLAOD_NOACK 0x68       //see W_TX_PAYLOAD_NOACK

// Interrupt status
#define STATUS_RX_TX_MAX        0x70
#define STATUS_RX_TX_MAX_TXFULL 0x71
#define STATUS_RX_DR            0x40
#define STATUS_TX_DS            0x20
#define STATUS_MAX_RT           0x10
#define STATUS_TX_FULL          0x01

#define RF_CMD_NOP                   {BK2401_CMD = CMD_NOP;}
#define RF_CMD_FLUSH_TX              {BK2401_CMD = CMD_FLUSH_TX;}    //Clear tx buffer
#define RF_CMD_FLUSH_RX              {BK2401_CMD = CMD_FLUSH_RX;}    //Clear rx buffer
#define RF_CMD_REUSE_TX_PL           {BK2401_CMD = CMD_REUSE_TX_PL;}
#define RF_CMD_R_RX_PAYLOAD          {BK2401_CMD = CMD_R_RX_PAYLOAD;}
#define RF_CMD_W_TX_PAYLOAD          {BK2401_CMD = CMD_W_TX_PAYLOAD;}
#define RF_CMD_W_ACK_PAYLOAD(pipe)   {BK2401_CMD = CMD_W_ACK_PAYLOAD+pipe;}
#define RF_CMD_W_TX_PAYLAOD_NOACK    {BK2401_CMD = CMD_W_TX_PAYLAOD_NOACK;}

#define RF_STATUS_CLEAR_ALL          {BK2401_STATUS = STATUS_RX_TX_MAX;}
#define RF_STATUS_CLEAR_RX           {BK2401_STATUS = STATUS_RX_DR;}
#define RF_STATUS_CLEAR_TX           {BK2401_STATUS = STATUS_TX_DS;}
#define RF_STATUS_CLEAR_MAX_RT       {BK2401_STATUS = STATUS_MAX_RT;}


//// fifostatus
#define RX_EMPTY				0x01
#define RX_FULL					0x02
#define TX_EMPTY				0x10
#define TX_FULL					0x20
#define TX_REUSE				0x40


//extern code unsigned char code hooping_dyn[16];
extern code unsigned char code hooping_dyn[16+16+15];
extern XDATA unsigned char rf_address_rx2[5];

extern void BK2433_RF_Initial(void);
//extern void PowerUp_RF(void);
//extern void PowerDown_RF(void);

#define PowerUp_RF()    BK2401_CONFIG=(BK2401_CONFIG|0x02)
#define PowerDown_RF()  BK2401_CONFIG=(BK2401_CONFIG&0xfd)

extern void RF_Set_Mode(BYTE mode);

extern void SwitchToRxMode(void);
extern void SwitchToTxMode(void);

extern void rf_set_channel(unsigned char ch);
extern void rf_set_baud(unsigned char baud);
extern void rf_set_dbm(unsigned char dbm);
extern void rf_set_retr(unsigned char retr);

extern void SetChannel(void);

extern void set_rf_constant_address(void);
#define rf_set_publice_address()  set_rf_constant_address();
//extern void set_rf_5byte_address(void);
extern void set_rf_5byte_address(unsigned char addr0);

//extern void rf_set_tx_addr(void);
extern void rf_set_rx_addr(void);

extern void rf_clrint(void);

extern void W_BANK1_DIGITAL_REG(UINT8 *paddr, UINT8* pvalue, int len);
extern void W_BANK1_ANALOG_REG(UINT8 addr, UINT8* pvalue);
extern void W_BANK1_ANALOG_REG1(UINT8 addr, UINT32 value);
extern void W_BANK1_ANALOG_REG2(UINT8 addr, UINT32 value);


//RF output power in TX mode:       0:Low power(-30dB down)     1:High power 
#if MCU_TYPE == BK2533
#define rf_tx_high_power() W_BANK1_ANALOG_REG2(4, 0x9f823c28)
#define rf_tx_low_power()  W_BANK1_ANALOG_REG2(4, 0x9e11bc28)
#elif MCU_TYPE == BK2433
#define rf_tx_high_power() W_BANK1_ANALOG_REG2(4, 0xe19e860b) 
//#define rf_tx_low_power()  W_BANK1_ANALOG_REG2(4, 0xd18e860b)
#define rf_tx_low_power()  W_BANK1_ANALOG_REG2(4, 0xe18e860b)
#elif MCU_TYPE == BK2535 
#define rf_tx_high_power() ANA_CTRL11 = 0xff
#define rf_tx_low_power()  ANA_CTRL11 = 0xa1   /*调节数值*/
#endif

//RSSI Threshold for CD detect    0: -97 dBm, 2 dB/step, 15: -67 dBm 
#define rf_set_rssi_dbm(dbm)  W_BANK1_ANALOG_REG2(5, 0x01027FA6|((dbm&0xf)<<26))  //0x0d027FA6


// 0 - 1.2v, 1 - vdd/2
#define ADC_REF_2DOT4  0
#define ADC_REF_VDD    1

#define ADC_REF_VOL		ADC_REF_2DOT4

extern void adc_set_ref(bit ref, bit cel);




#define RF_CLR_ALL_INT  0x70
#define RF_TX_MODE      0x5E
#define RF_RX_MODE      0x3F
#define RF_TX_MODE2     0x0E
#define RF_RX_MODE2     0x0F



#endif
