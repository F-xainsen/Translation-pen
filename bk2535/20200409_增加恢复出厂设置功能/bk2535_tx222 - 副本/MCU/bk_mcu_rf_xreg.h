#ifndef __BK_MCU_RF_XREG_H__
#define __BK_MCU_RF_XREG_H__

#include <absacc.h>
#define  PCHAR    (unsigned char volatile xdata * )

/*****************************************************/
//USB
#define FRAM_NO_0           XBYTE[0x0808]
#define FRAM_NO_1           XBYTE[0x0809]

#define USBINT0             XBYTE[0x080a]
#define USBINT1             XBYTE[0x080b]
#define USB_EN0             XBYTE[0x080c]
#define USB_EN1             XBYTE[0x080d]
#if MCU_TYPE == BK2433
#define EP_STATUS           XBYTE[0x080e]
#else
#define EP_STATUS_IN        XBYTE[0x080e]
#define EP_STATUS_OUT       XBYTE[0x080f]
#endif

#define CFG_EP0_1           XBYTE[0x0810]
#define CFG_EP0_0           XBYTE[0x0811]

#define CFG_EP1_1           XBYTE[0x0812]
#define CFG_EP1_0           XBYTE[0x0813]

#define CFG_EP2_1           XBYTE[0x0814]
#define CFG_EP2_0           XBYTE[0x0815]

#define CFG_EP3_1           XBYTE[0x0816]
#define CFG_EP3_0           XBYTE[0x0817]

#define CFG_EP4_1           XBYTE[0x0818]
#define CFG_EP4_0           XBYTE[0x0819]

#define CFG_EP5_1           XBYTE[0x081a]
#define CFG_EP5_0           XBYTE[0x081b]

#define CFG_EP6_1           XBYTE[0x081c]
#define CFG_EP6_0           XBYTE[0x081d]

#define CFG_EP7_1           XBYTE[0x081e]
#define CFG_EP7_0           XBYTE[0x081f]

#define EP_HALT             XBYTE[0x0820]
#define EP_RDY              XBYTE[0x0821]
#define FUNCT_ADR           XBYTE[0x0822]


#define  CNT0               XBYTE[0x0823]
#define  CNT1               XBYTE[0x0824]
#define  CNT2               XBYTE[0x0825]
#define  CNT3               XBYTE[0x0826]
#define  CNT4               XBYTE[0x0827]
#define  CNT5               XBYTE[0x0828]
#define  CNT6               XBYTE[0x0829]
#define  CNT7               XBYTE[0x082a]
#define  CNT0_HBIT          XBYTE[0x082b]
#define  CNT1_HBIT          XBYTE[0x082c]
#define  CNT2_HBIT          XBYTE[0x082d]
#define  CNT3_HBIT          XBYTE[0x082e]
#define  CNT4_HBIT          XBYTE[0x082f]
#define  CNT5_HBIT          XBYTE[0x0830]
#define  CNT6_HBIT          XBYTE[0x0831]
#define  CNT7_HBIT          XBYTE[0x0832]
#define  EP_ADDR_MSB        XBYTE[0x0840]

#define  USB_PWR_CN         XBYTE[0x0841]
#define  USB_DEBUG          XBYTE[0x0842]
#define  USB_SOFT_RST       XBYTE[0x0843]

// temp


//#define  EP_PP_DEF          XBYTE[0x0370]

// BK2401
#define  BK2401_CONFIG      XBYTE[0x0880]
#define  BK2401_ENAA        XBYTE[0x0881]
#define  BK2401_ENRX        XBYTE[0x0882]
#define  BK2401_AW          XBYTE[0x0883]
#define  BK2401_RETR        XBYTE[0x0884]
#define  BK2401_RFCH        XBYTE[0x0885]
#define  BK2401_SETUP       XBYTE[0x0886]


#define  BK2401_R0_ADDR    (PCHAR(0x0887))
#define  BK2401_R0_ADDR_ARRAY  (PCHAR(0x0887))
#define  BK2401_R1_ADDR    (PCHAR(0x088C))
#define  BK2401_R1_ADDR_ARRAY  (PCHAR(0x088C))
#define  BK2401_R2_ADDR    XBYTE[0x0891]
#define  BK2401_R3_ADDR    XBYTE[0x0892]
#define  BK2401_R4_ADDR    XBYTE[0x0893]
#define  BK2401_R5_ADDR    XBYTE[0x0894]

#define  BK2401_TX_ADDR    (PCHAR(0x0895))
#define  BK2401_TX_ADDR_ARRAY  (PCHAR(0x0895))

#define  BK2401_R0_ADDR_0   XBYTE[0x0887]
#define  BK2401_R0_ADDR_1   XBYTE[0x0888]
#define  BK2401_R0_ADDR_2   XBYTE[0x0889]
#define  BK2401_R0_ADDR_3   XBYTE[0x088A]
#define  BK2401_R0_ADDR_4   XBYTE[0x088B]
#define  BK2401_R1_ADDR_0   XBYTE[0x088C]
#define  BK2401_R1_ADDR_1   XBYTE[0x088D]
#define  BK2401_R1_ADDR_2   XBYTE[0x088E]
#define  BK2401_R1_ADDR_3   XBYTE[0x088F]
#define  BK2401_R1_ADDR_4   XBYTE[0x0890]
#define  BK2401_TX_ADDR_0   XBYTE[0x0895]
#define  BK2401_TX_ADDR_1   XBYTE[0x0896]
#define  BK2401_TX_ADDR_2   XBYTE[0x0897]
#define  BK2401_TX_ADDR_3   XBYTE[0x0898]
#define  BK2401_TX_ADDR_4   XBYTE[0x0899]

#define  BK2401_R0_PW       XBYTE[0x089A]
#define  BK2401_R1_PW       XBYTE[0x089B]
#define  BK2401_R2_PW       XBYTE[0x089C]
#define  BK2401_R3_PW       XBYTE[0x089D]
#define  BK2401_R4_PW       XBYTE[0x089E]
#define  BK2401_R5_PW       XBYTE[0x089F]
#define  BK2401_DYNPD       XBYTE[0x08A0]
#define  BK2401_FEATURE     XBYTE[0x08A1]

#define  BK2401_CFG_0C_0    XBYTE[0x08A2]
//#define  BK2401_CFG_0D_3    XBYTE[0x08A9]

#define  BK2401_CFG_0C_1    XBYTE[0x08A3]
#define  BK2401_CFG_0C_2    XBYTE[0x08A4]
#define  BK2401_CFG_0C_3    XBYTE[0x08A5]
#define  BK2401_CFG_0D_0    XBYTE[0x08A6]
#define  BK2401_CFG_0D_1    XBYTE[0x08A7]
#define  BK2401_CFG_0D_2    XBYTE[0x08A8]
#define  BK2401_CFG_0D_3    XBYTE[0x08A9]
#define  BK2401_RAMP_TAB_0  XBYTE[0x08AA]
#define  BK2401_RAMP_TAB_1  XBYTE[0x08AB]
#define  BK2401_RAMP_TAB_2  XBYTE[0x08AC]
#define  BK2401_RAMP_TAB_3  XBYTE[0x08AD]
#define  BK2401_RAMP_TAB_4  XBYTE[0x08AE]
#define  BK2401_RAMP_TAB_5  XBYTE[0x08AF]
#define  BK2401_RAMP_TAB_6  XBYTE[0x08B0]
#define  BK2401_RAMP_TAB_7  XBYTE[0x08B1]
#define  BK2401_RAMP_TAB_8  XBYTE[0x08B2]
#define  BK2401_RAMP_TAB_9  XBYTE[0x08B3]
#define  BK2401_RAMP_TAB_A  XBYTE[0x08B4]

#define  BK2401_CFG_0C_ARRAY    (PCHAR(0X08A2))
#define  BK2401_CFG_0D_ARRAY    (PCHAR(0X08A6))
#define  BK2401_RAMP_TAB_ARRAY  (PCHAR(0X08AA))

#define  BK2401_CE          XBYTE[0x08B5]
#define  BK2401_CMD         XBYTE[0x08B6]
#define  BK2401_FIFO        XBYTE[0x08B7]
#define  BK2401_SDATA       (PCHAR(0x08B8))
#define  BK2401_SDATA_0     XBYTE[0x08B8]
#define  BK2401_SDATA_1     XBYTE[0x08B9]
#define  BK2401_SDATA_2     XBYTE[0x08BA]
#define  BK2401_SDATA_3     XBYTE[0x08BB]
/*
#define  BK2401_sdata_1     XBYTE[0x08B9]
#define  BK2401_sdata_2     XBYTE[0x08BA]
#define  BK2401_sdata_3     XBYTE[0x08BB]
*/
#define  BK2401_SCTRL       XBYTE[0x08BC]
#define  BK2401_KCALOUT_H   XBYTE[0x08BE]
#define  BK2401_KCALOUT_L   XBYTE[0x08BF]
#define  BK2401_STATUS      XBYTE[0x08C0]
#define  BK2401_OBSERVETX   XBYTE[0x08C1]
#define  BK2401_CDSTATUS    XBYTE[0x08C2]
#define  BK2401_FIFOSTATUS  XBYTE[0x08C3]
#define  BK2401_RPL_WIDTH   XBYTE[0x08C4]
#define  BK2401_MBIST_ST    XBYTE[0x08C5]
#define  BK2401_CHIP_ID_LOW    XBYTE[0X08C6]
#define  BK2401_CHIP_ID_HIGH   XBYTE[0X08C7]
#define  BK2401_BIT_CNT    (PCHAR(0x08C8))
#define  BK2401_BIT_CNT_ARRAY  (PCHAR(0X08C8))
/*
#define  BK2401_bit_cnt_1   XBYTE[0x08C9]
#define  BK2401_bit_cnt_2   XBYTE[0x08CA]
#define  BK2401_bit_cnt_3   XBYTE[0x08CB]
*/
#define  BK2401_ERR_CNT     (PCHAR(0x08CC))
#define  BK2401_ERR_CNT_ARRAY  (PCHAR(0X08CC))
/*
#define  BK2401_err_cnt_1   XBYTE[0x08CD]
#define  BK2401_err_cnt_2   XBYTE[0x08CE]
#define  BK2401_err_cnt_3   XBYTE[0x08CF]
*/

#if MCU_TYPE == BK2535

#define  TX_FREQ_OFFSET     (PCHAR(0x08D0))
/*
#define  TX_FREQ_OFFSET_1   XBYTE[0x08D1]
#define  TX_FREQ_OFFSET_2   XBYTE[0x08D2]
#define  TX_FREQ_OFFSET_3   XBYTE[0x08D3]
*/
#define  RX_FREQ_OFFSET     (PCHAR(0x08D4))
/*
#define  TX_FREQ_OFFSET_1   XBYTE[0x08D5]
#define  TX_FREQ_OFFSET_2   XBYTE[0x08D6]
#define  TX_FREQ_OFFSET_3   XBYTE[0x08D7]
*/
#define  MOD_DELAY          XBYTE[0x08D8]
#define  PLL_SDM            XBYTE[0x08D9]
#define  FM_GAIN_H          XBYTE[0x08DA]
#define  FM_GAIN_L          XBYTE[0x08DB]
#define  FM_KMOD_SET_H      XBYTE[0x08DC]
#define  FM_KMOD_SET_L      XBYTE[0x08DD]
#define  MOD_COEFFICIENT_H  XBYTE[0x08DE]
#define  MOD_COEFFICIENT_L  XBYTE[0x08DF]
#define  ANA_CTRL10         XBYTE[0x08E0]
#define  ANA_CTRL11         XBYTE[0x08E1]   //
#define  ANA_CTRL12         XBYTE[0x08E2]
#define  ANA_CTRL13         XBYTE[0x08E3]

#define  ANA_IND            XBYTE[0x08F9]
#define  CHIP_ID_H          XBYTE[0x08FA]
#define  CHIP_ID_L          XBYTE[0x08FB]
#define  DEVICE_ID          (PCHAR(0x08FC))
/*
#define  device_id_l       XBYTE[0x08FD]
#define  device_id_2       XBYTE[0x08FE]
#define  device_id_3       XBYTE[0x08FF]
*/
#endif

#if MCU_TYPE & MCU_FLASH
// EEPROM
#define  EEPROM_KEY         XBYTE[0x0910]
#define  EEPROM_STAT        XBYTE[0x0911]
#define  EEPROM_ADDR        XBYTE[0x0912]
#define  EEPROM_DAT         XBYTE[0x0913]
#define  EEPROM_CLK_EN      XBYTE[0x0914]
// e2prom
#define E2PROM_KEY          XBYTE[0x0910]
#define E2PROM_STA          XBYTE[0x0911]
#define E2PROM_ADD          XBYTE[0x0912]
#define E2PROM_DAT          XBYTE[0x0913]
#define E2PROM_CKEN         XBYTE[0x0914]

#endif

#if MCU_TYPE == BK2535

//FLASH¡¡control
#define  FLASH_KEY          XBYTE[0x0900]
#define  FLASH_CTL          XBYTE[0x0901]
#define  FLASH_ADR          XBYTE[0x0902]
#define  FLASH_ADR_H        XBYTE[0x0903]
#define  FLASH_DAT          XBYTE[0x0904]
#define  NVR_WP0            XBYTE[0x0905]
#define  NVR_WP1            XBYTE[0x0906]
#define  NVR_WF0            XBYTE[0x0907]
#define  NVR_WF1            XBYTE[0x0908]
#define  NVR_WF_MAIN        XBYTE[0x0909]

#define  MTP_KEY            XBYTE[0x0900]
#define  MTP_CTL            XBYTE[0x0901]
#define  MTP_ADR            XBYTE[0x0902]
#define  MTP_ADR_H          XBYTE[0x0903]
#define  MTP_DAT            XBYTE[0x0904]
#define  nvr_wp0            XBYTE[0x0905]
#define  nvr_wp1            XBYTE[0x0906]
#define  nvr_wf0            XBYTE[0x0907]
#define  nvr_wf1            XBYTE[0x0908]
#define  nvr_wf_main        XBYTE[0x0909]


//RTC TIMER
#define  RTC_TIMER_CTRL     XBYTE[0x0918]
#define  RTC_TIMER_CTL      XBYTE[0x0918]
#define  RTC_COUNT_H        XBYTE[0x0919]
#define  RTC_COUNT_L        XBYTE[0x091a]

//ADC
#define  adc_datal          XBYTE[0x0920]
#define  adc_datah          XBYTE[0x0921]
#define  adc_ctl            XBYTE[0x0922]
#define  adc_rate           XBYTE[0x0923]
#define  adc_ctl2           XBYTE[0x0924]

#define  ADC_DATA_L         XBYTE[0x0920]
#define  ADC_DATA_H         XBYTE[0x0921]
#define  ADC_CTL            XBYTE[0x0922]
#define  ADC_RATE           XBYTE[0x0923]
#define  ADC_CTL2           XBYTE[0x0924]


//KEY WAKE
#define  KEY_EN             XBYTE[0x0928]
#define  KEY_STATUS         XBYTE[0x0929]

//RNG
#define  RNG_DATA           XBYTE[0x0930]
#define  RNG_CTRL           XBYTE[0x0931]

//PCA
#define  CCAP0H             XBYTE[0x0960]
#define  CCAP1H             XBYTE[0x0961]
#define  CCAP2H             XBYTE[0x0962]
#define  CCAP3H             XBYTE[0x0963]
#define  CCAP4H             XBYTE[0x0964]
#define  CCAP5H             XBYTE[0x0965]
#define  CCAP0L             XBYTE[0x0966]
#define  CCAP1L             XBYTE[0x0967]
#define  CCAP2L             XBYTE[0x0968]
#define  CCAP3L             XBYTE[0x0969]
#define  CCAP4L             XBYTE[0x096a]
#define  CCAP5L             XBYTE[0x096b]
#define  CCAMP0             XBYTE[0x096c]
#define  CCAMP1             XBYTE[0x096d]
#define  CCAMP2             XBYTE[0x096e]
#define  CCAMP3             XBYTE[0x096f]
#define  CCAMP4             XBYTE[0x0970]
#define  CCAMP5             XBYTE[0x0971]
#define  CCON               XBYTE[0x0972]
#define  CH                 XBYTE[0x0973]
#define  CL                 XBYTE[0x0974]
#define  CMOD               XBYTE[0x0975]
#define  CCAPO              XBYTE[0x0976]



//AES
#define  AES_KEY00          XBYTE[0x0980]
#define  AES_KEY01          XBYTE[0x0981]
#define  AES_KEY02          XBYTE[0x0982]
#define  AES_KEY03          XBYTE[0x0983]
#define  AES_KEY04          XBYTE[0x0984]
#define  AES_KEY05          XBYTE[0x0985]
#define  AES_KEY06          XBYTE[0x0986]
#define  AES_KEY07          XBYTE[0x0987]
#define  AES_KEY08          XBYTE[0x0988]
#define  AES_KEY09          XBYTE[0x0989]
#define  AES_KEY0A          XBYTE[0x098A]
#define  AES_KEY0B          XBYTE[0x098B]
#define  AES_KEY0C          XBYTE[0x098C]
#define  AES_KEY0D          XBYTE[0x098D]
#define  AES_KEY0E          XBYTE[0x098E]
#define  AES_KEY0F          XBYTE[0x098F]
#define  AES_TEXTIN00       XBYTE[0x0990]
#define  AES_TEXTIN01       XBYTE[0x0991]
#define  AES_TEXTIN02       XBYTE[0x0992]
#define  AES_TEXTIN03       XBYTE[0x0993]
#define  AES_TEXTIN04       XBYTE[0x0994]
#define  AES_TEXTIN05       XBYTE[0x0995]
#define  AES_TEXTIN06       XBYTE[0x0996]
#define  AES_TEXTIN07       XBYTE[0x0997]
#define  AES_TEXTIN08       XBYTE[0x0998]
#define  AES_TEXTIN09       XBYTE[0x0999]
#define  AES_TEXTIN0A       XBYTE[0x099A]
#define  AES_TEXTIN0B       XBYTE[0x099B]
#define  AES_TEXTIN0C       XBYTE[0x099C]
#define  AES_TEXTIN0D       XBYTE[0x099D]
#define  AES_TEXTIN0E       XBYTE[0x099E]
#define  AES_TEXTIN0F       XBYTE[0x099F]
#define  AES_TEXTOUT00      XBYTE[0x09A0]
#define  AES_TEXTOUT01      XBYTE[0x09A1]
#define  AES_TEXTOUT02      XBYTE[0x09A2]
#define  AES_TEXTOUT03      XBYTE[0x09A3]
#define  AES_TEXTOUT04      XBYTE[0x09A4]
#define  AES_TEXTOUT05      XBYTE[0x09A5]
#define  AES_TEXTOUT06      XBYTE[0x09A6]
#define  AES_TEXTOUT07      XBYTE[0x09A7]
#define  AES_TEXTOUT08      XBYTE[0x09A8]
#define  AES_TEXTOUT09      XBYTE[0x09A9]
#define  AES_TEXTOUT0A      XBYTE[0x09AA]
#define  AES_TEXTOUT0B      XBYTE[0x09AB]
#define  AES_TEXTOUT0C      XBYTE[0x09AC]
#define  AES_TEXTOUT0D      XBYTE[0x09AD]
#define  AES_TEXTOUT0E      XBYTE[0x09AE]
#define  AES_TEXTOUT0F      XBYTE[0x09AF]
#define  AES_CTL            XBYTE[0x09B0]
#define  AES_INT            XBYTE[0x09B1]

#define  AES_KEY_ARRAY      (PCHAR(0x0980))
#define  AES_TEXTIN_ARRAY   (PCHAR(0x0990))
#define  AES_TEXTOUT_ARRAY  (PCHAR(0x09A0))

#endif


#endif
