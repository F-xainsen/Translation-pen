/**
 ****************************************************************************************
 *
 * @file otas.h
 *
 * @brief Header file - OTA Profile Server Role
 *
 * Copyright (C) beken 2009-2015
 *
 *
 ****************************************************************************************
 */
#ifndef _OTAS_H_
#define _OTAS_H_

/**
 ****************************************************************************************
 * @addtogroup  OTA 'Profile' Server
 * @ingroup OTAS
 * @brief OTAS 'Profile' Server
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "rwprf_config.h"

#if (BLE_OADS_SERVER)

#include "oads_task.h"
#include "atts.h"
#include "prf_types.h"
#include "prf.h"

/*
 * DEFINES
 ****************************************************************************************
 */
 
 

#define OAD_SERVICE_UUID       0xFFC0
#define OAD_IMG_IDENTIFY_UUID  0xFFC1
#define OAD_IMG_BLOCK_UUID     0xFFC2




#define OAD_CHAR_CNT          2



// OAD Characteristic Indices
#define OAD_CHAR_IMG_IDENTIFY 0
#define OAD_CHAR_IMG_BLOCK    1


// Image Identification size
#define OAD_IMG_ID_SIZE       4


// Image header size (version + length +  image id size)
#define OAD_IMG_HDR_SIZE      (  2 + 2  + OAD_IMG_ID_SIZE )

// The Image is transporte in 16-byte blocks in order to avoid using blob operations.
//需要根据实际的值修改
#define OAD_BLOCK_SIZE        16
#define HAL_FLASH_PAGE_SIZE   256

#define HAL_FLASH_WORD_SIZE   4

#define OAD_BLOCKS_PER_PAGE  (HAL_FLASH_PAGE_SIZE / OAD_BLOCK_SIZE)


#define OAD_BLOCK_APP_MAX        (0x2A00)  //(128K *1024*4) /64
#define OAD_BLOCK_STACK_MAX         (0x3E00)

typedef void (*FUNCPTR)(void);

#if defined(CFG_ALLROLES)

#if defined(__CC_ARM)
#define SEC_IMAGE_OAD_HEADER_APP_FADDR              (0x2f030)//for keil
#else
#define SEC_IMAGE_OAD_HEADER_APP_FADDR              (0x30ac0)//for gcc
#endif

#else
#ifdef __MOUSE__
#define SEC_IMAGE_OAD_HEADER_APP_FADDR                (0x2b070) 
#else
#define SEC_IMAGE_OAD_HEADER_APP_FADDR                (0x280a0)     
#endif
#endif

#ifdef __MOUSE__
#define SEC_IMAGE_OAD_HEADER_STACK_FADDR            (0x5006) 
#else
#define SEC_IMAGE_OAD_HEADER_STACK_FADDR            (0x20E0) 
#endif

#define SEC_BACKUP_APP_PART_UID_OAD_HEADER_FADDR    (0x52000) //328kb * 1024

#ifdef __MOUSE__
#define SEC_BACKUP_APP_STACK_UID_OAD_HEADER_FADDR    (0x41000) //256kb * 1024
#else
#define SEC_BACKUP_APP_STACK_UID_OAD_HEADER_FADDR    (0x40000) //256kb * 1024
#endif


#define SEC_BACKUP_ALLOC_END_FADDR                    (0x80000) //(512KB)
/*********************************************************************
 * MACROS
 */   
     
// Macros to get Image ID (LSB) and Version Number
#define OAD_IMG_ID( ver )    (ver)//( (ver) & 0x01 )
#define OAD_VER_NUM( ver )   ( (ver) >> 0x01 )

#define OAD_APP_PART_UID    (0x42424242)
#define OAD_APP_STACK_UID    (0x53535353)


/*********************************************************************
 * GLOBAL VARIABLES
 */ 

/*********************************************************************
 * TYPEDEFS
 */
// The Image Header will not be encrypted, but it will be included in a Signature.
typedef struct
{
    // Secure OAD uses the Signature for image validation instead of calculating a CRC, but the use
    // of CRC==CRC-Shadow for quick boot-up determination of a validated image is still used.
    uint32_t crc;       // CRC must not be 0x0000 or 0xFFFF.
    // User-defined Image Version Number - default logic uses simple a '!=' comparison to start an OAD.
    uint16_t ver;
    uint16_t len;        // Image length in 4-byte blocks (i.e. HAL_FLASH_WORD_SIZE blocks).
    uint32_t  uid;     // User-defined Image Identification bytes.
    uint8_t  crc_status;     // cur image crc status
    uint8_t  sec_status;     // cur image sec status
    uint16_t  rom_ver;     // Reserved space for future use.
} img_hdr_t;


enum
{
    SSTATUS_UNKOWN           = 0,
    SSTATUS_SECT_ERASED,
    SSTATUS_UPDATE_NORMAL,
    SSTATUS_UPDATE_ABNORMAL,
};

typedef struct
{
    img_hdr_t sec_hdr; 
    uint32_t erase_offset;
    uint32_t update_offset;
    uint32_t data_cnt;
    uint8_t  flag_write;
    uint8_t  flag_sleep;
    uint8_t  data[2000];
} OAD_SECTION_T, *OAD_SECTION_PTR;

extern OAD_SECTION_T bsec;
extern uint8_t latency_disable_state;



#define OADS_OAD_MANDATORY_MASK       (0xFFFFFFFF)


/// Bracelet Profile  Attributes Indexes
enum
{
ATT_USER_SERVER_SVC_OAD                    = ATT_UUID_16(OAD_SERVICE_UUID),

ATT_USER_SERVER_CHAR_IDENTIFY            = ATT_UUID_16(OAD_IMG_IDENTIFY_UUID),

ATT_USER_SERVER_CHAR_BLOCK                = ATT_UUID_16(OAD_IMG_BLOCK_UUID),
    
};


/// OAD service Attributes Indexes // 
enum
{
    OADS_IDX_SVC,          // 0XFFC0
    ///*************************  refer TI OAD design  ******************///
    OADS_IDX_FFC1_LVL_CHAR, 
    OADS_IDX_FFC1_LVL_VAL,
    OADS_IDX_FFC1_LVL_NTF_CFG,
    OADS_IDX_FFC1_USER_DECL,
    OADS_IDX_FFC2_LVL_CHAR,  
    OADS_IDX_FFC2_LVL_VAL,
    OADS_IDX_FFC2_LVL_NTF_CFG,
      OADS_IDX_FFC2_USER_DECL,
     
    OADS_IDX_NB,
    ///**************************************************************************///
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// OAD 'Profile' Server environment variable
struct oads_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// On-going operation
    struct ke_msg * operation;
    ///  Services Start Handle
    uint16_t oads_start_hdl; 
    ///  control point char
    uint8_t ffc1_value[OADS_FFC1_DATA_LEN]; 
    /// alarm 1 char
    uint8_t ffc2_value[OADS_FFC2_DATA_LEN];
    /// OADS task state
    ke_state_t state[OADS_IDX_MAX];
    /// Notification configuration of peer devices.
    uint8_t ffc1_ntf_cfg[BLE_CONNECTION_MAX];
    /// Notification configuration of peer devices.
    uint16_t ffc2_ntf_cfg[BLE_CONNECTION_MAX];
    /// Database features
    uint8_t features;

    uint8_t cursor;

};



/**
 ****************************************************************************************
 * @brief Retrieve Bracelet service profile interface
 *
 * @return Brace service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs* oads_prf_itf_get(void);

void oads_exe_operation(void);

//static void oadImgBlockReq(uint16_t connHandle, uint16_t blkNum);

//static void oadImgIdentifyReq(uint16_t connHandle, img_hdr_t *pImgHdr);

uint8_t oadImgIdentifyWrite( uint16_t connHandle, uint16_t length, uint8_t *pValue );

uint8_t oadImgBlockWrite( uint16_t connHandle, uint8_t *pValue );

void select_image_run(void);

uint32_t oad_updating_user_section_begin(uint32_t version, uint32_t total_len);
uint32_t oad_updating_user_section_pro(void);
uint32_t oad_updating_user_section_end(void);
    
#endif /* #if (BLE_OADS_SERVER) */



#endif /*  _OTAS_H_ */





