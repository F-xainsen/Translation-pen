/**
 ****************************************************************************************
 *
 * @file app.c
 *
 * @brief Application entry point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "user_config.h"
  
#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)
#include "rwapp_config.h"
#include <string.h>

#include "rwip.h"
#include "app_task.h"                // Application task Definition
#include "app.h"                     // Application Definition
#include "gap.h"                     // GAP Definition
#include "gapm_task.h"               // GAP Manager Task API
#include "gapc_task.h"               // GAP Controller Task API

#include "co_bt.h"                   // Common BT Definition
#include "co_math.h"                 // Common Maths Definition

#if (BLE_APP_SEC)
#include "app_sec.h"                 // Application security Definition
#endif // (BLE_APP_SEC)

#if (BLE_APP_FEE0S)
#include "app_fee0.h"                  // USER define FEE0S Application Definitions
#endif //(BLE_APP_FEE0S)

#if (BLE_APP_FCC0S)
#include "app_fcc0.h"                  // USER define FCC0S Application Definitions
#endif //(BLE_APP_FCC0S)

#if(BLE_APP_ANCS)
#include "app_ancsc.h"               // Application ancs Definition
#endif
#if(BLE_APP_OADS)
#include "app_oads.h"                 // Application oads Definition
#endif

#if (BLE_APP_HT)
#include "app_ht.h"                  // Health Thermometer Application Definitions
#endif //(BLE_APP_HT)

#if (BLE_APP_DIS)
#include "app_dis.h"                 // Device Information Service Application Definitions
#endif //(BLE_APP_DIS)

#if (BLE_APP_BATT)
#include "app_batt.h"                // Battery Application Definitions
#endif //(BLE_APP_DIS)

#if (BLE_APP_HID)
#include "app_hid.h"                 // HID Application Definitions
#endif //(BLE_APP_HID)

#if (DISPLAY_SUPPORT)
#include "app_display.h"             // Application Display Definition
#endif //(DISPLAY_SUPPORT)

#if (BLE_APP_AM0)
#include "app_am0.h"                 // Audio Mode 0 Application
#endif //(BLE_APP_AM0)

#if (NVDS_SUPPORT)
#include "nvds.h"                    // NVDS Definitions
#endif //(NVDS_SUPPORT)
#include "flash.h"
#include "app_hid.h"
#include "app_key.h"
#include "icu.h"
#include "BK3633_RegList.h"
#include "driver_rf.h"
#include "driver_timer.h"
#include "intc.h"
/*
 * DEFINES
 ****************************************************************************************
 */

#if (BLE_APP_HID)
// HID Mouse
#define DEVICE_NAME        "Hid Mouse"
#else
#define DEVICE_NAME        "BK DEVICE"
#endif

#define DEVICE_NAME_SIZE    sizeof(DEVICE_NAME)

/**
 * UUID List part of ADV Data
 * --------------------------------------------------------------------------------------
 * x03 - Length
 * x03 - Complete list of 16-bit UUIDs available
 * x09\x18 - Health Thermometer Service UUID
 *   or
 * x12\x18 - HID Service UUID
 * --------------------------------------------------------------------------------------
 */

#if (BLE_APP_HT)
#define APP_HT_ADV_DATA_UUID        "\x03\x03\x09\x18"
#define APP_HT_ADV_DATA_UUID_LEN    (4)
#endif //(BLE_APP_HT)

#if (BLE_APP_HID)
#define APP_HID_ADV_DATA_UUID       "\x03\x03\x12\x18"
#define APP_HID_ADV_DATA_UUID_LEN   (4)
#endif //(BLE_APP_HID)

/**
 * Appearance part of ADV Data
 * --------------------------------------------------------------------------------------
 * x03 - Length
 * x19 - Appearance
 * x03\x00 - Generic Thermometer
 *   or
 * xC2\x04 - HID Mouse
 * --------------------------------------------------------------------------------------
 */

#if (BLE_APP_HT)
#define APP_HT_ADV_DATA_APPEARANCE    "\x03\x19\x00\x03"
#endif //(BLE_APP_HT)

#if (BLE_APP_HID)
#define APP_HID_ADV_DATA_APPEARANCE   "\x03\x19\xC2\x03"
#define APP_HID_ADV_K_DATA_APPEARANCE   "\x03\x19\xC1\x03"

// //c1 keyboard  c2 mouse
#endif //(BLE_APP_HID)

#define APP_ADV_DATA_APPEARANCE_LEN  (4)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

typedef void (*appm_add_svc_func_t)(void);


/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// List of service to add in the database
enum appm_svc_list
{
    #if (BLE_APP_FEE0S)
    APPM_SVC_FEE0S,
    #endif //(BLE_APP_FEE0S)

    #if (BLE_APP_FCC0S)
    APPM_SVC_FCC0S,
    #endif //(BLE_APP_FCC0S)

    #if (BLE_APP_HT)
    APPM_SVC_HTS,
    #endif //(BLE_APP_HT)
    #if (BLE_APP_DIS)
    APPM_SVC_DIS,
    #endif //(BLE_APP_DIS)
    #if (BLE_APP_BATT)
    APPM_SVC_BATT,
    #endif //(BLE_APP_BATT)
    #if (BLE_APP_HID)
    APPM_SVC_HIDS,
    #endif //(BLE_APP_HID)
    #if (BLE_APP_AM0)
    APPM_SVC_AM0_HAS,
    #endif //(BLE_APP_AM0)

    #if(BLE_APP_ANCS)
    APPM_SVC_ANCSC,
    #endif
    #if(BLE_APP_OADS)
    APPM_SVC_OADS,
    #endif

    APPM_SVC_LIST_STOP,
};

/*
 * LOCAL VARIABLES DEFINITIONS
 ****************************************************************************************
 */

/// Application Task Descriptor
extern const struct ke_task_desc TASK_DESC_APP;
extern volatile uint32_t sys_flag;
extern void ble_addr_add1(void);
extern void ke_timer_set(ke_msg_id_t const timer_id, ke_task_id_t const task_id, uint32_t delay);
uint8_t rf_mode=0;// 0:2.4,1:BLE

/// List of functions used to create the database
static const appm_add_svc_func_t appm_add_svc_func_list[APPM_SVC_LIST_STOP] =
{
    #if (BLE_APP_FEE0S)
    (appm_add_svc_func_t)app_fee0_add_fee0s,
    #endif //(BLE_APP_FEE0S)

    #if (BLE_APP_FCC0S)
    (appm_add_svc_func_t)app_fcc0_add_fcc0s,
    #endif //(BLE_APP_FCC0S)

    #if (BLE_APP_HT)
    (appm_add_svc_func_t)app_ht_add_hts,
    #endif //(BLE_APP_HT)
    #if (BLE_APP_DIS)
    (appm_add_svc_func_t)app_dis_add_dis,
    #endif //(BLE_APP_DIS)
    #if (BLE_APP_BATT)
    (appm_add_svc_func_t)app_batt_add_bas,
    #endif //(BLE_APP_BATT)
    #if (BLE_APP_HID)
    (appm_add_svc_func_t)app_hid_add_hids,
    #endif //(BLE_APP_HID)
    #if (BLE_APP_AM0)
    (appm_add_svc_func_t)app_am0_add_has,
    #endif //(BLE_APP_AM0)
    #if (BLE_APP_ANCS)
    (appm_add_svc_func_t)app_ancs_add_ancsc,
    #endif
    #if (BLE_APP_OADS)
    (appm_add_svc_func_t)app_oad_add_oads,
    #endif
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Application Environment Structure
struct app_env_tag app_env;

uint8_t  remote_pair_adv_data[] =
{
    0x07, 0x09, 'K', 'T', 'P', 'A', 'I', 'R',
    //0x09, 0xff, 0x00, 0x03, 0x01, 0x40, 0x03, 0x10, 0x06, 0x01
    //0x09, 0xff, 0x00, 0x03, 0x01, 0x40, 0x03, 0x10, 0x06, 0x01
    0x09, 0xFF, 0x00, 0x03, 0x96, 0x00, 0x03, 0x95, 0x10, 0x00
};

uint8_t  remote_scan_rsp_data[] =
{
    0x07, 0x09, 't', 'S', 'e', 'l', 'f', 'i',
    //0x09, 0xff, 0x00, 0x03, 0x01, 0x40, 0x03, 0x10, 0x06, 0x01
    //0x09, 0xff, 0x00, 0x03, 0x01, 0x40, 0x03, 0x10, 0x06, 0x01
    0x09, 0xFF, 0x00, 0x03, 0x96, 0x00, 0x03, 0x95, 0x10, 0x00
};


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void appm_add_dev_to_wlist(struct gap_bdaddr bdaddr)
{
     /*  rsp:
      * GAPM_CMP_EVT: When operation completed.
      */
    
     uart_printf("%s\r\n",__func__);
     struct gapm_list_set_wl_cmd* cmd = KE_MSG_ALLOC_DYN(GAPM_WHITE_LIST_MGT_CMD,
                    TASK_GAPM, TASK_APP, gapm_list_set_wl_cmd,sizeof(struct gap_bdaddr));
     
     
      cmd->operation = GAPM_ADD_DEV_IN_WLIST;
      cmd->size= 1;
      cmd->wl_info[0] = bdaddr;
     
      uart_printf("addr_type = %x\r\n", cmd->wl_info[0].addr_type);
      uart_printf("addr = ");
      for(int i = 0;i < sizeof(bd_addr_t);i++)
      {
        uart_printf("%x ",cmd->wl_info[0].addr.addr[i]);
      }
      uart_printf("\r\n");
        
      ke_msg_send(cmd);
}


void appm_rmv_dev_from_wlist(struct gap_bdaddr bdaddr)
{
     /*  rsp:
      *  GAPM_CMP_EVT: When operation completed.
      */
    
     uart_printf("%s\r\n",__func__);
     struct gapm_list_set_wl_cmd* cmd = KE_MSG_ALLOC_DYN(GAPM_WHITE_LIST_MGT_CMD,
                    TASK_GAPM, TASK_APP, gapm_list_set_wl_cmd,sizeof(struct gap_bdaddr));
     
     
      cmd->operation = GAPM_RMV_DEV_FRM_WLIST;
      cmd->size= 1;
      cmd->wl_info[0] = bdaddr;
     
      uart_printf("addr_type = %x\r\n", cmd->wl_info[0].addr_type);
      uart_printf("addr = ");
      for(int i = 0;i < sizeof(bd_addr_t);i++)
      {
        uart_printf("%x ",cmd->wl_info[0].addr.addr[i]);
      }
      uart_printf("\r\n");
        
      ke_msg_send(cmd);
     

}




void appm_clear_wlist(void)
{
     /*  rsp:
      * GAPM_CMP_EVT: When operation completed.
      * GAPM_WHITE_LIST_SIZE_IND: If white list size is requested
      */
    
     uart_printf("%s\r\n",__func__);
     struct gapm_list_set_wl_cmd* cmd = KE_MSG_ALLOC(GAPM_WHITE_LIST_MGT_CMD,
                    TASK_GAPM, TASK_APP, gapm_list_set_wl_cmd);
     
     
    cmd->operation = GAPM_CLEAR_WLIST;
     
    ke_msg_send(cmd);
}


#if (!BLE_APP_AM0)
static void appm_build_adv_data(uint16_t max_length, uint16_t *p_length, uint8_t *p_buf)
{
    // Remaining Length
    uint8_t rem_len = max_length;

    #if 0
    *p_length = sizeof( remote_pair_adv_data );
    memcpy( p_buf, remote_pair_adv_data, *p_length);
    #else
    //adv type flag
    p_buf[0]=0x02;
    p_buf[1]=0x01;
    //p_buf[2]=0x06;//06:gen mode 05:limit 04 non
    if(sys_flag & FLAG_KEY_PAIRED)
    {
        p_buf[2]=0x05;
    }
    else
    {
        p_buf[2]=0x05;
    }
    p_buf += 3;
    *p_length += 3;
    uart_printf("adv flag\r\n",p_buf[2]);
    #if (BLE_APP_HT)
    // Set list of UUIDs
    memcpy(p_buf, APP_HT_ADV_DATA_UUID, APP_HT_ADV_DATA_UUID_LEN);
    *p_length += APP_HT_ADV_DATA_UUID_LEN;
    p_buf += APP_HT_ADV_DATA_UUID_LEN;

    // Set appearance
    memcpy(p_buf, APP_HT_ADV_DATA_APPEARANCE, APP_ADV_DATA_APPEARANCE_LEN);
    *p_length += APP_ADV_DATA_APPEARANCE_LEN;
    p_buf += APP_ADV_DATA_APPEARANCE_LEN;
    #endif //(BLE_APP_HT)

    #if (BLE_APP_HID)
    memcpy(p_buf, APP_HID_ADV_DATA_APPEARANCE, APP_ADV_DATA_APPEARANCE_LEN);
    *p_length += APP_ADV_DATA_APPEARANCE_LEN;
    p_buf += APP_ADV_DATA_APPEARANCE_LEN;
    // Set list of UUIDs
    memcpy(p_buf, APP_HID_ADV_DATA_UUID, APP_HID_ADV_DATA_UUID_LEN);
    *p_length += APP_HID_ADV_DATA_UUID_LEN;
    p_buf += APP_HID_ADV_DATA_UUID_LEN;

    // Set appearance

    #endif //(BLE_APP_HID)

    // Sanity check
    ASSERT_ERR(rem_len >= max_length);

    // Get remaining space in the Advertising Data - 2 bytes are used for name length/flag
    rem_len -= *p_length;

    // Check if additional data can be added to the Advertising data - 2 bytes needed for type and length
    if (rem_len > 2)
    {
        uint8_t dev_name_length = co_min(app_env.dev_name_len, (rem_len - 2));

        // Device name length
        *p_buf = dev_name_length + 1;
        // Device name flag (check if device name is complete or not)
        *(p_buf + 1) = (dev_name_length == app_env.dev_name_len) ? '\x09' : '\x08';
        // Copy device name
        memcpy(p_buf + 2, app_env.dev_name, dev_name_length);

        // Update advertising data length
        *p_length += (dev_name_length + 2);
    }
#endif
}
#endif //(!BLE_APP_AM0)

void appm_start_advertising(void)
{
    // Prepare the GAPM_ACTIVITY_START_CMD message
    struct gapm_activity_start_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_START_CMD,
                                                         TASK_GAPM, TASK_APP,
                                                         gapm_activity_start_cmd);

    p_cmd->operation = GAPM_START_ACTIVITY;
    p_cmd->actv_idx = app_env.adv_actv_idx;
    #if (BLE_APP_HID)
    p_cmd->u_param.adv_add_param.duration = APP_DFLT_ADV_DURATION;
    #else //(BLE_APP_HID)
    p_cmd->u_param.adv_add_param.duration = 0;
    #endif //(BLE_APP_HID)
    p_cmd->u_param.adv_add_param.max_adv_evt = 0;

    // Send the message
    ke_msg_send(p_cmd);

    // Keep the current operation
    app_env.adv_state = APP_ADV_STATE_STARTING;
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_START_ACTIVITY;

    
}


void appm_stop_advertising(void)
{
    // Prepare the GAPM_ACTIVITY_STOP_CMD message
    struct gapm_activity_stop_cmd *cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_STOP_CMD,
                                                      TASK_GAPM, TASK_APP,
                                                      gapm_activity_stop_cmd);

    // Fill the allocated kernel message
    cmd->operation = GAPM_STOP_ACTIVITY;
    cmd->actv_idx = app_env.adv_actv_idx;

    // Send the message
    ke_msg_send(cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_STOPPING;
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_STOP_ACTIVITY;
}


void appm_set_adv_data(void)
{
    // Prepare the GAPM_SET_ADV_DATA_CMD message
    struct gapm_set_adv_data_cmd *p_cmd = KE_MSG_ALLOC_DYN(GAPM_SET_ADV_DATA_CMD,
                                                           TASK_GAPM, TASK_APP,
                                                           gapm_set_adv_data_cmd,
                                                           ADV_DATA_LEN);

    // Fill the allocated kernel message
    p_cmd->operation = GAPM_SET_ADV_DATA;
    p_cmd->actv_idx = app_env.adv_actv_idx;

    p_cmd->length = 0;
    // GAP will use 3 bytes for the AD Type
    appm_build_adv_data(ADV_DATA_LEN, &p_cmd->length, &p_cmd->data[0]);

    // Send the message
    ke_msg_send(p_cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_SETTING_ADV_DATA;
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_SET_ADV_DATA;
}

void appm_set_scan_rsp_data(void)
{
    // Prepare the GAPM_SET_ADV_DATA_CMD message
    struct gapm_set_adv_data_cmd *p_cmd = KE_MSG_ALLOC_DYN(GAPM_SET_ADV_DATA_CMD,
                                                           TASK_GAPM, TASK_APP,
                                                           gapm_set_adv_data_cmd,
                                                           ADV_DATA_LEN);

    // Fill the allocated kernel message
    p_cmd->operation = GAPM_SET_SCAN_RSP_DATA;
    p_cmd->actv_idx = app_env.adv_actv_idx;
    #if 0
    p_cmd->length = sizeof( remote_scan_rsp_data );
    memcpy( &p_cmd->data[0],remote_scan_rsp_data, p_cmd->length );
    #else

        #if (BLE_APP_AM0)
        app_am0_fill_scan_rsp_data(p_cmd);
        #else //(BLE_APP_AM0)
        p_cmd->length = APP_SCNRSP_DATA_LEN;
        memcpy(&p_cmd->data[0], APP_SCNRSP_DATA, APP_SCNRSP_DATA_LEN);
        #endif //(BLE_APP_AM0)
    #endif
    // Send the message
    ke_msg_send(p_cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_SETTING_SCAN_RSP_DATA;
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_SET_SCAN_RSP_DATA;
}


static void appm_send_gapm_reset_cmd(void)
{
    // Reset the stack
    struct gapm_reset_cmd *p_cmd = KE_MSG_ALLOC(GAPM_RESET_CMD,
                                                TASK_GAPM, TASK_APP,
                                                gapm_reset_cmd);
    p_cmd->operation = GAPM_RESET;

    ke_msg_send(p_cmd);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void appm_init()
{
    // Reset the application manager environment
    memset(&app_env, 0, sizeof(app_env));

    // Create APP task
    ke_task_create(TASK_APP, &TASK_DESC_APP);

    // Initialize Task state
    ke_state_set(TASK_APP, APPM_INIT);

    flash_read_data(&app_env.dev_name[0],(flash_env.bdaddr_def_addr_abs+8),APP_DEVICE_NAME_MAX_LEN);

    for(uint8_t i=0;i<APP_DEVICE_NAME_MAX_LEN;i++)
    {
        if( (app_env.dev_name[i]!=0xff) && (app_env.dev_name[i]!=0x00) )
        {
            app_env.dev_name_len++;
        }
        else
        {
            break;
        }

    }
    if( (app_env.dev_name[0]==0xff) && (app_env.dev_name[1]==0xff) )
    {
        // Get default Device Name (No name if not enough space)
        memcpy(app_env.dev_name, APP_DFLT_DEVICE_NAME, APP_DFLT_DEVICE_NAME_LEN);
        app_env.dev_name_len = APP_DFLT_DEVICE_NAME_LEN;

        // TODO update this value per profiles
    }


    /*------------------------------------------------------
     * INITIALIZE ALL MODULES
     *------------------------------------------------------*/

    // load device information:

    #if (DISPLAY_SUPPORT)
    app_display_init();
    #endif //(DISPLAY_SUPPORT)

    #if (BLE_APP_SEC)
    // Security Module
    app_sec_init();
    #endif // (BLE_APP_SEC)

    #if (BLE_APP_HT)
    // Health Thermometer Module
    app_ht_init();
    #endif //(BLE_APP_HT)

    #if (BLE_APP_DIS)
    // Device Information Module
    app_dis_init();
    #endif //(BLE_APP_DIS)

    #if (BLE_APP_HID)
    // HID Module
    app_hid_init();
    #endif //(BLE_APP_HID)

    #if (BLE_APP_BATT)
    // Battery Module
    app_batt_init();
    #endif //(BLE_APP_BATT)

    #if (BLE_APP_AM0)
    // Audio Mode 0 Module
    app_am0_init();
    #endif //(BLE_APP_AM0)

    #if (BLE_APP_ANCS)
    app_ancsc_init();
    #endif
    #if (BLE_APP_OADS)
    app_oads_init();
    #endif

    // Reset the stack
    appm_send_gapm_reset_cmd();
}

bool appm_add_svc(void)
{
    // Indicate if more services need to be added in the database
    bool more_svc = false;

    // Check if another should be added in the database
    if (app_env.next_svc != APPM_SVC_LIST_STOP)
    {
        ASSERT_INFO(appm_add_svc_func_list[app_env.next_svc] != NULL, app_env.next_svc, 1);

        // Call the function used to add the required service
        appm_add_svc_func_list[app_env.next_svc]();

        // Select following service to add
        app_env.next_svc++;
        more_svc = true;
    }
    uart_printf("%s more_svc:%d\r\n",__func__,more_svc);
    return more_svc;
}

void appm_disconnect(void)
{
    struct gapc_disconnect_cmd *cmd = KE_MSG_ALLOC(GAPC_DISCONNECT_CMD,
                                                   KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                                   gapc_disconnect_cmd);

    cmd->operation = GAPC_DISCONNECT;
    cmd->reason    = CO_ERROR_REMOTE_USER_TERM_CON;

    // Send the message
    ke_msg_send(cmd);
}
void rw_ip_sleep_test(uint32_t time);
void rw_ip_deep_sleep_test(void);
void appm_create_advertising(void)
{
    if (app_env.adv_state == APP_ADV_STATE_IDLE)
    {
        // Prepare the GAPM_ACTIVITY_CREATE_CMD message
        struct gapm_activity_create_adv_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_CREATE_CMD,
                                                                  TASK_GAPM, TASK_APP,
                                                                  gapm_activity_create_adv_cmd);

        // Set operation code
        p_cmd->operation = GAPM_CREATE_ADV_ACTIVITY;

        // Fill the allocated kernel message
        p_cmd->own_addr_type = GAPM_STATIC_ADDR;
        p_cmd->adv_param.type = GAPM_ADV_TYPE_LEGACY;
        p_cmd->adv_param.prop = GAPM_ADV_PROP_UNDIR_CONN_MASK;
        p_cmd->adv_param.prim_cfg.chnl_map = APP_ADV_CHMAP;
        p_cmd->adv_param.prim_cfg.phy = GAP_PHY_LE_1MBPS;

        #if (APP_WHITE_LIST_ENABLE)
        if(app_sec_get_bond_status())
        {
            struct gap_bdaddr whitelist_bdaddr;
            uint8_t peer_address_len = NVDS_LEN_PEER_BD_ADDRESS;

            uart_printf("appm_create_white_list_dvertising\r\n");

            if(nvds_get(NVDS_TAG_PEER_BD_ADDRESS, &peer_address_len, (uint8_t *)&whitelist_bdaddr) != NVDS_OK)
            {
                // An error has occurred during access to the NVDS
                ASSERT_INFO(0,NVDS_TAG_PEER_BD_ADDRESS,peer_address_len);
            }
            appm_add_dev_to_wlist(whitelist_bdaddr);
            p_cmd->adv_param.filter_pol = ADV_ALLOW_SCAN_WLST_CON_WLST;
            p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_NON_DISC;
        }
        else
        {
            p_cmd->adv_param.filter_pol = ADV_ALLOW_SCAN_ANY_CON_ANY;            
        }
        #else
        p_cmd->adv_param.filter_pol = ADV_ALLOW_SCAN_ANY_CON_ANY;
        #endif

        if(sys_flag & FLAG_KEY_PAIRED)
        {
            p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_GEN_DISC;//GAPM_ADV_MODE_LIM_DISC;
            p_cmd->adv_param.prim_cfg.adv_intv_min = APP_ADV_INT_MIN;
            p_cmd->adv_param.prim_cfg.adv_intv_max = APP_ADV_INT_MAX;
        }
        else if(app_sec_get_bond_status())
        {
            p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_NON_DISC;
            /*
             * If the peripheral is already bonded with a central device, use the direct advertising
             * procedure (BD Address of the peer device is stored in NVDS.
             */
            if (sys_flag & FLAG_PEER_PUBLIC_ADDR)
            {
                // BD Address of the peer device
                struct gap_bdaddr peer_bd_addr;

                uint8_t length = NVDS_LEN_PEER_BD_ADDRESS;
                uart_printf("appm_create_advertising derect\r\n");
                //Get the BD Address of the peer device in NVDS
                if (nvds_get(NVDS_TAG_PEER_BD_ADDRESS, &length, (uint8_t *)&peer_bd_addr) != NVDS_OK)
                {
                    // The address of the bonded peer device should be present in the database
                    ASSERT_ERR(0);
                }

                //p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_NON_DISC;
                //Set the DIRECT ADVERTISING mode
                //p_cmd->adv_param.prop = GAPM_ADV_PROP_DIR_CONN_MASK;
                //Copy the BD address of the peer device and the type of address
                memcpy(&p_cmd->adv_param.peer_addr, &peer_bd_addr, NVDS_LEN_PEER_BD_ADDRESS);

                p_cmd->adv_param.prim_cfg.adv_intv_min = APP_ADV_FAST_INT;
                p_cmd->adv_param.prim_cfg.adv_intv_max = APP_ADV_FAST_INT;
                uart_printf("peeraddr1=%x,%x,%x,%x,%x,%x\r\n",p_cmd->adv_param.peer_addr.addr.addr[0],p_cmd->adv_param.peer_addr.addr.addr[1],p_cmd->adv_param.peer_addr.addr.addr[2],
                            p_cmd->adv_param.peer_addr.addr.addr[3],p_cmd->adv_param.peer_addr.addr.addr[4],p_cmd->adv_param.peer_addr.addr.addr[5]);
                uart_printf("peeraddr2=%x,%x,%x,%x,%x,%x\r\n",peer_bd_addr.addr.addr[0],peer_bd_addr.addr.addr[1],peer_bd_addr.addr.addr[2],
                            peer_bd_addr.addr.addr[3],peer_bd_addr.addr.addr[4],peer_bd_addr.addr.addr[5]);
                app_env.adv_prop = APP_ADV_PROP_DERECT;

                p_cmd->adv_param.prop = GAPM_ADV_PROP_DIR_CONN_MASK;
            }
            else
            {
                uart_printf("appm_create_advertising nonderect \r\n");
                //p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_LIM_DISC;
                p_cmd->adv_param.prim_cfg.adv_intv_min = APP_ADV_INT_MIN;
                p_cmd->adv_param.prim_cfg.adv_intv_max = APP_ADV_INT_MAX;
                app_env.adv_prop = APP_ADV_PROP_NONDERECT;
            }
        }
        else
        {
            uart_printf("no create_advertising\r\n");
            return;
        }
        // Send the message
        ke_msg_send(p_cmd);

        // Keep the current operation
        app_env.adv_state = APP_ADV_STATE_CREATING;
        // And the next expected operation code for the command completed event
        app_env.adv_op = GAPM_CREATE_ADV_ACTIVITY;
    }
}


void appm_delete_advertising(void)
{
    // Prepare the GAPM_ACTIVITY_CREATE_CMD message
    struct gapm_activity_delete_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_DELETE_CMD,
                                                              TASK_GAPM, TASK_APP,
                                                              gapm_activity_delete_cmd);

    // Set operation code
    p_cmd->operation = GAPM_DELETE_ALL_ACTIVITIES;

    // Send the message
    ke_msg_send(p_cmd);
    uart_printf("appm_delete_advertising \r\n");
    // Keep the current operation
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_DELETE_ALL_ACTIVITIES;
}
#if 0
void appm_adv_fsm_next(void)
{
    uart_printf("adv_state:%x\r\n",app_env.adv_state);

    switch (app_env.adv_state)
    {
        case (APP_ADV_STATE_IDLE):
        {
            // Create advertising
            appm_create_advertising();
        } break;

        case (APP_ADV_STATE_CREATING):
        {
            // Set advertising data
            appm_set_adv_data();
        } break;

        case (APP_ADV_STATE_SETTING_ADV_DATA):
        {
            // Set scan response data
            appm_set_scan_rsp_data();
        } break;

        case (APP_ADV_STATE_CREATED):
        case (APP_ADV_STATE_SETTING_SCAN_RSP_DATA):
        {
            // Start advertising activity
            appm_start_advertising();
        } break;

        case (APP_ADV_STATE_STARTING):
        {
            #if (DISPLAY_SUPPORT)
            // Update advertising state screen
            app_display_set_adv(true);
            #endif //(DISPLAY_SUPPORT)

            // Go to started state
            app_env.adv_state = APP_ADV_STATE_STARTED;
        } break;

        case (APP_ADV_STATE_STARTED):
        {
            // Stop advertising activity
            appm_stop_advertising();
        } break;

        case (APP_ADV_STATE_STOPPING):
        {
            #if (DISPLAY_SUPPORT)
            // Update advertising state screen
            app_display_set_adv(false);
            #endif //(DISPLAY_SUPPORT)

            // Go created state
            app_env.adv_state = APP_ADV_STATE_CREATED;
        } break;



        default:
        {
            ASSERT_ERR(0);
        } break;
    }
}
#endif
void appm_update_param(struct gapc_conn_param *conn_param)
{
    // Prepare the GAPC_PARAM_UPDATE_CMD message
    struct gapc_param_update_cmd *cmd = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD,
                                                     KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                                     gapc_param_update_cmd);

    cmd->operation  = GAPC_UPDATE_PARAMS;
    cmd->intv_min   = conn_param->intv_min;
    cmd->intv_max   = conn_param->intv_max;
    cmd->latency    = conn_param->latency;
    cmd->time_out   = conn_param->time_out;

    // not used by a slave device
    cmd->ce_len_min = 0xFFFF;
    cmd->ce_len_max = 0xFFFF;

    // Send the message
    ke_msg_send(cmd);
}

uint8_t appm_get_dev_name(uint8_t* name)
{
    // copy name to provided pointer
    memcpy(name, app_env.dev_name, app_env.dev_name_len);
    // return name length
    return app_env.dev_name_len;
}
void appm_change_device_mode(void)
{
    rf_mode ^= 1;
    rf_mode &=0x01;
    if (nvds_put(NVDS_TAG_RF_MODE, NVDS_LEN_RF_MODE, (uint8_t *)&rf_mode) != NVDS_OK)
    {
        ASSERT_ERR(0);
    }

    if(rf_mode == RF_BLE_MODE)
    {
        uart_printf("chg to BLE\r\n");
        #if 0
        cpu_reset();
        #else
        GLOBAL_INT_DISABLE();
        Timer0_Stop(1);
       // Rf_Init();
        RF_CHIP_DISABLE;
        PowerDown_Rf();
        setf_SYS_Reg0x3_bk24_pwd;
        intc_init();
        rwip_init(0);
        GLOBAL_INT_RESTORE();
        
        #endif
    }
    else
    {
        uart_printf("chg to 24\r\n");


    }
}

#endif //(BLE_APP_PRESENT)

/// @} APP
