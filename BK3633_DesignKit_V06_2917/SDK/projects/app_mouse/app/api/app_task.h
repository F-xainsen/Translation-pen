/**
 ****************************************************************************************
 *
 * @file app_task.h
 *
 * @brief Header file - APPTASK.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef APP_TASK_H_
#define APP_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APPTASK Task
 * @ingroup APP
 * @brief Routes ALL messages to/from APP block.
 *
 * The APPTASK is the block responsible for bridging the final application with the
 * RWBLE software host stack. It communicates with the different modules of the BLE host,
 * i.e. @ref SMP, @ref GAP and @ref GATT.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // SW configuration
#include "rwapp_config.h"
#if (BLE_APP_PRESENT)

#include <stdint.h>         // Standard Integer
#include "rwip_task.h"      // Task definitions
#include "ke_task.h"        // Kernel Task

/*
 * DEFINES
 ****************************************************************************************
 */

/// Number of APP Task Instances
#define APP_IDX_MAX                 (1)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
    /// States of the Application HID Module
    enum app_hid_states
    {
        /// Module is disabled (Service not added in DB)
        APP_HID_DISABLED,
        /// Module is idle (Service added but profile not enabled)
        APP_HID_IDLE,
        /// Module is enabled (Device is connected and the profile is enabled)
        APP_HID_ENABLED,
        /// The application can send reports
        APP_HID_READY,
        /// Waiting for a report
        APP_HID_WAIT_REP,

        APP_HID_STATE_MAX,
    };

/// States of APP task
enum appm_state
{
    /// Initialization state
    APPM_INIT,
    /// Database create state
    APPM_CREATE_DB,
    /// Ready State
    APPM_READY,
    /// Connected state
    APPM_CONNECTED,

    /// Number of defined states.
    APPM_STATE_MAX
};


/// APP Task messages
/*@TRACE*/
enum app_msg_id
{
    APPM_DUMMY_MSG = TASK_FIRST_MSG(TASK_ID_APP),

    #if (BLE_APP_HT)
    /// Timer used to refresh the temperature measurement value
    APP_HT_MEAS_INTV_TIMER,
    #endif //(BLE_APP_HT)

    #if (BLE_APP_HID)
    /// Timer used to disconnect the moue if no activity is detecter
    APP_HID_MOUSE_TIMEOUT_TIMER,
    #endif //(BLE_APP_HID)

    APP_PARAM_UPDATE_REQ_IND,

    APP_SEND_SECURITY_REQ,

    APP_ANCS_REQ_IND,

    APP_PERIOD_TIMER,

    APP_GATTC_EXC_MTU_CMD,

    APP_MOUSE_LED_TIMER,
};


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/// @} APPTASK
void app_check_key(void);

#endif //(BLE_APP_PRESENT)

#endif // APP_TASK_H_
