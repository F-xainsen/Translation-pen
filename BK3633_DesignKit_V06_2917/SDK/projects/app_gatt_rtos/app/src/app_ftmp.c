/**
 ****************************************************************************************
 *
 * @file app_ftmp.c
 *
 * @brief ftmp Application Module entry point
 *
 * @auth  gang.cheng
 *
 * @date  2016.05.31
 *
 * Copyright (C) Beken 2009-2016
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

#include "rwip_config.h"     // SW configuration
#if BLE_APP_FTMPS
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>
#include "app_ftmp.h"              // Battery Application Module Definitions
#include "app.h"                    // Application Definitions
#include "app_task.h"             // application task definitions
#include "ftmps_task.h"           // health thermometer functions
#include "co_bt.h"
#include "prf_types.h"             // Profile common types definition
#include "arch.h"                    // Platform Definitions
#include "prf.h"
#include "ftmps.h"
#include "ke_timer.h"
#include "uart.h"
#include "gpio.h"
#include "co_utils.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// ftmp Application Module Environment Structure
struct app_ftmp_env_tag app_ftmp_env;

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


void app_ftmp_init(void)
{
    // Reset the environment
    memset(&app_ftmp_env, 0, sizeof(struct app_ftmp_env_tag));


    // 以下部分是测试代码，做实际产品时需要用功能代替
    //
    //uint32_t featureField = treadmill_total_distance;
    //uint32_t targetSettingFeatures = 0;
    
    FitnessMachineFeatureRead(0x1204, 0x01);
    SpeedRangeRead(100,5000,100);
    //HeartRateRangeRead(0x40,0x80,0x01);
    TrainingStatusRead(0x01);

    // test
    //my_user_init();
}

void app_ftmp_add_ftmps(void)
{

    struct ftmps_db_cfg *db_cfg;

    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                            TASK_GAPM, TASK_APP,
                                            gapm_profile_task_add_cmd, sizeof(struct ftmps_db_cfg));
    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl =   0;
    req->prf_task_id = TASK_ID_FTMPS;
    req->app_task = TASK_APP;
    req->start_hdl = 0; //req->start_hdl = 0; dynamically allocated

uart_printf("%s %d\r\n",__func__,__LINE__);
    // Set parameters
    db_cfg = (struct ftmps_db_cfg* ) req->param;

    // Sending of notifications is supported
    db_cfg->features = FTMP_NTF_SUP;
    // Send the message
    ke_msg_send(req);
}



void app_send_lvl(uint8_t* buf, uint8_t len,uint16_t uuidindx)
{
     //Allocate the message 
    struct ftmps_level_upd_req * req = KE_MSG_ALLOC(FTMPS_LEVEL_UPD_REQ,
                                            prf_get_task_from_id(TASK_ID_FTMPS),
                                            TASK_APP,
                                            ftmps_level_upd_req);
    //Fill in the parameter structure
    req->length = len;
    memcpy(req->level, buf, len);
    req->uuid_indx = uuidindx;
    // Send the message
    ke_msg_send(req);
}


static int ftmps_level_ntf_cfg_ind_handler(ke_msg_id_t const msgid,
        struct ftmps_level_ntf_cfg_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    uart_printf("param->ntf_cfg = %x\r\n",param->ntf_cfg);
    if(param->ntf_cfg == PRF_CLI_STOP_NTFIND)
    {
        
    }
    else
    {
        
    }

    return (KE_MSG_CONSUMED);
}

static int ftmp_level_upd_handler(ke_msg_id_t const msgid,
                                  struct ftmps_level_upd_rsp const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    if(param->status == GAP_ERR_NO_ERROR)
    {
        uint8_t buf[128];
        memset(buf, 0xcc, 128);
        //app_fff1_send_lvl(buf, 128);
    }

    return (KE_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int app_ftmp_msg_dflt_handler(ke_msg_id_t const msgid,
                                     void const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    uart_printf("%s\r\n", __func__);

    // Drop the message
    return (KE_MSG_CONSUMED);
}


static int ftmp_writer_req_handler(ke_msg_id_t const msgid,
                                   struct ftmps_writer_ind *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
#if 0
    // Drop the message
    uart_printf("ftmp_writer_req_handler: %d\r\n", param->flag);

    for(uint8_t i = 0; i < param->length; i++)
    {
        uart_printf("%02x ",param->value[i]);
    }
    uart_printf("\r\n");
#endif
    if(param->flag == FTMP_IDX_FITNESS_MACHINE_CONTROL_POINTE_VAL)
    {
        FitnessMachineControlPointWrite(param->value, param->length);
    }
    else if(param->flag == FTMP_IDX_FITNESS_EXTENSION_DATA_VAL)
    {
        fitnessExtensionDataWrite(param->value, param->length);
    }
    
    return (KE_MSG_CONSUMED);
}

void SendUartDataToApp(uint8_t* data,uint8_t length, uint16_t uuidIndex)// 发送到APP的函数
{
    app_send_lvl(data,length,uuidIndex);
}


void SendAppMachineControlPointToUart(uint8_t opCode,control_point_target_union *param) // 发送控制命令到串口的函数
{
    uint8_t status;
    
    uart_printf("MachineControlPoint: 0x%x\r\n", opCode);
    if(opCode == stop)
    {
        uart_printf("app stop\r\n");
        status = 0;
        apptc_cmd_encode(0xA3, &status, 1);

        TrainingStatusRead(post_workout);
        FitnessMachineStatusnotify(stop_by_user, NULL, 0);
    }
    else if(opCode == pause)
    {
        uart_printf("app pause\r\n");
        status = 3;
        apptc_cmd_encode(0xA3, &status, 1);
    }
    else if((opCode == start) || (opCode == resume))
    {
        uart_printf("app start\r\n");
        status = 1;
        apptc_cmd_encode(0xA3, &status, 1);

        TrainingStatusRead(pre_workout);    //手机主动开启跑步机，需要设置app状态

    }
    else if(opCode == set_target_speed)
    {
        //60 = 6km/h
        status = param->targetSpeed/10;
        uart_printf("set_target_speed: %d\r\n", status);
        apptc_cmd_encode(0x01, &status, 1);
    }
      return ;
}

void SendAppExtensionDataToUart(huawei_fitness_extension_data_t *extensionData) // 发送华为扩展字段到串口的函数
{
    uint8_t send[8] = {0};
    
    //uart_printf("DataToUart: 0x%x\r\n", extensionData->flags);

    if(extensionData->flags & 0x02)    //心率
    {
        send[0] |= 0x01;
        send[1] = extensionData->heartRate;
    }
    else if(extensionData->flags & 0x04)    //总卡路里，热量
    {
        send[0] |= 0x04;
        send[4] = (extensionData->totalEnergy >> 8) & 0xff;
        send[5] = extensionData->totalEnergy & 0xff;
    }
    else if(extensionData->flags & 0x08)    //动态卡路里
    {
        send[0] |= 0x08;
        send[6] = (extensionData->dynamicEnerty >> 8) & 0xff;
        send[7] = extensionData->dynamicEnerty & 0xff;
    }
    else if(extensionData->flags & 0x10)    //步数
    {
        send[0] |= 0x02;
        send[2] = (extensionData->steps >> 8) & 0xff;
        send[3] = extensionData->steps & 0xff;
    }
    
    apptc_cmd_encode(0xA1, send, 8);
}

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
const struct ke_msg_handler app_ftmp_msg_handler_list[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,        (ke_msg_func_t)app_ftmp_msg_dflt_handler},
    {FTMPS_LEVEL_NTF_CFG_IND,  (ke_msg_func_t)ftmps_level_ntf_cfg_ind_handler},
    {FTMPS_LEVEL_UPD_RSP,      (ke_msg_func_t)ftmp_level_upd_handler},
    {FTMPS_WRITER_REQ_IND,        (ke_msg_func_t)ftmp_writer_req_handler},
    //{FTMPS_LEVEL_PERIOD_NTF,    (ke_msg_func_t)fff1_period_ntf_handler},
};

const struct app_subtask_handlers app_ftmp_table_handler = APP_HANDLERS(app_ftmp);


#if 0

#define POWER_ON_OFF     0x31    //开机按键

uint8_t g_status = 0;
unsigned short g_totalDistance = 0;
unsigned short g_totalTime = 0;
#endif

void my_user_init(void)
{
#if 0
    gpio_config(POWER_ON_OFF, INPUT, PULL_HIGH);
    REG_APB5_GPIO_WUATOD_TYPE |= 0x02000000;    //下降沿
    REG_APB5_GPIO_WUATOD_ENABLE |= 0x02000000;    //使能P33中断
    REG_AHB0_ICU_INT_ENABLE |=  (0x01 << 9);    //GPIO总中断
    gpio_cb_register(user_gpio_int_cb);
#endif
}

void user_gpio_int_cb(void)
{
#if 0
    if(!(REG_APB5_GPIOD_DATA & 0x200))
    {
        if(g_status == 0)
        {
            ke_timer_set(APP_KEY_TIMER, TASK_APP, 10);
        }
        else if(g_status == 2)
        {
            g_status = 0;
            ke_timer_clear(APP_KEY_TIMER, TASK_APP);
        }
    }
#endif
}

int app_key_timer_handler(ke_msg_id_t const msgid,
                                          void *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
#if 0
       UART_PRINTF("key short: %d\r\n", g_status);
    if(g_status == 0)
    {
        g_status = 1;
        TrainingStatusRead(pre_workout);    //pre_workout        开始状态
        ke_timer_set(APP_KEY_TIMER, TASK_APP, 200);
    }
    else if(g_status == 1)
    {
        g_status = 2;
        TrainingStatusRead(quick_start);    //quick_start        开始状态
        ke_timer_set(APP_KEY_TIMER, TASK_APP, 10);
    }
    else if(g_status == 2)
    {
        g_status = 3;
        FitnessMachineStatusnotify(start_by_user, NULL, 0);    //开始
        ke_timer_set(APP_KEY_TIMER, TASK_APP, 10);
    }
    else if(g_status == 3)
    {
        g_status = 3;

        treadmill_Data_t data;
        data.flags = instantaneous_speed | treadmill_total_distance 
            | treadmill_expended_energy | treadmill_elapsed_time
            |treadmill_elapsed_time;
        data.instantaneousSpeed = 150;    //瞬时速度
        data.totalDistance = ++g_totalDistance;    //总距离每秒加一
        data.totalEnergy = 30;
        data.energyPerHour = 50;
        data.elapsedTime = ++g_totalTime;

        UART_PRINTF("totalDistance: %dm, %ds\r\n", g_totalDistance, g_totalTime);
        
        TreadmillDataNotify(data);    //每秒发送一次跑步数据
        ke_timer_set(APP_KEY_TIMER, TASK_APP, 100);
    }
#endif
    return KE_MSG_CONSUMED;
}
#endif

