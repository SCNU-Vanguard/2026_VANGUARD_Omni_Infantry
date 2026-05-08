#include "super_cap.h"
#include "bsp_dwt.h"
#include "referee_task.h"



static uint8_t idx;
static Super_Capacitor_t *Super_Capacitor_instances[SUPER_CAP_CNT];

static void Decode_Super_Capacitor(CAN_instance_t *super_cap)
{
    uint8_t *rxbuff = super_cap->rx_buff;
    Super_Capacitor_t *super_capacitor = (Super_Capacitor_t *)super_cap->id;
    Super_Capacitor_callback_t *receive_data = &(super_capacitor->receive_data);

    Supervisor_Reload(super_capacitor->supervisor);
    super_capacitor->dt = DWT_GetDeltaT(&super_capacitor->feed_cnt);

    if(super_capacitor->LostFlag==true)
    {
        super_capacitor->LostFlag = false;
    }

    memcpy(receive_data, rxbuff, 8);

}

static void Super_Capacitor_Lost_Callback(void *super_cap_ptr)
{
    Super_Capacitor_t *super_capacitor = (Super_Capacitor_t *)super_cap_ptr;
    super_capacitor->LostFlag = true;
}

Super_Capacitor_t *Super_Capacitor_Init(Super_Capacitor_t *config)
{
    Super_Capacitor_t *instance = (Super_Capacitor_t *)malloc(sizeof(Super_Capacitor_t));

    if (instance == NULL)
    {
        return NULL;
    }
    memset(instance, 0, sizeof(Super_Capacitor_t));

    config->can_init_config.can_module_callback = Decode_Super_Capacitor;
    config->can_init_config.id = instance;
    instance->super_capacitor_can_instance = CAN_Register(&config->can_init_config);

    // 注册守护线程
    supervisor_init_config_t supervisor_config = {
        .handler_callback = Super_Capacitor_Lost_Callback,
        .owner_id = instance,
        .reload_count = 1000, // 1000ms未收到数据则丢失
    };
    instance->supervisor = Supervisor_Register(&supervisor_config);

    instance->LostFlag = true; // 刚初始化时视为离线状态,直到收到第一条数据

    DWT_GetDeltaT(&instance->feed_cnt);

    DWT_Delay(0.1);

    instance->error_cnt = 0;
    instance->error_pro_cnt = 0;

    Super_Capacitor_instances[idx++] = instance;

    return instance;
}

// 错误判断
void Super_Cap_Error_Judge(Super_Capacitor_t *supercap)
{
    if(supercap->receive_data.statusCode == 2U) // 可发送消息恢复错误
    {
        supercap->error_cnt++;
        supercap->transmit_data.command.clearError = 0; 
        supercap->transmit_data.command.systemRestart = 0;

        if(supercap->error_cnt > 50)   //50*4 = 200ms
        {
            supercap->transmit_data.command.clearError = 1; // 发送清除错误命令
            supercap->error_cnt = 0;
            supercap->error_pro_cnt ++ ;
            if(supercap->error_pro_cnt >= 5)    //200*5 = 1000ms
            {
                supercap->transmit_data.command.systemRestart = 1; // 发送系统重启命令
                supercap->error_pro_cnt = 0;
            }
        }
    }
    else
    {
        supercap->transmit_data.command.clearError = 0; 
        supercap->transmit_data.command.systemRestart = 0; 
        supercap->error_cnt = 0;
        supercap->error_pro_cnt = 0;
    }
}

void Super_Capacitor_Task(void)         //在底盘任务中调用，周期为4ms
{
    Super_Capacitor_t *super_capacitor;

    for (size_t i = 0; i < idx; ++i)
    {
        super_capacitor = Super_Capacitor_instances[i];
        super_capacitor->transmit_data.refereePowerLimit = referee_outer_info->RobotPerformance.chassis_power_limit>200 ? 200 : referee_outer_info->RobotPerformance.chassis_power_limit; //功率限制上限为200W
        super_capacitor->transmit_data.refereeEnergyBuffer = referee_outer_info->PowerHeatData.buffer_energy;
        Super_Cap_Error_Judge(super_capacitor);

        memcpy(super_capacitor->super_capacitor_can_instance->tx_buff, &super_capacitor->transmit_data, 8);
        CAN_Transmit(super_capacitor->super_capacitor_can_instance, 2);
        
    }

}
