/*仿照guatai版本的大疆电机库改的摩擦轮电机库,整个库都是围绕摩擦轮电机依靠G4电调只需要，使用can2通道写的，
只需要将目标转速发给电调，缺少泛用性，如果需要使用其他can线，需要再进行修改*/
//   miracle-cloud    2025/11/27


//algorithmn


//application


//bsp
#include "bsp_can.h"
#include "bsp_dwt.h"

//module
#include "shoot_motor.h"

static uint8_t idx = 0; //  摩擦轮电机全局索引，相当于摩擦轮电机数量

static shoot_motor_instance_t *shoot_motor_instances[Shoot_MOTOR_CNT] = {NULL}; // 对应电机数组

/*负责储存电机发送数据的数组*/
static CAN_instance_t shoot_sender_assignment[2] = {
    [0] = {.can_handle = &hfdcan2, .tx_header.Identifier = 0x010, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [1] = {.can_handle = &hfdcan2, .tx_header.Identifier = 0x01f, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}}, 
};

/*static*/  
uint8_t shoot_sender_enable_flag[2] = {0}; //对应数组是否使用的标志


// static void Motor_Sender_Grouping(Shoot_motor_instance_t *motor, can_init_config_t *config)（未完成，如果需要使用依然需要修改）
// {
//     if(config->id<1||config->id>8)  //检验电机编号是否超出限制，超出限制则进入死循环
//     {
//         while (1);
//     }
//     uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值
//     uint8_t motor_send_num;
//     uint8_t motor_grouping;
//     // switch (motor->motor_type)
//     // {
//     // case Shoot:   //虽然摩擦轮就一种电机，实际上没必要分类处理，但这是为了符合框架结构，而且遇到型号不对还可以进入死循环，起到保护作用
//     // if (motor_id < 4) // 根据ID分组
//     //     {
//     //         motor_send_num = motor_id;
//     //         motor_grouping = config->can_handle == &hfdcan1 ? 1 : (config->can_handle == &hfdcan2 ? 3 : 5);
//     //     }
//     // else
//     //     {
//     //         motor_send_num = motor_id - 4;
//     //         motor_grouping = config->can_handle == &hfdcan1 ? 0 : (config->can_handle == &hfdcan2 ? 2 : 4);
//     //     }

//         // 计算接收id并设置分组发送id
//         config->rx_id = 0x010 + motor_id + 1;   // 把ID+1,进行分组设置（）
//         sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
//         motor->message_num = motor_send_num;
//         motor->sender_group = motor_grouping;
// }

/*特异化符合G4电调控制，泛用性低，仅用在当前发射机构上，其他情况下参考DJI_Motor*/
static void Shoot_Motor_Sender_Grouping(shoot_motor_instance_t *motor, can_init_config_t *config)
{
    uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值

    motor->message_num = motor_id;
}


/*摩擦轮电机接收回调函数*/
static void Shoot_Motor_Decode(CAN_instance_t *_instance)  
{
    uint8_t *rxbuff = _instance->rx_buff;
    shoot_motor_instance_t *motor = (shoot_motor_instance_t *)_instance->id;

	Supervisor_Reload(motor->supervisor);
	motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

	if (motor->error_code & MOTOR_LOST_ERROR)
	{
		motor->error_code &= ~(MOTOR_LOST_ERROR);
	}

    //对接收数据的处理(接收到的数据只有data[0]中包含电机状态码，其他位均是无效数据)
    motor->receive_flag = rxbuff[0];
}

/*摩擦轮电机发送函数*/
void Shoot_Motor_Send(void)  //发送时直接用了sender_assignment[0],如果需要使用其他来发送，记得更改
{
    uint8_t num; // 电机编号
    
    shoot_motor_instance_t *motor;
    // 遍历所有电机实例，设置发送报文的值
    for (size_t i = 0; i < idx; ++i)
    {   
        motor = shoot_motor_instances[i];
        num = motor->message_num;
        /* ------------------------------handler------------------------------------*/
        // 填入发送数据
        shoot_sender_assignment[0].tx_buff[2 * num] = (uint8_t)(motor->target >> 8);   //偶数数组发高八位      
        shoot_sender_assignment[0].tx_buff[2 * num + 1] = (uint8_t)(motor->target & 0x00ff); //奇数数组发低八位

        // 若该电机处于停止状态,直接将对应buff置零
        if (motor->motor_state_flag == MOTOR_DISABLE)
        {
            memset(shoot_sender_assignment[0].tx_buff + 2 * num, 0, sizeof(uint16_t));
			motor -> target = 0;
        }
    }
    /* ------------------------------handler------------------------------------*/

    // 遍历flag,检查是否要发送这一帧报文TODO(GUATAI):后续应解耦，能够由开发者来选择何时发送，来达到每个模块不同控制频率的需求
            //CAN_Transmit(&shoot_sender_assignment[0], 2);
    for (size_t i = 0; i < 2; ++i)
    {
        if (shoot_sender_enable_flag[i])
        {
            CAN_Transmit(&shoot_sender_assignment[i], 2);
        }
    }
}

/*电机使能（允许发送目标转速）*/
void Shoot_Motor_Enable(shoot_motor_instance_t *motor)
{
    motor->motor_state_flag = MOTOR_ENABLE;
}

/*电机失能（将目标转速置零）*/
void Shoot_Motor_Stop(shoot_motor_instance_t *motor)
{
    motor->motor_state_flag = MOTOR_DISABLE;
}

/*设定电机目标转速*/
void  Shoot_Motor_SetTar(shoot_motor_instance_t *motor , int16_t tar)
{
    motor->target = tar;
}

/*摩擦轮电机出错时的错误回调函数(目前没有任何处理)*/
static void Shoot_Motor_Lost_Callback(void *motor_ptr)
{
    // uint16_t can_bus;
    // shoot_motor_instance_t *motor = (shoot_motor_instance_t *)motor_ptr;
    // can_bus = motor->motor_can_instance->can_handle == &hfdcan1 ? 1 : (motor->motor_can_instance->can_handle == &hfdcan2 ? 2 : 3);

	shoot_motor_instance_t *motor = (shoot_motor_instance_t *) motor_ptr;
	motor->error_code |= MOTOR_LOST_ERROR;
}

/*电机结构体初始化函数 */
shoot_motor_instance_t *Shoot_Motor_Init(motor_init_config_t *config)
{
    //uint8_t motor_grouping;
    shoot_motor_instance_t *instance = (shoot_motor_instance_t *)malloc(sizeof(shoot_motor_instance_t));

    if (instance == NULL)
	{
		return NULL;
	}

    memset(instance, 0, sizeof(shoot_motor_instance_t));
    
    instance->motor_type = config->motor_type; 
    Shoot_Motor_Sender_Grouping(instance, &config->can_init_config);

    // 注册电机到CAN总线
    config->can_init_config.can_module_callback = Shoot_Motor_Decode; 
    config->can_init_config.id = instance;                          
    instance->motor_can_instance = CAN_Register(&config->can_init_config);  

    // 注册守护线程
    supervisor_init_config_t supervisor_config = {
        .handler_callback = Shoot_Motor_Lost_Callback,
        .owner_id = instance,
        .reload_count = 100, // 100ms未收到数据则丢失
    };
    instance->supervisor = Supervisor_Register(&supervisor_config);

    Shoot_Motor_Stop(instance);
    shoot_sender_enable_flag[0] = 1;   //设置发送标志位,防止发送空帧(只使用了数组[0]，直接赋1，证明有电机注册到这个分组;在发送函数中会通过此标志判断是否有电机注册)
    shoot_motor_instances[idx++] = instance;
    return instance;
}

