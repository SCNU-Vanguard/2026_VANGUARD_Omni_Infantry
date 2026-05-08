#ifndef __ShootTION_MOTOR_H__
#define __ShootTION_MOTOR_H__

#include "drv_motor.h"

#include "defense_center.h"

#define Shoot_MOTOR_CNT 3

/* 滤波系数设置为1的时候即关闭滤波 */    //（未修改）
#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制

// typedef enum  //(未修改)
// {
//     ORIGIN = 0,
//     RAD = 1,
//     DEGREE = 2,
// } Shoot_motor_feedback_data_e;

/* 摩擦轮电机CAN反馈信息*/ 
// typedef struct  //与DJI电机相同can协议
// {
//     int16_t flag;   // 电机返回状态值
// } Shoot_motor_callback_t;

// /* 发送给摩擦轮电调的信息*/
// typedef struct
// {
//     int16_t speed; // 转速 rpm
// } Shoot_motor_fillmessage_t;

/**
 * @brief 摩擦轮电机的初始化结构体
 *
 */
typedef struct
{
    motor_model_e motor_type; // 电机类型
    motor_working_type_e motor_state_flag; //电机状态标志，用于控制电机使能或失能，与下文的

    motor_error_e error_code;


    CAN_instance_t *motor_can_instance; // 电机CAN实例
    
    int16_t receive_flag;   // 电机返回状态值  为0xA5时正常转动，为0xFF时停止转动
    int16_t target; // 电机设定转速 rpm

    supervisor_t *supervisor;

    uint32_t feed_cnt;
    float dt;

    // 发送编号位置设置
    uint8_t message_num;
} shoot_motor_instance_t;


shoot_motor_instance_t *Shoot_Motor_Init(motor_init_config_t *config);

void Shoot_Motor_Send(void);

void  Shoot_Motor_SetTar(shoot_motor_instance_t *motor , int16_t tar);

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 *
 */
void Shoot_Motor_Stop(shoot_motor_instance_t *motor);

/**
 * @brief 启动电机,此时电机会响应设定值
 *        初始化时不需要此函数,因为stop_flag的默认值为0
 *
 */
void Shoot_Motor_Enable(shoot_motor_instance_t *motor);


#endif 