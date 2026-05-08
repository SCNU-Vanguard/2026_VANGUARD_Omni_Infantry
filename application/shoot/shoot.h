/**
* @file shoot.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __SHOOT_H__
#define __SHOOT_H__

#include <stdint.h>
#include "DJI_motor.h"
#include "shoot_motor.h"
#include "rs485.h"
#include "remote_control.h"
#include "remote_vt03.h"




typedef struct 
{
    /* data */
}__attribute__((packed))shoot_behaviour_t;

typedef struct
{
    /* data */
}__attribute__((packed))shoot_cmd_t;   

typedef enum
{
    shoot_state_disable = 0, //摩擦轮命令为失能
    shoot_state_enable = 1,  //摩擦轮命令为使能，等待摩擦轮转速达到目标值
}shoot_state_e;


extern uint16_t target_shoot_frequence;

extern DJI_motor_instance_t *chassis_shoot_motor;
extern shoot_motor_instance_t *friction_motor[3];




void Shoot_Init(void);
void Shoot_Control_Remote(void);
void Shoot_Control_Keyboard(void);


#endif /* __SHOOT_H__ */