/**
* @file gimbal.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "robot_frame_config.h"
#include "message_center.h"

#include "DJI_motor.h"
#include "DM_motor.h"

#include "remote_control.h"
#include "INS.h"
#include "chassis.h"
#include "shoot.h"
#include "pid.h"
#include "Serial.h"
#include <stdbool.h>
#include "remote_vt03.h"

typedef enum
{
	AUTO_FOLLOW = 1,//云台跟随
	SHOOT = 3,//手动射击
	STOP_SHOOT = 2,//停止发射
	AUTO_SHOOT = 4,//自瞄+手打
	AUTO_FIRE = 5  //自瞄+火控
} shooter_strategy_e;//开火策略

typedef enum
{
	AUTOMATIC_AIMING = 1,
	STAND_NECK = 3,//启动云台控制
	SIT_NECK = 2,//停止云台控制
} gimbal_ctrl_e;//云台控制

typedef enum
{
	GIMBAL_DISABLE = 0,
	GIMBAL_ENABLE = 1,
} gimbal_status_e;//云台使能

typedef struct
{
	shooter_strategy_e shooter;
	gimbal_ctrl_e ctrl_mode;
	gimbal_status_e status;

	uint8_t friction_state;//摩擦轮状态:0-关闭，1-完全开启
	uint8_t shoot_advice;
	uint8_t mouse_flag;//优化操作手界面超调现象
	
	bool Neckflag;//脖子伸起标志位，true为伸出，false为缩回
	bool Yawflag;//yaw电机角度在0点附近为true,允许缩头
	uint8_t Auto_Aim_flag;//自瞄标志位，0为不自瞄,1为自瞄，2为自瞄+开火控制
	

	float pitch_init;//上电初始化pitch角度
	float pitch;
	float yaw;

	float v_yaw;          //YAW_角速度
	float v_pitch_neck;		
	float v_pitch_head;	

	float v_shoot;
}__attribute__((__packed__)) Gimbal_CmdTypedef;


typedef struct
{
	float yaw_diff;
	float pitch_diff;

	float yaw;
	float pitch;
}__attribute__((__packed__)) Gimbal_Auto; //自瞄所传参数结构体


/**/
extern Gimbal_CmdTypedef gimbal_cmd;
extern DM_motor_instance_t *DM_6220_pitch_head;
extern DM_motor_instance_t *DM_4310_pitch_neck;
extern DM_motor_instance_t *DM_6006_yaw;



/**/
void Gimbal_Init(void);
void Gimbal_Control_Remote(void);
void Gimbal_Control_Keyboard(void);

#endif /* __GIMBAL_H__ */
