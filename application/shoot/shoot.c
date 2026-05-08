/**
******************************************************************************
* @file    shoot.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include <string.h>
#include <stdlib.h>

#include "robot_frame_init.h"
#include "shoot.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot_motor.h"
#include "referee_task.h"



#define SHOOT_V 8000  //摩擦轮转速 rpm

DJI_motor_instance_t *chassis_shoot_motor;
shoot_motor_instance_t *friction_motor[3];

uint16_t target_shoot_frequence = 0;
shoot_state_e shoot_state = shoot_state_disable;


extern RC_ctrl_t *rc_ctl;

float t_2006;

PID_t chassis_2006_speed_pid = {
    .kp = 2.0f,
    .ki = 0.8f,
    .kd = 1.0f,
    .output_limit = 10000.0f,
    .integral_limit = 10000.0f,
    .dead_band = 0.0f,
};

PID_t friction_angle_pid = {
    .kp = 12.0f,
    .ki = 0.0f,
    .kd = 480.0f,
    .output_limit = 10.0f,
    .integral_limit = 0.0f,
    .dead_band = 0.0f,
};

PID_t friction_speed_pid = {
    .kp = 400.0f,
    .ki = 400.0f,
    .kd = 0.0f,
    .output_limit = 25000.0f,
    .integral_limit = 25000.0f,
    .dead_band = 0.0f,
};

motor_init_config_t chassis_2006_init = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_2006_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
        .control_button = POLYCYCLIC_LOOP_CONTROL,
    },

    .motor_type = M2006,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x05,
        .rx_id = 0x05,
    },

};

motor_init_config_t friction_motor_init = {
    .controller_param_init_config = {
        .angle_PID = &friction_angle_pid,
        .speed_PID = &friction_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_AND_SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = SNAIL,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x01,
        .rx_id = 0x011,
    },
};

void Shoot_Init(void)
{
    chassis_shoot_motor = DJI_Motor_Init(&chassis_2006_init);
	
	for(int i = 0;i < 3;i++)
	{
		friction_motor_init.can_init_config.tx_id = 0x01 + i;
        friction_motor_init.can_init_config.rx_id = 0x011 + i;
		friction_motor[i] = Shoot_Motor_Init(&friction_motor_init);
	}
}

static void Shoot_Enable(void)
{
    DJI_Motor_Enable(chassis_shoot_motor);
	
	for(int i = 0; i <3; i++)
    {
        Shoot_Motor_Enable(friction_motor[i]);
    }
}

static void Shoot_Stop(void)
{
    DJI_Motor_Disable(chassis_shoot_motor);
	
	for(int i = 0; i <3; i++)
    {
        Shoot_Motor_Stop(friction_motor[i]);
    }
}

static void Shoot_Set_All_Friction(int16_t speed)
{
	// for(int i = 0; i <3; i++)
    // {
    //     Shoot_Motor_SetTar(friction_motor[i],speed);
    // }
    Shoot_Motor_SetTar(friction_motor[0],speed);//右
    Shoot_Motor_SetTar(friction_motor[1],speed);//下
    Shoot_Motor_SetTar(friction_motor[2],-speed);//左
}




void Shoot_Control_Remote(void)
{
    if((rc_ctl->rc.switch_left == 1) /*&& rc_ctl->rc.switch_right == 1 */&& gimbal_cmd.Neckflag == true)
    {
        Shoot_Enable();
        Shoot_Set_All_Friction(SHOOT_V);
        if(friction_motor[0] -> receive_flag == 0xA5 && friction_motor[1] -> receive_flag == 0xA5 && friction_motor[2] -> receive_flag == 0xA5)//摩擦轮转速达到目标值后再给拨弹盘设置转速
        {
            gimbal_cmd.friction_state = 1;
            if(rc_ctl->rc.switch_right == 3)
            {
                gimbal_cmd.Auto_Aim_flag = 1;//开启自瞄
                target_shoot_frequence = abs(rc_ctl->rc.dial) /660.0f * 3000.0f; //手动开火
                DJI_Motor_Set_Ref(chassis_shoot_motor, target_shoot_frequence);
            }
            else if(rc_ctl->rc.switch_right == 1)
            {
                gimbal_cmd.Auto_Aim_flag = 2;//开启自瞄+开火控制

                if(vs_aim_packet_from_nuc.mode==2)
                {
                    target_shoot_frequence = 1200;//低弹频800
                    DJI_Motor_Set_Ref(chassis_shoot_motor, target_shoot_frequence);
                }
                else
                {
                    target_shoot_frequence = 0;
                    DJI_Motor_Set_Ref(chassis_shoot_motor, target_shoot_frequence);

                    chassis_shoot_motor->motor_controller.speed_PID->i_term = 0;
                    chassis_shoot_motor->motor_controller.angle_PID->i_term = 0;
                    chassis_shoot_motor->motor_controller.speed_PID->output = 0;
                    chassis_shoot_motor->motor_controller.angle_PID->output = 0;
                }
            }
            else
            {
                gimbal_cmd.Auto_Aim_flag = 0;//关闭自瞄

                target_shoot_frequence = 0;
                DJI_Motor_Set_Ref(chassis_shoot_motor, target_shoot_frequence);

                chassis_shoot_motor->motor_controller.speed_PID->i_term = 0;
                chassis_shoot_motor->motor_controller.angle_PID->i_term = 0;
                chassis_shoot_motor->motor_controller.speed_PID->output = 0;
                chassis_shoot_motor->motor_controller.angle_PID->output = 0;

            }
        }
        else
        {
            gimbal_cmd.friction_state = 0;
            target_shoot_frequence = 0;
            DJI_Motor_Set_Ref(chassis_shoot_motor, target_shoot_frequence);

            chassis_shoot_motor->motor_controller.speed_PID->i_term = 0;
            chassis_shoot_motor->motor_controller.angle_PID->i_term = 0;
            chassis_shoot_motor->motor_controller.speed_PID->output = 0;
            chassis_shoot_motor->motor_controller.angle_PID->output = 0;

        }
    }
    else
    {
        Shoot_Stop();
        if(friction_motor[0] -> receive_flag == 0xFF && friction_motor[1] -> receive_flag == 0xFF && friction_motor[2] -> receive_flag == 0xFF)//摩擦轮停止后才能缩脖子
        {
            gimbal_cmd.friction_state = 0;
        }
        gimbal_cmd.Auto_Aim_flag = 0;//关闭自瞄

        target_shoot_frequence = 0;
        DJI_Motor_Set_Ref(chassis_shoot_motor, target_shoot_frequence);
        
        chassis_shoot_motor->motor_controller.speed_PID->i_term = 0;
		chassis_shoot_motor->motor_controller.angle_PID->i_term = 0;
		chassis_shoot_motor->motor_controller.speed_PID->output = 0;
		chassis_shoot_motor->motor_controller.angle_PID->output = 0;
    }

    t_2006 = chassis_shoot_motor->receive_data.real_current;

    // t=chassis_shoot_motor->receive_data.real_current;
    // chassis_shoot_speed = -chassis_shoot_motor->receive_data.speed;
    // chassis_shoot_motor->motor_controller.speed_PID->kp = kp;
    // chassis_shoot_motor->motor_controller.speed_PID->ki = ki;
    // chassis_shoot_motor->motor_controller.speed_PID->kd = kd;
    // chassis_shoot_motor->motor_controller.speed_PID->output_limit = out_limit;
    // chassis_shoot_motor->motor_controller.speed_PID->integral_limit = integral_limit;


    for(int i = 0; i <3; i++)
    {
        if(friction_motor[i]->error_code & MOTOR_LOST_ERROR)
        {
            friction_motor[i] -> receive_flag = 0;
            gimbal_cmd.friction_state = 0;

            chassis_shoot_motor->motor_controller.speed_PID->i_term = 0;
            chassis_shoot_motor->motor_controller.angle_PID->i_term = 0;
            chassis_shoot_motor->motor_controller.speed_PID->output = 0;
            chassis_shoot_motor->motor_controller.angle_PID->output = 0;
        }
    }

    if(chassis_shoot_motor->error_code & MOTOR_LOST_ERROR)
    {
        chassis_shoot_motor->motor_controller.speed_PID->i_term = 0;
        chassis_shoot_motor->motor_controller.angle_PID->i_term = 0;
        chassis_shoot_motor->motor_controller.speed_PID->output = 0;
        chassis_shoot_motor->motor_controller.angle_PID->output = 0;
    }

	//DJI_Motor_Control();
	Shoot_Motor_Send();
}





void Shoot_Control_Keyboard(void)
{

    if((robot_state == robot_stretch)&& gimbal_cmd.Neckflag == true) //伸头
    {
        if(rc_vt03->key->f) //开摩擦轮
        {
            shoot_state = shoot_state_enable;
        }
        if(rc_vt03->key->g) //关摩擦轮
        {
            shoot_state = shoot_state_disable;
        }
        if(rc_vt03->mouse.press_r) //开自瞄
        {
            gimbal_cmd.Auto_Aim_flag = 1;
        }
        else //关自瞄
        {
            gimbal_cmd.Auto_Aim_flag = 0;
        }
         
        if(shoot_state == shoot_state_enable)
        {
            Shoot_Enable();
            Shoot_Set_All_Friction(SHOOT_V);
        }
        else
        {
            Shoot_Stop();
        }

        if(friction_motor[0] -> receive_flag == 0xA5 && friction_motor[1] -> receive_flag == 0xA5 && friction_motor[2] -> receive_flag == 0xA5)//摩擦轮转速达到目标值后再给拨弹盘设置转速
        {
            gimbal_cmd.friction_state = 1;
            if(rc_vt03->mouse.press_l && (referee_outer_info->PowerHeatData.shooter_17mm_barrel_heat < (referee_outer_info->RobotPerformance.shooter_barrel_heat_limit-5*10)))//手动开火 gimbal_cmd.Auto_Aim_flag = 0
            {
                target_shoot_frequence = 2000.0f; 
                DJI_Motor_Set_Ref(chassis_shoot_motor, target_shoot_frequence);
            }
            // else if(gimbal_cmd.Auto_Aim_flag == 2)//自瞄+开火控制
            // {
            //     if(vs_aim_packet_from_nuc.mode==2)
            //     {
            //         target_shoot_frequence = 1200;//低弹频800
            //         DJI_Motor_Set_Ref(chassis_shoot_motor, target_shoot_frequence);
            //     }
            //     else
            //     {
            //         target_shoot_frequence = 0;
            //         DJI_Motor_Set_Ref(chassis_shoot_motor, target_shoot_frequence);

            //         chassis_shoot_motor->motor_controller.speed_PID->i_term = 0;
            //         chassis_shoot_motor->motor_controller.angle_PID->i_term = 0;
            //         chassis_shoot_motor->motor_controller.speed_PID->output = 0;
            //         chassis_shoot_motor->motor_controller.angle_PID->output = 0;
            //     }
            // }
            else
            {
                // gimbal_cmd.Auto_Aim_flag = 0;//关闭自瞄
                target_shoot_frequence = 0; 
                DJI_Motor_Set_Ref(chassis_shoot_motor, target_shoot_frequence);

                chassis_shoot_motor->motor_controller.speed_PID->i_term = 0;
                chassis_shoot_motor->motor_controller.angle_PID->i_term = 0;
                chassis_shoot_motor->motor_controller.speed_PID->output = 0;
                chassis_shoot_motor->motor_controller.angle_PID->output = 0;

            }
        }
        else
        {
            gimbal_cmd.friction_state = 0;
            target_shoot_frequence = 0;
            DJI_Motor_Set_Ref(chassis_shoot_motor, target_shoot_frequence);

            chassis_shoot_motor->motor_controller.speed_PID->i_term = 0;
            chassis_shoot_motor->motor_controller.angle_PID->i_term = 0;
            chassis_shoot_motor->motor_controller.speed_PID->output = 0;
            chassis_shoot_motor->motor_controller.angle_PID->output = 0;

        }
    }
    else
    {
        Shoot_Stop();
        shoot_state = shoot_state_disable;
        if(friction_motor[0] -> receive_flag == 0xFF && friction_motor[1] -> receive_flag == 0xFF && friction_motor[2] -> receive_flag == 0xFF)//摩擦轮停止后才能缩脖子
        {
            gimbal_cmd.friction_state = 0;
        }
        // gimbal_cmd.Auto_Aim_flag = 0;//关闭自瞄
        target_shoot_frequence = 0;
        DJI_Motor_Set_Ref(chassis_shoot_motor, target_shoot_frequence);
        chassis_shoot_motor->motor_controller.speed_PID->i_term = 0;
		chassis_shoot_motor->motor_controller.angle_PID->i_term = 0;
		chassis_shoot_motor->motor_controller.speed_PID->output = 0;
		chassis_shoot_motor->motor_controller.angle_PID->output = 0;
    }

    for(int i = 0; i <3; i++)
    {
        if(friction_motor[i]->error_code & MOTOR_LOST_ERROR)
        {
            friction_motor[i] -> receive_flag = 0;
            gimbal_cmd.friction_state = 0;

            chassis_shoot_motor->motor_controller.speed_PID->i_term = 0;
            chassis_shoot_motor->motor_controller.angle_PID->i_term = 0;
            chassis_shoot_motor->motor_controller.speed_PID->output = 0;
            chassis_shoot_motor->motor_controller.angle_PID->output = 0;
        }
    }

    if(chassis_shoot_motor->error_code & MOTOR_LOST_ERROR)
    {
        chassis_shoot_motor->motor_controller.speed_PID->i_term = 0;
        chassis_shoot_motor->motor_controller.angle_PID->i_term = 0;
        chassis_shoot_motor->motor_controller.speed_PID->output = 0;
        chassis_shoot_motor->motor_controller.angle_PID->output = 0;
    }

	Shoot_Motor_Send();
}




