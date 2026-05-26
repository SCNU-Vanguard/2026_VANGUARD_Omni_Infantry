/**
******************************************************************************
* @file    chassis.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/
#include "chassis.h"
#include "vofa.h"
#include "signal_generator.h"


extern RC_ctrl_t *rc_ctl;

DJI_motor_instance_t *chassis_m3508[4];
Chassis_CmdTypedef chassis_cmd;
float target_speed[4] = {0};//底盘解算出的电机目标值（rad/s）
float target_omega = 0;//底盘旋转角速度

Super_Capacitor_t *chassis_super_capacitor;

//注意堆栈大小，使用同一个结构体，堆栈太小到会导致配置错误
//PID_t chassis_3508_speed_pid = {
//    .kp = 28.005f,
//    .ki = 0.26f,
//    .kd = 3.0f,
//    .output_limit = 15000.0f, 
//    .integral_limit = 1000.0f,
//    .dead_band = 0.0f,
//};

PID_t chassis_3508_speed_pid = {
    .kp = 28.005f,
    .ki = 0.26f,
    .kd = 3.0f,
    .output_limit = 16000.0f,//16000.0f
    .integral_limit = 1000.0f,//1000.f
    .dead_band = 0.0f,
};

motor_init_config_t chassis_3508_init = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_3508_speed_pid,
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

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
        .control_button = POLYCYCLIC_LOOP_CONTROL
    },

    .motor_type = M3508,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x02,
        .rx_id = 0x02,
    },

    // .motor_control_type = 0,//和dji无关
};

static Super_Capacitor_t chassis_super_cap_cfg = {
    
    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x06A,
        .rx_id = 0x05A,
    },
};

/*
 * @brief  	全向轮运动学逆解算(3508电机)
 * @param	底盘控制结构体指针
 * @retval 	float[4], 各轮角速度
 * @note:   仅考虑全向轮等距放置夹角为45°情况
 */
void Omni_Solve(Chassis_CmdTypedef *cmd, float *ret, float Omega)
{
 	/*         vx
				^
				|
		   0//      \\1
		   // \    / \\
			   top      -->vy
		   \\ /    \ //
		   3\\      //2
					
	*/
    ret[0] =      ((cmd->vx)*M_SQRT1_2 +         (cmd->vy)*M_SQRT1_2 + Omega*CHASSIS_RADIUS) /WHEEL_RADIUS*M3508_REDUCTION_RATIO;
    ret[1] = ((-1)*(cmd->vx)*M_SQRT1_2 +         (cmd->vy)*M_SQRT1_2 + Omega*CHASSIS_RADIUS) /WHEEL_RADIUS*M3508_REDUCTION_RATIO;
    ret[2] = ((-1)*(cmd->vx)*M_SQRT1_2 +	(-1)*(cmd->vy)*M_SQRT1_2 + Omega*CHASSIS_RADIUS) /WHEEL_RADIUS*M3508_REDUCTION_RATIO;
    ret[3] =      ((cmd->vx)*M_SQRT1_2 +    (-1)*(cmd->vy)*M_SQRT1_2 + Omega*CHASSIS_RADIUS) /WHEEL_RADIUS*M3508_REDUCTION_RATIO;

}

/*
 * @brief  	底盘坐标系转云台坐标系
 * @param	底盘命令结构体指针
 * @param   底盘坐标系中，云台yaw与底盘x平行时yaw轴电机角度（编码值）
 * @param   yaw电机当前角度（-PI ~ PI）
 * @retval 	无
 */
void Chassis_Cmd_Trans (Chassis_CmdTypedef *cmd, float chs_zeropoint, float gim_ang)
{
	float bias_theta = gim_ang - chs_zeropoint; 

	if ( bias_theta > 2 * PI )
		bias_theta -= 2 * PI;
	else if ( bias_theta < 0 )
		bias_theta += 2 * PI;
	
	if(bias_theta < 0.1)//BIAS_DEADBAND死区
	{
		bias_theta = 0;
	}
	
	// float vx_tmp = cmd -> vx * cosf (bias_theta) - cmd -> vy * sinf (bias_theta);
	// float vy_tmp = cmd -> vx * sinf (bias_theta) + cmd -> vy * cosf (bias_theta);
    float vx_tmp =  cmd -> vx * cosf (bias_theta) + cmd -> vy * sinf (bias_theta);
	float vy_tmp = -cmd -> vx * sinf (bias_theta) + cmd -> vy * cosf (bias_theta);

	cmd -> vx = vx_tmp;
	cmd -> vy = vy_tmp;
	
}


// /*
//  * @brief  	全向轮逆解算(3508电机)
//  * @param	底盘四电机结构体指针ID，顺序1-4
//  * @retval 	底盘自旋速度，rad/s
//  * @todo    按需增加对里程的解算
//  */
float Chassis_Get_Omega (void)
{
	uint8_t i = 0;
	float v_total = 0;//四个3508转子的角速度之和

	for(i = 0; i < 4; i++)
    {
        v_total += chassis_m3508[i]->receive_data.speed; //单位rpm
    }
		 
	v_total = v_total/4;                                            //平均一个3508 ， 单位rpm
	v_total = v_total * RPM_2_RAD_PER_SEC /M3508_REDUCTION_RATIO;   // 转子 --> 转轴   rad/s
	v_total = v_total * WHEEL_RADIUS / CHASSIS_RADIUS; // rps （轮子的线速度 = 底盘的线速度） 单位变换时要调内环PID
	return v_total;    
}


void Chassis_Super_Capacitor_Init(void)
{
    chassis_super_capacitor = Super_Capacitor_Init(&chassis_super_cap_cfg);
    chassis_super_capacitor->transmit_data.refereeEnergyBuffer = 60; //初始裁判系统缓冲能量为60j
    chassis_super_capacitor->transmit_data.refereePowerLimit = 45; //血量优先1级时功率限制为45W
    chassis_super_capacitor->transmit_data.command.enableDCDC = 1; // 默认使能DCDC
}
/*
 * @brief  	底盘电机初始化
 * @param	无
 * @retval 	无
 */
void Chassis_Init(void)
{
    for(int i = 0; i <4; i++)
    {
        chassis_3508_init.can_init_config.tx_id = 0x01 + i;
        chassis_3508_init.can_init_config.rx_id = 0x01 + i;
        chassis_m3508[i] = DJI_Motor_Init(&chassis_3508_init);
        chassis_m3508[i]->motor_feedback = RAD;
    }
		
		// Super_Capacitor_instance = CAN_Register(&super_capacitor_can_init_config);
    Chassis_Super_Capacitor_Init();
}

void Chassis_Enable(void)
{
    for(int i = 0; i <4; i++)
    {
        DJI_Motor_Enable(chassis_m3508[i]);
    }
}

void Chassis_Stop(void)
{
    for(int i = 0; i <4; i++)
    {
        DJI_Motor_Disable(chassis_m3508[i]);
    }
}

/////////////////////////////////////////////
static void Remote_Ctrl(Chassis_CmdTypedef *chs)
{
	if(rc_ctl -> rc . rocker_l1 < 5 && rc_ctl -> rc . rocker_l1 > -5)//摇杆死区
	{
		chs -> vx = 0;
	}
	else
	{
		chs -> vx = (float) rc_ctl -> rc . rocker_l1 * REMOTE_X_SEN ;
	}
	if(rc_ctl -> rc . rocker_l_ < 5 && rc_ctl -> rc . rocker_l_ > -5)//摇杆死区
	{
		chs -> vy = 0;
	}
	else
	{
		chs -> vy = (float) rc_ctl -> rc . rocker_l_ * REMOTE_Y_SEN ;
	}
    if((rc_ctl -> rc . dial > 5)||(rc_ctl -> rc . dial < -5))
    {
        chs->omega_z = rc_ctl -> rc . dial * REMOTE_OMEGA_Z_SEN; 
    }
    else
    {
        chs->omega_z = 0;
    }
}
///////////////////////////////////////////



#include "robot_frame_init.h"

// float chassis_speed=0;
// float chassis_target_speed=0;

void Chassis_Ctrl_Remote(void)
{
    // chassis_speed = chassis_m3508[0]->receive_data.speed * 2 * PI / 60.0f; 
    // chassis_target_speed = target_speed[0];
    Remote_Ctrl(&chassis_cmd);

        //模式处理
        if((rc_ctl -> rc . switch_left == 3||rc_ctl -> rc . switch_left == 1)&&gimbal_cmd.Neckflag==false) //缩头
        {
            //底盘解算
            Omni_Solve(&chassis_cmd, target_speed,rc_ctl->rc.rocker_r_ * 0.01f);
            //设目标值
            for(int i = 0; i < 4; i++)
            {
                DJI_Motor_Set_Ref(chassis_m3508[i], target_speed[i]);
            }
            Chassis_Enable();
        }
        else if((rc_ctl -> rc . switch_left == 3/*||rc_ctl -> rc . switch_left == 1*/)&&gimbal_cmd.Neckflag==true) //伸头
        {
            //坐标系转化
            Chassis_Cmd_Trans(&chassis_cmd, 0, DM_6006_yaw->receive_data.position);
            if(chassis_cmd.omega_z)
            {
                target_omega = chassis_cmd.omega_z; 
            }
            else //底盘跟随
            {
                target_omega = -DM_6006_yaw->receive_data.position*4.0f;
            }
			//底盘解算
            Omni_Solve(&chassis_cmd, target_speed,target_omega);
            //设目标值
            for(int i = 0; i < 4; i++)
            {
                DJI_Motor_Set_Ref(chassis_m3508[i], target_speed[i]);
            }            
            Chassis_Enable();
        }
        // else if(rc_ctl -> rc . switch_left == 1&&gimbal_cmd.Neckflag==true) //视觉用
        // {
        //     //坐标系转化
        //     Chassis_Cmd_Trans(&chassis_cmd, 0, DM_6006_yaw->receive_data.position);
            
        //     target_omega = -DM_6006_yaw->receive_data.position*4.0f;//底盘跟随
                
		// 	//底盘解算
        //     Omni_Solve(&chassis_cmd, target_speed,target_omega);
        //     //设目标值
        //     for(int i = 0; i < 4; i++)
        //     {
        //         DJI_Motor_Set_Ref(chassis_m3508[i], target_speed[i]);
        //     }            
        //     Chassis_Enable();
        // }
        else
        {
            
            chassis_cmd.vx = 0;
            chassis_cmd.vy = 0; 
            Chassis_Stop();

            for(int i = 0; i <4; i++)
            {
                chassis_m3508[i]->motor_controller.speed_PID->i_term = 0;
                chassis_m3508[i]->motor_controller.speed_PID->output = 0;
            }
           
        }

        for(int i = 0; i <4; i++)
        {
            if(chassis_m3508[i]->error_code&MOTOR_LOST_ERROR)
            {
                chassis_m3508[i]->motor_controller.speed_PID->i_term = 0;
                chassis_m3508[i]->motor_controller.speed_PID->output = 0;
            }
        }

        DJI_Motor_Control(NULL);//电机pid计算及发送控制报文 , 与波弹盘拆解 
}


static float temp_vx = 0;
static float temp_vy = 0;
static float chassis_spin_wobble_phase = 0.0f;   // 弧度

static void Keyboard_Ctrl(Chassis_CmdTypedef *chs)
{
    temp_vx += (rc_vt03->key->w - rc_vt03->key->s) * KETBOARD_X_SEN;
    temp_vx = abs_limit(temp_vx, CHASSIS_SPEED_MAX);
    if((!rc_vt03->key->w)&&(!rc_vt03->key->s))
    {
        if(temp_vx > 0)
        {
            temp_vx -= 4*KETBOARD_X_SEN;
            if(temp_vx < 0) temp_vx = 0;
        }
        else if(temp_vx < 0)
        {
            temp_vx += 4*KETBOARD_X_SEN;
            if(temp_vx > 0) temp_vx = 0;
        }
    }

    temp_vy += (rc_vt03->key->d - rc_vt03->key->a) * KETBOARD_Y_SEN;
    temp_vy = abs_limit(temp_vy, CHASSIS_SPEED_MAX);
    if((!rc_vt03->key->a)&&(!rc_vt03->key->d))
    {
        if(temp_vy > 0)
        {
            temp_vy -= 4*KETBOARD_Y_SEN;
            if(temp_vy < 0) temp_vy = 0;
        }
        else if(temp_vy < 0)
        {
            temp_vy += 4*KETBOARD_Y_SEN;
            if(temp_vy > 0) temp_vy = 0;
        }
    }

    if(rc_vt03->key->shift)
    {
        chs->mode = SPIN;
    }
    if(rc_vt03->key->ctrl)
    {
        chs->mode = FOLLOW;
    }

    if(chs->mode == SPIN)
    {
        chassis_spin_wobble_phase += 2.0f * PI * CHASSIS_SPIN_WOBBLE_FREQ / 250.0f;  // 250Hz 更新频率
        if (chassis_spin_wobble_phase > 2.0f * PI) chassis_spin_wobble_phase -= 2.0f * PI;
        chs->omega_z = CHASSIS_OMEGA_Z_MAX + CHASSIS_SPIN_WOBBLE_AMP * arm_sin_f32(chassis_spin_wobble_phase);
    }
    else if(chs->mode == FOLLOW)
    {
        chs->omega_z = 0;
        chassis_spin_wobble_phase = 0;
    }

    chs->vx = temp_vx;
    chs->vy = temp_vy;


}

void Chassis_Control_Keyboard(void)
{
    Keyboard_Ctrl(&chassis_cmd);

    if((robot_state==robot_shrink)&&gimbal_cmd.Neckflag==false) //缩头
    {
        // //底盘解算
        // Omni_Solve(&chassis_cmd, target_speed,rc_vt03->mouse.x * 0.01f);
        // //设目标值
        // for(int i = 0; i < 4; i++)
        // {
        //     DJI_Motor_Set_Ref(chassis_m3508[i], target_speed[i]);
        // }
        // Chassis_Enable();
    }
    else if((robot_state==robot_stretch)&&gimbal_cmd.Neckflag==true) //伸头
    {
        //坐标系转化
        Chassis_Cmd_Trans(&chassis_cmd, 0, DM_6006_yaw->receive_data.position);
        if(chassis_cmd.omega_z)
        {
            target_omega = chassis_cmd.omega_z; 
        }
        else //底盘跟随
        {
            target_omega = -DM_6006_yaw->receive_data.position*4.0f;
        }
        //底盘解算
        Omni_Solve(&chassis_cmd, target_speed,target_omega);
        //设目标值
        for(int i = 0; i < 4; i++)
        {
            DJI_Motor_Set_Ref(chassis_m3508[i], target_speed[i]);
        }            
        Chassis_Enable();
    }
    else if((robot_state==robot_tunnel)&&gimbal_cmd.Neckflag==false) //隧道模式
    {
        //底盘解算
        Omni_Solve(&chassis_cmd, target_speed,rc_vt03->mouse.x * 0.01f);
        //设目标值
        for(int i = 0; i < 4; i++)
        {
            DJI_Motor_Set_Ref(chassis_m3508[i], target_speed[i]);
        }
        Chassis_Enable();
    }
    else
    {
        
        chassis_cmd.vx = 0;
        chassis_cmd.vy = 0; 
        Chassis_Stop();

        for(int i = 0; i <4; i++)
        {
            chassis_m3508[i]->motor_controller.speed_PID->i_term = 0;
            chassis_m3508[i]->motor_controller.speed_PID->output = 0;
        }
        
    }

    for(int i = 0; i <4; i++)
    {
        if(chassis_m3508[i]->error_code&MOTOR_LOST_ERROR)
        {
            chassis_m3508[i]->motor_controller.speed_PID->i_term = 0;
            chassis_m3508[i]->motor_controller.speed_PID->output = 0;
        }
    }

    DJI_Motor_Control(NULL);//电机pid计算及发送控制报文 , 与波弹盘拆解 
    
}








