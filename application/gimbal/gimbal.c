/**
******************************************************************************
* @file    gimbal.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include "gimbal.h"

#include "user_lib.h"
#include "signal_generator.h"

Gimbal_CmdTypedef gimbal_cmd;

DM_motor_instance_t *DM_6220_pitch_head; // 云台头，imu串级控制
DM_motor_instance_t *DM_4310_pitch_neck; // 云台脖子，串级
DM_motor_instance_t *DM_6006_yaw;		 // 云台yaw电机，imu串级控制

extern RC_ctrl_t *rc_ctl;




PID_t yaw_6006_angle_pid = {
	.kp = 15,//21.0f, 
	.ki = 0.0f,
	.kd = 4,//9.0f,
	.integral_limit = 6.0f,
	.output_limit = 10.0f,
	.dead_band = 0.0f,
};
PID_t yaw_6006_speed_pid = {
	.kp = 0.9f,//1.2f,
	.ki = 0.0045f,//0.006f,
	.kd = 0.0f,
	.integral_limit = 1.5f,//5.0f,
	.output_limit = 1.5f,//5.0f,
	.dead_band = 0.0f,
};

PID_t angle_pid_pitch_head = {
	.kp = 18.0f,//10.0f, 
	.ki = 0.0f,
	.kd = 3.0f,
	.integral_limit = 0.0f,
	.output_limit = 10.0f,
	.dead_band = 0.0f,
};

PID_t speed_pid_pitch_head = {
	.kp = 1.5f,//2.25f,
	.ki = 0.007f,//0.009f,
	.kd = 0.0f,
	.kf = 0.0f,
	.integral_limit = 2.5f,
	.output_limit = 2.5f,
	.dead_band = 0.0f,
};

PID_t angle_pid_pitch_neck = {
	.kp = 13.5f,//12.5
	.ki = 0.0f,
	.kd = 35.0f,//35
	.integral_limit = 0.0f,//0
	.output_limit = 10.0f,//10
	.dead_band = 0.0f,
};

PID_t speed_pid_pitch_neck = {
	.kp = 3.6f,//3.6
	.ki = 0.03f,//0.0079
	.kd = 0.0f,
	.integral_limit = 3.0f,//4
	.output_limit = 4.0f,//4
	.dead_band = 0.0f,
};

/*
 * @brief 达妙电机配置初始化结构体
 */
// DM_MOTOR_ABSOLUTE
motor_init_config_t dm_6006_yaw = {
	.controller_param_init_config = {
		.angle_PID = &yaw_6006_angle_pid,
		.speed_PID = &yaw_6006_speed_pid,
		.current_PID = NULL,
		.torque_PID = NULL,

		.other_angle_feedback_ptr = &INS.Yaw,	  
		.other_speed_feedback_ptr = &INS.Gyro[2], 

		.angle_feedforward_ptr = NULL,
		.speed_feedforward_ptr = NULL,
		.current_feedforward_ptr = NULL,
		.torque_feedforward_ptr = NULL,

		.pid_ref = 0.0f,
	},
	.controller_setting_init_config = {
		.outer_loop_type = ANGLE_LOOP,
		.close_loop_type = ANGLE_AND_SPEED_LOOP,
		// .outer_loop_type = SPEED_LOOP,
		// .close_loop_type = SPEED_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = OTHER_FEED, // 使用imu作为角度反馈
		// .angle_feedback_source = MOTOR_FEED,//使用电机can反馈信息
		// .speed_feedback_source = MOTOR_FEED,
		.speed_feedback_source = OTHER_FEED,

		.feedforward_flag = FEEDFORWARD_NONE, 
		.control_button = POLYCYCLIC_LOOP_CONTROL,

	},

	.motor_type = DM6006,

	.can_init_config = {
		.can_handle = &hfdcan1,
		.tx_id = 0x01,
		.rx_id = 0x11,
	},
	// .motor_control_type = MIT_MODE_E,//达妙电机力矩模式
};

motor_init_config_t p_n_4310 = {

	.motor_type = DM4310,

	.controller_param_init_config = {
		.angle_PID = &angle_pid_pitch_neck, // 电机控制器的pid指针
		.speed_PID = &speed_pid_pitch_neck,
		.current_PID = NULL,
		.torque_PID = NULL,

		.other_angle_feedback_ptr = NULL, 
		.other_speed_feedback_ptr = NULL, 

		.angle_feedforward_ptr = NULL,
		.speed_feedforward_ptr = NULL,
		.current_feedforward_ptr = NULL,
		.torque_feedforward_ptr = NULL, // ptr,前馈数据指针

		.pid_ref = 0.0f, // pid目标值
	},

	.controller_setting_init_config = {
		.outer_loop_type =ANGLE_LOOP ,//,OPEN_LOOP 
		.close_loop_type = ANGLE_AND_SPEED_LOOP,//, OPEN_LOOP

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,		// 默认为正转模式
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL, // 反馈量正向

		.angle_feedback_source = MOTOR_FEED, // 角度反馈来源，可以为电机can反馈信息或其他来源
		.speed_feedback_source = MOTOR_FEED, // 速度反馈来源，一般选择电机can反馈信息 ， 其他来源则使用other_speed_feedback_ptr指针

		.feedforward_flag = FEEDFORWARD_NONE, // 前馈反馈模式，可选择扭矩/速度，或两者皆有
		.control_button = POLYCYCLIC_LOOP_CONTROL,

	},

	.can_init_config = {
		// can配置
		.can_handle = &hfdcan2,
		.tx_id = 0x06,
		.rx_id = 0x16,
		
	},

	// .motor_control_type = POS_MODE_E,//电机控制模式，用于初始化，适配达妙电机三种模式
};

float pitch_head_torque_feedforward = 0.0f;
float kp_torque_ff = 0.0f;//-1.0
float torque_ff=0.54f;

motor_init_config_t p_h_6220 = {

	.motor_type = DM6220,

	.controller_param_init_config = {
		.angle_PID = &angle_pid_pitch_head, // 电机控制器的pid指针
		.speed_PID = &speed_pid_pitch_head,
		.current_PID = NULL,
		.torque_PID = NULL,

		.other_angle_feedback_ptr = &INS.Pitch,
		.other_speed_feedback_ptr = &INS.Gyro[0], // 其他反馈来源的数据指针

		.angle_feedforward_ptr = NULL,
		.speed_feedforward_ptr = NULL,
		.current_feedforward_ptr = NULL,
		.torque_feedforward_ptr = &pitch_head_torque_feedforward, // ptr,前馈数据指针
		// .torque_feedforward_ptr = NULL ,
		.pid_ref = 0.0f, // pid目标值
	},

	.controller_setting_init_config = {
		.outer_loop_type = ANGLE_LOOP,
		.close_loop_type = ANGLE_AND_SPEED_LOOP,
		// .outer_loop_type = SPEED_LOOP,
		// .close_loop_type = SPEED_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,		// 默认为正转模式
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL, // 反馈量正向

		// .angle_feedback_source = MOTOR_FEED, // 角度反馈来源，可以为电机can反馈信息或其他来源
		// .speed_feedback_source = MOTOR_FEED, // 速度反馈来源，一般选择电机can反馈信息 ， 其他来源则使用other_speed_feedback_ptr指针
		.angle_feedback_source = OTHER_FEED, // 角度反馈来源，可以为电机can反馈信息或其他来源
		.speed_feedback_source = OTHER_FEED, // 速度反馈来源，一般选择电机can反馈信息 ， 其他来源则使用other_speed_feedback_ptr指针

		.feedforward_flag = TORQUE_FEEDFORWARD, // 前馈反馈模式，可选择扭矩/速度，或两者皆有
		// .feedforward_flag = FEEDFORWARD_NONE,
		.control_button = POLYCYCLIC_LOOP_CONTROL,
	},

	.can_init_config = {
		// can配置
		.can_handle = &hfdcan2,
		.tx_id = 0x05,
		.rx_id = 0x15,
	},
};



void Gimbal_Init(void)
{

	
	DM_6220_pitch_head = DM_Motor_Init(&p_h_6220);
	DM_6220_pitch_head->dm_mode = MIT_MODE; // 设置为MIT模式
	//DM_6220_pitch_head->motor_feedback = DM_MOTOR_ABSOLUTE; // 设置电机反馈数据为绝对值

	DM_4310_pitch_neck = DM_Motor_Init(&p_n_4310);
	DM_4310_pitch_neck->dm_mode = MIT_MODE,//POS_MODE; 
	DM_4310_pitch_neck->motor_feedback = DM_MOTOR_ABSOLUTE; // 设置电机反馈数据为绝对值

	DM_6006_yaw = DM_Motor_Init(&dm_6006_yaw);
	DM_6006_yaw->dm_mode = MIT_MODE; // 设置为MIT模式
	
}

void Gimbal_Enable(void)
{
	if (DM_6220_pitch_head->receive_data.state == 0)
	{
		DM_Motor_Enable(DM_6220_pitch_head);
	}

	if (DM_4310_pitch_neck->receive_data.state == 0)
	{
		DM_Motor_Enable(DM_4310_pitch_neck);
	}
	// DM_Motor_Enable(DM_6006_yaw);
}

void Gimbal_Disable(void)
{
	if (DM_6220_pitch_head->receive_data.state == 1)
	{
		DM_Motor_Disable(DM_6220_pitch_head);
	}

	if (DM_4310_pitch_neck->receive_data.state == 1)
	{
		DM_Motor_Disable(DM_4310_pitch_neck);
	}
	if (DM_6006_yaw->receive_data.state == 1)
	{
		DM_Motor_Disable(DM_6006_yaw);
	}
}

void Gimbal_Stop(void)
{
	DM_Motor_Stop(DM_6220_pitch_head);
	DM_Motor_Stop(DM_4310_pitch_neck);
	DM_Motor_Stop(DM_6006_yaw);
}

#include "robot_frame_init.h"

float target_speed_yaw = 0.0f;
float yaw_speed = 0;

float kp = 0.9f;
float ki = 0.0045f;
float kd = 0;
float kf = 0;
float out_limit = 1.5f;
float integral_limit = 1.5f;

float AA = 0;
float frq = 0;

float target_angle_yaw = 0.0f;
float yaw_angle = 0;

float kp_A = 15;//21.0;
float ki_A = 0;
float kd_A = 4.0;//9.0
float Angle_out_limit = 10.0f;
float Angle_integral_limit = 0.0f;


float target_angle_head = 0;
float pitch_angle = 0;

float measure_head_speed = 0;
float motor_head_v = 0;


float k1 = 0.04f;

float target_angle_neck = 0;

float yaw_t;
float neck_t;
float head_t;

float target_speed_yaw;
float speed_ref;

// float Gimbal_Limit(float temp)
// {
//    if(temp>GIMBAL_LIMIT_MAX)
//       temp = GIMBAL_LIMIT_MAX;
//    if(temp<-GIMBAL_LIMIT_MAX)
//       temp = -GIMBAL_LIMIT_MAX;
	  
// 	  return temp;
// }

// void Mouse_control_gimbal(Gimbal_CmdTypedef *gim)
// {
// 	if(rc_ctl ->mouse.x)
// 	{
// 	   gim->yaw = rc_ctl ->mouse.x *KEYBOARD_YAW_SEN;
// 	   gim->yaw = Gimbal_Limit(gim->yaw);
// 	}
// 	if(rc_ctl ->mouse.y)
// 	{
// 	   gim->pitch = rc_ctl ->mouse.y * KEYBOARD_HEAD_SEN;
// 	   gim->pitch = Gimbal_Limit(gim->pitch);
// 	}

// }

void Gimbal_Control_Remote(void)
{

	if ((rc_ctl->rc.switch_left == 3 || rc_ctl->rc.switch_left == 1) && rc_ctl->rc.switch_right != 2) // 伸头
	{

		Gimbal_Enable();
		DM_Motor_Start(DM_6220_pitch_head);

		DM_Motor_Start(DM_4310_pitch_neck);
		DM_Motor_Start(DM_6006_yaw);

		if (DM_6006_yaw->receive_data.state == 0)
		{
			DM_Motor_Enable(DM_6006_yaw);
		}

		DM_Motor_SetTar(DM_4310_pitch_neck, 0);

		if (gimbal_cmd.Neckflag == true)
		{
			

			

			
			if((vs_aim_packet_from_nuc.mode == 1||vs_aim_packet_from_nuc.mode == 2) && gimbal_cmd.Auto_Aim_flag != 0)//开启自瞄且视觉有数据输入
			{
				// 检测并过滤 NaN 数据
                if (isnan(vs_aim_packet_from_nuc.yaw)) 
                {
                    ;
                }
				// else
				// {
				// 	if(fabsf(target_angle_yaw - vs_aim_packet_from_nuc.yaw) <= 0.003f)
				// 	{
				// 		target_angle_yaw = vs_aim_packet_from_nuc.yaw;
				// 	}
				// 	else if(target_angle_yaw < vs_aim_packet_from_nuc.yaw)
				// 	{
				// 		target_angle_yaw += 0.003f;
				// 	}
				// 	else if (target_angle_yaw > vs_aim_packet_from_nuc.yaw)
				// 	{
				// 		target_angle_yaw -= 0.003f;
				// 	}
				// }
				else // 只有当数据不是NaN时，才进行处理
                {
					float yaw_diff = vs_aim_packet_from_nuc.yaw - INS.Yaw;
					// 处理角度差，使其在[-pi, pi]范围内
					while (yaw_diff > M_PI)
					{
						yaw_diff -= 2 * M_PI;
					}
					while (yaw_diff < -M_PI)
					{
						yaw_diff += 2 * M_PI;
					}
					// 判断归一化后的差值绝对值是否在安全范围内
					if (fabsf(yaw_diff) < M_PI_2) // 安全范围设为90度（pi/2弧度）
					{
						// 数据正常，接受视觉数据
						target_angle_yaw = vs_aim_packet_from_nuc.yaw;
					}
					else				
					{
						;
					}
				}

				// 检测并过滤 NaN 数据
                if (isnan(vs_aim_packet_from_nuc.pitch)) 
                {
                    ;
                }
				else
				{
					if(fabsf(target_angle_head - vs_aim_packet_from_nuc.pitch) <= 0.0025f)
					{
						target_angle_head = vs_aim_packet_from_nuc.pitch;
					}
					else if(target_angle_head < vs_aim_packet_from_nuc.pitch)
					{
						target_angle_head += 0.0025f;
					}
					else if (target_angle_head > vs_aim_packet_from_nuc.pitch)
					{
						target_angle_head -= 0.0025f;
					}
					
				}
                // else // 只有当数据不是NaN时，才进行处理
                // {
				// 	float pitch_diff = vs_aim_packet_from_nuc.pitch - INS.Pitch;
				// 	// 处理角度差，使其在[-pi, pi]范围内
				// 	while (pitch_diff > M_PI)
				// 	{
				// 		pitch_diff -= 2 * M_PI;
				// 	}
				// 	while (pitch_diff < -M_PI)
				// 	{
				// 		pitch_diff += 2 * M_PI;
				// 	}
				// 	// 判断归一化后的差值绝对值是否在安全范围内
				// 	if (fabsf(pitch_diff) < 0.25f) // 安全范围设为0.34f
				// 	{
				// 		// 数据正常，接受视觉数据
				// 		target_angle_head = vs_aim_packet_from_nuc.pitch;
				// 	}
				// 	else				
				// 	{
				// 		;
				// 	}
				// }
			}
			else
			{
				target_angle_yaw += rc_ctl->rc.rocker_r_ * - (REMOTE_YAW_SEN);
				target_angle_head += rc_ctl->rc.rocker_r1 * - (REMOTE_HEAD_SEN);
				// target_angle_yaw = gimbal_cmd.yaw;
				// target_angle_head = gimbal_cmd.pitch;
			}
			
			if (target_angle_head > GIMBAL_LIMIT_MAX)
			{
				target_angle_head = GIMBAL_LIMIT_MAX;
			}
			if (target_angle_head < GIMBAL_LIMIT_MIN)
			{
				target_angle_head = GIMBAL_LIMIT_MIN;
			}


			// if(target_angle_yaw > 0.7f)//0.7
			// {
			// 	target_angle_yaw = 0.7f;
			// }
			// if(target_angle_yaw < -0.7f)
			// {
			// 	target_angle_yaw = -0.7f;
			// }

			while (target_angle_yaw - INS.Yaw > (PI))
			{
				target_angle_yaw -= 2 * PI;
			}
			while (target_angle_yaw - INS.Yaw < (-PI))
			{
				target_angle_yaw += 2 * PI;
			}

			
			DM_Motor_SetTar(DM_6006_yaw, target_angle_yaw);

			// t = DM_6220_pitch_head->transmit_data.torque_des;
			DM_Motor_SetTar(DM_6220_pitch_head, target_angle_head);
			// yaw_speed=DM_6006_yaw->receive_data.velocity;
			//yaw_speed = INS.Gyro[2];

			// pitch_head_torque_feedforward = 1 * kp_torque_ff * torque_ff * cosf(INS.Pitch);
			// pitch_head_torque_feedforward = kp_torque_ff * torque_ff*cosf(DM_6220_pitch_head->receive_data.position);

			// measure_head_speed = DM_6220_pitch_head->motor_controller.speed_PID->measure;
			// motor_head_v = DM_6220_pitch_head->receive_data.velocity;

			// DM_6220_pitch_head->motor_controller.speed_PID->kp = kp;
			// DM_6220_pitch_head->motor_controller.speed_PID->ki = ki;
			// DM_6220_pitch_head->motor_controller.speed_PID->kd = kd;
			// DM_6220_pitch_head->motor_controller.speed_PID->kf = kf;
			// DM_6220_pitch_head->motor_controller.speed_PID->output_limit = out_limit;
			// DM_6220_pitch_head->motor_controller.speed_PID->integral_limit = integral_limit;
			// pitch_angle = INS.Pitch;//DM_6220_pitch_head->receive_data.position;
			// motor_head_v=INS.Gyro[0];
			// DM_6220_pitch_head->motor_controller.angle_PID->kp = kp_A;
			// DM_6220_pitch_head->motor_controller.angle_PID->ki = ki_A;
			// DM_6220_pitch_head->motor_controller.angle_PID->kd = kd_A;
			// DM_6220_pitch_head->motor_controller.angle_PID->output_limit = Angle_out_limit;
			// DM_6220_pitch_head->motor_controller.angle_PID->integral_limit = Angle_integral_limit;

			// DM_6006_yaw->motor_controller.speed_PID->kp=kp;
			// DM_6006_yaw->motor_controller.speed_PID->ki=ki;
			// DM_6006_yaw->motor_controller.speed_PID->kd=kd;
			// DM_6006_yaw->motor_controller.speed_PID->output_limit=out_limit;
			// DM_6006_yaw->motor_controller.speed_PID->integral_limit=integral_limit;
			// yaw_angle=INS.Yaw;
			// DM_6006_yaw->motor_controller.angle_PID->kp=kp_A;
			// DM_6006_yaw->motor_controller.angle_PID->ki=ki_A;
			// DM_6006_yaw->motor_controller.angle_PID->kd=kd_A;
			// DM_6006_yaw->motor_controller.angle_PID->output_limit=Angle_out_limit;
			// DM_6006_yaw->motor_controller.angle_PID->integral_limit=Angle_integral_limit;
			// target_speed_yaw = DM_6006_yaw->motor_controller.angle_PID ->output;
			// speed_ref=INS.Gyro[2];

			// DM_4310_pitch_neck->motor_controller.speed_PID->kp = kp;
			// DM_4310_pitch_neck->motor_controller.speed_PID->ki = ki;
			// DM_4310_pitch_neck->motor_controller.speed_PID->kd = kd;
			// // DM_4310_pitch_neck->motor_controller.speed_PID->kf = kf;
			// DM_4310_pitch_neck->motor_controller.speed_PID->output_limit = out_limit;
			// DM_4310_pitch_neck->motor_controller.speed_PID->integral_limit = integral_limit;
			// pitch_angle = DM_4310_pitch_neck->receive_data.position;
			// DM_4310_pitch_neck->motor_controller.angle_PID->kp = kp_A;
			// DM_4310_pitch_neck->motor_controller.angle_PID->ki = ki_A;
			// DM_4310_pitch_neck->motor_controller.angle_PID->kd = kd_A;
			// DM_4310_pitch_neck->motor_controller.angle_PID->output_limit = Angle_out_limit;
			// DM_4310_pitch_neck->motor_controller.angle_PID->integral_limit = Angle_integral_limit;

			// t= DM_4310_pitch_neck->receive_data.torque;
			// target_angle_neck += rc_ctl->rc.rocker_r1 * -(0.000002f);

			yaw_t = DM_6006_yaw->receive_data.torque;
			neck_t = DM_4310_pitch_neck->receive_data.torque;
			head_t = DM_6220_pitch_head->receive_data.torque;

			// if(target_angle_neck>1.0)
			// {
			// 	target_angle_neck=1.0;
			// }
			// if(target_angle_neck<0)
			// {
			// 	target_angle_neck=0;
			// }
//			float Neck_Angle_output,Neck_speed_output;
//      target_angle_neck = ramp_calc(Neck_Motor,target_angle_neck);
//			Neck_Angle_output = PID_Position(DM_4310_pitch_neck->motor_controller.angle_PID,DM_4310_pitch_neck->receive_data.position,target_angle_neck);
//			Neck_speed_output = PID_Increment(DM_4310_pitch_neck->motor_controller.speed_PID,DM_4310_pitch_neck->receive_data.velocity,Neck_Angle_output);
//			DM_Motor_SetTar(DM_4310_pitch_neck, Neck_speed_output);//这里要直接在初始化使用开环控制
			// DM_Motor_SetTar(DM_4310_pitch_neck, target_angle_neck);
		}
	}
	else if (rc_ctl->rc.switch_left == 3 && rc_ctl->rc.switch_right == 2) // 缩头
	{
		if (gimbal_cmd.Yawflag == true)
		{
			Gimbal_Enable();
			DM_Motor_Start(DM_6220_pitch_head);

			DM_Motor_Start(DM_4310_pitch_neck);
			// DM_Motor_Start(DM_6006_yaw);

			DM_Motor_SetTar(DM_4310_pitch_neck, 1.0f);

			target_angle_head = -0.058268439f;
			DM_Motor_SetTar(DM_6220_pitch_head, target_angle_head);
			
		}

		if (DM_6006_yaw->receive_data.state == 0)
		{
			DM_Motor_Enable(DM_6006_yaw);
		}

		DM_Motor_Start(DM_6006_yaw);
		if(DM_6006_yaw->receive_data.position > 0.131f || DM_6006_yaw->receive_data.position < -0.07f)
		{
			//yaw电机角度不在0点附近，控制yaw目标让角度正确，不过在底盘跟随状态下yaw电机角度一般都是允许缩头的
			if(DM_6006_yaw->receive_data.position > 0.05f)
			{
				target_angle_yaw -= k1*0.1f;
			}
			else if(DM_6006_yaw->receive_data.position < -0.02f)
			{
				target_angle_yaw -= k1*(-0.1f);
			}
		}
		DM_Motor_SetTar(DM_6006_yaw, target_angle_yaw);


	}
	else
	{
		Gimbal_Stop();
		Gimbal_Disable();

		DM_6006_yaw->motor_controller.speed_PID->i_term = 0;
		DM_6006_yaw->motor_controller.angle_PID->i_term = 0;
		DM_6006_yaw->motor_controller.speed_PID->output = 0;
		DM_6006_yaw->motor_controller.angle_PID->output = 0;

		DM_6220_pitch_head->motor_controller.speed_PID->i_term = 0;
		DM_6220_pitch_head->motor_controller.angle_PID->i_term = 0;
		DM_6220_pitch_head->motor_controller.speed_PID->output = 0;
		DM_6220_pitch_head->motor_controller.angle_PID->output = 0;

		DM_4310_pitch_neck->motor_controller.speed_PID->i_term = 0;
		DM_4310_pitch_neck->motor_controller.angle_PID->i_term = 0;
		DM_4310_pitch_neck->motor_controller.speed_PID->output = 0;
		DM_4310_pitch_neck->motor_controller.angle_PID->output = 0;

		target_angle_yaw = INS.Yaw;
		DM_Motor_SetTar(DM_6006_yaw, target_angle_yaw);
	}

	// gimbal_cmd.Neckflag = true;

	if((DM_4310_pitch_neck->receive_data.position < 0.2f&&DM_4310_pitch_neck->receive_data.position > -0.23f )&&
		(DM_6220_pitch_head->receive_data.position < (GIMBAL_LIMIT_MAX+0.03f)||DM_6220_pitch_head->receive_data.position > (GIMBAL_LIMIT_MIN-0.03f)))
	{
		gimbal_cmd.Neckflag = true;//脖子必须抬起才能小陀螺和底盘跟随
	}
	else
	{
		gimbal_cmd.Neckflag = false;
	}

	if (DM_6006_yaw->receive_data.position < 0.131f && DM_6006_yaw->receive_data.position > -0.07f) 
	{
		gimbal_cmd.Yawflag = true;//yaw电机角度必须在0点附近才能缩脖子
	}
	else
	{
		gimbal_cmd.Yawflag = false;
	}


	if(DM_6006_yaw->error_code&MOTOR_LOST_ERROR||DM_6220_pitch_head->error_code&MOTOR_LOST_ERROR||
		DM_4310_pitch_neck->error_code&MOTOR_LOST_ERROR)
	{
		DM_6006_yaw->motor_controller.speed_PID->i_term = 0;
		DM_6006_yaw->motor_controller.angle_PID->i_term = 0;
		DM_6006_yaw->motor_controller.speed_PID->output = 0;
		DM_6006_yaw->motor_controller.angle_PID->output = 0;

		DM_6220_pitch_head->motor_controller.speed_PID->i_term = 0;
		DM_6220_pitch_head->motor_controller.angle_PID->i_term = 0;
		DM_6220_pitch_head->motor_controller.speed_PID->output = 0;
		DM_6220_pitch_head->motor_controller.angle_PID->output = 0;

		DM_4310_pitch_neck->motor_controller.speed_PID->i_term = 0;
		DM_4310_pitch_neck->motor_controller.angle_PID->i_term = 0;
		DM_4310_pitch_neck->motor_controller.speed_PID->output = 0;
		DM_4310_pitch_neck->motor_controller.angle_PID->output = 0;
		
	}

	// 全部电机计算
	DM_Motor_Control(NULL);

}





void Gimbal_Control_Keyboard(void)
{
	if(rc_vt03->key->c) //伸头
	{
		robot_state = robot_stretch;
	}
	if(rc_vt03->key->v) //缩头
	{
		robot_state = robot_shrink;
	}
	if(rc_vt03->key->b) //失能
	{
		robot_state = robot_disable;
	}
	if(rc_vt03->key->x) //隧道模式
	{
		robot_state = robot_tunnel;
	}



	if (robot_state == robot_stretch) // 伸头
	{

		Gimbal_Enable();
		DM_Motor_Start(DM_6220_pitch_head);

		DM_Motor_Start(DM_4310_pitch_neck);
		DM_Motor_Start(DM_6006_yaw);

		if (DM_6006_yaw->receive_data.state == 0)
		{
			DM_Motor_Enable(DM_6006_yaw);
		}

		DM_Motor_SetTar(DM_4310_pitch_neck, 0);

		if (gimbal_cmd.Neckflag == true)
		{
			if((vs_aim_packet_from_nuc.mode == 1||vs_aim_packet_from_nuc.mode == 2) && gimbal_cmd.Auto_Aim_flag != 0)//开启自瞄且视觉有数据输入
			{
				// 检测并过滤 NaN 数据
                if (isnan(vs_aim_packet_from_nuc.yaw)) 
                {
                    ;
                }
				else // 只有当数据不是NaN时，才进行处理
                {
					float yaw_diff = vs_aim_packet_from_nuc.yaw - INS.Yaw;
					// 处理角度差，使其在[-pi, pi]范围内
					while (yaw_diff > M_PI)
					{
						yaw_diff -= 2 * M_PI;
					}
					while (yaw_diff < -M_PI)
					{
						yaw_diff += 2 * M_PI;
					}
					// 判断归一化后的差值绝对值是否在安全范围内
					if (fabsf(yaw_diff) < M_PI_2) // 安全范围设为90度（pi/2弧度）
					{
						// 数据正常，接受视觉数据
						target_angle_yaw = vs_aim_packet_from_nuc.yaw;
					}
					else				
					{
						;
					}
				}

				// 检测并过滤 NaN 数据
                if (isnan(vs_aim_packet_from_nuc.pitch)) 
                {
                    ;
                }
				else
				{
					if(fabsf(target_angle_head - vs_aim_packet_from_nuc.pitch) <= 0.0025f)
					{
						target_angle_head = vs_aim_packet_from_nuc.pitch;
					}
					else if(target_angle_head < vs_aim_packet_from_nuc.pitch)
					{
						target_angle_head += 0.0025f;
					}
					else if (target_angle_head > vs_aim_packet_from_nuc.pitch)
					{
						target_angle_head -= 0.0025f;
					}
					
				}


			}
			else
			{
				target_angle_yaw += rc_vt03->mouse.x * - (KEYBOARD_YAW_SEN);
				target_angle_head += rc_vt03->mouse.y * - (KEYBOARD_HEAD_SEN);
			}
			
			
			if (target_angle_head > GIMBAL_LIMIT_MAX)
			{
				target_angle_head = GIMBAL_LIMIT_MAX;
			}
			if (target_angle_head < GIMBAL_LIMIT_MIN)
			{
				target_angle_head = GIMBAL_LIMIT_MIN;
			}

			while (target_angle_yaw - INS.Yaw > (PI))
			{
				target_angle_yaw -= 2 * PI;
			}
			while (target_angle_yaw - INS.Yaw < (-PI))
			{
				target_angle_yaw += 2 * PI;
			}

			
			DM_Motor_SetTar(DM_6006_yaw, target_angle_yaw);
			DM_Motor_SetTar(DM_6220_pitch_head, target_angle_head);

		}
	}
	else if (robot_state == robot_shrink) // 缩头
	{
		if (gimbal_cmd.Yawflag == true && gimbal_cmd.friction_state == 0)
		{
			Gimbal_Enable();
			DM_Motor_Start(DM_6220_pitch_head);
			DM_Motor_Start(DM_4310_pitch_neck);

			DM_Motor_SetTar(DM_4310_pitch_neck, 1.0f);

			target_angle_head = -0.058268439f;
			DM_Motor_SetTar(DM_6220_pitch_head, target_angle_head);
			
		}

		if (DM_6006_yaw->receive_data.state == 0)
		{
			DM_Motor_Enable(DM_6006_yaw);
		}

		DM_Motor_Start(DM_6006_yaw);
		if(DM_6006_yaw->receive_data.position > 0.131f || DM_6006_yaw->receive_data.position < -0.07f)
		{
			//yaw电机角度不在0点附近，控制yaw目标让角度正确，不过在底盘跟随状态下yaw电机角度一般都是允许缩头的
			if(DM_6006_yaw->receive_data.position > 0.05f)
			{
				target_angle_yaw -= k1*0.1f;
			}
			else if(DM_6006_yaw->receive_data.position < -0.02f)
			{
				target_angle_yaw -= k1*(-0.1f);
			}
		}
		DM_Motor_SetTar(DM_6006_yaw, target_angle_yaw);


	}
	else
	{
		Gimbal_Stop();
		Gimbal_Disable();

		DM_6006_yaw->motor_controller.speed_PID->i_term = 0;
		DM_6006_yaw->motor_controller.angle_PID->i_term = 0;
		DM_6006_yaw->motor_controller.speed_PID->output = 0;
		DM_6006_yaw->motor_controller.angle_PID->output = 0;

		DM_6220_pitch_head->motor_controller.speed_PID->i_term = 0;
		DM_6220_pitch_head->motor_controller.angle_PID->i_term = 0;
		DM_6220_pitch_head->motor_controller.speed_PID->output = 0;
		DM_6220_pitch_head->motor_controller.angle_PID->output = 0;

		DM_4310_pitch_neck->motor_controller.speed_PID->i_term = 0;
		DM_4310_pitch_neck->motor_controller.angle_PID->i_term = 0;
		DM_4310_pitch_neck->motor_controller.speed_PID->output = 0;
		DM_4310_pitch_neck->motor_controller.angle_PID->output = 0;

		target_angle_yaw = INS.Yaw;
		DM_Motor_SetTar(DM_6006_yaw, target_angle_yaw);
	}

	if((DM_4310_pitch_neck->receive_data.position < 0.2f&&DM_4310_pitch_neck->receive_data.position > -0.23f )&&
		(DM_6220_pitch_head->receive_data.position < (GIMBAL_LIMIT_MAX+0.03f)||DM_6220_pitch_head->receive_data.position > (GIMBAL_LIMIT_MIN-0.03f)))
	{
		gimbal_cmd.Neckflag = true;//脖子必须抬起才能小陀螺和底盘跟随
	}
	else
	{
		gimbal_cmd.Neckflag = false;
	}

	if (DM_6006_yaw->receive_data.position < 0.131f && DM_6006_yaw->receive_data.position > -0.07f) 
	{
		gimbal_cmd.Yawflag = true;//yaw电机角度必须在0点附近才能缩脖子
	}
	else
	{
		gimbal_cmd.Yawflag = false;
	}


	if(DM_6006_yaw->error_code&MOTOR_LOST_ERROR||DM_6220_pitch_head->error_code&MOTOR_LOST_ERROR||
		DM_4310_pitch_neck->error_code&MOTOR_LOST_ERROR)
	{
		DM_6006_yaw->motor_controller.speed_PID->i_term = 0;
		DM_6006_yaw->motor_controller.angle_PID->i_term = 0;
		DM_6006_yaw->motor_controller.speed_PID->output = 0;
		DM_6006_yaw->motor_controller.angle_PID->output = 0;

		DM_6220_pitch_head->motor_controller.speed_PID->i_term = 0;
		DM_6220_pitch_head->motor_controller.angle_PID->i_term = 0;
		DM_6220_pitch_head->motor_controller.speed_PID->output = 0;
		DM_6220_pitch_head->motor_controller.angle_PID->output = 0;

		DM_4310_pitch_neck->motor_controller.speed_PID->i_term = 0;
		DM_4310_pitch_neck->motor_controller.angle_PID->i_term = 0;
		DM_4310_pitch_neck->motor_controller.speed_PID->output = 0;
		DM_4310_pitch_neck->motor_controller.angle_PID->output = 0;
		
	}

	// 全部电机计算
	DM_Motor_Control(NULL);

	
}







