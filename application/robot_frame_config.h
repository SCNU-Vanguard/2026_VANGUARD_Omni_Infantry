/**
* @file robot_frame_config.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __ROBOT_FRAME_CONFIG_H__
#define __ROBOT_FRAME_CONFIG_H__

/*常用函数定义*/
#define USER_LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))






/*归一化参数*/
#define RAD_2_DEGREE 57.2957795f    //rad/s to °/s 180/pi
#define DEGREE_2_RAD 0.01745329252f //°/s to rad/s pi/180

#define RPM_2_ANGLE_PER_SEC 6.0f       //rpm to °/s ×360°/60sec
#define RPM_2_RAD_PER_SEC 0.104719755f //rpm to rad/s ×2pi/60sec    

#define BMI088_Frame 1

/*底盘参数*/
#define M3508_REDUCTION_RATIO (3591.0f/187.0f) //M3508减速比
#define CHASSIS_RADIUS 0.25f  //底盘半径（全向轮到底盘中心的距离）
#define WHEEL_RADIUS 0.077f	//驱动轮半径（diameter）

#define CHASSIS_FOLLOW_ANGLE 0.0f  //底盘跟随电机编码器偏置

/*云台参数*/
#define GIMBAL_LIMIT_MAX  0.11f
#define GIMBAL_LIMIT_MIN -0.35f //-0.17f

    
/*遥控器参数*/
#define REMOTE_X_SEN 0.005f   //660 ~ -660 0.009f
#define REMOTE_Y_SEN 0.005f
#define REMOTE_OMEGA_Z_SEN 0.01f  //6.6

#define REMOTE_YAW_SEN 0.000006f
#define REMOTE_HEAD_SEN 0.000002f

/*键鼠参数*/
#define KETBOARD_X_SEN 0.02f   
#define KETBOARD_Y_SEN 0.02f
#define KETBOARD_OMEGA_Z_SEN 0.01f  

#define KEYBOARD_YAW_SEN 0.00003f
#define KEYBOARD_HEAD_SEN 0.00001f


/*速度限幅*/
#define CHASSIS_SPEED_MAX 3.3f
#define CHASSIS_OMEGA_Z_MAX 4.0f//3.3f
#define CHASSIS_SPIN_WOBBLE_AMP 0.5f //变速小陀螺叠加的正弦波幅值
#define CHASSIS_SPIN_WOBBLE_FREQ 0.5f   //变速小陀螺叠加的正弦波频率

#endif /* __ROBOT_FRAME_CONFIG_H__ */
