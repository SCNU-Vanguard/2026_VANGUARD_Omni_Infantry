#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "DJI_motor.h"
#include "DM_motor.h"
#include "robot_frame_config.h"
#include "user_lib.h"

#include "super_cap.h"
#include "gimbal.h"
#include "remote_vt03.h"

typedef enum
{
    SPIN    = 1, //小陀螺
    FOLLOW  = 3,//底盘跟随
    STOP_C    = 2//不动
}chassis_mode_e;


typedef struct
{
    float vx;
    float vy;

    float omega_z;       //底盘小陀螺时的角速度(rad/s)
//    float omega_follow;   //底盘跟随时的角速度  (rad/s)
    chassis_mode_e mode;
}__attribute__((__packed__))Chassis_CmdTypedef;

//typedef struct
//{
//    float target;
//    float value;
//    float error;
//    float output;

//    float omega_z_ref;
//    PID_TypeDef pid[2];    //outer(or angle)(0)、inner(or speed)(1) circle
//}__attribute__((__packed__))Chassis_TypeDef;

////////////////////////超电部分
// typedef struct
// {
//     uint8_t statusCode; // 状态信息
//     float chassisPower; // 底盘功率，单位W
//     uint16_t chassisPowerLimit; // 底盘最大可用功率 （包括裁判系统）
//     uint8_t capEnergy;  // 电容现有能量，0-255
// }__attribute__((__packed__))Super_Capacitor_callback_t; //ID 0x051


/////////////////////////

/*外部参数*/
extern DJI_motor_instance_t *chassis_m3508[4];
extern Chassis_CmdTypedef chassis_cmd;
extern float target_speed[4];
extern Super_Capacitor_t *chassis_super_capacitor;


void Chassis_Init(void);
float Chassis_Get_Omega(void);
void Chassis_Ctrl_Remote(void);
void Chassis_Control_Keyboard(void);



#endif /* __CHASSIS_H__ */
