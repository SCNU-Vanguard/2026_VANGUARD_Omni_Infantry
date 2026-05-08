/**
* @file robot_frame_init.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __ROBOT_FRAME_INIT_H__
#define __ROBOT_FRAME_INIT_H__

#include <stdint.h>

#include "remote_control.h"
#include "remote_vt03.h"

typedef enum
{
    robot_disable = 0,
    robot_stretch = 1, //伸头
    robot_shrink = 2,  //缩头
    robot_tunnel = 3,  //穿洞
}robot_state_e;

extern void Robot_Frame_Init(void);
extern RC_ctrl_t *rc_ctl; 
extern VT03_ctrl_t *rc_vt03;
extern robot_state_e robot_state;

/*

wsad

c --->伸头
v --->缩头
b --->失能
x --->隧道模式

shift --->小陀螺
ctrl  --->停止小陀螺

f --->开摩擦轮
g --->关摩擦轮

右键长按 --->开自瞄
//e --->关自瞄

r ??? --->开火控

z --> 刷新UI

*/



#endif /* __ROBOT_FRAME_INIT_H__ */
