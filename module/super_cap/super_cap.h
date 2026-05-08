#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "main.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "bsp_can.h"
#include "defense_center.h"

#define SUPER_CAP_CNT 1

typedef union
{
	struct
	{
        uint8_t enableDCDC      : 1;   // 置1使能DCDC
        uint8_t systemRestart   : 1;   // 置1系统重启
        uint8_t clearError      : 1;   // 置1手动清除可清除的错误
        uint8_t reserved        : 5;   // 保留位
	};
	uint8_t commands;
} SPC_Ctrl_Command_t;

typedef struct
{
    SPC_Ctrl_Command_t command;     // 控制命令
    uint16_t refereePowerLimit;     // 裁判系统限制的最大功率，单位W
    uint16_t refereeEnergyBuffer;   // 裁判系统限制的能量缓冲区，单位J
    uint8_t reserved[3];            // 保留字节
}__attribute__((packed)) Super_Capacitor_sendmessage_t;

typedef struct
{
    uint8_t statusCode; // 状态信息 0:无错误 1:可自动恢复错误 2:可发送消息恢复错误 3:不可恢复错误   只有低两位有效
    float chassisPower; // 底盘功率，单位W
    uint16_t chassisPowerLimit; // 底盘最大可用功率 （包括裁判系统）
    uint8_t capEnergy;  // 电容现有能量，0-255
}__attribute__((packed)) Super_Capacitor_callback_t; //ID 0x051

typedef struct
{
    bool LostFlag;

    can_init_config_t can_init_config;
    CAN_instance_t *super_capacitor_can_instance;

    Super_Capacitor_sendmessage_t transmit_data; // 发送数据结构体
    Super_Capacitor_callback_t receive_data;     // 接收数据结构体

	supervisor_t *supervisor;

	uint32_t feed_cnt;
	float dt;

    uint8_t error_cnt;
    uint8_t error_pro_cnt;

}Super_Capacitor_t;


Super_Capacitor_t *Super_Capacitor_Init(Super_Capacitor_t *config);
void Super_Capacitor_Task(void);


#endif

