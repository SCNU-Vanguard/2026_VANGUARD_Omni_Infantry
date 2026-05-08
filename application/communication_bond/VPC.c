/*
 * VPC.c
 * author: miracle-cloud
 * Created on: 2025��10��31��
 */

#include "VPC.h"
#include "Serial.h"
#include "INS.h"
#include "gimbal.h"

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "semphr.h"
#include "usbd_cdc_if.h"

extern Frame_header_t header;

void VPC_Init(void)
{
  #if(Tongji == 1) 
	{
    //NV_Send_Packet_Init(&nv_aim_packet_to_nuc);
    VS_Send_Packet_Init(&vs_aim_packet_to_nuc);
    VS_Receive_Packet_Init(&vs_aim_packet_from_nuc);
    // Send_Packet_Init(&aim_packet_to_nuc);

	}
	#elif(TianJin == 1)
	{
    header.sof = 0xA6; // 帧头
    header.crc8 = 0 ;// CRC8校验，暂时不计算

    auto_send_data.state = 0; //默认状态
    auto_send_data.autoaim = 1;
    auto_send_data.enemy_color = 0; //0 --- 红色
	}
	#endif
}

void VPC_UpdatePackets(void)
{
  #if(Tongji == 1) 
	{
    vs_aim_packet_to_nuc.head[0] = 'S';
    vs_aim_packet_to_nuc.head[1] = 'P';
    vs_aim_packet_to_nuc.mode = 1;
    vs_aim_packet_to_nuc.q[0] = INS.q[0];
    vs_aim_packet_to_nuc.q[1] = INS.q[1];
    vs_aim_packet_to_nuc.q[2] = INS.q[2];
    vs_aim_packet_to_nuc.q[3] = INS.q[3];
    vs_aim_packet_to_nuc.yaw = INS.Yaw;
    vs_aim_packet_to_nuc.yaw_vel = 0; 

    vs_aim_packet_to_nuc.pitch = INS.Pitch;
    vs_aim_packet_to_nuc.pitch_vel = 0;
    vs_aim_packet_to_nuc.bullet_speed = 22.9f; 
    vs_aim_packet_to_nuc.bullet_count = 0;  

	}
	#elif(TianJin == 1)
	{
    auto_send_data.curr_yaw = INS.Yaw;
		auto_send_data.curr_pitch = INS.Pitch;
		auto_send_data.curr_roll = INS.Roll;

	}
	#endif


}






