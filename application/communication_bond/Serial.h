#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "main.h"
#include "usart.h"
#include "CRC.h"

//#include "remote.h"
#include "usbd_cdc_if.h"

/*使用普通串口/虚拟串口*/
#define NORMAL_UART 0
#define CDC_UART 1

/*适配同济开源*/
#define Tongji 1
#define TianJin 0

#if(Tongji == 1) 
   #define vs_rx_len 29
#elif(TianJin == 1)
   #define vs_rx_len 13
#endif


typedef struct {
   uint8_t     sof;
   uint8_t     crc8;
}Frame_header_t;

typedef struct {
   uint16_t    crc16;
}frame_tailer_t;

//无人机对NX
typedef struct {
   float       curr_yaw;
   float       curr_pitch;
   float       curr_roll;
   uint8_t     state;
   uint8_t     autoaim;
   uint8_t     enemy_color;
}input_data_t;

//
typedef struct {
   uint8_t     fire;
   float       shoot_yaw;
   float       shoot_pitch;
}output_data_t;

//15
typedef struct {                     //电控传给自瞄系统的云台数据
   Frame_header_t frame_header;//2
   input_data_t   input_data;//17
   frame_tailer_t frame_tailer; //19 
}state_bytes_t;

//
typedef struct  {                   // 自瞄返回给电控的控制数据
    Frame_header_t frame_header;//2
    output_data_t  output_data;//11
    frame_tailer_t frame_tailer;//13
}output_bytes_t;


/*适配同济开源*/
typedef struct {
    uint8_t head[2];//2
    uint8_t mode;//3
    float q[4];//19 WXYZ
    float yaw;//23
    float yaw_vel;//27
    float pitch;//31
    float pitch_vel;//35
    float bullet_speed;//39
    uint16_t bullet_count;//41
    uint16_t crc16;//43
}__attribute__((packed)) vs_send_packet_t;

typedef struct {
    uint8_t head[2];//2
    uint8_t mode;//3
    float yaw;//7
    float yaw_vel;//11
    float yaw_acc;//15
    float pitch;//19
    float pitch_vel;//23
    float pitch_acc;//27
    uint16_t crc16;//29
}__attribute__((packed)) vs_receive_packet_t;


/*外部变量声明*/
extern input_data_t auto_send_data;
extern output_data_t auto_receive_data;
extern output_bytes_t nx_auto_datas;
extern uint8_t auto_shoot_rx_buf[sizeof(output_bytes_t)];

extern vs_receive_packet_t vs_aim_packet_from_nuc;
extern vs_send_packet_t vs_aim_packet_to_nuc;

/*函数定义*/
void VS_Init(UART_HandleTypeDef *vs_usart_handle);
void Serial_Init(void);
void NX_Pack_And_Send_Data(input_data_t *send_data);
void NV_UnPack_Data_ROS2(uint8_t *receive_buf, output_bytes_t *receive_packet, uint16_t Len);

void VS_Pack_And_Send_Data_ROS2(vs_send_packet_t *send_packet);
void VS_Send_Packet_Init(vs_send_packet_t *send_packet);
void VS_Receive_Packet_Init(vs_receive_packet_t *receive_packet);
void VS_UnPack_Data_ROS2(uint8_t *receive_buf, vs_receive_packet_t *receive_packet, uint16_t Len);
#endif /* __SERIAL_H__ */




