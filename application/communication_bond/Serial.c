#include "serial.h"
#include "bsp_usart.h"
#include "defense_center.h"
#include "usart.h"
#include <string.h>
#include "usbd_cdc_if.h"

/*天大开源*/
input_data_t auto_send_data;//电控发送给自瞄系统的数据
state_bytes_t state_bytes;//电控发送给自瞄系统的数据帧(结构体)

output_data_t auto_receive_data;//接收上位机的数据
output_bytes_t nx_auto_datas;//自瞄系统返回给电控的数据帧(结构体)

Frame_header_t header;
frame_tailer_t tailer;
//#define AUTO_RXBUFF_SIZE 60
uint8_t auto_shoot_rx_buf[sizeof(output_bytes_t)]; // 接收缓冲区

/*适配同济开源*/
vs_receive_packet_t vs_aim_packet_from_nuc;
vs_send_packet_t vs_aim_packet_to_nuc;

// int CDC_SendFeed(uint8_t *Fed, uint16_t Len)
// {
//     CDC_Transmit_HS(Fed, Len);
//     return 0;
// }

static uint8_t vs_init_flag = 0;
USART_instance_t *vs_usart_instance;
static supervisor_t *vs_supervisor_instance; 
uint8_t frame_buf[vs_rx_len*2];

uint8_t frq_vs=0;
static void Vs_Rx_Callback(void)   //接收到数据后的中断回调处理
{
	Supervisor_Reload(vs_supervisor_instance); //喂狗
    #if(Tongji == 1) 
	{
        memcpy(frame_buf, vs_usart_instance->recv_buff, sizeof(vs_receive_packet_t)); 
        VS_UnPack_Data_ROS2(frame_buf, &vs_aim_packet_from_nuc, sizeof(vs_receive_packet_t));
	}
	#elif(TianJin == 1)
	{
        memcpy(frame_buf, vs_usart_instance->recv_buff, sizeof(output_bytes_t));
        NV_UnPack_Data_ROS2(frame_buf, &nx_auto_datas, sizeof(output_bytes_t));
	}
	#endif
} 

static void Vs_Lost_Callback(void) //视觉掉线未正常工作则重启
{
    #if(Tongji == 1) 
	{
        memset(&vs_aim_packet_from_nuc,0,sizeof(vs_aim_packet_from_nuc));
	}
	#elif(TianJin == 1)
	{
        memset(&nx_auto_datas,0,sizeof(nx_auto_datas));
	}
	#endif
}

void VS_Init(UART_HandleTypeDef *vs_usart_handle) //视觉串口注册函数
{
	usart_init_config_t conf;
	conf.module_callback = Vs_Rx_Callback;
	conf.usart_handle    = vs_usart_handle;
	conf.recv_buff_size  = vs_rx_len;
	vs_usart_instance   = USART_Register(&conf);


	// 进行守护进程的注册,用于定时检查裁判系统是否正常工作
//	supervisor_init_config_t supervisor_conf = 
//	{
//		.reload_count = 1000, // 1000ms未收到数据视为离线
//		.handler_callback = Referee_Lost_Callback,
//		.owner_id = NULL, 
//	};
//	referee_supervisor_instance = Supervisor_Register(&supervisor_conf);

	vs_init_flag = 1;
}

void Serial_Init(void)
{
   //开启DMA接收
//    #if (NORMAL_UART == 1)
//    {
// 		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, auto_shoot_rx_buf, sizeof(output_bytes_t));
// 		// __HAL_DMA_DISABLE_IT(&hdma_uart7_rx,DMA_IT_HT);
//    }
//    #endif
   header.sof = 0xA6; // 帧头
   header.crc8 = 0 ;// CRC8校验，暂时不计算

   auto_send_data.state = 0; //默认状态
   auto_send_data.autoaim = 1;
   auto_send_data.enemy_color = 0; //0 --- 红色

}

//特定数据打包发送
void NX_Pack_And_Send_Data(input_data_t *send_data) //天津开源数据打包发送函数
{
   uint16_t len = sizeof(state_bytes_t) + 1;
   static uint8_t temp_data[sizeof(state_bytes_t) + 1]; //static：防止CDC异步DMA读取悬空栈指针

   state_bytes.frame_header = header;
   state_bytes.input_data = *send_data;
   state_bytes.frame_tailer = tailer;

   memcpy(temp_data, &state_bytes, len - 3);
   uint16_t w_crc = Get_CRC16_Check_Sum(temp_data, len - 3, 0xFFFF);

   temp_data[len - 3] = (uint8_t)(w_crc & 0x00ff);
   temp_data[len - 2] = (uint8_t)((w_crc >> 8) & 0x00ff);
   temp_data[len - 1] = '\n'; //定于帧尾为换行符
   #if(NORMAL_UART == 1)
   {
		// HAL_UART_Transmit(&huart7, temp_data, len, HAL_MAX_DELAY);
        HAL_UART_Transmit_DMA(&huart7, temp_data, len);
   }
   #elif(CDC_UART == 1)
   {
        CDC_Transmit_HS(temp_data,len);
   }
   #endif

}


void NV_UnPack_Data_ROS2(uint8_t *receive_buf, output_bytes_t *receive_packet, uint16_t Len) //天津开源数据解包函数
{
   if (receive_buf[0] == 0xA6)
   {
       uint16_t w_expected = Get_CRC16_Check_Sum(receive_buf, Len - 2, 0xFFFF);
       if ((w_expected & 0xff) == receive_buf[Len - 2] && ((w_expected >> 8) & 0xff) == receive_buf[Len - 1])
    	{
           memcpy(receive_packet, receive_buf, Len);
      	}
   }
   memset(receive_buf, 0, Len);
}



//void Pack_And_Send_Data_ROS2(send_packet_t *send_packet)
//{
//    uint16_t len = sizeof(send_packet_t);
//    uint8_t tmp[sizeof(send_packet_t)];
//    memcpy(tmp, send_packet, len - 2);
//    uint16_t w_crc = Get_CRC16_Check_Sum(tmp, len - 2, 0xFFFF);

//    tmp[len - 2] = (uint8_t)(w_crc & 0x00ff);
//    tmp[len - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
//    CDC_SendFeed(tmp, len);
//}

void VS_Send_Packet_Init(vs_send_packet_t *send_packet)
{
    /*TJ*/
    send_packet->head[0] = 'S';
    send_packet->head[1] = 'P';
    send_packet->mode = 0;
    send_packet->q[0] = 0.0f;
    send_packet->q[1] = 0.0f;
    send_packet->q[2] = 0.0f;
    send_packet->q[3] = 0.0f;
    send_packet->yaw = 0.0f;
    send_packet->yaw_vel = 0.0f;
    send_packet->pitch = 0.0f;
    send_packet->pitch_vel = 0.0f;
    send_packet->bullet_speed = 0;
    send_packet->bullet_count = 0;

}

void VS_Receive_Packet_Init(vs_receive_packet_t *receive_packet)
{
    receive_packet->head[0] = 'S';
    receive_packet->head[1] = 'P';
    receive_packet->mode = 0;
    receive_packet->yaw = 0.0f;
    receive_packet->yaw_vel = 0.0f;
    receive_packet->yaw_acc = 0.0f;
    receive_packet->pitch = 0.0f;
    receive_packet->pitch_vel = 0.0f;
    receive_packet->pitch_acc = 0.0f;
    receive_packet->crc16 = 0;
}

// uint8_t tmp[43] ={0x53,0x50,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,
//                             0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,
//                             0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,
//                             0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,
//                             0x41,0xAA};
 
// uint8_t tmp[43] ={0x53,0x50,0x01,0x81,0xaa,0x19,0x3f,0x80,0x35,0xae,0x3a,0xea,
//                             0x14,0x25,0x3c,0x96,0x31,0x4c,0x3f,0xba,0x65,0xed,
//                             0x3f,0x00,0x00,0x00,0x00,0xbb,0xb1,0x91,0x3c,0x00,
//                             0x00,0x00,0x00,0x00,0x00,0xc0,0x41,0x00,0x00,
//                             0x00,0x00};
// uint8_t tmp[43];
void VS_Pack_And_Send_Data_ROS2(vs_send_packet_t *send_packet)  //同济开源数据打包发送函数
{
    uint16_t len = sizeof(vs_send_packet_t);
    static uint8_t tmp[sizeof(vs_send_packet_t)];

    memcpy(tmp, send_packet, len - 2);
    uint16_t w_crc = Get_CRC16_Check_Sum(tmp, len - 2, 0xFFFF);
    

    tmp[len - 2] = (uint8_t)(w_crc & 0x00ff);
    tmp[len - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);

    // USART_Send(vs_usart_instance, tmp,len,USART_TRANSFER_DMA);

   #if(NORMAL_UART == 1)
   {
        USART_Send(vs_usart_instance, tmp,len,USART_TRANSFER_DMA);
   }
   #elif(CDC_UART == 1)
   {
  	    CDC_Transmit_HS(tmp,len);
   }
   #endif
    
    // CDC_Transmit_HS(tmp,len);
    

}


void VS_UnPack_Data_ROS2(uint8_t *receive_buf, vs_receive_packet_t *receive_packet, uint16_t Len) //同济开源数据解包函数
{
    // /*LenΪԭʼ�������ݳ��ȣ����Ҫ�ų�ĩβ�Ļ��з�������Ҫ��ȥ���һλ���ݰ�*/
    // uint16_t actual_Len = Len - 1;
    if (receive_buf[0] == 'S' && receive_buf[1] == 'P')
    {
        uint16_t w_expected;
        w_expected = Get_CRC16_Check_Sum(receive_buf, Len - 2, 0xFFFF);
        if ((w_expected & 0xff) == receive_buf[Len - 2] && ((w_expected >> 8) & 0xff) == receive_buf[Len - 1]) // CRC�����������
        {
            memcpy(receive_packet, receive_buf, Len);
        }

        frq_vs++;

    }
    memset(receive_buf, 0, Len);
}



