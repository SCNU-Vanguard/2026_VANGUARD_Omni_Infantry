/**
******************************************************************************
 * @file    gimbal_task.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "gimbal_task.h"
#include "gimbal.h"

#include "message_center.h"

#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "rs485.h"
#include "defense_center.h"

#define GIMBAL_TASK_PERIOD 1 // ms

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

__weak void Gimbal_Publish(void);

__weak void Gimbal_Init(void);

__weak void Gimbal_Observer(void);

__weak void Gimbal_Handle_Exception(void);

__weak void Gimbal_Set_Mode(void);

__weak void Gimbal_Reference(void);

__weak void Gimbal_Console(void);

__weak void Gimbal_Send_Cmd(void);

osThreadId_t gimbal_task_handel;

static publisher_t *gimbal_publisher;
static subscriber_t *gimbal_subscriber;

static void Gimbal_Task(void *argument);

void Gimbal_Task_Init(void)
{
	const osThreadAttr_t attr = {
		.name = "Gimbal_Task",
		.stack_size = 128 * 8,
		.priority = (osPriority_t) osPriorityRealtime4,
	};
	gimbal_task_handel = osThreadNew(Gimbal_Task, NULL, &attr);

//	gimbal_publisher  = Publisher_Register("gimbal_transmit_feed", sizeof(gimbal_behaviour_t));
//	gimbal_subscriber = Subscriber_Register("gimbal_receive_cmd", sizeof(gimbal_cmd_t));
}

uint32_t gimbal_task_diff;




static void Gimbal_Task(void *argument)
{
	Gimbal_Publish( );

	uint32_t time = osKernelGetTickCount( );

	osDelay(2);

	for (; ;)
	{
		if(DT7_Is_Online())
		{
			//云台遥控器控制
			Gimbal_Control_Remote();
		}
		else
		{
			//云台键鼠控制
			Gimbal_Control_Keyboard();
		}

		Supervisor_Task();


		gimbal_task_diff = osKernelGetTickCount( ) - time;
		time             = osKernelGetTickCount( );
		osDelayUntil(time + GIMBAL_TASK_PERIOD);
		
#if INCLUDE_uxTaskGetStackHighWaterMark
		gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

__weak void Gimbal_Publish(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Gimbal_Init(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Gimbal_Observer(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Gimbal_Handle_Exception(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Gimbal_Set_Mode(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Gimbal_Reference(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Gimbal_Console(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Gimbal_Send_Cmd(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}
