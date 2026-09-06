#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "BlueSerial.h"
#include "NRF24L01.h"
#include "PID.h"
#include "DataBuffer.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "MyTask.h"

/*事件标志组*/
#define LedState		(1<<0)		//Bit0：控制Led用于指示运行状态，1=运行，0=停止

EventBits_t EventBits;
EventGroupHandle_t EventGroup_Handle;

/*测试任务配置*/
#define Test_Priority		1
#define Test_StackDepth		128

TaskHandle_t Test_Handle;

/*按键任务配置*/
#define Key_Priority		2
#define Key_StackDepth		128

TaskHandle_t Key_Handle;

/*Led任务配置*/
#define Led_Priority		1
#define Led_StackDepth		128

TaskHandle_t Led_Handle;

/*OLED任务配置*/
#define OLED_Priority		1
#define OLED_StackDepth		256

TaskHandle_t OLED_Handle;

/*串口任务配置*/
#define UART_Priority		1
#define UART_StackDepth		256

TaskHandle_t UART_Handle;

/*蓝牙任务配置*/
#define BlueSerial_Priority			3	
#define BlueSerial_StackDepth		256

TaskHandle_t BlueSerial_Handle;

/*NRF24L01任务配置*/
#define NRF24L01_Priority		3
#define NRF24L01_StackDepth		128

TaskHandle_t NRF24L01_Handle;

/*变量定义与声明*/
uint8_t SetDataChang_Flag;		//0:数据未变化，1：数据变化

extern PID_SetData_t PID_SetDataBuf[2];
extern PID_GetData_t PID_GetDataBuf[2];

extern uint8_t RunFlag;
extern uint8_t SetBackBuf;

extern volatile uint8_t SetActiveBuf;
extern volatile uint8_t GetBackBuf;

extern PID_t AnglePID,SpeedPID,YawPID;

extern int16_t AvePWM;

void MyTask_Init(void)
{
	OLED_Init();
	LED_Init();
	Key_Init();
	Serial_Init();
	BlueSerial_Init();
	NRF24L01_Init();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	EventGroup_Handle=xEventGroupCreate();

	xTaskCreate(TasK_Test,"TestTask",Test_StackDepth,NULL,Test_Priority,&Test_Handle);

	
	xTaskCreate(Task_Key_Get,"KeyTask",Key_StackDepth,NULL,Key_Priority,&Key_Handle);
	xTaskCreate(Task_Led_Display,"LedTask",Led_StackDepth,NULL,Led_Priority,&Led_Handle);
	xTaskCreate(Task_OLED_Display,"OLEDTask",OLED_StackDepth,NULL,OLED_Priority,&OLED_Handle);
	xTaskCreate(Task_UART_Service,"UARTTask",UART_StackDepth,NULL,UART_Priority,&UART_Handle);
	xTaskCreate(Task_BlueSerial_Service,"BuleSerialTask",BlueSerial_StackDepth,NULL,BlueSerial_Priority,&BlueSerial_Handle);
	xTaskCreate(Task_NRF24L01_Service,"NRF24L01Task",NRF24L01_StackDepth,NULL,NRF24L01_Priority,&NRF24L01_Handle);
}

void TasK_Test(void *pvParemeters)
{
	UBaseType_t Key_MinStack,Led_MinStack,OLED_MinStack,UART_MinStack,BlueSerial_MinStack,NRF24L01_MinStack,Test_MinStack;	
	
	while(1)
	{	
		Test_MinStack=uxTaskGetStackHighWaterMark(NULL);
		Key_MinStack=uxTaskGetStackHighWaterMark(Key_Handle);
		Led_MinStack=uxTaskGetStackHighWaterMark(Led_Handle);
		OLED_MinStack=uxTaskGetStackHighWaterMark(OLED_Handle);
		UART_MinStack=uxTaskGetStackHighWaterMark(UART_Handle);
		BlueSerial_MinStack=uxTaskGetStackHighWaterMark(BlueSerial_Handle);
		NRF24L01_MinStack=uxTaskGetStackHighWaterMark(NRF24L01_Handle);
		
		vTaskSuspendAll();	//临界区保护
		
		Serial_Printf("-----------------------------------\n");		//数据分隔
		
		Serial_Printf("Test任务栈最小值:%d\n",Test_MinStack);
		Serial_Printf("Key任务栈最小值:%d\n",Key_MinStack);
		Serial_Printf("Led任务栈最小值:%d\n",Led_MinStack);
		Serial_Printf("OLED任务栈最小值:%d\n",OLED_MinStack);
		Serial_Printf("UART任务栈最小值:%d\n",UART_MinStack);
		Serial_Printf("BlueSerial任务栈最小值:%d\n",BlueSerial_MinStack);
		Serial_Printf("NRF24L01任务栈最小值:%d\n",NRF24L01_MinStack);
		
		Serial_Printf("-----------------------------------\n");		//数据分隔
		
		xTaskResumeAll();	//退出临界区
		
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}

void Task_Key_Get(void *pvParameters)
{
	static uint8_t Count;
	static uint8_t CurrState, PrevState;
	
	uint8_t Key_Num=0;
	
	while(1)
	{
		Count ++;
		if (Count >= 2)
		{
			Count = 0;
			
			PrevState = CurrState;
			CurrState = Key_GetState();
			
			if (CurrState == 0 && PrevState != 0)
			{
				Key_Num = PrevState;
			}
			else
			{
				Key_Num=0;
			}
		}
			
		if(Key_Num==1)
		{
			Key_Num=0;
			
			RunFlag=!RunFlag;
			
			if(RunFlag==1)
			{
				PID_Init(&AnglePID);
				PID_Init(&SpeedPID);
				PID_Init(&YawPID);			
			}	
		}
		
		if(RunFlag==1)
		{	
			xEventGroupSetBits(EventGroup_Handle,LedState);
		}
		else
		{		
			xEventGroupClearBits(EventGroup_Handle,LedState);
		}
		
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void Task_Led_Display(void *pvParameters)
{
	while(1)
	{	
		EventBits=xEventGroupWaitBits(EventGroup_Handle,LedState,pdFALSE,pdFALSE,pdMS_TO_TICKS(100));
		
		if(EventBits&LedState)
		{
			LED_ON();
		}
		else
		{
			LED_OFF();
		}
	}
}

void Task_OLED_Display(void *pvParameters)
{	
	uint8_t Local_GetBackBuf=0;
	
	while(1)
	{		
		Local_GetBackBuf=GetBackBuf;
		
		OLED_Printf(0,0,OLED_6X8,"AX=%+05d",PID_GetDataBuf[Local_GetBackBuf].AX);
		OLED_Printf(0,8,OLED_6X8,"AY=%+05d",PID_GetDataBuf[Local_GetBackBuf].AY);
		OLED_Printf(0,16,OLED_6X8,"AZ=%+05d",PID_GetDataBuf[Local_GetBackBuf].AZ);
		
		OLED_Printf(24,56,OLED_6X8,"Agl=+%05.2f",PID_GetDataBuf[Local_GetBackBuf].Angle);
		
		OLED_Printf(30,32,OLED_6X8,"LS=%+04.2f",PID_GetDataBuf[Local_GetBackBuf].LSpeed);
		OLED_Printf(30,40,OLED_6X8,"RS=%+04.2f",PID_GetDataBuf[Local_GetBackBuf].RSpeed);
		
		OLED_Printf(60,0,OLED_6X8,"GX=%+05d",PID_GetDataBuf[Local_GetBackBuf].GX);
		OLED_Printf(60,8,OLED_6X8,"GY=%+05d",PID_GetDataBuf[Local_GetBackBuf].GY);
		OLED_Printf(60,16,OLED_6X8,"GZ=%+05d",PID_GetDataBuf[Local_GetBackBuf].GZ);
		
		OLED_Update();
	}
}

void Task_UART_Service(void *pvParameters)
{	
	uint8_t Local_GetBackBuf=0;
	
	while(1)
	{			
		Local_GetBackBuf=GetBackBuf;
		
		vTaskSuspendAll();	//临界区保护
		
		Serial_Printf("%f,%f,%f,\n",PID_GetDataBuf[Local_GetBackBuf].AngleAcc,
						PID_GetDataBuf[Local_GetBackBuf].AngleGyro,
						PID_GetDataBuf[Local_GetBackBuf].Angle);

		xTaskResumeAll();	//退出临界区
	}
}

void Task_BlueSerial_Service(void *pvParameters)
{	
	uint8_t Local_GetBackBuf=0;
	
	while(1)
	{			
		Local_GetBackBuf=GetBackBuf;
		
		if (BlueSerial_RxFlag == 1&&SetDataChang_Flag==0)
		{	
			SetDataChang_Flag=1;		//数据变化

			char *Tag = strtok(BlueSerial_RxPacket, ",");
			if (strcmp(Tag, "key") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Action = strtok(NULL, ",");
				
				if (strcmp(Name, "1") == 0 && strcmp(Action, "up") == 0)
				{
					vTaskSuspendAll();	//临界区保护

					RunFlag=!RunFlag;
					
					xTaskResumeAll();	//退出临界区
					
					if(RunFlag==1)
					{
						xEventGroupSetBits(EventGroup_Handle,LedState);
						
						PID_Init(&AnglePID);
						PID_Init(&SpeedPID);
						PID_Init(&YawPID);
					}
					else
					{
						xEventGroupClearBits(EventGroup_Handle,LedState);
					}
				}
			}
			else if (strcmp(Tag, "slider") == 0)
			{	
				char *Name = strtok(NULL, ",");
				char *Value = strtok(NULL, ",");
				
				if (strcmp(Name, "AngleKp") == 0)
				{					
					PID_SetDataBuf[SetBackBuf].AKp = atof(Value);
				}
				else if (strcmp(Name, "AngleKi") == 0)
				{
					PID_SetDataBuf[SetBackBuf].AKi = atof(Value);
				}
				else if (strcmp(Name, "AngleKd") == 0)
				{					
					PID_SetDataBuf[SetBackBuf].AKd = atof(Value);					
				}
				else if (strcmp(Name, "SpeedKp") == 0)
				{
					PID_SetDataBuf[SetBackBuf].SKp = atof(Value);
				}
				else if (strcmp(Name, "SpeedKi") == 0)
				{					
					PID_SetDataBuf[SetBackBuf].SKi = atof(Value);	
				}
				else if (strcmp(Name, "SpeedKd") == 0)
				{					
					PID_SetDataBuf[SetBackBuf].SKd = atof(Value);
				}
				else if (strcmp(Name, "YawKp") == 0)
				{
					PID_SetDataBuf[SetBackBuf].YKp = atof(Value);					
				}
				else if (strcmp(Name, "YawKi") == 0)
				{
					PID_SetDataBuf[SetBackBuf].YKi = atof(Value);	
				}
				else if (strcmp(Name, "YawKd") == 0)
				{
					PID_SetDataBuf[SetBackBuf].YKd = atof(Value);
				}
			}	
			else if (strcmp(Tag, "joystick") == 0)
			{		
				int8_t LH = atoi(strtok(NULL, ","));
				int8_t LV = atoi(strtok(NULL, ","));
				int8_t RH = atoi(strtok(NULL, ","));
				int8_t RV = atoi(strtok(NULL, ","));
				
				vTaskSuspendAll();	//临界区保护
				
				PID_SetDataBuf[SetBackBuf].SpeedTarget=LV/25.0;		//限制小车平均速度在-4Rounds/S~4Rounds/S之间变化
				PID_SetDataBuf[SetBackBuf].YawTarget=RH/25.0;				//限制小车的差分速度在-4Rounds/S~4Rounds/S之间变化
				
				xTaskResumeAll();	//退出临界区
			}
			BlueSerial_RxFlag = 0;
		}

		vTaskSuspendAll();	//临界区保护				

		if(SetDataChang_Flag==1)
		{
			SetActiveBuf=SetBackBuf;
			SetBackBuf=!SetBackBuf;
			
			SetDataChang_Flag=0;
		}
		
		xTaskResumeAll();	//退出临界区
		
		BlueSerial_Printf("[plot,%f,%f,%f]",PID_GetDataBuf[Local_GetBackBuf].AngleAcc,
							PID_GetDataBuf[Local_GetBackBuf].AngleGyro,
							PID_GetDataBuf[Local_GetBackBuf].Angle);
		
		vTaskDelay(pdMS_TO_TICKS(60));
	}
}

void Task_NRF24L01_Service(void *pvParameters)
{	
	uint8_t Local_GetBackBuf;
	
	while(1)
	{		
		Local_GetBackBuf=GetBackBuf;
		
		if (NRF24L01_Receive() == 1&SetDataChang_Flag==0)
		{
			uint8_t ID = NRF24L01_RxPacket[0];
			
			SetDataChang_Flag=1;
			
			if (ID == 0x00 || ID == 0x01)
			{
				if (ID == 0x01)
				{	
					NRF24L01_TxPacket[0] = 0x02;
					NRF24L01_TxPacket[1] = (int8_t)PID_GetDataBuf[Local_GetBackBuf].LPWM;
					NRF24L01_TxPacket[2] = (int8_t)PID_GetDataBuf[Local_GetBackBuf].RPWM;
					*(float *)&NRF24L01_TxPacket[4] = PID_GetDataBuf[Local_GetBackBuf].Angle;		
					*(float *)&NRF24L01_TxPacket[8] = PID_GetDataBuf[Local_GetBackBuf].LSpeed;		
					*(float *)&NRF24L01_TxPacket[12] = PID_GetDataBuf[Local_GetBackBuf].RSpeed;
					
					NRF24L01_Send();
				}
				
//				int8_t LH = NRF24L01_RxPacket[1];
				int8_t LV = NRF24L01_RxPacket[2];
				int8_t RH = NRF24L01_RxPacket[3];
//				int8_t RV = NRF24L01_RxPacket[4];
				uint8_t KEY = NRF24L01_RxPacket[5];
				
				vTaskSuspendAll();	//临界区保护
				
				PID_SetDataBuf[SetBackBuf].SpeedTarget = LV / 25.0;
				PID_SetDataBuf[SetBackBuf].YawTarget = RH / 25.0;				

				if(SetDataChang_Flag==1)
				{
					SetActiveBuf=SetBackBuf;
					SetBackBuf=!SetBackBuf;
					
					SetDataChang_Flag=0;
				}
				
				xTaskResumeAll();	//退出临界区
				
				if (KEY == 1)
				{	
					
					vTaskSuspendAll();	//临界区保护
					
					RunFlag=!RunFlag;
					
					xTaskResumeAll();	//临界区保护
					
					if(RunFlag==1)
					{
						PID_Init(&AnglePID);
						PID_Init(&SpeedPID);
						PID_Init(&YawPID);					
					}
				}
			}
		}
		
		if(RunFlag==1)
		{		
			xEventGroupSetBits(EventGroup_Handle,LedState);
		}
		else
		{
			xEventGroupClearBits(EventGroup_Handle,LedState);
		}
		
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
