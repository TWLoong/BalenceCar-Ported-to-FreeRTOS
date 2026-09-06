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

PID_t AnglePID={
	.Kp=3,
	.Ki=0.1,
	.Kd=3,
	
	.Target=0,
	
	.ErrorIntMax=500,
	.ErrorIntMin=-500,
	.OutPut_Offset=4,	//左侧电机死区为4，右侧死区为5
	
	.OutMax=600,		//限制直立环输出平均PWM最大100
	.OutMin=-600,		//限制直立环输出平均PWM最小-100
};

PID_t SpeedPID={
	.Kp=2,
	.Ki=0.05,
	.Kd=0,
	
	.Target=0,
	
	.ErrorIntMax=150,
	.ErrorIntMin=-150,
	
	.OutMax=20,		//限制速度环输出最大20°
	.OutMin=-20,	//限制速度环输出最小-20°
};

PID_t YawPID={
	.Kp=4,
	.Ki=3,
	.Kd=0,
	
	.Target=0,
	
	.ErrorIntMax=+20,
	.ErrorIntMin=-20,
	
	.OutMax=50,		//限转向环输出差分PWM最大为50
	.OutMin=-50,	//限制速度环输出差分PWM最小-50
};

PID_SetData_t PID_SetDataBuf[2];
PID_GetData_t PID_GetDataBuf[2];

volatile uint8_t SetActiveBuf;
uint8_t SetBackBuf=1;

uint8_t GetActiveBuf;
volatile uint8_t GetBackBuf=1;

uint8_t RunFlag;

uint8_t GetDataChang_Flag;		//0:数据未变化，1：数据变化

int16_t AvePWM,DifPWM,LPWM,RPWM;

float AveSpeed,DifSpeed;

extern EventGroupHandle_t EventGroup_Handle;

int main(void)
{
	PID_SetDataBuf[SetBackBuf].AKp=AnglePID.Kp;
	PID_SetDataBuf[SetBackBuf].AKi=AnglePID.Ki;
	PID_SetDataBuf[SetBackBuf].AKd=AnglePID.Kd;
	
	PID_SetDataBuf[SetBackBuf].SKp=SpeedPID.Kp;
	PID_SetDataBuf[SetBackBuf].SKi=SpeedPID.Ki;
	PID_SetDataBuf[SetBackBuf].SKd=SpeedPID.Kd;
	
	PID_SetDataBuf[SetBackBuf].YKp=YawPID.Kp;
	PID_SetDataBuf[SetBackBuf].YKi=YawPID.Ki;
	PID_SetDataBuf[SetBackBuf].YKd=YawPID.Kd;

	PID_SetDataBuf[SetBackBuf].SpeedTarget=SpeedPID.Target;
	PID_SetDataBuf[SetBackBuf].YawTarget=YawPID.Target;
	
	PID_SetDataBuf[SetActiveBuf].AKp=AnglePID.Kp;
	PID_SetDataBuf[SetActiveBuf].AKi=AnglePID.Ki;
	PID_SetDataBuf[SetActiveBuf].AKd=AnglePID.Kd;
	                  
	PID_SetDataBuf[SetActiveBuf].SKp=SpeedPID.Kp;
	PID_SetDataBuf[SetActiveBuf].SKi=SpeedPID.Ki;
	PID_SetDataBuf[SetActiveBuf].SKd=SpeedPID.Kd;
	                  
	PID_SetDataBuf[SetActiveBuf].YKp=YawPID.Kp;
	PID_SetDataBuf[SetActiveBuf].YKi=YawPID.Ki;
	PID_SetDataBuf[SetActiveBuf].YKd=YawPID.Kd;
		
	Timer1_Init();
	Encoder_Init();
	Motor_Init();
	MPU6050_Init();

	MyTask_Init();
	xPortStartScheduler();
	
	while(1)
	{
		
	}
}

void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count0,Count1,Count2;
	static float Alpha=0.01;
		
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{	
		Delay_Tick();		//定时器资源紧张，仅MPU6050初始化使用Delay，不追求精确性，故以TIM1作为节拍

		
		/*--------------------------直立环--------------------------*/	
		Count0++;
		
		if(Count0>=10)
		{
			Count0=0;
			
			GetDataChang_Flag=1;
			
			MPU6050_GetData(&PID_GetDataBuf[GetActiveBuf].AX,&PID_GetDataBuf[GetActiveBuf].AY,&PID_GetDataBuf[GetActiveBuf].AZ,
							&PID_GetDataBuf[GetActiveBuf].GX,&PID_GetDataBuf[GetActiveBuf].GY,&PID_GetDataBuf[GetActiveBuf].GZ);
			
			PID_GetDataBuf[GetActiveBuf].AX-=20;		//零漂补偿
			PID_GetDataBuf[GetActiveBuf].AngleAcc=-atan2(PID_GetDataBuf[GetActiveBuf].AX,PID_GetDataBuf[GetActiveBuf].AZ)/3.14159*180;
			
			PID_GetDataBuf[GetActiveBuf].GY-=25;		//零漂补偿
			PID_GetDataBuf[GetActiveBuf].AngleGyro=PID_GetDataBuf[GetActiveBuf].Angle+PID_GetDataBuf[GetActiveBuf].GY/32768.0*2000*0.01;
			
			PID_GetDataBuf[GetActiveBuf].Angle= Alpha*PID_GetDataBuf[GetActiveBuf].AngleAcc+(1-Alpha)*PID_GetDataBuf[GetActiveBuf].AngleGyro;
			
			AnglePID.Actual=PID_GetDataBuf[GetActiveBuf].Angle;
			
			AnglePID.Kp=PID_SetDataBuf[SetActiveBuf].AKp;
			AnglePID.Ki=PID_SetDataBuf[SetActiveBuf].AKi;
			AnglePID.Kd=PID_SetDataBuf[SetActiveBuf].AKd;
			
			PID_Update(&AnglePID);
			
			AvePWM=-AnglePID.Out;

			/*--------------------------电机控制--------------------------*/		
			LPWM=AvePWM+DifPWM/2;
			RPWM=AvePWM-DifPWM/2;
			
			if (LPWM > 100) {LPWM = 100;} else if (LPWM < -100) {LPWM = -100;}
			if (RPWM > 100) {RPWM = 100;} else if (RPWM < -100) {RPWM = -100;}
			
			if(PID_GetDataBuf[GetActiveBuf].Angle>=50||PID_GetDataBuf[GetActiveBuf].Angle<=-50)
			{
				RunFlag=0;
			}
			
			if(RunFlag==1)
			{
				Motor_SetPWM(LPWM,1);
				Motor_SetPWM(RPWM,2);
			}
			else
			{
				Motor_SetPWM(0,1);
				Motor_SetPWM(0,2);
			}
		}

		/*--------------------------速度环--------------------------*/		
		Count1++;
		
		if(Count1>=50)
		{
			Count1=0;
			
			GetDataChang_Flag=1;
			
			PID_GetDataBuf[GetActiveBuf].LSpeed=Encoder_Get(1)/0.05/44.0/9.2766;		//单位：转/秒,最大转速为14.11转每秒	
			PID_GetDataBuf[GetActiveBuf].RSpeed=Encoder_Get(2)/0.05/44.0/9.2766;		//单位：转/秒,最大转速为14.11转每秒
			
			AveSpeed=(PID_GetDataBuf[GetActiveBuf].LSpeed+PID_GetDataBuf[GetActiveBuf].RSpeed)/2.0;		//最大为14.11转每秒，最小为0转每秒
			
			SpeedPID.Actual=AveSpeed;
			
			SpeedPID.Kp=PID_SetDataBuf[SetActiveBuf].SKp;
			SpeedPID.Ki=PID_SetDataBuf[SetActiveBuf].SKi;
			SpeedPID.Kd=PID_SetDataBuf[SetActiveBuf].SKd;
			SpeedPID.Target=PID_SetDataBuf[SetActiveBuf].SpeedTarget;
			
			PID_Update(&SpeedPID);
			
			AnglePID.Target=SpeedPID.Out;
		}
		
		/*--------------------------转向环--------------------------*/		
		Count2++;
		
		if(Count2>=50)
		{
			Count2=0;
			
			GetDataChang_Flag=1;
			
			DifSpeed=PID_GetDataBuf[GetActiveBuf].LSpeed-PID_GetDataBuf[GetActiveBuf].RSpeed;		//最大为25转每秒，最小为-25转每秒
			
			YawPID.Actual=DifSpeed;
			
			YawPID.Kp=PID_SetDataBuf[SetActiveBuf].YKp;
			YawPID.Ki=PID_SetDataBuf[SetActiveBuf].YKi;
			YawPID.Kd=PID_SetDataBuf[SetActiveBuf].YKd;
			YawPID.Target=PID_SetDataBuf[SetActiveBuf].YawTarget;
			
			PID_Update(&YawPID);
			
			DifPWM=YawPID.Out;
		}
		
		if(GetDataChang_Flag==1)
		{		
			GetBackBuf=GetActiveBuf;
			GetActiveBuf=!GetActiveBuf;
			
			GetDataChang_Flag=0;
		}
	
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
