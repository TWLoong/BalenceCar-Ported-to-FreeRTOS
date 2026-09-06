#ifndef __DATABUFFER_H
#define __DATABUFFER_H

typedef struct{
	float AKp;
	float AKi;
	float AKd;
	
	float SKp;
	float SKi;
	float SKd;
	
	float YKp;
	float YKi;
	float YKd;
	
	float SpeedTarget;
	float YawTarget;
}PID_SetData_t;		//两级缓存用于调控PID的参数

typedef struct{
	int16_t AX;
	int16_t AY;
	int16_t AZ;
	
	int16_t GX;
	int16_t GY;
	int16_t GZ;
	
	float Angle;
	float AngleAcc;
	float AngleGyro;
	
	float LSpeed;
	float RSpeed;
	
	float LPWM;
	float RPWM;
}PID_GetData_t;		//两级缓存用于显示PID状态的参数

#endif
