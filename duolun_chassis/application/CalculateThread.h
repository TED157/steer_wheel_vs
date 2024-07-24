#ifndef __CalculateThread_H
#define __CalculateThread_H
#include "struct_typedef.h"
#include "feet_motor.h"

typedef enum
{
	NOFORCE,
	STOP,
	FALLOW,
	ROTING,
	HIGHSPEED,
	RESERVE_ROTATE
}ChassisMode_e;

typedef struct
{
	motor_measure_t Motor6020[4];
	motor_measure_t Motor3508[4];
	motor_measure_t YawMotor;
	ChassisMode_e Mode;
	
	float vx;
	float vy;
	float wz;
	
	float vx_last[2]; //[0]是稳定时的v_last,[1]是实时的v_last
	float vy_last[2];
	uint8_t fast_turning_flag;
	uint16_t fast_turning_counter;

	float Current[8];
	float WheelSpeed[4];
	float WheelAngle[4];
	float speed_6020[4];
	uint8_t CapKey;
	
}Chassis_t;

typedef struct{
	uint8_t AimbotState;
	uint8_t AimbotTarget;
	int16_t TargetX;
	int16_t TargetY;
}Aimbot_Message_t;

extern Chassis_t Chassis;
extern Aimbot_Message_t        Aimbot_Message;

void CalculateThread(void const * pvParameters);
void ChassisCurrentUpdate();





#endif



