/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "struct_typedef.h"
#include "main.h"

//达妙电机的数据范围
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

//rm motor data
typedef struct{
    uint16_t    ecd;
    int16_t     speed_rpm;
    int16_t     given_current;
    uint8_t     temperate;
    int16_t     last_ecd;
} motor_measure_t;

// 电机回传信息结构体
typedef struct
{
	int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
} motor_fbpara_t;

typedef struct
{
	int8_t id;
	uint8_t start_flag;
	motor_fbpara_t para;
	uint8_t error_clear_flag;
	uint32_t error_clear_time;
} DM_motor_t;

typedef enum
{
  DaMiao_DISABLE = 0,
  DaMiao_ENABLE,
  DaMiao_CLear_ERROR,
}DaMiao_CMD_e;

extern void GimbalMotorControl(int16_t YawMotor, int16_t PitchMotor, int16_t RotorMotor, int16_t AmmoLeftMotor, int16_t AmmoRightMotor);
extern void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t* message);
extern void GimbalMotorMeasureUpdate(GimbalMotorMeasure_t* Gimbal);
extern void ShootMotorMeasureUpdate(ShootMotorMeasure_t* Shoot);
extern void DaMiaoCanSend(float DaMiao);

#endif
