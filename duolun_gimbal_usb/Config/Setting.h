#ifndef SETTING_H
#define SETTING_H

#define MOTOR_OFFLINE_TIMEMAX   50
#define REMOTE_OFFLINE_TIMEMAX  550
#define AIMBOT_OFFLINE_TIMEMAX  550
#define REFEREE_OFFLINE_TIMEMAX 3000
#define FT_REMOTE_OFFLINE_TIMEMAX  200

// 用于区分三辆舵轮
//#define YELLOW_STEERWHEEL
//#define GREEN_STEERWHEEL
#define BLACK_STEERWHEEL

//区分pitch轴电机类型
#if defined YELLOW_STEERWHEEL
#define PARAMETER_FILE "YellowSteeringWheelParameter.h"
#define DAMIAO_PITCH
#elif defined GREEN_STEERWHEEL
#define PARAMETER_FILE "GreenSteeringWheelParameter.h"
#define DAMIAO_PITCH
#elif defined BLACK_STEERWHEEL
#define PARAMETER_FILE "BlackSteeringWheelParameter.h"
#define GM6020_PITCH
#endif

// ���������ļ�
//#define PARAMETER_FILE "Infantry3Parameter.h"
// ��λ�����ļ�
#define KEYMAP_FILE "Infantry4KeyMap.h"
// imu��װ����
#define IMU_DIRECTION_rzyx_XYZ
// gyro yaw��ƫ��
#define GYRO_YAW_BIAS -0.0019f//-0.00770,-0.01065f,INCREASE
// �������������
#define MAIN_SHOOTER_TYPE_NORMAL
//#define MAIN_SHOOTER_TYPE_HEAVY
// ���ID����
#define YAW_MOTOR_ID                0x205
#define PITCH_MOTOR_ID              0x206
#define DAMIAO_PITCH_MOTOR_SLAVE_ID 0x08
#define DAMIAO_PITCH_MOTOR_MASTER_ID 0x09
#define ROTOR_MOTOR_ID              0x203
#define AMMO_LEFT_MOTOR_ID          0x201
#define AMMO_RIGHT_MOTOR_ID         0x202
// �����װ����
// ��̨��������˶��������̨��̬�����ƣ�����ϵͬ��Ϊ1������Ϊ-1
// ���̵�������˶�����͵������ǹ�ܷ���ͬ��Ϊ1������Ϊ-1
// Ħ���ֵ�������˶�����͵���ͬ��Ϊ1������Ϊ-1
#if (defined YELLOW_STEERWHEEL) || (defined GREEN_STEERWHEEL)
#define YAW_MOTOR_DIRECTION 1
#define PITCH_MOTOR_DIRECTION -1
#define DAMIAO_PITCH_MOTOR_DIRECTION 1
#define ROTOR_MOTOR_DIRECTION 1
#define AMMO_LEFT_MOTOR_DIRECTION -1
#define AMMO_RIGHT_MOTOR_DIRECTION 1
#elif defined BLACK_STEERWHEEL
#define YAW_MOTOR_DIRECTION 1
#define PITCH_MOTOR_DIRECTION -1
#define DAMIAO_PITCH_MOTOR_DIRECTION 1
#define ROTOR_MOTOR_DIRECTION 1
#define AMMO_LEFT_MOTOR_DIRECTION 1
#define AMMO_RIGHT_MOTOR_DIRECTION -1
#endif
// ��̨YAW�����͸����޷�
#define YAW_ZERO_ECDANGLE           -2.5//180.0f-142.5f//21.4f
#define PITCH_MIN_ANGLE             -19.5f//-27.0f//-24.0f//-26.0f
#define PITCH_MAX_ANGLE             17.0f//15.0f
// Ĭ��Ħ�����ٶ�
#define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_30MS
#define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_30MS
#define DEFAULT_AMMO_SPEEDSET       AMMO_SPEEDSET_30MS
// ͨ��can����λ��
#define COMMUNICATE_CANPORT         hcan1






#endif

