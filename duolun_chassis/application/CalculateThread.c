#include "CalculateThread.h"
#include "feet_motor.h"
#include "Remote.h"
#include "AttitudeThread.h"
#include "cmsis_os.h"
#include "pid.h"
#include "Setting.h"
#include "user_lib.h"
#include "CanPacket.h"
#include "stdio.h"
#include "InterruptService.h"
#include "RefereeBehaviour.h"
#include "usart.h"
#include "MahonyAHRS.h"
#include "arm_math.h"
#include "CMS.h"

#define START_POWER 15.0f

Chassis_t Chassis;
RC_ctrl_t Remote;
EulerSystemMeasure_t Imu;
Aim_t Aim;
PTZ_t PTZ;
ext_game_robot_status_t Referee;
Aimbot_Message_t Aimbot_Message;
extern ext_power_heat_data_t power_heat_data_t;
uint32_t F_Motor[8];

uint16_t last_HP=0;
uint16_t reset_time=0;

#if defined GREEN_STEERWHEEL
fp32 Angle_zero_6020[4] = {47.1810455, 128.138214, 98.9989014, 67.6620636};
#elif defined YELLOW_STEERWHEEL
fp32 Angle_zero_6020[4] = {75.8808594, -54.7405701, 100.625061, 158.815765};
#elif defined BLACK_STEERWHEEL
fp32 Angle_zero_6020[4] = {-21.3380585,-21.3380585 , -78.7376404, 69.376144};
#endif
//fp32 Angle_zero_6020[4] = {0, 0, 0, 0};
fp32 Direction[5] = {-1.0, -1.0, 1.0, 1.0, -1.0};
fp32 Maxspeed = 6000.0f;
fp32 speed[4];
KFP Power_kf;
fp32 Power_Max = 45.0f;

float angle_minus;
float last_speed[8] = {0},stall_kp[4]={0};
uint8_t offline_solve[2] = {1,0};

fp32 he = 0;
float kp = 1.30 * 1.99999999e-06;
float lijupower = 0.0f;
uint8_t power_flag=0;
float power_scale;
fp32 v_gain=0,cap_gain=1;

uint8_t stop_flag=0;
int16_t stop_countdown=0;
uint16_t speed_up_time=0;


pid_type_def follow_yaw;
pid_type_def follow;
pid_type_def left_front_6020_speed_pid;
pid_type_def right_front_6020_speed_pid;
pid_type_def right_back_6020_speed_pid;
pid_type_def left_back_6020_speed_pid;
pid_type_def left_front_6020_position_pid;
pid_type_def right_front_6020_position_pid;
pid_type_def right_back_6020_position_pid;
pid_type_def left_back_6020_position_pid;
pid_type_def left_front_3508_pid;
pid_type_def right_front_3508_pid;
pid_type_def right_back_3508_pid;
pid_type_def left_back_3508_pid;
pid_type_def speed_adjust_pid;
//pid_type_def low_power_adjust_pid;
first_order_filter_type_t current_6020_filter_type;
first_order_filter_type_t current_3508_filter_type;
first_order_filter_type_t referee_power;
first_order_filter_type_t wheel_angle_1;
first_order_filter_type_t wheel_angle_2;
first_order_filter_type_t wheel_angle_3;
first_order_filter_type_t wheel_angle_4;
first_order_filter_type_t wz_filter;


fp32 follow_angle;
fp32 follow_yaw_PID[3]={0.08,0,1};
fp32 follow_PID[3]={FOLLOW_KP,FOLLOW_KI,FOLLOW_KD};
fp32 left_front_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 right_front_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 right_back_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 left_back_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 left_front_6020_position_PID[3] = {POSITION_6020_KP_1, POSITION_6020_KI_1, POSITION_6020_KD_1};
fp32 right_front_6020_position_PID[3] = {POSITION_6020_KP_2, POSITION_6020_KI_2, POSITION_6020_KD_2};
fp32 right_back_6020_position_PID[3] = {POSITION_6020_KP_1, POSITION_6020_KI_1, POSITION_6020_KD_1};
fp32 left_back_6020_position_PID[3] = {POSITION_6020_KP_2, POSITION_6020_KI_2, POSITION_6020_KD_2};
fp32 left_front_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 right_front_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 right_back_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 left_back_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 speed_adjust_PID[3] = {speed_adjust_KP, speed_adjust_KI, speed_adjust_KD};
//fp32 low_power_adjust_PID[3] = {low_power_KP, low_power_KI, low_power_KD};

extern motor_measure_t LEFT_FRONT_6020_Measure;
extern motor_measure_t RIGHT_FRONT_6020_Measure;
extern motor_measure_t RIGHT_BACK_6020_Measure;
extern motor_measure_t LEFT_BACK_6020_Measure;
extern motor_measure_t LEFT_FRONT_3508_Measure;
extern motor_measure_t RIGHT_FRONT_3508_Measure;
extern motor_measure_t RIGHT_BACK_3508_Measure;
extern motor_measure_t LEFT_BACK_3508_Measure;
extern motor_measure_t YawMotorMeasure;
OfflineMonitor_t Offline;

void ChassisInit();
void ChassisModeUpdate();
void ChassisPidUpadte();
void ChassisCommandUpdate();
void ChassisCurrentUpdate();
void RefereeInfUpdate(ext_game_robot_status_t *referee);
void ChassisInfUpdate();
void Angle_Speed_calc();
void CMS__();
uint8_t chassis_powerloop(Chassis_t *Chassis);
void Chassis_motor3508_speed_adjust(Chassis_t *Chassis,fp32 *kp);
void Fast_Turning_Control(Chassis_t* Chassis);

float vx_last=0,vy_last=0;
float speed_adjust_test=0;
void CalculateThread(void const *pvParameters)
{

	ChassisInit();

	while (1)
	{
		//Remote = *get_remote_control_point();
		
		DeviceOfflineMonitorUpdate(&Offline);
		ChassisModeUpdate();
		ChassisInfUpdate();
		RefereeInfUpdate(&Referee);
		GimbalEulerSystemMeasureUpdate(&Imu);
		ChassisCommandUpdate();
		chassis_powerloop(&Chassis);
		CMS__();
		Chassis_Control(Chassis.Current[0]*offline_solve[Offline.Motor[0]],
						Chassis.Current[1]*offline_solve[Offline.Motor[1]],
						Chassis.Current[2]*offline_solve[Offline.Motor[2]],
						Chassis.Current[3]*offline_solve[Offline.Motor[3]],
						Chassis.Current[4],
						Chassis.Current[5],
						Chassis.Current[6]/*speed_adjust_test*/,
						Chassis.Current[7]/*speed_adjust_test*/);
	
		osDelay(1);
	}
}

void ChassisInit()
{
	PID_init(&left_front_6020_speed_pid, PID_POSITION, left_front_6020_speed_PID, 30000, 10000);
	PID_init(&right_front_6020_speed_pid, PID_POSITION, right_front_6020_speed_PID, 30000, 10000);
	PID_init(&right_back_6020_speed_pid, PID_POSITION, right_back_6020_speed_PID, 30000, 10000);
	PID_init(&left_back_6020_speed_pid, PID_POSITION, left_back_6020_speed_PID, 30000, 10000);
	PID_init(&left_front_6020_position_pid, PID_POSITION, left_front_6020_position_PID, 300, 60);              //6020
	PID_init(&right_front_6020_position_pid, PID_POSITION, right_front_6020_position_PID, 300, 60);
	PID_init(&right_back_6020_position_pid, PID_POSITION, right_back_6020_position_PID, 300, 60);
	PID_init(&left_back_6020_position_pid, PID_POSITION, left_back_6020_position_PID, 300, 60);

	PID_init(&left_front_3508_pid, PID_POSITION, left_front_3508_PID, 8000, 2000);
	PID_init(&right_front_3508_pid, PID_POSITION, right_front_3508_PID, 8000, 2000);
	PID_init(&right_back_3508_pid, PID_POSITION, right_back_3508_PID, 8000, 2000);							//3508
	PID_init(&left_back_3508_pid, PID_POSITION, left_back_3508_PID, 8000, 2000);
	
	PID_init(&follow_yaw,PID_POSITION,follow_yaw_PID,1,1);
	PID_init(&follow,PID_POSITION,follow_PID,2,1);
	
	PID_init(&speed_adjust_pid,PID_POSITION,speed_adjust_PID,1.0,0);
	
	//PID_init(&low_power_adjust_pid,PID_POSITION,low_power_adjust_PID,0.5,1);
	//KalmanFilter_init(&Power_kf, 0.0f , 0.0001f,0.0118f ,0.0,50.0,2.0);//A,B,P,Q,R                   //����
	first_order_filter_init(&current_6020_filter_type,0.002,0.1);
	first_order_filter_init(&current_3508_filter_type,0.002,0.1);
	first_order_filter_init(&wheel_angle_1,0.001,0.1);
	first_order_filter_init(&wheel_angle_2,0.001,0.1);
	first_order_filter_init(&wheel_angle_3,0.001,0.1);
	first_order_filter_init(&wheel_angle_4,0.001,0.1);

	CMS_Data.charge_flag=1;
	Chassis.vx_last[0] = 0;
	Chassis.vy_last[0] = 0;
	Chassis.fast_turning_flag = 0;
};

void ChassisInfUpdate()
{
	memcpy(&Chassis.Motor3508[0], &LEFT_FRONT_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor3508[1], &RIGHT_FRONT_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor3508[2], &RIGHT_BACK_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor3508[3], &LEFT_BACK_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[0], &LEFT_FRONT_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[1], &RIGHT_FRONT_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[2], &RIGHT_BACK_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[3], &LEFT_BACK_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
}

void ChassisModeUpdate()
{
	
	switch (PTZ.ChassisStatueRequest)
	{
	case 0x01:
		Chassis.Mode = NOFORCE;
		break;
	case 0x12:
	case 0x32:
	case 0x52:
	case 0x72:
		Chassis.Mode = ROTING;
		break;
	case 0x82:
		Chassis.Mode = RESERVE_ROTATE;
	break;
	case 0x0A:
	case 0x2A:
		Chassis.Mode = FALLOW;
		break;
	case 0x06:
	case 0x26:
		Chassis.Mode = STOP;
		break;

	default:
		break;
	}
	if((PTZ.ChassisStatueRequest & (0x01 << 5)) != 0)
	{
		Chassis.CapKey = 1;
	}
	else Chassis.CapKey = 0;
}

void ChassisCommandUpdate()
{
	
	
	
	//Chassis.wz = -Remote.rc.ch[2] / 660.0f * (1.0f + Chassis.Power_Proportion / Power_Max);
	robot_state.current_HP == 0 ? reset_time = 0 : reset_time>3000 ? reset_time=3000 : reset_time++;
	if (Chassis.Mode == NOFORCE || Offline.PTZnode ==1 || ((Offline.Motor[4] || Offline.Motor[5] || Offline.Motor[6] || Offline.Motor[7]) && reset_time<3000))
	{
		
		Chassis.Current[0] = 0;
		Chassis.Current[1] = 0;
		Chassis.Current[2] = 0;
		Chassis.Current[3] = 0;
		Chassis.Current[4] = 0;
		Chassis.Current[5] = 0;
		Chassis.Current[6] = 0;
		Chassis.Current[7] = 0;
		return;
	}
	if (Chassis.Mode == FALLOW || Chassis.Mode == ROTING || Chassis.Mode == STOP || Chassis.Mode == RESERVE_ROTATE) //
	{
		follow_angle = loop_fp32_constrain(FollowAngle, YawMotorMeasure.angle - 180.0f,YawMotorMeasure.angle + 180.0f);
		
		if (Chassis.Mode == FALLOW)
		{
			angle_minus = -YawMotorMeasure.angle + FollowAngle;
			if(angle_minus>180) angle_minus-=360;
			else if(angle_minus<-180) angle_minus+=360;
			Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus/180.0*PI) - (PTZ.LRSpeed / 32767.0f) * sin(angle_minus/180.0*PI)) * (v_gain );
			Chassis.vy =  ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus/180.0*PI) + (PTZ.LRSpeed / 32767.0f) * cos(angle_minus/180.0*PI)) * (v_gain );
			Chassis.wz = -PID_calc(&follow,YawMotorMeasure.angle,follow_angle); //* (1.0f + Chassis.Power_Proportion /Power_Max );
			if(Fabs(Chassis.wz)<0.5*v_gain&&Fabs(angle_minus)<0.2){
				Chassis.wz=0.001*Chassis.wz/Fabs(Chassis.wz);
			}
//			if(Fabs(Chassis.wz)<0.1) Chassis.wz=0;
			//Chassis.wz = 0;
		}
		else if (Chassis.Mode == ROTING)
		{
			Chassis.wz = sin(v_gain*0.74)*1.45;
			if(Power_Max>85)
				Chassis.wz = sin(1.27*0.74)*1.45;
			if((PTZ.ChassisStatueRequest&64)==64)
			{
				Chassis.wz = sin(v_gain*0.74)*2.05;
			}
			angle_minus = -YawMotorMeasure.angle + FollowAngle - YawMotorMeasure.speed_rpm * 0.45;
			Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus/180.0*PI) - (PTZ.LRSpeed / 32767.0f) * sin(angle_minus/180.0*PI))*v_gain/1.8;//* (1.0f + Chassis.Power_Proportion /Power_Max );
			Chassis.vy = ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus/180.0*PI) + (PTZ.LRSpeed / 32767.0f) * cos(angle_minus/180.0*PI))*v_gain/1.8;//* (1.0f + Chassis.Power_Proportion /Power_Max );
		}
		else if(Chassis.Mode == RESERVE_ROTATE)
		{
			Chassis.wz = -sin(v_gain*0.74)*1.45;
			angle_minus = -YawMotorMeasure.angle + FollowAngle - YawMotorMeasure.speed_rpm * 0.45;
			Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus/180.0*PI) - (PTZ.LRSpeed / 32767.0f) * sin(angle_minus/180.0*PI))*v_gain/1.8;
			Chassis.vy = ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus/180.0*PI) + (PTZ.LRSpeed / 32767.0f) * cos(angle_minus/180.0*PI))*v_gain/1.8;
		}
		else if (Chassis.Mode == STOP)
		{
			angle_minus = -YawMotorMeasure.angle + FollowAngle;
			Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus/180.0*PI) - (PTZ.LRSpeed / 32767.0f) * sin(angle_minus/180.0*PI));
			Chassis.vy = ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus/180.0*PI) + (PTZ.LRSpeed / 32767.0f) * cos(angle_minus/180.0*PI));
			Chassis.wz = 0.0;
		}
		
	}
			/********************************	6020�ǶȽ���         ***********************/ // ����   i++  &&

		if(Chassis.Mode == FALLOW){
			Fast_Turning_Control(&Chassis);
		}
		if (Fabs(PTZ.FBSpeed / 32767.0) > 0.05 || Fabs(PTZ.LRSpeed / 32767.0) > 0.05 )
		{
			
			for (uint8_t i = 0; i < 4; )
			{
				Chassis.WheelAngle[i] = atan2((Chassis.vy) + Chassis.wz * gen2 * Direction[i], (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1])) / 3.1415927 * 180.0 + Angle_zero_6020[i]; // ?????????
				i++;
			}
			vx_last=Chassis.vx;
			vy_last=Chassis.vy;
			stop_flag=1;
		}
		else
		{
			if(stop_flag==1 && Chassis.Mode==FALLOW&&Fabs(angle_minus)  <2.0)
			{
				stop_flag=2;
				stop_countdown=500;
			}else if(Fabs(angle_minus)  >10.0){
				stop_flag=0;
			}
			if(stop_countdown<=0)
				stop_flag=0;
			if(stop_flag==2 && ((Fabs(LEFT_BACK_3508_Measure.speed_rpm)>100 || Fabs(RIGHT_BACK_3508_Measure.speed_rpm)>100 || Fabs(LEFT_FRONT_3508_Measure.speed_rpm)>100 || Fabs(RIGHT_FRONT_3508_Measure.speed_rpm)>100)) && Chassis.Mode==FALLOW)
			{
				Chassis.wz=0;
				for (uint8_t i = 0; i < 4; )
				{
				Chassis.WheelAngle[i] = atan2(vy_last, (vx_last)) / 3.1415927 * 180.0 + Angle_zero_6020[i]; 
				i++;
				}
				//stop_countdown--;
			}
			else{
			vx_last=0;
			vy_last=0;
			stop_flag=0;
			if(Chassis.wz == 0)
			{
			Chassis.WheelAngle[0] = 0 + Angle_zero_6020[0]+angle_minus;
			Chassis.WheelAngle[1] = 0 + Angle_zero_6020[1]+angle_minus; // Ĭ�ϽǶ�
			Chassis.WheelAngle[2] = 0 + Angle_zero_6020[2]+angle_minus;
			Chassis.WheelAngle[3] = 0 + Angle_zero_6020[3]+angle_minus;						
			}
			else if(Chassis.wz > 0)
			{
			Chassis.WheelAngle[0] = -135.0f + Angle_zero_6020[0];
			Chassis.WheelAngle[1] = -45.0f + Angle_zero_6020[1]; // Ĭ�ϽǶ�
			Chassis.WheelAngle[2] = 45.0f + Angle_zero_6020[2];
			Chassis.WheelAngle[3] = 135.0f + Angle_zero_6020[3];
			}
			else if(Chassis.wz < 0)
			{
			Chassis.WheelAngle[0] = 45.0f + Angle_zero_6020[0];
			Chassis.WheelAngle[1] = 135.0f + Angle_zero_6020[1]; // Ĭ�ϽǶ�
			Chassis.WheelAngle[2] = -135.0f + Angle_zero_6020[2];
			Chassis.WheelAngle[3] = -45.0f + Angle_zero_6020[3];							
			}	
		}
		speed_up_time=0;
		}
		Chassis.WheelAngle[0] = loop_fp32_constrain(Chassis.WheelAngle[0], LEFT_FRONT_6020_Measure.angle - 180.0f, LEFT_FRONT_6020_Measure.angle + 180.0f);
		Chassis.WheelAngle[1] = loop_fp32_constrain(Chassis.WheelAngle[1], RIGHT_FRONT_6020_Measure.angle - 180.0f, RIGHT_FRONT_6020_Measure.angle + 180.0f);
		Chassis.WheelAngle[2] = loop_fp32_constrain(Chassis.WheelAngle[2], RIGHT_BACK_6020_Measure.angle - 180.0f, RIGHT_BACK_6020_Measure.angle + 180.0f);
		Chassis.WheelAngle[3] = loop_fp32_constrain(Chassis.WheelAngle[3], LEFT_BACK_6020_Measure.angle - 180.0f, LEFT_BACK_6020_Measure.angle + 180.0f);		
		/***********************                 3508�ٶȽ���                    ******************************/
				//���ݵ�ʹ��
		if(((CMS_Data.cms_status) & (uint16_t) 1) != 1 && CMS_Data.Mode == FLY)
		{
			speed_up_time++;
			if(speed_up_time>1250){
				speed_up_time=1250;
			}
			Chassis.vx = ((cap_gain-1) * speed_up_time *0.0008 +1) * Chassis.vx ;
			speed_adjust_test=Chassis.vx;
			Chassis.vy = ((cap_gain-1) * speed_up_time *0.0008 +1) * Chassis.vy ;
			Chassis.wz = 1.0 * Chassis.wz ;
			
		}
		else if(((CMS_Data.cms_status) & (uint16_t) 1) != 1 && CMS_Data.Mode == HIGH_SPEED &&Power_Max<=120)
		{
			speed_up_time=0;
			Chassis.vx = Chassis.vx * 1.4;
			Chassis.vy = Chassis.vy * 1.4;
			Chassis.wz = 1.0 * Chassis.wz ;
		}
		else{
			speed_up_time=0;
		}
		for (uint8_t i = 0; i < 4;)
		{
			speed[i] = sqrtf((Chassis.vy + Chassis.wz * gen2 * Direction[i]) * (Chassis.vy + Chassis.wz * gen2 * Direction[i]) 
							+ (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1]) * (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1]));
			i++;
		}
		if(((CMS_Data.cms_status) & (uint16_t) 1) != 1 && CMS_Data.Mode == FLY){
			Chassis.WheelSpeed[0] = -0.75*speed[0];
			Chassis.WheelSpeed[1] = 0.75*speed[1];
			Chassis.WheelSpeed[2] = 1.34*speed[2];
			Chassis.WheelSpeed[3] = -1.34*speed[3];
		}
		else{
			Chassis.WheelSpeed[0] = -speed[0];
			Chassis.WheelSpeed[1] = speed[1];
			Chassis.WheelSpeed[2] = speed[2];
			Chassis.WheelSpeed[3] = -speed[3];
		}
		
//		if(Fabs(Fabs(LEFT_FRONT_6020_Measure.angle-Chassis.WheelAngle[0])>1.5
//			||RIGHT_FRONT_6020_Measure.angle-Chassis.WheelAngle[1])>1.5 
//			|| Fabs(RIGHT_BACK_6020_Measure.angle-Chassis.WheelAngle[2])>1.5
//		    ||Fabs(LEFT_BACK_6020_Measure.angle-Chassis.WheelAngle[3])>1.5)
//			wheel_flag=1;

		Angle_Speed_calc(); // �Ƕ��Ż�

		//Fast_Turning_Control(&Chassis);

		Chassis.speed_6020[0] = PID_calc(&left_front_6020_position_pid, LEFT_FRONT_6020_Measure.angle, Chassis.WheelAngle[0]);
		Chassis.speed_6020[1] = PID_calc(&right_front_6020_position_pid, RIGHT_FRONT_6020_Measure.angle, Chassis.WheelAngle[1]);
		Chassis.speed_6020[2] = PID_calc(&right_back_6020_position_pid, RIGHT_BACK_6020_Measure.angle, Chassis.WheelAngle[2]);
		Chassis.speed_6020[3] = PID_calc(&left_back_6020_position_pid, LEFT_BACK_6020_Measure.angle, Chassis.WheelAngle[3]);
	


		
	ChassisCurrentUpdate();

}

	

void Angle_Speed_calc()
{
	for (uint8_t i = 0; i < 4; )
	{
		if (Chassis.WheelAngle[i] - Chassis.Motor6020[i].angle > 90.0f)
		{
			Chassis.WheelAngle[i] -= 180.0f;
			Chassis.WheelSpeed[i] = -Chassis.WheelSpeed[i];
		}
		if (Chassis.WheelAngle[i] - Chassis.Motor6020[i].angle < -90.0f)
		{
			Chassis.WheelAngle[i] += 180.0f;
			Chassis.WheelSpeed[i] = -Chassis.WheelSpeed[i];
		}
		i++;
	}
	
//	first_order_filter_cali(&wheel_angle_1,Chassis.WheelAngle[0]);
//			first_order_filter_cali(&wheel_angle_2,Chassis.WheelAngle[1]);
//			first_order_filter_cali(&wheel_angle_3,Chassis.WheelAngle[2]);
//			first_order_filter_cali(&wheel_angle_4,Chassis.WheelAngle[3]);
//			Chassis.WheelAngle[0] = wheel_angle_1.out;
//			Chassis.WheelAngle[1] = wheel_angle_2.out;
//			Chassis.WheelAngle[2] = wheel_angle_3.out;
//			Chassis.WheelAngle[3] = wheel_angle_4.out;
	
}

void ChassisCurrentUpdate()
{
	if(stop_flag==2){
		Chassis.WheelSpeed[0] = 0;
		Chassis.WheelSpeed[1] = 0;
		Chassis.WheelSpeed[2] = 0;
		Chassis.WheelSpeed[3] = 0;
		left_front_3508_pid.Kp=6000;
		right_front_3508_pid.Kp=6000;
		right_back_3508_pid.Kp=6000;
		left_back_3508_pid.Kp=6000;
	}
	else{
		left_front_3508_pid.Kp=3600;
		right_front_3508_pid.Kp=3600;
		right_back_3508_pid.Kp=3600;
		left_back_3508_pid.Kp=3600;
	}
	Chassis.Current[0] = PID_calc(&left_front_6020_speed_pid, LEFT_FRONT_6020_Measure.speed_rpm, Chassis.speed_6020[0]);;
	Chassis.Current[1] = PID_calc(&right_front_6020_speed_pid, RIGHT_FRONT_6020_Measure.speed_rpm, Chassis.speed_6020[1]);
	Chassis.Current[2] = PID_calc(&right_back_6020_speed_pid, RIGHT_BACK_6020_Measure.speed_rpm, Chassis.speed_6020[2]);;
	Chassis.Current[3] = PID_calc(&left_back_6020_speed_pid, LEFT_BACK_6020_Measure.speed_rpm, Chassis.speed_6020[3]);
	//wz=LEFT_FRONT_3508_Measure.speed_rpm / Maxspeed;
	Chassis_motor3508_speed_adjust(&Chassis,stall_kp);
	Chassis.Current[4] = stall_kp[0]*PID_calc(&left_front_3508_pid, LEFT_FRONT_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[0]);;
	Chassis.Current[5] = stall_kp[1]*PID_calc(&right_front_3508_pid, RIGHT_FRONT_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[1]);
	Chassis.Current[6] = stall_kp[2]*PID_calc(&right_back_3508_pid, RIGHT_BACK_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[2]);
	Chassis.Current[7] = stall_kp[3]*PID_calc(&left_back_3508_pid, LEFT_BACK_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[3]);
	if(stop_flag==2){
		Chassis.Current[0] *= 1.2;
		Chassis.Current[1] *= 1.2;
		Chassis.Current[2] *= 1.2;
		Chassis.Current[3] *= 1.2;
	}
}

void RefereeInfUpdate(ext_game_robot_status_t *referee)
{
	memcpy(referee, &robot_state, sizeof(ext_game_robot_status_t));
	switch(referee->chassis_power_limit)
	{
		case 45:
			Power_Max = 45;v_gain=0.91;cap_gain=2.75;break;
		case 50:
			Power_Max = 50;v_gain=0.95;cap_gain=2.63;break;
		case 55:
			Power_Max = 55;v_gain=0.99;cap_gain=2.53;break;
		case 60:	
			Power_Max = 60;v_gain=1.02;cap_gain=2.45;break;
		case 65:	
			Power_Max = 65;v_gain=1.06;cap_gain=2.36;break;
		case 70:	
			Power_Max = 70;v_gain=1.12;cap_gain=2.23;break;
		case 75:	
			Power_Max = 75;v_gain=1.16;cap_gain=2.16;break;
		case 80:
			Power_Max = 80;v_gain=1.25;cap_gain=2.0;break;//基准
		case 85:	
			Power_Max = 85;v_gain=1.27;cap_gain=1.97;break;
		case 90:	
			Power_Max = 90;v_gain=1.29;cap_gain=1.94;break;
		case 95:	
			Power_Max = 95;v_gain=1.33;cap_gain=1.88;break;
		case 100:	
			Power_Max = 100;v_gain=1.36;cap_gain=1.84;break;
		case 120:
			Power_Max = 120;v_gain=1.51;cap_gain=1.66;break;
		case 130:
			Power_Max = 130;v_gain=1.58;cap_gain=1.58;break;
		case 140:
			Power_Max = 140;v_gain=1.65;cap_gain=1.52;break;
		case 150:
			Power_Max = 150;v_gain=1.73;cap_gain=1.45;break;
		case 160:
			Power_Max = 160;v_gain=1.80;cap_gain=1.39;break;
		case 170:
			Power_Max = 170;v_gain=1.87;cap_gain=1.34;break;
		case 180:
			Power_Max = 180;v_gain=1.93;cap_gain=1.30;break;
		case 190:
			Power_Max = 190;v_gain=2.00;cap_gain=1.25;break;
		case 200:
			Power_Max = 200;v_gain=2.100;cap_gain=1.19;break;
		default:
			Power_Max = 45;v_gain=0.85;cap_gain=1.0;break;
		
	}
}


extern uint16_t cms_offline_counter;
void CMS__()
{
	if(CMS_Data.cms_cap_v < 12){
		CMS_Data.charge_flag=0;
	}
	else if(CMS_Data.cms_cap_v > 18 && (Chassis.CapKey)){
		CMS_Data.charge_flag=1;
	}
	else if(CMS_Data.cms_cap_v > 15 && (!Chassis.CapKey)){
		CMS_Data.charge_flag=2;
	}
	else if(CMS_Data.charge_flag==1 && CMS_Data.cms_cap_v > 12 && (!Chassis.CapKey)){
		CMS_Data.charge_flag=2;
	}
	if((Chassis.CapKey) && CMS_Data.cms_cap_v > 12 && CMS_Data.charge_flag==1 )
	{
		CMS_Data.Mode =FLY;
	}
	else if((!Chassis.CapKey) && CMS_Data.cms_cap_v > 12 && CMS_Data.charge_flag==2 && (PTZ.PTZStatusInformation&16)==16){
		CMS_Data.Mode = HIGH_SPEED;
	}
	else{
		CMS_Data.Mode = NORMAL;
	}
	if(power_heat_data_t.buffer_energy < 20 || cms_offline_counter > 200) //cms�ò���
	{
		CMS_Data.Mode = NORMAL;	
	}
	cms_offline_counter ++;
}
uint8_t chassis_limit_update_flag=0;
void chassis_limit_update(void)
{
	if(Referee.chassis_power_limit!=Power_Max)
	{
		chassis_limit_update_flag=1;
	}
	else{
		chassis_limit_update_flag=0;
	}
}

uint8_t cms_flag=0;
float Plimit = 0;
uint8_t chassis_powerloop(Chassis_t *Chassis)
{

	// ��ع���
	// ����CMS��繦��

	// ��ѧģ��Ԥ�⹦�ʣ���ʹ��CMS���ʼƵĹ��ʼ��㣩
//	he = fabs(2 * Chassis->Motor3508[0].speed_rpm - last_speed[0]) * fabs((float)Chassis->Current[4]) * kp +
//		 fabs(2 * Chassis->Motor3508[1].speed_rpm - last_speed[1]) * fabs((float)Chassis->Current[5]) * kp +
//		 fabs(2 * Chassis->Motor3508[2].speed_rpm - last_speed[2]) * fabs((float)Chassis->Current[6]) * kp +
//		 fabs(2 * Chassis->Motor3508[3].speed_rpm - last_speed[3]) * fabs((float)Chassis->Current[7]) * kp+
//		 fabs(2 * Chassis->Motor6020[0].speed_rpm - last_speed[4]) * fabs((float)Chassis->Current[0]) * kp+
//		 fabs(2 * Chassis->Motor6020[1].speed_rpm - last_speed[5]) * fabs((float)Chassis->Current[1]) * kp+
//		 fabs(2 * Chassis->Motor6020[2].speed_rpm - last_speed[6]) * fabs((float)Chassis->Current[2]) * kp+
//		 fabs(2 * Chassis->Motor6020[3].speed_rpm - last_speed[7]) * fabs((float)Chassis->Current[3]) * kp;
	he = fabs((float) Chassis->Motor3508[0].speed_rpm) * fabs((float)Chassis->Current[4]) * kp +
		 fabs((float) Chassis->Motor3508[1].speed_rpm) * fabs((float)Chassis->Current[5]) * kp +
		 fabs((float) Chassis->Motor3508[2].speed_rpm) * fabs((float)Chassis->Current[6]) * kp +
		 fabs((float) Chassis->Motor3508[3].speed_rpm) * fabs((float)Chassis->Current[7]) * kp+
		 fabs((float) Chassis->Motor6020[0].speed_rpm) * fabs((float)Chassis->Current[0]) * kp+
		 fabs((float) Chassis->Motor6020[1].speed_rpm) * fabs((float)Chassis->Current[1]) * kp+
		 fabs((float) Chassis->Motor6020[2].speed_rpm) * fabs((float)Chassis->Current[2]) * kp+
		 fabs((float) Chassis->Motor6020[3].speed_rpm) * fabs((float)Chassis->Current[3]) * kp;
//	last_speed[0] = Chassis->Motor3508[0].speed_rpm;
//	last_speed[1] = Chassis->Motor3508[0].speed_rpm;
//	last_speed[2] = Chassis->Motor3508[0].speed_rpm;
//	last_speed[3] = Chassis->Motor3508[0].speed_rpm;

	lijupower = he + START_POWER;

	if (CMS_Data.cms_cap_v <= 15 || cms_offline_counter > 500 || power_heat_data_t.buffer_energy<40)
	{
		power_flag = 0;
		cms_flag=0;
	}
	else{
		power_flag = 1;
	}
	if (CMS_Data.Mode==FLY)
	{
		Power_Max = 200;
		cms_flag=1;
		if(power_heat_data_t.buffer_energy<20)
			power_flag=0;
		else{
			power_flag=1;
		}
		//DMA_printf("%d,%d\r\n",Chassis->Motor3508[0].speed_rpm,power_heat_data_t.buffer_energy);
	}
	else if(CMS_Data.Mode==HIGH_SPEED)
	{
		Power_Max += 50;
	}
    if(power_flag == 0){
		/*if (power_heat_data_t.buffer_energy < 40 && power_heat_data_t.buffer_energy >= 35)
			
		{
			Plimit = 0.95;
		}
		else */if (power_heat_data_t.buffer_energy < 35 && power_heat_data_t.buffer_energy >= 30)
		{
			Plimit = 0.6;
			//power_scale = (Power_Max-2) / lijupower;
		}
		else if (power_heat_data_t.buffer_energy < 30 && power_heat_data_t.buffer_energy >= 20)
		{
			Plimit = 0.25;
			//power_scale = (Power_Max-2) / lijupower;
			
		}
		else if (power_heat_data_t.buffer_energy < 20 && power_heat_data_t.buffer_energy >= 10 && cms_flag==0)
		{
			Plimit = 0.02;
			//power_scale = (Power_Max-2) / lijupower;
		}
		else if (power_heat_data_t.buffer_energy < 10 && power_heat_data_t.buffer_energy >= 0)
		{
			Plimit = 0.01;
			//power_scale = (Power_Max-2) / lijupower;}
		}
		else
		{
			Plimit = 1;
			//power_scale = 1;
		}
	}
	if (lijupower > Power_Max && power_flag == 0)
	{
		power_scale = (Power_Max-2) / lijupower;
		Chassis->Current[0] *= (power_scale) * (Plimit);
		Chassis->Current[1] *= (power_scale) * (Plimit);
		Chassis->Current[2] *= (power_scale) * (Plimit);
		Chassis->Current[3] *= (power_scale) * (Plimit);
		Chassis->Current[4] *= (power_scale) * (Plimit);
		Chassis->Current[5] *= (power_scale) * (Plimit);
		Chassis->Current[6] *= (power_scale) * (Plimit);
		Chassis->Current[7] *= (power_scale) * (Plimit);

	}
	//			if(CMS_charge_power < 0.0f)
	//			{
	//			CMS_charge_power = 0.0f;
	//			}

	//		if(chassis_power_buffer < chassis_buffer_limit)

	//	if(Chassis->power_mode == BATTERY)
	//	{
	//		if(lijupower > powermax)
	//		{
	//			power_scale = powermax/lijupower;
	//
	//			Chassis->Output.LF *= (power_scale);
	//			Chassis->Output.LB *= (power_scale);
	//			Chassis->Output.RF *= (power_scale);
	//			Chassis->Output.RB *= (power_scale);
	//
	//			CMS_charge_power = 0.0f;
	//		}else
	//		{
	//			CMS_charge_power = powermax - lijupower;
	////			CMS_charge_power = 0 ;
	//		}
	//			if(CMS_charge_power < 0.0f)
	//			{
	//			CMS_charge_power = 0.0f;
	//			}
	//	}
	//
	////���ݹ���
	//	if(Chassis->power_mode == CAPACITY)
	//	{
	//		CMS_charge_power = powermax;
	//		if(CMS_charge_power < 0.0f)
	//		{
	//			CMS_charge_power = 0.0f;
	//		}
	//	}

	return 0;
}

/*@brief：轮速调整*/
fp32 k_wheel_speed[4]={1,1,1,1};
//fp32 wheel_speed_last[4]={0,0,0,0};
//uint8_t speed_adiust_flag[4]={1,1,1,1};//轮速调整中
void Chassis_motor3508_speed_adjust(Chassis_t *Chassis,fp32 *kp)
{
	fp32 ave=0;
	for(uint8_t i=0;i<4;i++){
		if( Fabs(Chassis->WheelSpeed[i])>0)
			k_wheel_speed[i]=Fabs(Chassis->Motor3508[i].speed_rpm / Chassis->WheelSpeed[i] / Maxspeed);
		else k_wheel_speed[i]=1;
	}
	 
	ave=(k_wheel_speed[0]+k_wheel_speed[1]+k_wheel_speed[2]+k_wheel_speed[3])/4;
	for(uint8_t i=0;i<4;i++){
		kp[i]=1.0+PID_calc(&speed_adjust_pid,k_wheel_speed[i]/ave,1);
//		if(k_wheel_speed[i] > ave*1.1){
//			kp[i]=0.5; 
//		}
//		else if(k_wheel_speed[i] < ave*0.9){
//			kp[i]=2;
//		}
//		else{
//			kp[i]=1;
//		}
	}
}

//void Fast_Turning_Control(Chassis_t* chassis)
//{
//	for(uint8_t i = 0;i<4;i++){
//		if(chassis->Motor3508[i].speed>1000){
//			if((chassis->WheelAngle[i]-chassis->Motor6020[i].angle) > (90000/chassis->Motor3508[i].speed)+45)
//				chassis->WheelAngle[i] = chassis->Motor6020[i].angle + (90000/chassis->Motor3508[i].speed+45);
//			else if((chassis->WheelAngle[i]-chassis->Motor6020[i].angle) < -(90000/chassis->Motor3508[i].speed)+45)
//				chassis->WheelAngle[i] = chassis->Motor6020[i].angle - (90000/chassis->Motor3508[i].speed+45);
//		}
//	}
//}

uint16_t vx_stable_num=0,vy_stable_num=0;
float k_turn=0;
void Fast_Turning_Control(Chassis_t* Chassis)
{
	if((Chassis->vx != 0 || Chassis->vy != 0) && (Fabs(atan2(Chassis->vx,Chassis->vy)-atan2(Chassis->vx_last[0],Chassis->vy_last[0]))-PI/2) < 0.1 && (Fabs(atan2(Chassis->vx,Chassis->vy)-atan2(Chassis->vx_last[0],Chassis->vy_last[0]))-PI/2) > -0.1)
	{
		if(!Chassis->fast_turning_flag){
			if(CMS_Data.Mode == FLY){
				Chassis->fast_turning_counter=200;
				k_turn = 0.005;
			}
			else if(Power_Max>=100){
				Chassis->fast_turning_counter=40;
				k_turn = 0.025;
			}
			else if(Power_Max>=80){
				Chassis->fast_turning_counter=20;
				k_turn = 0.05;
			}
			else if(Power_Max>=60){
				Chassis->fast_turning_counter=15;
				k_turn = 0.066;
			}
			else{
				Chassis->fast_turning_counter=10;
				k_turn = 0.1;
			}
		}
		Chassis->fast_turning_flag = 1;
	}
	else
	{
		Chassis->fast_turning_flag = 0;
		Chassis->fast_turning_counter = 0;
	}
	if(Chassis->fast_turning_flag == 1 && Chassis->fast_turning_counter != 0)
	{
		Chassis->vx = ((Chassis->vx_last[0] - Chassis->vx) * Chassis->fast_turning_counter * k_turn * Chassis->fast_turning_counter * k_turn + Chassis->vx)* 0.01f;
		Chassis->vy = ((Chassis->vy_last[0] - Chassis->vy) * Chassis->fast_turning_counter * k_turn * Chassis->fast_turning_counter * k_turn + Chassis->vy)* 0.01f;
		Chassis->fast_turning_counter--;
	}
	else if(Chassis->fast_turning_flag == 1 && Chassis->fast_turning_counter == 0)
	{
		Chassis->fast_turning_flag = 0;
		Chassis->vx_last[1] = Chassis->vx;
		Chassis->vy_last[1] = Chassis->vy;
		Chassis->vx_last[0] = Chassis->vx_last[1];
		Chassis->vy_last[0] = Chassis->vy_last[1];
	}
	if(!Chassis->fast_turning_flag)
	{
		//DMA_printf("%f,%d\r\n",Chassis->vx_last[0],Chassis->fast_turning_flag);
		if(Fabs(Chassis->vx_last[1] - Chassis->vx) < 0.1)
			vx_stable_num++;
		else
			vx_stable_num = 0;
		if(Fabs(Chassis->vy_last[1] - Chassis->vy) < 0.1)
			vy_stable_num++;
		else
			vy_stable_num = 0;
		Chassis->vx_last[1] = Chassis->vx;
		Chassis->vy_last[1] = Chassis->vy;
		if(Chassis->vx == 0 && Chassis->vy == 0)
		{
			Chassis->vx_last[1] = 0.01;
			Chassis->vy_last[1] = 0.01;
		}
		if(vx_stable_num == 100){
			Chassis->vx_last[0] = Chassis->vx_last[1];
			vx_stable_num = 0;
		}
		if(vy_stable_num == 100){
			Chassis->vy_last[0] = Chassis->vy_last[1];
			vy_stable_num = 0;
		}
	}
}
