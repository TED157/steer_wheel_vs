#include "CalculateThread.h"
#include "AttitudeThread.h"
#include "InterruptService.h"
#include "Remote.h"
#include "AimbotCan.h"
#include "user_lib.h"
#include "pid.h"
#include "Motor.h"
#include "RefereeCan.h"
#include "tim.h"
#include "bsp_can.h"
#include "stdio.h"
#include "loop_fifo.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
#include "Infantry4KeyMap.h"
#include "Setting.h"
#include "kalman filter.h"
#include "Usb.h"

#include PARAMETER_FILE
#include KEYMAP_FILE
uint8_t resul=0;
extern DMA_HandleTypeDef hdma_usart1_tx;
#define DMA_printf(...)      __HAL_DMA_DISABLE(&hdma_usart1_tx);\
																				HAL_UART_Transmit_DMA(&huart1,\
																				(uint8_t  *)u1_buf,\
																				sprintf((char*)u1_buf,__VA_ARGS__))
extern uint8_t u1_buf[30];

Gimbal_t                Gimbal;//��̨״̬�ṹ
Chassis_t               Chassis;//����״̬
RC_ctrl_t               Remote;//ң��������
AimbotFrame_SCM_t         Aimbot;//��������
OfflineMonitor_t        Offline;//���߼��ṹ��
RefereeInformation_t    Referee;//����ϵͳ����
GimbalRequestState_t RequestStatePacket;//��̨����can��
Aimbot_Message_t Aimbot_Message;
																				
first_order_filter_type_t  pitch_aimbot_filter;

fp32 pitch_aimbot_filter_param = 0.10f;
fp32 rotate_yaw_bias=0.004777;
void GimbalStateMachineUpdate(void);
void ChassisStateMachineUpdate(void);
void GimbalControlModeUpdate(void);
void GimbalFireModeUpdate(void);
void SetGimbalDisable(void);
void GimbalPIDUpdate(void);
void RotorPIDUpdate(void);
void AmmoPIDUpdate(void);
void GimbalMeasureUpdate(void);
void GimbalCommandUpdate(void);
void ChassisCommandUpdate(void);
void RotorCommandUpdate(void);
void AmmoCommandUpdate(void);
void DebugLEDShow(void);
void GimbalRequestStatePacketSend(void);
//void BoomBayCover(void);
void ShootSpeedAdopt(void);
//int dafu_flag = 0;


bool_t single_shoot_flag=1;//��������
bool_t auto_fire_flag=1;//�Զ����𿪹�
bool_t switch_flag=0;//����л�����
uint8_t No_noforce_flag=1;
int16_t dealta_heat=0;
int32_t onelasttime=0;
int16_t onelastheat=0;
uint16_t count=0;

int32_t    gimbal_init_countdown = 0;          //  ��̨��ʼ������ʱ��
int32_t    gimbal_fire_countdown = 0;          //  ��̨�������ת������ʱ��
int32_t    gimbal_lagging_counter = 0;         //  ��̨��ת������


GimbalControlMode_e Mode_Last;//������һʱ�̵�״̬
fp32 LimitNormalization(fp32 input);
extern ImuPacketNormal_t ImuPacket;
extern ImuPacketMini_t ImuPackageMini;
int16_t minus = 0;
uint8_t pitch_flag=0;
float ammo_speed_l=AMMO_SPEEDSET_30MS_L;
float ammo_speed_r=AMMO_SPEEDSET_30MS_R;
extern uint8_t ammo_speed_ad_flag;
//extern uint8_t rune_shoot_flag;
extern DM_motor_t DamiaoPitchMotorMeasure;
uint16_t shoot_delay=0;
uint8_t auto_cap_flag=0;
void CalculateThread(void const * pvParameters)
{
	
	uint16_t control_counter = 0;
	
    osDelay(500);
    PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);//����Ħ����pid��ʼ��
    PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
//    LoopFifoFp32_init(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.ImuBuffer.YawAddress, 64);//��������fifo��ʼ��
//    LoopFifoFp32_init(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.ImuBuffer.PitchAddress, 64);
    first_order_filter_init(&pitch_aimbot_filter, 1000, &pitch_aimbot_filter_param);//�˲�����ʼ��
	DaMiao_Motor_Init(&DamiaoPitchMotorMeasure);
	//HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
	while(1)
    {
		Remote = *get_remote_control_point();//����ң��������
        Aimbot=*get_usb_aimbot_command_point();//��ȡ����ָ��
        GetRefereeInformation(&Referee);//��ȡ����ϵͳ��Ϣ ����ǹ�ڵ�����
        DeviceOfflineMonitorUpdate(&Offline);//��ȡģ��������Ϣ
        
//        LoopFifoFp32_push(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.Imu.YawAngle);
//        LoopFifoFp32_push(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.Imu.PitchAngle);//������������ջ
        
        GimbalStateMachineUpdate();//����ң�������˾�����ǰ״̬����������ʼ�������ԣ�������
        ChassisStateMachineUpdate();//����״̬�ı�
        GimbalControlModeUpdate();//����Ȩ
        GimbalFireModeUpdate();//����״̬ת��
        GimbalPIDUpdate();//��̨pid��װ��
        RotorPIDUpdate();//����pid��װ��
        GimbalMeasureUpdate();//��ȡ�����imu����
        GimbalCommandUpdate();//ָ���ת��
        ChassisCommandUpdate();//����ָ��ת��
        RotorCommandUpdate();//���̿���ת��
//		if(ammo_speed_ad_flag==1){
//			ShootSpeedAdopt();
//			ammo_speed_ad_flag=0;
//			}//Ħ�����ٶȵ���
        AmmoCommandUpdate();//���䲿�ֿ���ת��
		
		if(control_counter > 10)
		{
			control_counter = 0;
			GimbalRequestStatePacketSend();//��ָ̨���·�
		}
		control_counter+=1;
			
        
		
		
        DebugLEDShow();
        //BoomBayCover();//���ոǿ���
				
				minus = Aimbot.SystemTimer - ImuPacket.TimeStamp;


				GimbalMotorControl( Gimbal.Output.Yaw * YAW_MOTOR_DIRECTION ,
                            Gimbal.Output.Pitch * PITCH_MOTOR_DIRECTION, 
                            Gimbal.Output.Rotor, //Gimbal.Output.Rotor
                            Gimbal.Output.AmmoLeft,
                            Gimbal.Output.AmmoRight
                        );
				DaMiaoCanSend(Gimbal.Output.DaMiao_Pitch * DAMIAO_PITCH_MOTOR_DIRECTION-0.5);
				osDelay(1);
    }
}

void GimbalStateMachineUpdate(void)
{
		// ������߱���
//    if(Offline.PitchMotor==DEVICE_OFFLINE||Offline.YawMotor == DEVICE_OFFLINE)
//		{
//        if(Gimbal.StateMachine!=GM_NO_FORCE)
//						Gimbal.StateMachine = GM_NO_FORCE;
//        return;
//    }
    // ң�������߱���
    if(Offline.Remote==DEVICE_OFFLINE && Offline.Ft_Remote==DEVICE_OFFLINE)
		{
        if(Gimbal.StateMachine!=GM_NO_FORCE)
						Gimbal.StateMachine=GM_NO_FORCE;
        return;
    }
    
    // ��̨״̬��
	if(Offline.Remote==0){
		switch (Remote.rc.s[0])
			{
			// �Ҳ��˴����ϣ���̨��λ��������ģʽ����ģʽ�¿�Ħ����
			case RC_SW_UP:
				if (Gimbal.StateMachine == GM_NO_FORCE)
							{
					Gimbal.StateMachine = GM_INIT;
					gimbal_init_countdown = 800;
				}
				else if (Gimbal.StateMachine == GM_INIT)
				{
					if (gimbal_init_countdown > 0){
						gimbal_init_countdown--;
					}
					else{
						Gimbal.StateMachine = GM_MATCH;//����ģʽ
					}
				}
				else{
					Gimbal.StateMachine = GM_MATCH;
				}
				break;
			
			// �Ҳ��˴��м䣬��̨��λ��������ģʽ
			case RC_SW_MID:
				if (Gimbal.StateMachine == GM_NO_FORCE){
					Gimbal.StateMachine = GM_INIT;
					gimbal_init_countdown = 800;
				}
				else if (Gimbal.StateMachine == GM_INIT)
				{
					if (gimbal_init_countdown > 0){
						gimbal_init_countdown--;
					}
					else{
						Gimbal.StateMachine = GM_TEST;
					}
				}
				else{
					Gimbal.StateMachine = GM_TEST;
				}
				break;
				
			// �Ҳ��˴����£���ң�������ݳ�������̨��������ģʽ
			case RC_SW_DOWN:
				if (Gimbal.StateMachine != GM_NO_FORCE){
					Gimbal.StateMachine = GM_NO_FORCE;
				}
				break;
			default:
				if (Gimbal.StateMachine != GM_NO_FORCE && Offline.Ft_Remote==DEVICE_OFFLINE){
					Gimbal.StateMachine = GM_NO_FORCE;
				}
				break;
		}
	}
	if(CheakKeyPressOnce(KEY_PRESSED_OFFSET_Z)){
		No_noforce_flag=(No_noforce_flag+1)%2;
	}
	if(No_noforce_flag && Offline.Remote==DEVICE_OFFLINE)
	{
		Gimbal.StateMachine=GM_NO_FORCE;
	}
	else if(!No_noforce_flag && Offline.Remote==DEVICE_OFFLINE){
		if (Gimbal.StateMachine == GM_NO_FORCE)
		{
            Gimbal.StateMachine = GM_INIT;
            gimbal_init_countdown = 800;
		}
        else if (Gimbal.StateMachine == GM_INIT)
        {
			if (gimbal_init_countdown > 0){
				gimbal_init_countdown--;
			}
            else{
                Gimbal.StateMachine = GM_MATCH;
            }
		}
        else{
            Gimbal.StateMachine = GM_MATCH;
        }
		Remote.rc.s[1]=2;
	}
	if(Gimbal.StateMachine == GM_MATCH)
		HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
}

void ChassisStateMachineUpdate(void)
{   
	
    //if ((Gimbal.StateMachine == GM_NO_FORCE)  ||  (Gimbal.StateMachine == GM_INIT)) {
		if((Gimbal.StateMachine==GM_NO_FORCE))
        Chassis.ChassisState=CHASSIS_NO_FORCE;//��̨����������ǿ�ƽ�������״̬
		if(Gimbal.StateMachine==GM_INIT)
		{
			 if(Remote.rc.s[1]==2)
					Chassis.ChassisState=CHASSIS_FOLLOW;
			 else
					Chassis.ChassisState=CHASSIS_NO_FORCE;
		}
    if(Gimbal.StateMachine==GM_TEST||Gimbal.StateMachine==GM_MATCH)
		{
		if(Remote.rc.s[1]==2)
				{//��ದ�����������ǵ�������
            if(CHASSIS_ROTATE_SWITCH_KEYMAP || (RemoteDial() == -1.0f && Gimbal.StateMachine == GM_TEST))//С����ģʽ
                Chassis.ChassisState=CHASSIS_ROTATE;
			else if(CHASSIS_ROTATE_RESERVE_KEYMAP && Gimbal.StateMachine==GM_TEST)
				Chassis.ChassisState=CHASSIS_ROTATE_RESERVE;
						else 
								if(CHASSIS_STOP_KEYMAP)
										Chassis.ChassisState=CHASSIS_NO_MOVE;
								else
										Chassis.ChassisState=CHASSIS_FOLLOW;												
            if(CHASSIS_HIGH_SPEED_KEYMAP || (RemoteDial() == -1.0f && Gimbal.StateMachine == GM_MATCH))
                Chassis.ChassisSpeed=CHASSIS_FAST_SPEED;
            else
                Chassis.ChassisSpeed=CHASSIS_NORMAL_SPEED;
        }
        else
            Chassis.ChassisState=CHASSIS_NO_FORCE;
    }
		if(AUTO_CAP_KEYMAP)
		{
			auto_cap_flag=(auto_cap_flag+1)%2;
		}
}
void SetGimbalDisable(void)
{
    Gimbal.StateMachine = GM_NO_FORCE;
    Gimbal.ControlMode = GM_NO_CONTROL;
    Gimbal.FireMode = GM_FIRE_UNABLE;
}


void GimbalControlModeUpdate(void)
{
    // ����ģʽ��
    if(Gimbal.StateMachine==GM_MATCH||Gimbal.StateMachine==GM_TEST)
		{
        // �����������Ҽ�or s[1]=1�����Ӿ�����Ŀ�꣬�����������
        if(((Remote.mouse.press_r==PRESS)||(Remote.rc.s[1]==RC_SW_UP))&&(Offline.AimbotDataNode == DEVICE_ONLINE)&&(Aimbot.AimbotState&AIMBOT_TARGET_INSIDE_OFFSET))
				{
					Gimbal.ControlMode = GM_AIMBOT_OPERATE;
				}
        else
            Gimbal.ControlMode = GM_MANUAL_OPERATE;//�ֶ�״̬
    }	
    if(Gimbal.StateMachine==GM_INIT)
        Gimbal.ControlMode=GM_RESET_POSITION;
    if(Gimbal.StateMachine==GM_NO_CONTROL)
        Gimbal.ControlMode=GM_NO_CONTROL;
}


// qylann: ���´���̫��
uint8_t big_rune_flag = 0;
uint8_t small_rune_flag = 0;
uint32_t time=0;
extern GimbalRequestState_t RequestStatePacket;
void GimbalFireModeUpdate(void)
{		
	
	
    //�Զ����𿪹�,key Q
	  if(FIRE_MODE_KEYMAP) 
        auto_fire_flag=(auto_fire_flag+1)%2;   
	
		//��������,key B
	  if(BIG_RUNE_KEYMAP&&((Remote.mouse.press_r==PRESS)||(Remote.rc.s[1]==RC_SW_UP))) 
	{
		small_rune_flag = 0;
		 big_rune_flag=1;//=(big_rune_flag+1)%2; 
	}
	else if(SMALL_RUNE_KEYMAP&&((Remote.mouse.press_r==PRESS)||(Remote.rc.s[1]==RC_SW_UP)))
	{	
		big_rune_flag = 0;
		 small_rune_flag=1;//(small_rune_flag+1)%2;
	}
	else
	{
		small_rune_flag = 0;
		 big_rune_flag=0;
	}
//	if(big_rune_flag){
//		DMA_printf("%d\n",big_rune_flag);
//	}
	  if( (big_rune_flag|| small_rune_flag)&&(Gimbal.StateMachine==GM_MATCH||Gimbal.StateMachine==GM_TEST)){
		  single_shoot_flag = 1;
		  Gimbal.ControlMode = GM_AIMBOT_RUNES;
	  }
	  else
		{
			if(SINGLE_SHOOT_KEMAP)  //key E
			single_shoot_flag=(single_shoot_flag+1)%2;			
		}	
		if(Mode_Last==GM_AIMBOT_RUNES && Gimbal.ControlMode!=GM_AIMBOT_RUNES)
		{
			single_shoot_flag=0;
		}
	Mode_Last=Gimbal.ControlMode;
		dealta_heat=Referee.Ammo0Limit.Heat-Referee.Realtime.Ammo0Heat;
		if(GetSystemTimer()-onelasttime>=1000)
		{
				onelasttime=GetSystemTimer(),
				onelastheat=dealta_heat,
				count=0; 
		}

		if(Gimbal.StateMachine!=GM_MATCH)
		{
				Gimbal.FireMode=GM_FIRE_UNABLE;
				gimbal_fire_countdown=0;
		}
    if(Gimbal.StateMachine==GM_MATCH) 
	{
		// if((Aimbot.AimbotState & 0x02) != 0){
		// 	HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_SET);
		// }
		// else{
		// 	HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_RESET);
		// }
        if(Gimbal.FireMode==GM_FIRE_UNABLE)
            Gimbal.FireMode=GM_FIRE_READY;
        if (Gimbal.FireMode==GM_FIRE_READY) 
				{						
						if((SHOOT_COMMAND_KEYMAP)//�յ������ַ���ָ��
							&&((Gimbal.ControlMode==GM_AIMBOT_RUNES&&((Aimbot.AimbotState & 0x02) != 0)&&auto_fire_flag==1)
								||(Gimbal.ControlMode==GM_AIMBOT_OPERATE&&((Aimbot.AimbotState & 0x02) != 0)&&auto_fire_flag==1)//�Զ�����
						        ||((Gimbal.ControlMode==GM_AIMBOT_OPERATE||Gimbal.ControlMode==GM_AIMBOT_RUNES)&&auto_fire_flag==0)//���Զ�����
								||((Gimbal.ControlMode==GM_MANUAL_OPERATE&&Remote.mouse.press_r!=PRESS)||auto_fire_flag==0))//�ֶ�����
									&&((count*10<=Referee.Ammo0Limit.Cooling+onelastheat&&dealta_heat>20)||Referee.Ammo0Limit.Heat==0xFFFF)	)//�������ջ����� 
						{	
							//DMA_printf("%d\n",GetSystemTimer());
							//rune_shoot_flag=0;
							Gimbal.FireMode=GM_FIRE_BUSY;									
								gimbal_fire_countdown=ROTOR_TIMESET_BUSY;
								if(single_shoot_flag)
									gimbal_fire_countdown=135;
								count++;
						}
        }
				if(Gimbal.FireMode==GM_FIRE_BUSY&&gimbal_fire_countdown<=0)
				{
						if(single_shoot_flag==1||Offline.RefereeAmmoLimitNode0==1)
								gimbal_fire_countdown=450;//450;//time interval
						else 
								gimbal_fire_countdown=(int)(10000.0/(dealta_heat/1.4+Referee.Ammo0Limit.Cooling/1.8+5)-45);
						Gimbal.FireMode=GM_FIRE_COOLING; //no shoot
				}
//				if(Gimbal.FireMode==GM_FIRE_COOLING && gimbal_fire_countdown>0  && gimbal_fire_countdown<0 && rune_shoot_flag<1 && Gimbal.ControlMode==GM_AIMBOT_RUNES)
//				{	gimbal_fire_countdown=57;
//					Gimbal.FireMode=GM_FIRE_BUSY;
//					rune_shoot_flag++;
//				}
				if(Gimbal.FireMode==GM_FIRE_COOLING&&gimbal_fire_countdown<=0) 
						Gimbal.FireMode=GM_FIRE_READY;    
				
        //  �쳣���ģʽ��״̬�������ڷ���ת
				if(Gimbal.FireMode==GM_FIRE_LAGGING)
				{
						if(gimbal_fire_countdown<=0)
								Gimbal.FireMode=GM_FIRE_READY;  
				}
				else
				{
						if ((Gimbal.FireMode == GM_FIRE_BUSY)&&(Gimbal.MotorMeasure.ShootMotor.RotorMotorSpeed<400))
								gimbal_lagging_counter++;
						else
								gimbal_lagging_counter=0;      
						if (gimbal_lagging_counter>ROTOR_LAGGING_COUNTER_MAX)//ROTOR_LAGGING_COUNTER_MAX
						{        
								gimbal_lagging_counter=0;
								gimbal_fire_countdown=ROTOR_TIMESET_RESERVE;
								Gimbal.FireMode=GM_FIRE_LAGGING;
						}
				}				
				gimbal_fire_countdown--;
		}
}

// qylann: "     "


GimbalControlMode_e CMthis = GM_NO_CONTROL;
GimbalControlMode_e CMlast = GM_NO_CONTROL;

void GimbalPIDUpdate(void)
{
    CMthis = Gimbal.ControlMode;
    
    if (CMthis == CMlast){
        return;
    }
    
    
    //  
    
    if (CMthis == GM_MANUAL_OPERATE){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_MANUAL_OPERATE, 
                            YAW_SPEED_MANUAL_OPERATE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_MANUAL_OPERATE, 
                            PITCH_SPEED_MANUAL_OPERATE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            PITCH_MAX_OUTPUT, 
                            PITCH_MAX_IOUTPUT
                            );
    }
    else if (CMthis == GM_AIMBOT_OPERATE){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_AIMBOT_OPERATE, 
                            YAW_SPEED_AIMBOT_OPERATE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_AIMBOT_OPERATE, 
                            PITCH_SPEED_AIMBOT_OPERATE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            PITCH_MAX_OUTPUT, 
                            PITCH_MAX_IOUTPUT
                            );
    }
    else if (CMthis == GM_AIMBOT_RUNES){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_AIMBOT_RUNES, 
                            YAW_SPEED_AIMBOT_RUNES, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_AIMBOT_RUNES, 
                            PITCH_SPEED_AIMBOT_RUNES, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            PITCH_MAX_OUTPUT, 
                            PITCH_MAX_IOUTPUT
                            );
    }
    else if (CMthis == GM_RESET_POSITION){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_RESET_POSITION, 
                            YAW_SPEED_RESET_POSITION, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_RESET_POSITION, 
                            PITCH_SPEED_RESET_POSITION, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            PITCH_MAX_OUTPUT, 
                            PITCH_MAX_IOUTPUT
                            );
    }
    else{
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_NO_FORCE, 
                            YAW_SPEED_NO_FORCE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_NO_FORCE, 
                            PITCH_SPEED_NO_FORCE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            PITCH_MAX_OUTPUT, 
                            PITCH_MAX_IOUTPUT
                            );
    }
    
    CMlast = CMthis;
}





GimbalFireMode_e FMthis = GM_FIRE_UNABLE;
GimbalFireMode_e FMlast = GM_FIRE_UNABLE;
void RotorPIDUpdate(void)
{
    FMthis = Gimbal.FireMode;
    
    if (FMthis == FMlast){
        return;
    }
    
    //
    
    if ((FMthis == GM_FIRE_READY)  ||  (FMthis == GM_FIRE_COOLING)){
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_STOP, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    else if (FMthis == GM_FIRE_BUSY){
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_FORWARD, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    else if (FMthis == GM_FIRE_LAGGING){
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_BACK, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    else{
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_UNABLE, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    
    FMlast = FMthis;
}

void GimbalMeasureUpdate(void)
{
    GimbalMotorMeasureUpdate(&Gimbal.MotorMeasure.GimbalMotor);
    ShootMotorMeasureUpdate(&Gimbal.MotorMeasure.ShootMotor);
    GimbalEulerSystemMeasureUpdate(&Gimbal.Imu);
}



int t=10;

fp32 aimbot_pitch_bias = 0;

void GimbalCommandUpdate(void)
{
	if (AIMBOT_PITCH_BIAS_LOW_KEYMAP) {
		aimbot_pitch_bias += 0.3;
	}
	else if (AIMBOT_PITCH_BIAS_HIGH_KEYMAP) {
		aimbot_pitch_bias -= 0.3;
	}
	else if (AIMBOT_PITCH_BIAS_ZERO_KEYMAP) {
		aimbot_pitch_bias = 0;
	}
	
	if (aimbot_pitch_bias > 6) {
		aimbot_pitch_bias = 6;
	}
	else if (aimbot_pitch_bias < -6) {
		aimbot_pitch_bias = -6;
	}
    if (Gimbal.ControlMode == GM_MANUAL_OPERATE){
        Gimbal.Command.Yaw += GIMBAL_CMD_YAW_KEYMAP;
        Gimbal.Command.Pitch += GIMBAL_CMD_PITCH_KEYMAP;
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*rotate_yaw_bias , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);//-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*rotate_yaw_bias;
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
		Gimbal.Output.DaMiao_Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
//		if(Gimbal.StateMachine ==GM_MATCH) {
//			Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, pitch_kf.x, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
//		}
		pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
    else if (Gimbal.ControlMode == GM_AIMBOT_OPERATE || (Gimbal.ControlMode == GM_AIMBOT_RUNES)){
		if((Gimbal.ControlMode == GM_AIMBOT_RUNES) && (Aimbot.AimbotState&1)==0)
		{
			Gimbal.Command.Yaw += GIMBAL_CMD_YAW_KEYMAP;
			Gimbal.Command.Pitch += GIMBAL_CMD_PITCH_KEYMAP;
		}
		else{
			Gimbal.Command.Yaw = Aimbot.YawRelativeAngle;
			Gimbal.Command.Pitch = Aimbot.PitchRelativeAngle;
		}
//        Gimbal.Command.Yaw = LoopFifoFp32_read(&Gimbal.ImuBuffer.YawLoopPointer, (GetSystemTimer() - Aimbot.SystemTimer+t)) + Aimbot.YawRelativeAngle;
//        Gimbal.Command.Pitch = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.SystemTimer+t)) + Aimbot.PitchRelativeAngle + aimbot_pitch_bias;
//        fp32 pitch_command = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.PitchRelativeAngle;
//        first_order_filter_cali(&pitch_aimbot_filter, pitch_command);
//        Gimbal.Command.Pitch = pitch_aimbot_filter.out;
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*rotate_yaw_bias , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
		Gimbal.Output.DaMiao_Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
    } 
//    else if (Gimbal.ControlMode == GM_AIMBOT_RUNES){
//        
//        Gimbal.Command.Yaw = Aimbot.YawRelativeAngle;
//        Gimbal.Command.Pitch = Aimbot.PitchRelativeAngle;
////        Gimbal.Command.Yaw = LoopFifoFp32_read(&Gimbal.ImuBuffer.YawLoopPointer, (GetSystemTimer() - Aimbot.SystemTimer)) + Aimbot.YawRelativeAngle;
////        Gimbal.Command.Pitch = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.SystemTimer)) + Aimbot.PitchRelativeAngle + aimbot_pitch_bias;
//        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
//        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
//        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*0.0132 , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
//        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);

//    }
    else if (Gimbal.ControlMode == GM_RESET_POSITION){
        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle;
        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle;
        fp32 YawTempCommand = loop_fp32_constrain(YAW_ZERO_ECDANGLE, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle - 180.0f, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle + 180.0f);
//        Gimbal.Output.Yaw = YAW_MOTOR_DIRECTION * cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle, Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed, YAW_ZERO_ECDANGLE);
        Gimbal.Pid.Yaw.v_set = PID_calc(&Gimbal.Pid.Yaw.pid_outside, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle, YawTempCommand);
        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, 0);
        Gimbal.Output.DaMiao_Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, 0);
		pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
    else{
        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle;
        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle;
        Gimbal.Output.Yaw = 0;
        Gimbal.Output.Pitch = 0;
		Gimbal.Output.DaMiao_Pitch = 0;
        pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
}

void ChassisCommandUpdate(void)
{
    if ((Chassis.ChassisState == CHASSIS_NO_FORCE) ) {
        Chassis.ChassisCommandX = 0.0f;
        Chassis.ChassisCommandY = 0.0f;
    }
    else {
        Chassis.ChassisCommandX = CHASSIS_CMD_X_KEYMAP;
        Chassis.ChassisCommandY = -CHASSIS_CMD_Y_KEYMAP;
    }
}

void RotorCommandUpdate(void)
{
    if (Gimbal.FireMode == GM_FIRE_BUSY){  
		Gimbal.Command.Rotor = ROTOR_SPEEDSET_FORWARD * ROTOR_MOTOR_DIRECTION;
		if(single_shoot_flag)
			Gimbal.Command.Rotor = ROTOR_MAX_SPEED  * ROTOR_MOTOR_DIRECTION;
    }
    else if (Gimbal.FireMode == GM_FIRE_LAGGING){
        Gimbal.Command.Rotor = ROTOR_SPEEDSET_BACKWARD * ROTOR_MOTOR_DIRECTION;

    }
    else if (Gimbal.FireMode == GM_FIRE_UNABLE){
        Gimbal.Command.Rotor = 0;
        Gimbal.Output.Rotor = 0;
        return;
    }
    else{
        Gimbal.Command.Rotor = 0;
		
    }
    
    Gimbal.Output.Rotor = PID_calc(&Gimbal.Pid.Rotor, Gimbal.MotorMeasure.ShootMotor.RotorMotorSpeed, Gimbal.Command.Rotor);
}






void AmmoCommandUpdate(void)
{
    if (Gimbal.FireMode == GM_FIRE_UNABLE){
        
		if(Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed < 2500*AMMO_LEFT_MOTOR_DIRECTION)
		{
			Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft,
                                        Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                        2000 * AMMO_LEFT_MOTOR_DIRECTION
										);
		}
		if(Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed > 2500*AMMO_RIGHT_MOTOR_DIRECTION){
			Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                        Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                        2000 * AMMO_RIGHT_MOTOR_DIRECTION
											);
		}
		else{
			Gimbal.Command.AmmoLeft = 0;
			Gimbal.Command.AmmoRight = 0;
			Gimbal.Output.AmmoLeft = 0;
			Gimbal.Output.AmmoRight = 0;
		}
        return;
    }
	if(CheakKeyPressOnce(KEY_PRESSED_OFFSET_X))
	{
		ammo_speed_l+=20;
		ammo_speed_r+=20;
	}
	if(CheakKeyPressOnce(KEY_PRESSED_OFFSET_V))
	{
		ammo_speed_l-=20;
		ammo_speed_r-=20;
	}
	if(ammo_speed_l<6500) ammo_speed_l=6500;
	if(ammo_speed_r<6500) ammo_speed_r=6500;
	if(ammo_speed_l>8000) ammo_speed_l=8000;
	if(ammo_speed_r>8000) ammo_speed_r=8000;
    Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft,
                                        Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                        ammo_speed_l * AMMO_LEFT_MOTOR_DIRECTION
                                      );
    Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                        Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                        ammo_speed_r * AMMO_RIGHT_MOTOR_DIRECTION
                                        );
//    if(Gimbal.StateMachine != GM_MATCH){
//		Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft,
//                                        Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
//                                        0
//                                      );
//    Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
//                                        Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
//                                        0
//                                        );
//	}
    
}


void GetGimbalMotorOutput(GimbalOutput_t *out)
{
    memcpy(out, &Gimbal.Output, sizeof(GimbalOutput_t));
}

//bool_t cover_flag = 0;
//void BoomBayCover(void)
//{
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2000);
//		
//    if (Gimbal.StateMachine == GM_MATCH) {
//        if (cover_flag == 0) {
//            cover_flag = 2;
//            
//        }
//        
//        if (COVER_SWITCH_KEYMAP) {
//            if (cover_flag == 1) {
//                cover_flag = 2;
//                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
//            }
//            else if (cover_flag == 2) {
//                cover_flag = 1;
//                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
//            }
//        }
//    }
//    else {
//        if (SHOOT_COMMAND_KEYMAP) {
//              cover_flag = 1;
//                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
//        }
//		else{
//		 cover_flag = 2;
//                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
//		}

//    }
//    // HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
//		
//   
//    if ((cover_flag == 1)  ||  (cover_flag == 0)) {
//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1400);//�ĺţ�2245 ���� 1250    ��
//     }
//    if (cover_flag == 2) {
//         __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 600);//�ĺţ�500   �ر�
//    }
//    
//    
//    
//}



void GetGimbalRequestState(GimbalRequestState_t *RequestState)
{
	   if (Gimbal.StateMachine == GM_NO_FORCE) {
         RequestState->GimbalState |= (uint8_t)(1 << 0);
    }
	
	
	GimabalImu.mode =0x00;
    RequestState->AimbotRequest = 0x00;
	if(Gimbal.StateMachine==GM_MATCH||Gimbal.StateMachine==GM_TEST)
	{
     // �����������Ҽ�or s[1]=1�����Ӿ�����Ŀ�꣬�����������
		if(((Remote.mouse.press_r==PRESS)||(Remote.rc.s[1]==RC_SW_UP)))
		{
			GimabalImu.mode |= (uint8_t)(1 << 0);
		}
	}
	if(small_rune_flag){		
		RequestState->AimbotRequest |= (uint8_t)(1 << 4);
		GimabalImu.mode |= (uint8_t)(1 << 3);
	}
	else if(big_rune_flag){
		RequestState->AimbotRequest |= (uint8_t)(1 << 5);
		GimabalImu.mode |= (uint8_t)(1 << 2);
	}
	else if(Gimbal.ControlMode==GM_AIMBOT_OPERATE){
		RequestState->AimbotRequest |= (uint8_t) (1 << 0);
		GimabalImu.mode |= (uint8_t)(1 << 1);
	}
	if(single_shoot_flag){
		RequestState->AimbotRequest |= (uint8_t) (1 << 1);
	}

    
    
     
    
    
    
    RequestState->ChassisMoveXRequest = Chassis.ChassisCommandX * 32767;
    RequestState->ChassisMoveYRequest = Chassis.ChassisCommandY * 32767;
    RequestState->ChassisStateRequest = 0x00;

    
    
    if (Chassis.ChassisState != CHASSIS_NO_FORCE) {
        RequestState->ChassisStateRequest |= (uint8_t)(1 << 1);
        // �˶�״̬
        if (Chassis.ChassisState == CHASSIS_NO_MOVE) {
            RequestState->ChassisStateRequest |= (uint8_t)(1 << 2);
        }
        else if (Chassis.ChassisState == CHASSIS_FOLLOW) {
            RequestState->ChassisStateRequest |= (uint8_t)(1 << 3);
        }
        else if (Chassis.ChassisState == CHASSIS_ROTATE) {
            RequestState->ChassisStateRequest |= (uint8_t)(1 << 4);
        }
       
		if(Chassis.ChassisSpeed == CHASSIS_FAST_SPEED){
			RequestState->ChassisStateRequest |= (uint8_t)(1<<5);
		}
		if(CHASSIS_HIGH_SPEED_ROTATE){
			RequestState->ChassisStateRequest |= (uint8_t)(1<<6);
		}
		if(Chassis.ChassisState == CHASSIS_ROTATE_RESERVE)
			RequestState->ChassisStateRequest |= (uint8_t)(1<<7);
    }
    else {
        RequestState->ChassisStateRequest |= (uint8_t)(1 << 0);
    }
    
    RequestState->GimbalState = 0x00;
    
    
     
    if ((Remote.mouse.press_r == PRESS)  ||  (Remote.rc.s[1] == RC_SW_UP)) {
        RequestState->GimbalState |= (uint8_t) (1 << 1);
    }
    
    if (auto_cap_flag) {
        RequestState->GimbalState |= (uint8_t) (1 << 4);
    }
	
		if(auto_fire_flag == 1){
			RequestState->GimbalState |= (uint8_t) (1 << 6);
		}
		if(Gimbal.StateMachine==GM_MATCH)
		{
			RequestState->GimbalState |= (uint8_t) (1 << 7);
		}
		
	GimabalImu.robot_id=Referee.RobotState.RobotID;
        
    Aimbot_Message.AimbotState = Aimbot.AimbotState;
	Aimbot_Message.AimbotTarget = Aimbot.AimbotTarget;
	Aimbot_Message.TargetX = Aimbot.TargetX;
	Aimbot_Message.TargetY = Aimbot.TargetY;
    
    RequestState->Reserve = 0x00;
    
}



void DebugLEDShow(void)
{
    if (Offline.AimbotDataNode == DEVICE_ONLINE){
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
        if ((Aimbot.AimbotState & AIMBOT_TARGET_INSIDE_OFFSET)){
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        }
        else{
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        }
    }
    else{
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    }
    
    
}


fp32 LimitNormalization(fp32 input)
{
    if (input > 1.0f){
        return 1.0f;
    }
    else if (input < -1.0f){
        return -1.0f;
    }
    else{
        return input;
    }
}



void RefereeHeatInterpolation(void)
{
    Referee.Realtime.Ammo0Heat -= Referee.Ammo0Limit.Cooling / 10;
    if (Referee.Realtime.Ammo0Heat < 0) {
        Referee.Realtime.Ammo0Heat = 0;
    }
}


void GimbalRequestStatePacketSend(void)
{
    GetGimbalRequestState(&RequestStatePacket);
    CanSendMessage(&COMMUNICATE_CANPORT, GINBAL_REQUEST_STATE_ID, 8, (uint8_t *)&RequestStatePacket);
}

fp32 shoot_speed_last;
fp32 shoot_speed_now;
fp32 shoot_limit=30;
fp32 speed_high_flg;
float shoot_adot=0;
char speed_dec_flag = 0;
char speed_add_flag = 0;
char low_speed_time_num = 0;
uint8_t shoot_ad_stop_flag=0;
void ShootSpeedAdopt(void)
{
	shoot_speed_now=Referee.Ammo0Speed;
	if(shoot_speed_last!=shoot_speed_now)
	{
		//������ٵ���26.5m/s
		if(shoot_speed_now < (shoot_limit - 3.5f) && shoot_speed_now >= (shoot_limit - 7.0f) && shoot_ad_stop_flag==0)
		{
			low_speed_time_num++;
		}
		/*�����ж�*/ 		/*�����ж�*/
		if(((shoot_limit - 2.0f) <= shoot_speed_now ) ||low_speed_time_num == 3 )
		{	
			if((shoot_limit - 2.0)<shoot_speed_now)
				{speed_high_flg = (shoot_limit - 2.5 - shoot_speed_now) * 90;}
			else if((shoot_limit - 2.0)>shoot_speed_now)
				{speed_high_flg = (shoot_limit - 2.5 - shoot_speed_now) * 30;}
			low_speed_time_num = 0;		
		}
		/*�жϵ����Ƿ���26.5��28.0֮��*/
		if(shoot_speed_now >= (shoot_limit - 2.0f))
		{
			speed_dec_flag ++;
		    speed_add_flag = 0;
		}
		if(shoot_speed_now <= (shoot_limit - 3.5f))
		{
			speed_dec_flag = 0;
			speed_add_flag++;
		}
//		if(shoot_speed_now >= (shoot_limit - 2.5f)){
//			shoot_ad_stop_flag=1;
//		}
//		if(shoot_speed_now >= (shoot_limit - 4.0f))
//		{
//			shoot_ad_stop_flag=0;
//		}
//		if(shoot_ad_stop_flag)
//			speed_add_flag=0;
		//���β���������������
		if(speed_dec_flag == 3)
		{
			shoot_adot++;
			speed_dec_flag = 0;
			speed_add_flag = 0;		
		}
		if(speed_add_flag == 3)
		{
			shoot_adot--;
			speed_dec_flag = 0;
			speed_add_flag = 0;		
		}
	}
	if(shoot_speed_now>30)
	{
		shoot_delay=1000;
	}
	if(shoot_delay>0)
	{
		shoot_delay--;
		Gimbal.Output.Rotor=0;
	}
	shoot_speed_last= shoot_speed_now;
	/*���ٴ��� ���ʹ���*/
	if(Gimbal.StateMachine == GM_MATCH){
	ammo_speed_l = ammo_speed_l + speed_high_flg;
	ammo_speed_r = ammo_speed_r + speed_high_flg;
	ammo_speed_l = ammo_speed_l - shoot_adot * 10;
	ammo_speed_r = ammo_speed_r - shoot_adot * 10;
	}
	if(shoot_adot>2) shoot_adot=0;
	if(ammo_speed_l<6850) ammo_speed_l=6850;
	else if(ammo_speed_l>7600) ammo_speed_l=7600;
	if(ammo_speed_r<6850) ammo_speed_r=6850;
	else if(ammo_speed_l>7600) ammo_speed_l=7600;
}
