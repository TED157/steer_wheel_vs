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
#include "math.h"

#include PARAMETER_FILE
#include KEYMAP_FILE
uint8_t resul=0;
extern DMA_HandleTypeDef hdma_usart1_tx;
#define DMA_printf(...)      __HAL_DMA_DISABLE(&hdma_usart1_tx);\
																				HAL_UART_Transmit_DMA(&huart1,\
																				(uint8_t  *)u1_buf,\
																				sprintf((char*)u1_buf,__VA_ARGS__))
extern uint8_t u1_buf[30];

Gimbal_t                Gimbal;//1¤71¤70Ŕ60ü80Á01¤75ú5
Chassis_t               Chassis;//1¤71¤71¤71¤70ü80Á0
RC_ctrl_t               Remote;//0Ű11¤71¤71¤71¤71¤71¤71¤71¤7
AimbotFrame_SCM_t         Aimbot;//1¤71¤71¤71¤71¤71¤71¤71¤7
OfflineMonitor_t        Offline;//1¤71¤71¤71˛81¤71¤75ú51¤71¤7
RefereeInformation_t    Referee;//1¤71¤71¤71¤70Đ30Č11¤71¤71¤71¤7
GimbalRequestState_t RequestStatePacket;//1¤71¤70Ŕ61¤71¤71¤71¤7can1¤71¤7
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


bool_t single_shoot_flag=0;//单发
bool_t auto_fire_flag=1;//自动开火
bool_t switch_flag=0;//弹舱盖
bool_t Yaw_Reverse_Flag=0;//yaw轴转180度
bool_t Chassis_Rotate_Flag=0;//点按小陀螺
uint8_t No_noforce_flag=1;
int16_t dealta_heat=0;
int32_t onelasttime=0;
int16_t onelastheat=0;
uint16_t count=0;

int32_t    gimbal_init_countdown = 0;          //  云台初始化倒计时器
int32_t    gimbal_fire_countdown = 0;          //  云台射击拨盘转动倒计时器
int32_t    gimbal_lagging_counter = 0;         //  云台堵转计数器

fp32       Reverse_Target_Yaw_Angle = 0;        //Yaw_Auto_Reverse状态下的目标角度

GimbalControlMode_e Mode_Last;//上一帧的云台模式
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
uint8_t Yaw_reverse_interval=0;//yaw转180度间的时间间隔
extern uint8_t shoot_flag;
void CalculateThread(void const * pvParameters)
{
	
	uint16_t control_counter = 0;
	
    osDelay(500);
    PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);//左右摩擦轮pid初始化
    PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
//    LoopFifoFp32_init(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.ImuBuffer.YawAddress, 64);//1¤71¤71¤71¤71¤71¤71¤71¤7fifo1¤71¤70ś31¤71¤7
//    LoopFifoFp32_init(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.ImuBuffer.PitchAddress, 64);
    first_order_filter_init(&pitch_aimbot_filter, 1000, &pitch_aimbot_filter_param);//滤波器初始化
	DaMiao_Motor_Init(&DamiaoPitchMotorMeasure);
	//HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
	while(1)
    {
		Remote = *get_remote_control_point();//更新遥控器数据
        Aimbot=*get_usb_aimbot_command_point();//获取自瞄指令
        GetRefereeInformation(&Referee);//获取裁判系统信息 包括枪口的限制
        DeviceOfflineMonitorUpdate(&Offline);//获取模块离线信息
        
//        LoopFifoFp32_push(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.Imu.YawAngle);
//        LoopFifoFp32_push(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.Imu.PitchAngle);//1¤71¤71¤71¤71¤71¤71¤71¤71¤71¤71¤71¤70đ7
        
        GimbalStateMachineUpdate();//根据遥控器拨杆决定当前状态（无力，初始化，测试，比赛）
        ChassisStateMachineUpdate();//底盘状态改变
        GimbalControlModeUpdate();//控制权
        GimbalFireModeUpdate();//开火状态转换
        GimbalPIDUpdate();//云台pid重装载
        RotorPIDUpdate();//拨盘pid重装载
        GimbalMeasureUpdate();//获取电机和imu数据
        GimbalCommandUpdate();//指令的转换
        ChassisCommandUpdate();//底盘指令转换
        RotorCommandUpdate();//拨盘控制转换
//		if(ammo_speed_ad_flag==1){
//			ShootSpeedAdopt();
//			ammo_speed_ad_flag=0;
//			}//031¤71¤71¤71¤71¤7180Š01¤71¤71¤7
        AmmoCommandUpdate();//发射部分控制转化
		
		if(control_counter > 10)
		{
			control_counter = 0;
			GimbalRequestStatePacketSend();//云台指令下发
		}
		control_counter+=1;
			
        
		
		
        DebugLEDShow();
        //BoomBayCover();//1¤71¤71¤70đ40Ł61¤71¤71¤7
				
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
		// 1¤71¤71¤71¤71¤71¤71ą71¤71¤71¤7
//    if(Offline.PitchMotor==DEVICE_OFFLINE||Offline.YawMotor == DEVICE_OFFLINE)
//		{
//        if(Gimbal.StateMachine!=GM_NO_FORCE)
//						Gimbal.StateMachine = GM_NO_FORCE;
//        return;
//    }
    // 遥控器离线保护
    if(Offline.Remote==DEVICE_OFFLINE && Offline.Ft_Remote==DEVICE_OFFLINE)
		{
        if(Gimbal.StateMachine!=GM_NO_FORCE)
						Gimbal.StateMachine=GM_NO_FORCE;
        return;
    }
    
    // 云台状态机
	if(Offline.Remote==0){
		switch (Remote.rc.s[0])
		{
			// 右拨杆打到最上，云台复位后进入比赛模式，该模式下开摩擦轮
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
						Gimbal.StateMachine = GM_MATCH;//比赛模式
					}
				}
				else{
					Gimbal.StateMachine = GM_MATCH;
				}
				break;
			
			// 右拨杆打到中间，云台复位后进入调试模式
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
				
			// 右拨杆打到最下，或遥控器数据出错，云台进入无力模式
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
	//图传链路下云台状态机
	if(!CheakKeyPress(KEY_PRESSED_OFFSET_SHIFT) && CheakKeyPressOnce(KEY_PRESSED_OFFSET_Z)){
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
        Chassis.ChassisState=CHASSIS_NO_FORCE;//云台无力，底盘强制进入无力状态
	if(Gimbal.StateMachine==GM_INIT)
	{
		 if(Remote.rc.s[1]==2)
			Chassis.ChassisState=CHASSIS_FOLLOW;
		 else
			Chassis.ChassisState=CHASSIS_NO_FORCE;
	}
    if(Gimbal.StateMachine==GM_TEST||Gimbal.StateMachine==GM_MATCH)
	{
		if(CHASSIS_ROTATE_KICK_KEYMAP){
			Chassis_Rotate_Flag = (Chassis_Rotate_Flag+1)%2;
		}
		if(Remote.rc.s[1]==2)
				{//左侧拨杆在最下面是底盘有力
            if(CHASSIS_ROTATE_SWITCH_KEYMAP || Chassis_Rotate_Flag || (RemoteDial() == -1.0f && Gimbal.StateMachine == GM_TEST))//小陀螺模式
			{
                Chassis.ChassisState=CHASSIS_ROTATE;
				if(CHASSIS_ROTATE_SWITCH_KEYMAP){
					Chassis_Rotate_Flag = 0;
				}
			}
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
    //模式切换后标志位自动清零
	if(Gimbal.ControlMode != GM_AUTO_REVERSE){
		Yaw_Reverse_Flag = 0;
		Yaw_reverse_interval = Yaw_reverse_interval<250?Yaw_reverse_interval+1:250;
	}
	// 比赛模式下或调试模式
    if(Gimbal.StateMachine==GM_MATCH||Gimbal.StateMachine==GM_TEST)
		{
        // 如果按下鼠标右键or s[1]=1并且视觉发现目标，进入自瞄控制
        if(((Remote.mouse.press_r==PRESS)||(Remote.rc.s[1]==RC_SW_UP))&&(Offline.AimbotDataNode == DEVICE_ONLINE)&&(Aimbot.AimbotState&AIMBOT_TARGET_INSIDE_OFFSET))
		{
			Gimbal.ControlMode = GM_AIMBOT_OPERATE;
		}
        else{
			if(GIMBAL_CMD_YAW_RESERVE__KEYMAP){
				Yaw_Reverse_Flag = Yaw_reverse_interval==250?(Yaw_Reverse_Flag + 1) % 2:0;
				Reverse_Target_Yaw_Angle = Gimbal.Imu.YawAngle>0?Gimbal.Imu.YawAngle-181:Gimbal.Imu.YawAngle+179;
			}
			if(Yaw_Reverse_Flag){
				if(fabs(Gimbal.Imu.YawAngle - Reverse_Target_Yaw_Angle)<5){
					Gimbal.ControlMode = GM_MANUAL_OPERATE;//手动状态
					Yaw_Reverse_Flag = 0;
				}
				else{
					Gimbal.ControlMode = GM_AUTO_REVERSE;//yaw轴自动转180度
					Yaw_reverse_interval = 0;
				}
			}
			else{
				Gimbal.ControlMode = GM_MANUAL_OPERATE;//手动状态
			}
		}
    }	
    if(Gimbal.StateMachine==GM_INIT)
        Gimbal.ControlMode=GM_RESET_POSITION;
    if(Gimbal.StateMachine==GM_NO_CONTROL)
        Gimbal.ControlMode=GM_NO_CONTROL;
}


// qylann: 1¤71¤71¤7071¤71¤71¤70Ŕ91¤71¤7
uint8_t big_rune_flag = 0;
uint8_t small_rune_flag = 0;
extern GimbalRequestState_t RequestStatePacket;
void GimbalFireModeUpdate(void)
{		
	
	
    //自动开火开关,key Q
	  if(FIRE_MODE_KEYMAP) 
        auto_fire_flag=(auto_fire_flag+1)%2;   
	
	//单发开关,key B
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
        if(Gimbal.ControlMode!=GM_AIMBOT_RUNES || !SHOOT_COMMAND_KEYMAP){
			Aimbot_s.Aimbot_Shoot_Flag = 0;
		}
		if(Gimbal.FireMode==GM_FIRE_UNABLE)
            Gimbal.FireMode=GM_FIRE_READY;
        if (Gimbal.FireMode==GM_FIRE_READY) 
				{						
						if((SHOOT_COMMAND_KEYMAP)//收到操作手发弹指令
							&&((Gimbal.ControlMode==GM_AIMBOT_RUNES&&Aimbot_s.Aimbot_Shoot_Flag==1&&auto_fire_flag==1)
								||(Gimbal.ControlMode==GM_AIMBOT_OPERATE&&((Aimbot.AimbotState & 0x02) != 0)&&auto_fire_flag==1)//1¤70é81¤71¤71¤71¤71¤7
						        ||((Gimbal.ControlMode==GM_AIMBOT_OPERATE||Gimbal.ControlMode==GM_AIMBOT_RUNES)&&auto_fire_flag==0)//1¤71¤71¤70é81¤71¤71¤71¤71¤7
								||((Gimbal.ControlMode==GM_MANUAL_OPERATE&&Remote.mouse.press_r!=PRESS)||auto_fire_flag==0))//1¤70ö61¤71¤71¤71¤71¤7
									&&((count*10<=Referee.Ammo0Limit.Cooling+onelastheat&&dealta_heat>20)||Referee.Ammo0Limit.Heat==0xFFFF)	)//1¤71¤71¤71¤71¤71¤71¤70đ71¤71¤71¤71¤71¤7 
						{	
							//DMA_printf("%d\n",GetSystemTimer());
							Aimbot_s.Aimbot_Shoot_Flag = 2;
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
								gimbal_fire_countdown=300;//time interval
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
				
        //异常射击模式的状态机，用于反堵转
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
	else if(CMthis == GM_AUTO_REVERSE){
		cascade_PID_init(   &Gimbal.Pid.Yaw, 
							YAW_ANGLE_AUTO_REVERSE_OPERATE,
                            YAW_SPEED_AUTO_REVERSE_OPERATE,  
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



//int t=10;
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
    if (Gimbal.ControlMode == GM_MANUAL_OPERATE || (__ARM_isnanf(Aimbot.YawRelativeAngle)||__ARM_isnanf(Aimbot.PitchRelativeAngle))){
        Gimbal.Command.Yaw += GIMBAL_CMD_YAW_KEYMAP;
        Gimbal.Command.Pitch += GIMBAL_CMD_PITCH_KEYMAP;
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*rotate_yaw_bias , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);//+Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*100;
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
	else if(Gimbal.ControlMode == GM_AUTO_REVERSE){
//		DMA_printf("%f\n",Gimbal.Pid.Yaw.out);
		if((Gimbal.Pid.Yaw.pid_outside.error[0]>20 || Gimbal.Pid.Yaw.pid_outside.error[0]>-20) && Chassis.ChassisState == CHASSIS_ROTATE)
			Gimbal.Pid.Yaw.pid_outside.Kp=7.4f;
		else if(Gimbal.Pid.Yaw.pid_outside.error[0]>20 || Gimbal.Pid.Yaw.pid_outside.error[0]>-20)
			Gimbal.Pid.Yaw.pid_outside.Kp=9.5f;
		else
			Gimbal.Pid.Yaw.pid_outside.Kp=7.3f;
		 Gimbal.Command.Yaw = Reverse_Target_Yaw_Angle;
        Gimbal.Command.Pitch += GIMBAL_CMD_PITCH_KEYMAP;
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);//-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*rotate_yaw_bias;
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
		Gimbal.Output.DaMiao_Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
	}
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
    if(AMMO_SPEED_UP_KEYMAP)
	{
		ammo_speed_l+=20;
		ammo_speed_r+=20;
	}
	if(AMMO_SPEED_DOWN_KEYMAP)
	{
		ammo_speed_l-=20;
		ammo_speed_r-=20;
	}
	if(ammo_speed_l<6500) ammo_speed_l=6500;
	if(ammo_speed_r<6500) ammo_speed_r=6500;
	if(ammo_speed_l>8000) ammo_speed_l=8000;
	if(ammo_speed_r>8000) ammo_speed_r=8000;
	if (Gimbal.FireMode == GM_FIRE_UNABLE){
        
		if(Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed*AMMO_LEFT_MOTOR_DIRECTION > 2500)
		{
			Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft,
                                        Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                        2000 * AMMO_LEFT_MOTOR_DIRECTION
										);
		}
		if(Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed*AMMO_RIGHT_MOTOR_DIRECTION > 2500){
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
//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1400);//1¤702001¤72245 1¤71¤71¤71¤7 1250    1¤71¤7
//     }
//    if (cover_flag == 2) {
//         __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 600);//1¤702001¤7500   1¤7191¤7
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
     // 1¤71¤71¤71¤71¤71¤71¤71¤71¤71¤71¤70Ý61¤7or s[1]=11¤71¤71¤71¤71¤70ä21¤71¤71¤71¤71¤7071¤76ţ71¤71¤71¤71¤71¤71¤71¤71¤71¤71¤71¤7
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
        // 1¤70ť61¤70ü80Á0
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
//		if(CHASSIS_ROTATE_KICK_KEYMAP){
//			RequestState->ChassisStateRequest |= (uint8_t)(1<<6);
//		}
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
//	Aimbot_Message.TargetX = Aimbot.TargetX;
//	Aimbot_Message.TargetY = Aimbot.TargetY;
    
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
		//1¤71¤71¤71¤71¤71¤7171¤71¤71¤726.5m/s
		if(shoot_speed_now < (shoot_limit - 3.5f) && shoot_speed_now >= (shoot_limit - 7.0f) && shoot_ad_stop_flag==0)
		{
			low_speed_time_num++;
		}
		/*1¤71¤71¤71¤71¤7§Ř1¤7*/ 		/*1¤71¤71¤71¤71¤7§Ř1¤7*/
		if(((shoot_limit - 2.0f) <= shoot_speed_now ) ||low_speed_time_num == 3 )
		{	
			if((shoot_limit - 2.0)<shoot_speed_now)
				{speed_high_flg = (shoot_limit - 2.5 - shoot_speed_now) * 90;}
			else if((shoot_limit - 2.0)>shoot_speed_now)
				{speed_high_flg = (shoot_limit - 2.5 - shoot_speed_now) * 30;}
			low_speed_time_num = 0;		
		}
		/*1¤7§Ř0Đ31¤71¤71¤71¤70˘91¤71¤71¤726.51¤71¤728.00ő81¤71¤7*/
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
		//1¤71¤71¤7ŚÂ1¤71¤71¤71¤71¤71¤71¤71¤71¤71¤71¤71¤71¤71¤71¤7
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
	/*1¤71¤71¤7161¤71¤71¤7 1¤71¤71¤70Č21¤71¤71¤7*/
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
