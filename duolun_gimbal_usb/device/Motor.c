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

#include "Motor.h"
#include "bsp_can.h"

#include "Setting.h"
#include "InterruptService.h"
#include "pid.h"
#include "CalculateThread.h"
#define DaMiao_Motor_Enable_Message(message)              for(uint8_t i=0;i<7;i++) message[i]=0xff; message[7]=0xfc;     
#define DaMiao_Motor_Disable_Message(message)             for(uint8_t i=0;i<7;i++) message[i]=0xff; message[7]=0xfd;
#define DaMiao_Motor_Error_Clear_Message(message)         for(uint8_t i=0;i<7;i++) message[i]=0xff; message[7]=0xfb;
#define DaMiao_Motor_ZERO_SET_Message(message)            for(uint8_t i=0;i<7;i++) message[i]=0xff; message[7]=0xfe;

#if defined GREEN_STEERWHEEL
#define DAMIAO_MAX_ANGLE 2.35
#define DAMIAO_MIN_ANGLE 1.61
float tor=-0.8;
#elif defined YELLOW_STEERWHEEL
#define DAMIAO_MAX_ANGLE 0.04
#define DAMIAO_MIN_ANGLE -0.80
float tor=-0.5;
#elif defined BLACK_STEERWHEEL
#define DAMIAO_MAX_ANGLE 0
#define DAMIAO_MIN_ANGLE 0
float tor=0;
#endif
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern OfflineMonitor_t OfflineMonitor;
extern Gimbal_t                Gimbal;
static uint32_t             send_mail_box;
static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t              MotorSendBuffer[16];
uint8_t              damiao_data[8] = {0};
static uint8_t              MotorBusPosition[8] = {0};
uint8_t                     DaMiao_message[8];

DM_motor_t DamiaoPitchMotorMeasure;

void GimbalMotorControl(int16_t YawMotor, int16_t PitchMotor, int16_t RotorMotor, int16_t AmmoLeftMotor, int16_t AmmoRightMotor);
void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t* message);
void GimbalMotorMeasureUpdate(GimbalMotorMeasure_t* Gimbal);
void ShootMotorMeasureUpdate(ShootMotorMeasure_t* Shoot);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void DaMiao_ctrl_Command(uint8_t *message, DaMiao_CMD_e CMD);
void dm4310_fbdata(DM_motor_t *motor, uint8_t *rx_data);
void DaMiao_mit_ctrl(uint8_t *data, float torq);
void DaMiaoCanSend(float DaMiao);
void DaMiao_Motor_Init(DM_motor_t* DM_Motor);
void DaMiao_Motor_Stall_Monitor(DM_motor_t* DM_Motor);

void GimbalMotorControl(int16_t YawMotor, int16_t PitchMotor, int16_t RotorMotor, int16_t AmmoLeftMotor, int16_t AmmoRightMotor)
{
    MotorSendBuffer[(YAW_MOTOR_ID - 0x201)*2]               =   YawMotor >> 8;
    MotorSendBuffer[(YAW_MOTOR_ID - 0x201)*2 + 1]           =   YawMotor;
    MotorSendBuffer[(PITCH_MOTOR_ID - 0x201)*2]             =   PitchMotor >> 8;
    MotorSendBuffer[(PITCH_MOTOR_ID - 0x201)*2 + 1]         =   PitchMotor;
    MotorSendBuffer[(ROTOR_MOTOR_ID - 0x201)*2]             =   RotorMotor >> 8;
    MotorSendBuffer[(ROTOR_MOTOR_ID - 0x201)*2 + 1]         =   RotorMotor;
    MotorSendBuffer[(AMMO_LEFT_MOTOR_ID - 0x201)*2]         =   AmmoLeftMotor >> 8;
    MotorSendBuffer[(AMMO_LEFT_MOTOR_ID - 0x201)*2 + 1]     =   AmmoLeftMotor;
    MotorSendBuffer[(AMMO_RIGHT_MOTOR_ID - 0x201)*2]        =   AmmoRightMotor >> 8;
    MotorSendBuffer[(AMMO_RIGHT_MOTOR_ID - 0x201)*2 + 1]    =   AmmoRightMotor;
#ifdef PITCH_AUX
    MotorSendBuffer[(PITCH_AUX_MOTOR_ID - 0x201)*2]         =   (-PitchMotor) >> 8;
    MotorSendBuffer[(PITCH_AUX_MOTOR_ID - 0x201)*2 + 1]     =   (-PitchMotor);
#endif
    
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    
    
    can_tx_message.StdId = 0x200;
    if ((MotorBusPosition[0]|MotorBusPosition[1]|MotorBusPosition[2]|MotorBusPosition[3])&0x01){
		HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, MotorSendBuffer, &send_mail_box);
    }
    if ((MotorBusPosition[0]|MotorBusPosition[1]|MotorBusPosition[2]|MotorBusPosition[3])&0x02){
        HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, MotorSendBuffer, &send_mail_box);
    }
    
    can_tx_message.StdId = 0x1FF;
    if ((MotorBusPosition[4]|MotorBusPosition[5]|MotorBusPosition[6]|MotorBusPosition[7])&0x01){
        HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, (MotorSendBuffer + 8), &send_mail_box);
    }
    if ((MotorBusPosition[4]|MotorBusPosition[5]|MotorBusPosition[6]|MotorBusPosition[7])&0x02){
        HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, (MotorSendBuffer + 8), &send_mail_box);
    }
}

/**
*@brief 向达妙电机发送控制指令
**/
void DaMiaoCanSend(float DaMiao)
{
	can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    // 达妙电机控制指令发送（扭矩）
    can_tx_message.StdId = DAMIAO_PITCH_MOTOR_SLAVE_ID;
    if(DamiaoPitchMotorMeasure.para.state != 1){
        DaMiao_ctrl_Command(DaMiao_message,DaMiao_ENABLE);
        HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, DaMiao_message, &send_mail_box);
    }
	if(DamiaoPitchMotorMeasure.para.state>7 && !DamiaoPitchMotorMeasure.error_clear_flag){
		DaMiao_ctrl_Command(DaMiao_message,DaMiao_CLear_ERROR);
		HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, DaMiao_message, &send_mail_box);
		DamiaoPitchMotorMeasure.error_clear_flag = 1;
		DamiaoPitchMotorMeasure.error_clear_time = GetSystemTimer();
	}
	if(GetSystemTimer() - DamiaoPitchMotorMeasure.error_clear_time > 5000 && !OfflineMonitor.DaMiao_PitchMotor && DamiaoPitchMotorMeasure.error_clear_flag)
		DamiaoPitchMotorMeasure.error_clear_flag = 0;
	if(DamiaoPitchMotorMeasure.para.pos >= DAMIAO_MAX_ANGLE && DaMiao > 0)
		DaMiao = 0;
	else if(DamiaoPitchMotorMeasure.para.pos <= DAMIAO_MIN_ANGLE && DaMiao < 0)
		DaMiao = tor;
	//DaMiao_Motor_Stall_Monitor(&DamiaoPitchMotorMeasure);
	if(DamiaoPitchMotorMeasure.stall_flag==1) DaMiao/=((DaMiao>0?DaMiao:-DaMiao)*2);
    DaMiao_mit_ctrl(damiao_data,DaMiao);
    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, damiao_data, &send_mail_box);
}


motor_measure_t YawMotorMeasure;
motor_measure_t PitchMotorMeasure;
motor_measure_t RotorMotorMeasure;
static motor_measure_t AmmoLeftMotorMeasure;
static motor_measure_t AmmoRightMotorMeasure;


//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t* message)
{
    switch (MotorID){
        case YAW_MOTOR_ID:
            get_motor_measure(&YawMotorMeasure, message);
            break;
        case PITCH_MOTOR_ID:
            get_motor_measure(&PitchMotorMeasure, message);
            break;
        case ROTOR_MOTOR_ID:
            get_motor_measure(&RotorMotorMeasure, message);
            break;
        case AMMO_LEFT_MOTOR_ID:
            get_motor_measure(&AmmoLeftMotorMeasure, message);
            break;
        case AMMO_RIGHT_MOTOR_ID:
            get_motor_measure(&AmmoRightMotorMeasure, message);
            break;
        case DAMIAO_PITCH_MOTOR_MASTER_ID:
            dm4310_fbdata(&DamiaoPitchMotorMeasure, message);
			break;
        default:
            break;
    }
    
    if (hcan == &hcan1){
        MotorBusPosition[MotorID - 0x201] = 1;
    }
    else{
        MotorBusPosition[MotorID - 0x201] = 2;
    }
}


void GimbalMotorMeasureUpdate(GimbalMotorMeasure_t* Gimbal)
{
    Gimbal->YawMotorAngle = YawMotorMeasure.ecd / 8192.0f * 360.0f - 180.0f;
    Gimbal->YawMotorSpeed = YawMotorMeasure.speed_rpm;
    Gimbal->PitchMotorAngle = PitchMotorMeasure.ecd / 8192.0f * 360.0f - 180.0f;
    Gimbal->PitchMotorSpeed = PitchMotorMeasure.speed_rpm;
}

void ShootMotorMeasureUpdate(ShootMotorMeasure_t* Shoot)
{
    Shoot->RotorMotorSpeed = RotorMotorMeasure.speed_rpm;
    Shoot->AmmoLeftMotorSpeed = AmmoLeftMotorMeasure.speed_rpm;
    Shoot->AmmoRightMotorSpeed = AmmoRightMotorMeasure.speed_rpm;
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
************************************************************************
* @brief:      	DaMiao_mit_ctrl: MIT模式下的电机控制信息生成函数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	获取应向can总线发送的数据信息
************************************************************************
**/
void DaMiao_mit_ctrl(uint8_t *data, float torq)
{
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

	pos_tmp = float_to_uint(0, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(0, V_MIN, V_MAX, 12);
	kp_tmp = float_to_uint(0, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(0, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN, T_MAX, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
	data[7] = tor_tmp;
}

/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩以及相关温度参数
************************************************************************
**/
void dm4310_fbdata(DM_motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0]) & 0x0F;
	motor->para.state = (rx_data[0]) >> 4;
	motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
	motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
	motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
}

/**
* @brief:      	DaMiao_ctr_Command: 控制达妙电机的状态
* @param[in]:   message： 存储待发送的控制指令
* @param[in]:   CMD: 控制指令
* @retval:     	void
* @details:    	由控制指令获取待发送信息
 **/
void DaMiao_ctrl_Command(uint8_t *message, DaMiao_CMD_e CMD)
{
    switch (CMD)
    {
    case DaMiao_DISABLE:
        DaMiao_Motor_Disable_Message(message)
        break;
    case DaMiao_ENABLE:
        DaMiao_Motor_Enable_Message(message)
        break;
    case DaMiao_CLear_ERROR:
        DaMiao_Motor_Error_Clear_Message(message)
        break;
    case DaMiao_Zero_SET:
        DaMiao_Motor_ZERO_SET_Message(message)
        break;
    default:
        break;
    }
}

void DaMiao_Motor_Init(DM_motor_t* DM_Motor)
{
	DM_Motor->error_clear_flag=0;
	DM_Motor->error_clear_time=0;
	DM_Motor->stall_flag=0;
	DM_Motor->stall_counter=0;
	DaMiao_ctrl_Command(DaMiao_message,DaMiao_ENABLE);
    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, DaMiao_message, &send_mail_box);
}

void DaMiao_Motor_Stall_Monitor(DM_motor_t* DM_Motor)
{
	if((DM_Motor->para.tor >= 1.0f  || DM_Motor->para.tor <= -1.0f || DM_Motor->stall_flag==1) && (DM_Motor->para.vel>0?DM_Motor->para.vel:-DM_Motor->para.vel)<0.2)
	{
		DM_Motor->stall_counter++;
		if(DM_Motor->stall_counter>=65535) DM_Motor->stall_counter=65535;
	}
	else if((DM_Motor->para.vel>0?DM_Motor->para.vel:-DM_Motor->para.vel)>0.5)
	{
		DM_Motor->stall_counter=0;
	}
	if(DM_Motor->stall_counter>=500){
		DM_Motor->stall_flag=1;
	}
	else{
		DM_Motor->stall_flag=0;
	}
}
