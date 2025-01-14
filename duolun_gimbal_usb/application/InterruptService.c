#include "InterruptService.h"
#include "main.h"
#include "Motor.h"
#include "Remote.h"
#include "RefereeCan.h"

#include "AimbotCan.h"
#include "CalculateThread.h"
#include "AttitudeThread.h"

#include "bsp_can.h"

#include "struct_typedef.h"
#include "CanPacket.h"

#include "Setting.h"



#include <string.h>
#include <stdio.h>

#include "UsbPackage.h"
#include "Usb.h"
#include "bsp_usart.h"
#include "Fs_remote.h"
#define UART_printf(...)  HAL_UART_Transmit(&huart1,\
																				(uint8_t  *)u1_buf,\
																				sprintf((char*)u1_buf,__VA_ARGS__),0x120)

#define DMA_printf(...)      __HAL_DMA_DISABLE(&hdma_usart1_tx);\
																				HAL_UART_Transmit_DMA(&huart1,\
																				(uint8_t  *)u1_buf,\
																				sprintf((char*)u1_buf,__VA_ARGS__))
uint8_t u1_buf[30];

HAL_StatusTypeDef status;
extern RefereeInformation_t    Referee;

fp32 speed;

uint8_t prescaler;
uint8_t ammo_speed_ad_flag=0;
//uint8_t rune_shoot_flag=0;
uint16_t time_period_rune=0;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim5;
extern DM_motor_t DamiaoPitchMotorMeasure;

OfflineCounter_t OfflineCounter;
OfflineMonitor_t OfflineMonitor;

RefereeChassisPowerShootHeat_t RefereeChassisPowerShootHeat;


void PitchMotorOfflineCounterUpdate(void);
void YawMotorOfflineCounterUpdate(void);
void RotorMotorOfflineCounterUpdate(void);
void AmmoLeftMotorMotorOfflineCounterUpdate(void);
void AmmoRightMotorMotorOfflineCounterUpdate(void);
void RefereePowerHeatNode0OfflineCounterUpdate(void);
void RefereePowerHeatNode1OfflineCounterUpdate(void);
void RefereeAmmoSpeedNode0OfflineCounterUpdate(void);
void RefereeAmmoSpeedNode1OfflineCounterUpdate(void);
void RefereeAmmoSpeedNode2OfflineCounterUpdate(void);
void RefereeAmmoLimitNode0OfflineCounterUpdate(void);
void RefereeAmmoLimitNode1OfflineCounterUpdate(void);
void RefereeAmmoLimitNode2OfflineCounterUpdate(void);
void RefereeSelfStateNodeOfflineCounterUpdate(void);
void RemoteOfflineCounterUpdate(void);
void DaMiaoPitchMotorOfflineCounterUpdate(void);
void GimbalImuSend(void);
void RefereeUsbSend(void);
void Ft_RemoteoOfflineStateNodeOffline(void);

uint32_t RefereeInterpolationTimer = 0;

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
uint16_t p;
uint32_t time_stap;
uint8_t ghost_data;
uint8_t ghost_data2;
extern AimbotCommand_t AimbotCommand;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
		case AIMBOT_GHOST:
		{
			ghost_data = rx_data[0];
			ghost_data2 = rx_data[7];
			break;
		}      
        case REFEREE_POWER_HEAT_NODE_0_ID:
        {
            RefereePowerHeatNode0OfflineCounterUpdate();
            RefereePowerHeatNode0InformationUpdate(rx_data);
            break;
        }
        case REFEREE_POWER_HEAT_NODE_1_ID:
        {
            RefereePowerHeatNode1OfflineCounterUpdate();
            RefereePowerHeatNode1InformationUpdate(rx_data);
            RefereeInterpolationTimer = 0;
            break;
        }
        case REFEREE_AMMO_SPEED_NODE_0_ID:
        {
            RefereeAmmoSpeedNode0OfflineCounterUpdate();
            RefereeAmmoSpeedNode0InformationUpdate(rx_data);
			ammo_speed_ad_flag=1;
			//rune_shoot_flag=2;
//			DMA_printf("%d\n",GetSystemTimer());
            break;
        }
        case REFEREE_AMMO_SPEED_NODE_1_ID:
        {
            RefereeAmmoSpeedNode1OfflineCounterUpdate();
            RefereeAmmoSpeedNode1InformationUpdate(rx_data);
            break;
        }
        case REFEREE_AMMO_SPEED_NODE_2_ID:
        {
            RefereeAmmoSpeedNode2OfflineCounterUpdate();
            RefereeAmmoSpeedNode2InformationUpdate(rx_data);
            break;
        }
        case REFEREE_AMMO_LIMIT_NODE_0_ID:
        {
            RefereeAmmoLimitNode0OfflineCounterUpdate();
            RefereeAmmoLimitNode0InformationUpdate(rx_data);
            break;
        }
        case REFEREE_AMMO_LIMIT_NODE_1_ID:
        {
            RefereeAmmoLimitNode1OfflineCounterUpdate();
            RefereeAmmoLimitNode1InformationUpdate(rx_data);
            break;
        }
        case REFEREE_AMMO_LIMIT_NODE_2_ID:
        {
            RefereeAmmoLimitNode2OfflineCounterUpdate();
            RefereeAmmoLimitNode2InformationUpdate(rx_data);
            break;
        }
        case REFEREE_SELF_STATE_NODE:
        {
            RefereeSelfStateNodeOfflineCounterUpdate();
            RefereeSelfStateNodeInformationUpdate(rx_data);
            break;
        }
				case 0x12b:
				{
					p++;
					break;
				}

        
        
        case YAW_MOTOR_ID:
        {
            YawMotorOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        }
        case PITCH_MOTOR_ID:
        {
            PitchMotorOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        }
        case ROTOR_MOTOR_ID:
        {
            RotorMotorOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        }
        case AMMO_LEFT_MOTOR_ID:
        {
            AmmoLeftMotorMotorOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        }
        case AMMO_RIGHT_MOTOR_ID:
        {
            AmmoRightMotorMotorOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        }
        case DAMIAO_PITCH_MOTOR_MASTER_ID:
        {
            DaMiaoPitchMotorOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        }
        default:
        {
            break;
        }
    }
}


//�����ж�
void USART3_IRQHandler(void)
{
    if (huart3.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if (USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                sbus_to_rc(0);
                //��¼���ݽ���ʱ��
                RemoteOfflineCounterUpdate();
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                sbus_to_rc(1);
                //��¼���ݽ���ʱ��
                RemoteOfflineCounterUpdate();
            }
        }
    }

}
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}
//�����ж�
void USART6_IRQHandler(void)
{
#if REMOTE_UART6==1
	static volatile uint8_t res;

    if(USART6->SR & UART_FLAG_IDLE)
    {
        if(OfflineMonitor.Ft_Remote && !OfflineMonitor.Remote)
		{
			usart6_rx_dma_restart(USART_RX_BUF_LENGHT);
		}
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
		if(USART6->SR & UART_FLAG_TC)
			__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TC);
        static uint16_t this_time_rx_len = 0;

        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
			FigureTransmission_to_rc(0);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
			FigureTransmission_to_rc(1);
        }
    }
#elif REMOTE_UART6==2
    if (huart6.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if (USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                ibus_to_rc(0);
                //��¼���ݽ���ʱ��
                RemoteOfflineCounterUpdate();
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                ibus_to_rc(1);
                //��¼���ݽ���ʱ��
                RemoteOfflineCounterUpdate();
            }
        }
    }
#endif
}
/** 
  * @brief This function handles TIM3 global interrupt.
  */
uint32_t SystemTimer = 0;

uint32_t GetSystemTimer(void)
{
    return SystemTimer;
}


//void GimbalMotorCommandSend(void);

void GimbalImuPacketSend(void);

void CommuniteOfflineCounterUpdate(void);
void CommuniteOfflineStateUpdate(void);


void TimerTaskLoop1000Hz(void)
{
    SystemTimer++;
//    GimbalMotorCommandSend();
    CommuniteOfflineCounterUpdate();
    CommuniteOfflineStateUpdate();
    
    RefereeInterpolationTimer++;
    if ((RefereeInterpolationTimer % 100) == 0) {
        AmmoHeatSettlementInterpolation();
    }
	//DMA_printf("%d,%d\r\n",Aimbot.TargetX,rece_num);
}



void TimerTaskLoop500Hz(void)
{
    //GimbalImuPacketSend();
	GimbalImuSend();
}


void TimerTaskLoop100Hz(void)
{
//	if(!rune_shoot_flag){
//		time_period_rune++;}
//	if(time_period_rune==40)
//	{
//		rune_shoot_flag=1;
//	}
	if(prescaler==20){
		prescaler=0;
		CanSendMessage(&hcan1,ENEMY_ID,6,(uint8_t *)(&Aimbot_Message.AimbotState));
    }
	prescaler++;
	//GimbalImuPacketSend();
	//UART_printf("%f,%f\n",Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed,Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed);
}

void TimerTaskLoop100Hz_1(void)
{
	//usart1_tx_dma_enable((uint8_t*)"1\n",2);
	
		//DMA_printf("%f,%f\n",Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed,Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed/*,pitch_kf.x*/);
	
}




void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
    TimerTaskLoop1000Hz();
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
    TimerTaskLoop500Hz();
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
    TimerTaskLoop100Hz_1();
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
    TimerTaskLoop100Hz();
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}




//GimbalOutput_t GimbalMotorOutput;
//void GimbalMotorCommandSend(void)
//{
//    GetGimbalMotorOutput(&GimbalMotorOutput);
//    GimbalMotorControl( GimbalMotorOutput.Yaw * YAW_MOTOR_DIRECTION,
//                        GimbalMotorOutput.Pitch * PITCH_MOTOR_DIRECTION, 
//                        GimbalMotorOutput.Rotor, 
//                        GimbalMotorOutput.AmmoLeft, 
//                        GimbalMotorOutput.AmmoRight
//                        );
//}

ImuPacketNormal_t ImuPacket;
void GimbalImuSend(void)
{
	ImuPacket.TimeStamp = SystemTimer;
	GimabalImu.TimeStamp = SystemTimer;
	fp32 quat[4];
	GetCurrentQuaternion(quat);
	GimabalImu.q0 = quat[0];
	GimabalImu.q1 = quat[1];
	GimabalImu.q2 = quat[2];
	GimabalImu.q3 = quat[3];
	UsbSendMessage((uint8_t *)&GimabalImu, (uint16_t)sizeof(GimabalImu), GIMBAL_IMU_0_ID);
}





void CommuniteOfflineCounterUpdate(void)
{
    OfflineCounter.PitchMotor++;
     OfflineCounter.DaMiao_PitchMotor++;
    OfflineCounter.YawMotor++;
    OfflineCounter.RotorMotor++;
    OfflineCounter.AmmoLeftMotor++;
    OfflineCounter.AmmoRightMotor++;
    OfflineCounter.AimbotDataNode++;
    OfflineCounter.RefereePowerHeatNode0++;
    OfflineCounter.RefereePowerHeatNode1++;
    OfflineCounter.RefereeAmmoSpeedNode0++;
    OfflineCounter.RefereeAmmoSpeedNode1++;
    OfflineCounter.RefereeAmmoSpeedNode2++;
    OfflineCounter.RefereeAmmoLimitNode0++;
    OfflineCounter.RefereeAmmoLimitNode1++;
    OfflineCounter.RefereeAmmoLimitNode2++;
    OfflineCounter.RefereeSelfStateNode++;
    OfflineCounter.Remote++;
	OfflineCounter.Ft_Remote++;
}

void CommuniteOfflineStateUpdate(void)
{
    // Motor
    if (OfflineCounter.PitchMotor > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.PitchMotor = 1;
    }
    else{
        OfflineMonitor.PitchMotor = 0;
    }
    if (OfflineCounter.DaMiao_PitchMotor > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.DaMiao_PitchMotor = 1;
    }
    else{
        OfflineMonitor.DaMiao_PitchMotor = 0;
    }
    if (OfflineCounter.YawMotor > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.YawMotor = 1;
    }
    else{
        OfflineMonitor.YawMotor = 0;
    }
    if (OfflineCounter.RotorMotor > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.RotorMotor = 1;
    }
    else{
        OfflineMonitor.RotorMotor = 0;
    }
    if (OfflineCounter.AmmoLeftMotor > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.AmmoLeftMotor = 1;
    }
    else{
        OfflineMonitor.AmmoLeftMotor = 0;
    }
    if (OfflineCounter.AmmoRightMotor > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.AmmoRightMotor = 1;
    }
    else{
        OfflineMonitor.AmmoRightMotor = 0;
    }
    
    // USB Bus Node
    if (OfflineCounter.AimbotDataNode > AIMBOT_OFFLINE_TIMEMAX){
        OfflineMonitor.AimbotDataNode = 1;
    }
    else{
        OfflineMonitor.AimbotDataNode = 0;
    }
    if (OfflineCounter.RefereePowerHeatNode0 > REFEREE_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereePowerHeatNode0 = 1;
    }
    else{
        OfflineMonitor.RefereePowerHeatNode0 = 0;
    }
    if (OfflineCounter.RefereePowerHeatNode1 > REFEREE_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereePowerHeatNode1 = 1;
    }
    else{
        OfflineMonitor.RefereePowerHeatNode1 = 0;
    }
    if (OfflineCounter.RefereeAmmoSpeedNode0 > REFEREE_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereeAmmoSpeedNode0 = 1;
    }
    else{
        OfflineMonitor.RefereeAmmoSpeedNode0 = 0;
    }
    if (OfflineCounter.RefereeAmmoSpeedNode1 > REFEREE_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereeAmmoSpeedNode1 = 1;
    }
    else{
        OfflineMonitor.RefereeAmmoSpeedNode1 = 0;
    }
    if (OfflineCounter.RefereeAmmoSpeedNode2 > REFEREE_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereeAmmoSpeedNode2 = 1;
    }
    else{
        OfflineMonitor.RefereeAmmoSpeedNode2 = 0;
    }
    if (OfflineCounter.RefereeAmmoLimitNode0 > REFEREE_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereeAmmoLimitNode0 = 1;
    }
    else{
        OfflineMonitor.RefereeAmmoLimitNode0 = 0;
    }
    if (OfflineCounter.RefereeAmmoLimitNode1 > REFEREE_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereeAmmoLimitNode1 = 1;
    }
    else{
        OfflineMonitor.RefereeAmmoLimitNode1 = 0;
    }
    if (OfflineCounter.RefereeAmmoLimitNode2 > REFEREE_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereeAmmoLimitNode2 = 1;
    }
    else{
        OfflineMonitor.RefereeAmmoLimitNode2 = 0;
    }
    if (OfflineCounter.RefereeSelfStateNode > REFEREE_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereeSelfStateNode = 1;
    }
    else{
        OfflineMonitor.RefereeSelfStateNode = 0;
    }

    // Remote
    if (OfflineCounter.Remote > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.Remote = 1;
    }
    else{
        OfflineMonitor.Remote = 0;
    }
	//ͼ������
	if(OfflineCounter.Ft_Remote > FT_REMOTE_OFFLINE_TIMEMAX){
		OfflineMonitor.Ft_Remote = 1;
	}
	else{
		OfflineMonitor.Ft_Remote = 0;
	}
}

void DeviceOfflineMonitorUpdate(OfflineMonitor_t *Monitor)
{
    memcpy(Monitor, &OfflineMonitor, sizeof(OfflineMonitor_t));
}


void PitchMotorOfflineCounterUpdate(void)
{
    OfflineCounter.PitchMotor = 0;
}

void DaMiaoPitchMotorOfflineCounterUpdate(void)
{
    OfflineCounter.DaMiao_PitchMotor = 0;
}

void YawMotorOfflineCounterUpdate(void)
{
    OfflineCounter.YawMotor = 0;
}

void RotorMotorOfflineCounterUpdate(void)
{
    OfflineCounter.RotorMotor = 0;
}

void AmmoLeftMotorMotorOfflineCounterUpdate(void)
{
    OfflineCounter.AmmoLeftMotor = 0;
}

void AmmoRightMotorMotorOfflineCounterUpdate(void)
{
    OfflineCounter.AmmoRightMotor = 0;
}
void AimbotDataNodeOfflineCounterUpdate(void)
{
    OfflineCounter.AimbotDataNode = 0;
}

void RefereePowerHeatNode0OfflineCounterUpdate(void)
{
    OfflineCounter.RefereePowerHeatNode0 = 0;
}

void RefereePowerHeatNode1OfflineCounterUpdate(void)
{
    OfflineCounter.RefereePowerHeatNode1 = 0;
}

void RefereeAmmoSpeedNode0OfflineCounterUpdate(void)
{
    OfflineCounter.RefereeAmmoSpeedNode0 = 0;
}

void RefereeAmmoSpeedNode1OfflineCounterUpdate(void)
{
    OfflineCounter.RefereeAmmoSpeedNode1 = 0;
}

void RefereeAmmoSpeedNode2OfflineCounterUpdate(void)
{
    OfflineCounter.RefereeAmmoSpeedNode2 = 0;
}

void RefereeAmmoLimitNode0OfflineCounterUpdate(void)
{
    OfflineCounter.RefereeAmmoLimitNode0 = 0;
}

void RefereeAmmoLimitNode1OfflineCounterUpdate(void)
{
    OfflineCounter.RefereeAmmoLimitNode1 = 0;
}

void RefereeAmmoLimitNode2OfflineCounterUpdate(void)
{
    OfflineCounter.RefereeAmmoLimitNode2 = 0;
}

void RefereeSelfStateNodeOfflineCounterUpdate(void)
{
    OfflineCounter.RefereeSelfStateNode = 0;
}

void RemoteOfflineCounterUpdate(void)
{
    OfflineCounter.Remote = 0;
}

void Ft_RemoteoOfflineStateNodeOffline(void)
{
	OfflineCounter.Ft_Remote = 0;
}