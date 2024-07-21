#include "RefereeThread.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "CMS.h"
#include "RefereeBehaviour.h"
#include "Client_UI.h"
#include "CanPacket.h"
#include "CalculateThread.h"
#include "UIThread.h"
#include "InterruptService.h"
//???????UI????
//??????????????UI
// Graph_Data deng1,kuang,deng2,deng3,deng4,fu,imagex,imagex1,imagex2,imagex3,imagey,imagey1,imagey2,imagey3,imagey4;
// String_Data bullet,bullet3,DAFU,Abuff,Pbuff,Cbuff,state,ZIMIAO,dafustate,zimiaostate,dafustate1,zimiaostate1,state4;
// Float_Data capacityD,Min,Sec;
// int Time=0,M=10,S=0;
// int BTime=0,BM=10,BS=0;
// Float_Data BMin,BSec;

// Graph_Data Frame;//?????
////??????
// Graph_Data Collimation_1;
// Graph_Data Collimation_2;
// Graph_Data Collimation_3;
// Graph_Data Collimation_4;
////???????
// Graph_Data Collimation_5;
////????????
// Graph_Data Collimation_6;
// Graph_Data Collimation_7;
// Graph_Data Collimation_8;
// Graph_Data Collimation_9;
// Graph_Data Collimation_10;
// Graph_Data Collimation_11;

// String_Data Capcity,HitRune,AiMBot;
// Float_Data CapData;
// String_Data HitRuneStatus,AiMBotStatus;

// String_Data BulletCover,ChassisStatue;
// Graph_Data BulletCircle,ChassisStatueCircle;

// String_Data ChassisMove;
// Graph_Data ChassisMoveCircle;

// String_Data s_rune,b_rune;

// uint32_t flash_time = 0;
// uint8_t s_rune_flag = 0;
// uint8_t b_rune_flag = 0;
// int s_time_rune;
// int b_time_rune;

Graph_Data imagex, imagey, x1, x6, x7, x8, x9, x10, x11, x12, x13, x14, HP_total, HP_real;
Graph_Data circle1, circle2, circle3, circle4;
Graph_Data Pingheng, Motor_offline, Posture_line;
String_Data Ammo, aimbot, autofire, Mode, open2, open3, open4, open5, noforce;
String_Data capcity;
String_Data rune;
Float_Data CapData, battery_voltage;
int mode_flag = 0;
uint8_t count = 0, count2 = 0;
extern float CapChageVoltage;
extern EulerSystemMeasure_t Imu;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern uint8_t enemy[2];
uint32_t HP_Now = 0;
extern OfflineMonitor_t Offline;
extern float angle_minus;
ChassisMode_e mode = NOFORCE;
uint8_t fire_mode, shoot_mode, aim_mode, match_mode;
uint8_t mode_change_flag; // bit 0-7 ??????,?????????????????????????????

void UI(void const *argument)
{
	uint16_t flashtime = 0;
	num = 0;
	top = 0;
	osDelay(1000);
	mode = NOFORCE;
	fire_mode = 0x00;
	shoot_mode = 0x00;
	match_mode = 0x00;
	aim_mode = (PTZ.AimTargetRequest & 0x31);
	//???UI???
	//???л?
	Char_Draw(&Mode, "mod", UI_Graph_ADD, 0, UI_Color_Green, 18, 21, 2, 78, 782, "MODE\nFIRE\nSINGLE\nAMMO");
	Char_ReFresh(Mode);
	usart6_tx_dma_enable(UIsend_buffer + head[num - 1], head[num] - head[num - 1]);
	num--;
	top = head[num];
	osDelay(100);
	//?ο???
	Line_Draw(&imagex, "xck", UI_Graph_ADD, 0, UI_Color_Green, 2, 6900, 540, 7310, 540);
	Line_Draw(&imagey, "yck", UI_Graph_ADD, 0, UI_Color_Green, 1, 7105, 900, 7105, 100);
	Line_Draw(&x1, "x01", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 7050, 520, 7160, 520);
	Line_Draw(&x6, "x06", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 7050, 420, 7160, 420);
	Line_Draw(&x13, "x13", UI_Graph_ADD, 0, UI_Color_Pink, 6, 6706, 40, 6910, 340);
	Line_Draw(&x14, "x14", UI_Graph_ADD, 0, UI_Color_Pink, 6, 7504, 40, 7300, 340);
	Float_Draw(&CapData, "cpd", UI_Graph_ADD, 0, UI_Color_Yellow, 20, 5, 2, 920, 158, CMS_Data.cms_cap_v * 1000);
	UI_ReFresh(7, imagex, imagey, x1, x6, x13, x14, CapData);
	usart6_tx_dma_enable(UIsend_buffer + head[num - 1], head[num] - head[num - 1]);
	num--;
	top = head[num];
	osDelay(100);
	//?????
	Char_Draw(&noforce, "nof", UI_Graph_ADD, 1, UI_Color_White, 16, 7, 2, 225, 786, "noforce");
	Char_ReFresh(noforce);
	usart6_tx_dma_enable(UIsend_buffer + head[num - 1], head[num] - head[num - 1]);
	num--;
	top = head[num];
	osDelay(100);
	Char_Draw(&open2, "opp", UI_Graph_ADD, 1, UI_Color_White, 16, 4, 2, 225, 760, "manu");
	Char_ReFresh(open2);
	usart6_tx_dma_enable(UIsend_buffer + head[num - 1], head[num] - head[num - 1]);
	num--;
	top = head[num];
	osDelay(100);
	Char_Draw(&open3, "oop", UI_Graph_ADD, 1, UI_Color_White, 16, 7, 2, 225, 731, "close ");
	Char_ReFresh(open3);
	usart6_tx_dma_enable(UIsend_buffer + head[num - 1], head[num] - head[num - 1]);
	num--;
	top = head[num];
	osDelay(100);
	Char_Draw(&rune, "run", UI_Graph_ADD, 1, UI_Color_White, 16, 7, 2, 225, 702, "NORMAL");
	Char_ReFresh(rune);
	usart6_tx_dma_enable(UIsend_buffer + head[num - 1], head[num] - head[num - 1]);
	num--;
	top = head[num];
	osDelay(100);
	while (1)
	{
		if (Chassis.Mode == FALLOW && mode == NOFORCE)
		{
			// 模式
			Char_Draw(&Mode, "mod", UI_Graph_ADD, 0, UI_Color_Green, 18, 21, 2, 78, 782, "MODE\nFIRE\nSINGLE\nAMMO");
			Char_ReFresh(Mode);
			usart6_tx_dma_enable(UIsend_buffer + head[num - 1], head[num] - head[num - 1]);
			num--;
			top = head[num];
			osDelay(100);
			Char_Draw(&noforce, "nof", UI_Graph_ADD, 1, UI_Color_Purplish_red, 16, 7, 2, 225, 786, "follow ");
			Char_ReFresh(noforce);
			usart6_tx_dma_enable(UIsend_buffer + head[num - 1], head[num] - head[num - 1]);
			num--;
			top = head[num];
			osDelay(100);
			// 基准线
			Line_Draw(&imagex, "xck", UI_Graph_ADD, 0, UI_Color_Green, 2, 6900, 540, 7310, 540);
			Line_Draw(&imagey, "yck", UI_Graph_ADD, 0, UI_Color_Green, 1, 7105, 900, 7105, 100);
			Line_Draw(&x1, "x01", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 7050, 520, 7160, 520);
			Line_Draw(&x6, "x06", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 7050, 420, 7160, 420);
			Line_Draw(&x13, "x13", UI_Graph_ADD, 0, UI_Color_Pink, 6, 6706, 40, 6910, 340);
			UI_ReFresh(5, imagex, imagey, x1, x6, x13);
			usart6_tx_dma_enable(UIsend_buffer + head[num - 1], head[num] - head[num - 1]);
			num--;
			top = head[num];
			osDelay(100);
			Line_Draw(&x14, "x14", UI_Graph_ADD, 0, UI_Color_Pink, 6, 7504, 40, 7300, 340);
			Float_Draw(&CapData, "cpd", UI_Graph_ADD, 0, UI_Color_White, 20, 5, 2, 920, 158, CMS_Data.cms_cap_v * 1000);
			Line_Draw(&Posture_line, "pol", UI_Graph_ADD, 0, UI_Color_Pink, 25, (uint32_t)(200 - 30 * sin(angle_minus)), (uint32_t)(230 - 30 * cos(angle_minus)), (uint32_t)(200 + 30 * sin(angle_minus)), (uint32_t)(230 + 30 * cos(angle_minus)));
			if (Offline.Motor[4] || Offline.Motor[5] || Offline.Motor[6] || Offline.Motor[7])
			{
				Circle_Draw(&Motor_offline, "mto", UI_Graph_ADD, 0, UI_Color_Purplish_red, 10, 950, 258, 5);
			}
			else
			{
				Circle_Draw(&Motor_offline, "mto", UI_Graph_ADD, 0, UI_Color_Green, 10, 950, 258, 5);
			}
			Float_Draw(&battery_voltage, "btv", UI_Graph_ADD, 0, UI_Color_White, 230, 5, 2, 920, 208, power_heat_data_t.chassis_voltage);
			Rectangle_Draw(&HP_total, "hpt", UI_Graph_ADD, 0, UI_Color_Yellow, 4, 696, 833, 1199, 863);
			Line_Draw(&HP_real, "hpr", UI_Graph_ADD, 0, UI_Color_Green, 28, 700, 836, 700, 836);
			UI_ReFresh(7, x14, CapData, Posture_line, Motor_offline, battery_voltage, HP_total, HP_real);
			usart6_tx_dma_enable(UIsend_buffer + head[num - 1], head[num] - head[num - 1]);
			num--;
			top = head[num];
			osDelay(100);
			// 模式
			if ((PTZ.PTZStatusInformation & 64) == 64)
			{
				Char_Draw(&open2, "opp", UI_Graph_ADD, 1, UI_Color_Purplish_red, 16, 4, 2, 225, 760, "auto");
				Char_ReFresh(open2);
			}
			else
			{
				Char_Draw(&open2, "opp", UI_Graph_ADD, 1, UI_Color_White, 16, 4, 2, 225, 760, "manu");
				Char_ReFresh(open2);
			}
			usart6_tx_dma_enable(UIsend_buffer + head[num - 1], head[num] - head[num - 1]);
			num--;
			top = head[num];
			osDelay(100);
			if ((PTZ.AimTargetRequest & 0x02) == 0x02)
			{
				Char_Draw(&open3, "oop", UI_Graph_ADD, 1, UI_Color_Purplish_red, 16, 7, 2, 225, 731, "open  ");
				Char_ReFresh(open3);
			}
			else
			{
				Char_Draw(&open3, "oop", UI_Graph_ADD, 1, UI_Color_White, 16, 7, 2, 225, 731, "close ");
				Char_ReFresh(open3);
			}
			usart6_tx_dma_enable(UIsend_buffer + head[num - 1], head[num] - head[num - 1]);
			num--;
			top = head[num];
			osDelay(100);
			if (PTZ.AimTargetRequest & 0x20)
			{
				Char_Draw(&rune, "run", UI_Graph_ADD, 1, UI_Color_Pink, 16, 7, 2, 225, 702, "BIG    ");
				Char_ReFresh(rune);
			}
			else if (PTZ.AimTargetRequest & 0x10)
			{
				Char_Draw(&rune, "run", UI_Graph_ADD, 1, UI_Color_Cyan, 16, 7, 2, 225, 702, "SMALL  ");
				Char_ReFresh(rune);
			}
			else
			{
				Char_Draw(&rune, "run", UI_Graph_ADD, 1, UI_Color_White, 16, 7, 2, 225, 702, "NORMAL");
				Char_ReFresh(rune);
			}
			usart6_tx_dma_enable(UIsend_buffer + head[num - 1], head[num] - head[num - 1]);
			num--;
			top = head[num];
			osDelay(100);
			mode = NOFORCE;
			fire_mode = (PTZ.PTZStatusInformation & 64);
			shoot_mode = (PTZ.AimTargetRequest & 0x02);
			aim_mode = (PTZ.AimTargetRequest & 0x31);
			match_mode = 0x00;
		}
		if (PTZ.PTZStatusInformation & 0x40)
		{
			Rectangle_Draw(&HP_total, "hpt", UI_Graph_Change, 0, UI_Color_Black, 4, 696, 833, 1199, 863);
		}
		else
		{
			Rectangle_Draw(&HP_total, "hpt", UI_Graph_Change, 0, UI_Color_Yellow, 4, 696, 833, 1199, 863);
		}
		// 敌方机器人血量
		if (enemy[0] & 0x01)
		{
			if (robot_state.robot_id < 10)
			{
				if (enemy[1] & 0x01)
				{
					HP_Now = game_robot_HP_t.blue_1_robot_HP;
				}
				else if (enemy[1] & 0x02)
				{
					HP_Now = game_robot_HP_t.blue_2_robot_HP;
				}
				else if (enemy[1] & 0x04)
				{
					HP_Now = game_robot_HP_t.blue_3_robot_HP;
				}
				else if (enemy[1] & 0x08)
				{
					HP_Now = game_robot_HP_t.blue_4_robot_HP;
				}
				else if (enemy[1] & 0x10)
				{
					HP_Now = game_robot_HP_t.blue_5_robot_HP;
				}
				else if (enemy[1] & 0x40)
				{
					HP_Now = game_robot_HP_t.blue_7_robot_HP;
				}
				else
					HP_Now = 0;
			}
			else if (robot_state.robot_id > 10)
			{
				if (enemy[1] & 0x01)
				{
					HP_Now = game_robot_HP_t.red_1_robot_HP;
				}
				else if (enemy[1] & 0x02)
				{
					HP_Now = game_robot_HP_t.red_2_robot_HP;
				}
				else if (enemy[1] & 0x04)
				{
					HP_Now = game_robot_HP_t.red_3_robot_HP;
				}
				else if (enemy[1] & 0x08)
				{
					HP_Now = game_robot_HP_t.red_4_robot_HP;
				}
				else if (enemy[1] & 0x10)
				{
					HP_Now = game_robot_HP_t.red_5_robot_HP;
				}
				else if (enemy[1] & 0x40)
				{
					HP_Now = game_robot_HP_t.red_7_robot_HP;
				}
				else
					HP_Now = 0;
			}
			else
				HP_Now = 0;
			if (enemy[0] & 0x02)
				Line_Draw(&HP_real, "hpr", UI_Graph_Change, 0, UI_Color_White, 28, 629, 836, 629 + HP_Now, 836);
			else
				Line_Draw(&HP_real, "hpr", UI_Graph_Change, 0, UI_Color_Main, 28, 629, 836, 629 + HP_Now, 836);
		}
		else
			Line_Draw(&HP_real, "hpr", UI_Graph_Change, 0, UI_Color_White, 28, 629, 836, 629, 836);
		/*电容电压*/
		if (CMS_Data.Mode == NORMAL)
			Float_Draw(&CapData, "cpd", UI_Graph_Change, 0, UI_Color_White, 30, 5, 2, 920, 158, CMS_Data.cms_cap_v * 1000);
		else if (CMS_Data.Mode == FLY)
			Float_Draw(&CapData, "cpd", UI_Graph_Change, 0, UI_Color_Yellow, 40, 5, 2, 920, 158, CMS_Data.cms_cap_v * 1000);
		else if (CMS_Data.Mode == HIGH_SPEED)
			Float_Draw(&CapData, "cpd", UI_Graph_Change, 0, UI_Color_Purplish_red, 40, 5, 2, 920, 158, CMS_Data.cms_cap_v * 1000);
		/*电池电压*/
		Float_Draw(&battery_voltage, "btv", UI_Graph_Change, 0, UI_Color_Purplish_red, 30, 5, 2, 920, 208, power_heat_data_t.chassis_voltage);
		/*底盘与云台的偏角及电机是否离线*/
		if (Chassis.Mode == FALLOW || Chassis.Mode == ROTING)
		{
			/*电机离线*/
			if (Offline.Motor[4] || Offline.Motor[5] || Offline.Motor[6] || Offline.Motor[7])
			{
				Circle_Draw(&Motor_offline, "mto", UI_Graph_Change, 0, UI_Color_Main, 10, (uint32_t)(961 - 50 * sin(angle_minus * 0.0175)), (uint32_t)(258 + 50 * cos(angle_minus * 0.0175)), 5);
			}
			else
			{
				Circle_Draw(&Motor_offline, "mto", UI_Graph_Change, 0, UI_Color_Green, 10, (uint32_t)(961 - 50 * sin(angle_minus * 0.0175)), (uint32_t)(258 + 50 * cos(angle_minus * 0.0175)), 5);
			}
			/*偏角*/
			Line_Draw(&Posture_line, "pol", UI_Graph_Change, 0, UI_Color_Yellow, 25, (uint32_t)(961 + 30 * sin(angle_minus * 0.0175)), (uint32_t)(258 - 30 * cos(angle_minus * 0.0175)), (uint32_t)(961 - 30 * sin(angle_minus * 0.0175)), (uint32_t)(258 + 30 * cos(angle_minus * 0.0175)));
		}
		else
		{
			/*电机离线*/
			if (Offline.Motor[4] || Offline.Motor[5] || Offline.Motor[6] || Offline.Motor[7])
			{
				Circle_Draw(&Motor_offline, "mto", UI_Graph_Change, 0, UI_Color_Main, 10, (uint32_t)(961), (uint32_t)(308), 5);
			}
			else
			{
				Circle_Draw(&Motor_offline, "mto", UI_Graph_Change, 0, UI_Color_Green, 10, (uint32_t)(961), (uint32_t)(308), 5);
			}
			Line_Draw(&Posture_line, "pol", UI_Graph_Change, 0, UI_Color_Yellow, 25, (uint32_t)(961), (uint32_t)(228), (uint32_t)(961), (uint32_t)(288));
		}
		Line_Draw(&x14, "x14", UI_Graph_Change, 0, UI_Color_Pink, 6, 7504, 40, 7300, 340);
		if (count2 == 0)
		{
			UI_ReFresh(7, CapData, battery_voltage, Motor_offline, Posture_line, HP_real, HP_total, x14);
		}
		mode_change_flag = 0x00;
		// 模式切换
		// 底盘模式
		if (Chassis.Mode != mode)
		{
			mode_change_flag |= (uint8_t)(1 << 0);
		}
		// 开火模式
		if (fire_mode != (PTZ.PTZStatusInformation & 64))
		{
			mode_change_flag |= (uint8_t)(1 << 1);
		}
		// 击打模式
		if (shoot_mode != (PTZ.AimTargetRequest & 0x02))
		{
			mode_change_flag |= (uint8_t)(1 << 2);
		}
		// 自瞄模式
		if (aim_mode != (PTZ.AimTargetRequest & 0x31))
		{
			mode_change_flag |= (uint8_t)(1 << 4);
		}
		//		//比赛模式
		//		if(match_mode!=(PTZ.PTZStatusInformation&0x40))
		//		{
		//			mode_change_flag |= (uint8_t) (1<<5);
		//		}
		mode = Chassis.Mode;
		fire_mode = (PTZ.PTZStatusInformation & 64);
		shoot_mode = (PTZ.AimTargetRequest & 0x02);
		aim_mode = (PTZ.AimTargetRequest & 0x31);
		//************************************底盘模式**********************************
		if (mode_change_flag & 0x01)
		{
			if (Chassis.Mode == NOFORCE)
			{
				Char_Draw(&noforce, "nof", UI_Graph_Change, 1, UI_Color_White, 16, 7, 2, 225, 786, "noforce");
				Char_ReFresh(noforce);
			}
			else if (Chassis.Mode == ROTING)
			{
				Char_Draw(&noforce, "nof", UI_Graph_Change, 1, UI_Color_Green, 16, 7, 2, 225, 786, "rotate ");
				Char_ReFresh(noforce);
			}
			else if (Chassis.Mode == FALLOW)
			{
				Char_Draw(&noforce, "nof", UI_Graph_Change, 1, UI_Color_Orange, 16, 7, 2, 225, 786, "follow ");
				Char_ReFresh(noforce);
			}
			else if (Chassis.Mode == STOP)
			{
				Char_Draw(&noforce, "nof", UI_Graph_Change, 1, UI_Color_Orange, 16, 7, 2, 225, 786, "stop   ");
				Char_ReFresh(noforce);
			}
		}
		//************************************开火模式********************************
		if (mode_change_flag & 0x02)
		{
			if ((PTZ.PTZStatusInformation & 64) == 64)
			{
				Char_Draw(&open2, "opp", UI_Graph_Change, 1, UI_Color_Purplish_red, 16, 4, 2, 225, 760, "auto");
				Char_ReFresh(open2);
			}
			else
			{
				Char_Draw(&open2, "opp", UI_Graph_Change, 1, UI_Color_White, 16, 4, 2, 225, 760, "manu");
				Char_ReFresh(open2);
			}
		}
		//*********************************击打模式*********************************
		if (mode_change_flag & 0x04)
		{
			if ((PTZ.AimTargetRequest & 0x02) == 0x02)
			{
				Char_Draw(&open3, "oop", UI_Graph_Change, 1, UI_Color_Purplish_red, 16, 7, 2, 225, 731, "open  ");
				Char_ReFresh(open3);
			}
			else
			{
				Char_Draw(&open3, "oop", UI_Graph_Change, 1, UI_Color_White, 16, 7, 2, 225, 731, "close ");
				Char_ReFresh(open3);
			}
		}
		/************************rune*******************************/
		if (mode_change_flag & 0x10)
		{
			if (PTZ.AimTargetRequest & 0x20)
			{
				Char_Draw(&rune, "run", UI_Graph_Change, 1, UI_Color_Pink, 16, 7, 2, 225, 702, "BIG    ");
				Char_ReFresh(rune);
			}
			else if (PTZ.AimTargetRequest & 0x10)
			{
				Char_Draw(&rune, "run", UI_Graph_Change, 1, UI_Color_Cyan, 16, 7, 2, 225, 702, "SMALL  ");
				Char_ReFresh(rune);
			}
			else
			{
				Char_Draw(&rune, "run", UI_Graph_Change, 1, UI_Color_White, 16, 7, 2, 225, 702, "NORMAL");
				Char_ReFresh(rune);
			}
		}
		if (count == 0)
		{
			if (num > 0)
			{
				while (locked)
					osDelay(1);
				locked = 1;
				usart6_tx_dma_enable(UIsend_buffer + head[num - 1], head[num] - head[num - 1]);
				num--;
				top = head[num];
				locked = 0;
			}
		}
		count++;
		if (count == 100)
			count = 0;
		count2 = (count2 + 1) % 200;
		osDelay(1);
	}
}