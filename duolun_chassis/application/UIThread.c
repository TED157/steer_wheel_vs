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
//Graph_Data deng1,kuang,deng2,deng3,deng4,fu,imagex,imagex1,imagex2,imagex3,imagey,imagey1,imagey2,imagey3,imagey4;
//String_Data bullet,bullet3,DAFU,Abuff,Pbuff,Cbuff,state,ZIMIAO,dafustate,zimiaostate,dafustate1,zimiaostate1,state4;
//Float_Data capacityD,Min,Sec;
//int Time=0,M=10,S=0;
//int BTime=0,BM=10,BS=0;
//Float_Data BMin,BSec;

//Graph_Data Frame;//?????
////??????
//Graph_Data Collimation_1;
//Graph_Data Collimation_2;
//Graph_Data Collimation_3;
//Graph_Data Collimation_4;
////???????
//Graph_Data Collimation_5;
////????????
//Graph_Data Collimation_6;
//Graph_Data Collimation_7;
//Graph_Data Collimation_8;
//Graph_Data Collimation_9;
//Graph_Data Collimation_10;
//Graph_Data Collimation_11;

//String_Data Capcity,HitRune,AiMBot;
//Float_Data CapData;
//String_Data HitRuneStatus,AiMBotStatus;

//String_Data BulletCover,ChassisStatue;
//Graph_Data BulletCircle,ChassisStatueCircle;

//String_Data ChassisMove;
//Graph_Data ChassisMoveCircle;

//String_Data s_rune,b_rune;

//uint32_t flash_time = 0;
//uint8_t s_rune_flag = 0;
//uint8_t b_rune_flag = 0;
//int s_time_rune;
//int b_time_rune;

Graph_Data CapData,imagex,imagey,x1,x6,x13,x14,HP_total,HP_real,Aimbot_Dot,Motor_offline,Posture_line,Cap_Line1,Cap_Line2,Bullet_permission,Bullet_line1,Bullet_line2,Bullet_line3;
String_Data Ammo,aimbot,autofire,Mode,open2,open3,open4,open5,noforce;
String_Data capcity;
String_Data rune;
Float_Data HP_real_num;
int mode_flag=0;
uint8_t count=0,count2=0;
extern float CapChageVoltage;
extern EulerSystemMeasure_t    Imu;
extern DMA_HandleTypeDef hdma_usart6_tx;
int32_t HP_Now=0;
extern OfflineMonitor_t Offline;
extern float angle_minus;
ChassisMode_e mode=NOFORCE;
uint8_t fire_mode,shoot_mode,aim_mode,match_mode;
uint8_t mode_change_flag;//bit 0-7 ??????,?????????????????????????????
uint16_t bullet_allowance=0;

void UI(void const * argument)
{
	uint16_t flashtime=0;
	num=0;top=0;
	osDelay(1000);
	mode=NOFORCE;
	fire_mode=0x00;
	shoot_mode=0x00;
	match_mode=0x00;
	aim_mode=(PTZ.AimTargetRequest&0x31);
	while(1)
    {
		if(Chassis.Mode==FALLOW&&mode==NOFORCE){
			//模式
			Char_Draw(&Mode,"mod",UI_Graph_ADD,0,UI_Color_Green,18,21,2,1663,851,"MODE\nFIRE\nSINGLE\nAMMO");
			Char_ReFresh(Mode);	
			usart6_tx_dma_enable(UIsend_buffer+head[num-1],head[num]-head[num-1]);
			num--;top=head[num];
			osDelay(100);
			Char_Draw(&noforce,"nof",UI_Graph_ADD,1,UI_Color_Purplish_red,16,7,2,1805,855,"follow ");
			Char_ReFresh(noforce);
			usart6_tx_dma_enable(UIsend_buffer+head[num-1],head[num]-head[num-1]);
			num--;top=head[num];	
			osDelay(100);
			//基准线
			Line_Draw(&imagex,"xck",UI_Graph_ADD,0,UI_Color_Green,2,6900,540,7310,540);
			Line_Draw(&imagey,"yck",UI_Graph_ADD,0,UI_Color_Green,1,7105,900,7105,100);
			Line_Draw(&x1,"x01",UI_Graph_ADD,0,UI_Color_Cyan,1,7050,520,7160,520);
			Line_Draw(&x6,"x06",UI_Graph_ADD,0,UI_Color_Cyan,1,7050,420,7160,420);
			Line_Draw(&x13,"x13",UI_Graph_ADD,0,UI_Color_Pink,6,6706,40,6910,340);
			Int_Draw(&HP_real_num,"hrn",UI_Graph_ADD,0,UI_Color_Pink,20,2,1216,890,0);
			Line_Draw(&x14,"x14",UI_Graph_ADD,0,UI_Color_Pink,6,7504,40,7300,340);
			UI_ReFresh(7,imagex,imagey,x1,x6,x13,HP_real_num,x14);	
			usart6_tx_dma_enable(UIsend_buffer+head[num-1],head[num]-head[num-1]);
			num--;top=head[num];
			osDelay(100);
			Circle_Draw(&Aimbot_Dot,"abd",UI_Graph_ADD,0,UI_Color_Pink,10,1210,593,10);
			Arc_Draw(&CapData,"cpd",UI_Graph_ADD,0,UI_Color_Green,210,(u32)(210+CMS_Data.cms_cap_v*4.07),7,775,540,207,311);
			Line_Draw(&Posture_line,"pol",UI_Graph_ADD,0,UI_Color_Pink,25,(uint32_t)(200-30*sin(angle_minus)),(uint32_t)(230-30*cos(angle_minus)),(uint32_t)(200+30*sin(angle_minus)),(uint32_t)(230+30*cos(angle_minus)));
			if(Offline.Motor[4] || Offline.Motor[5] || Offline.Motor[6] || Offline.Motor[7])
			{
				Circle_Draw(&Motor_offline,"mto",UI_Graph_ADD,0,UI_Color_Purplish_red,10,950,258,5);
			}
			else{
				Circle_Draw(&Motor_offline,"mto",UI_Graph_ADD,0,UI_Color_Green,10,950,258,5);
			}
			Line_Draw(&Cap_Line1,"cpl",UI_Graph_ADD,0,UI_Color_Main,3,595,727,625,727);
			Rectangle_Draw(&HP_total,"hpt",UI_Graph_ADD,0,UI_Color_Black,4,709,863,1210,893);
			Line_Draw(&HP_real,"hpr",UI_Graph_ADD,0,UI_Color_Green,28,700,867,700,867);
			UI_ReFresh(7,CapData,Posture_line,Motor_offline,Cap_Line1,HP_total,HP_real,Aimbot_Dot);	
			usart6_tx_dma_enable(UIsend_buffer+head[num-1],head[num]-head[num-1]);
			num--;top=head[num];
			osDelay(100);
			Arc_Draw(&Bullet_permission,"bup",UI_Graph_ADD,0,UI_Color_Pink,(u32)(149.99),150,7,1145,540,207,311);
			Line_Draw(&Bullet_line1,"bul",UI_Graph_ADD,0,UI_Color_Main,3,1308,383,1338,383);
			Line_Draw(&Bullet_line2,"bll",UI_Graph_ADD,0,UI_Color_Main,3,1335,538,1360,538);
			Line_Draw(&Bullet_line3,"bel",UI_Graph_ADD,0,UI_Color_Main,3,1309,693,1339,693);
			Line_Draw(&Cap_Line2,"cal",UI_Graph_ADD,0,UI_Color_Main,3,559,608,589,608);
			UI_ReFresh(5,Bullet_permission,Bullet_line1,Bullet_line2,Bullet_line3,Cap_Line2);	
			usart6_tx_dma_enable(UIsend_buffer+head[num-1],head[num]-head[num-1]);
			num--;top=head[num];
			osDelay(100);
			
			if(  (PTZ.PTZStatusInformation     &  64) == 64)
			{
				Char_Draw(&open2,"opp",UI_Graph_ADD,1,UI_Color_Purplish_red,16,4,2,1805,829,"auto");
				Char_ReFresh(open2);
			}
			else
			{
				Char_Draw(&open2,"opp",UI_Graph_ADD,1,UI_Color_White,16,4,2,1805,829,"manu");
				Char_ReFresh(open2);	
			}	
			usart6_tx_dma_enable(UIsend_buffer+head[num-1],head[num]-head[num-1]);
			num--;top=head[num];	
			osDelay(100);
			if((PTZ.AimTargetRequest & 0x02) == 0x02){
				Char_Draw(&open3,"oop",UI_Graph_ADD,1,UI_Color_Purplish_red,16,7,2,1805,800,"open  ");
				Char_ReFresh(open3);
			}
			else
			{
				Char_Draw(&open3,"oop",UI_Graph_ADD,1,UI_Color_White,16,7,2,1805,800,"close ");
				Char_ReFresh(open3);	
			}
			usart6_tx_dma_enable(UIsend_buffer+head[num-1],head[num]-head[num-1]);
			num--;top=head[num];	
			osDelay(100);		
			if(PTZ.AimTargetRequest & 0x20)
			{
				Char_Draw(&rune,"run",UI_Graph_ADD,1,UI_Color_Pink,16,7,2,1805,771,"BIG    ");
				Char_ReFresh(rune);
			}
			else if(PTZ.AimTargetRequest & 0x10)
			{
				Char_Draw(&rune,"run",UI_Graph_ADD,1,UI_Color_Cyan,16,7,2,1805,771,"SMALL  ");
				Char_ReFresh(rune);
			}
			else 
			{
				Char_Draw(&rune,"run",UI_Graph_ADD,1,UI_Color_White,16,7,2,1805,771,"NORMAL");
				Char_ReFresh(rune);	
			}
			usart6_tx_dma_enable(UIsend_buffer+head[num-1],head[num]-head[num-1]);
			num--;top=head[num];	
			osDelay(100);
			mode=NOFORCE;
			fire_mode=(PTZ.PTZStatusInformation&64);
			shoot_mode=(PTZ.AimTargetRequest&0x02);
			aim_mode=(PTZ.AimTargetRequest&0x31);
			match_mode=0x00;
		}
		if(PTZ.PTZStatusInformation&0x80){
			Rectangle_Draw(&HP_total,"hpt",UI_Graph_Change,0,UI_Color_Yellow,4,709,863,1210,893);
		}
		else{
			Rectangle_Draw(&HP_total,"hpt",UI_Graph_Change,0,UI_Color_Black,4,709,863,1210,893);
		}
		//敌方机器人血量
		if(Aimbot_Message.AimbotState & 0x01){
			if(robot_state.robot_id<10){
				if(Aimbot_Message.AimbotTarget & 0x01){
					HP_Now = game_robot_HP_t.blue_1_robot_HP;
				}
				else if(Aimbot_Message.AimbotTarget & 0x02){
					HP_Now = game_robot_HP_t.blue_2_robot_HP;
				}
				else if(Aimbot_Message.AimbotTarget & 0x04){
					HP_Now = game_robot_HP_t.blue_3_robot_HP;
				}
				else if(Aimbot_Message.AimbotTarget & 0x08){
					HP_Now = game_robot_HP_t.blue_4_robot_HP;
				}
				else if(Aimbot_Message.AimbotTarget & 0x10){
					HP_Now = game_robot_HP_t.blue_5_robot_HP;
				}
				else if(Aimbot_Message.AimbotTarget & 0x40){
					HP_Now = game_robot_HP_t.blue_7_robot_HP;
				}
				else
					HP_Now = 0;
			}
			else if(robot_state.robot_id>10)
			{
				if(Aimbot_Message.AimbotTarget & 0x01){
					HP_Now = game_robot_HP_t.red_1_robot_HP;
				}
				else if(Aimbot_Message.AimbotTarget & 0x02){
					HP_Now = game_robot_HP_t.red_2_robot_HP;
				}
				else if(Aimbot_Message.AimbotTarget & 0x04){
					HP_Now = game_robot_HP_t.red_3_robot_HP;
				}
				else if(Aimbot_Message.AimbotTarget & 0x08){
					HP_Now = game_robot_HP_t.red_4_robot_HP;
				}
				else if(Aimbot_Message.AimbotTarget & 0x10){
					HP_Now = game_robot_HP_t.red_5_robot_HP;
				}
				else if(Aimbot_Message.AimbotTarget & 0x40){
					HP_Now = game_robot_HP_t.red_7_robot_HP;
				}
				else
					HP_Now = 0;
			}
			else
				HP_Now = 0;
			if(Aimbot_Message.AimbotState & 0x02)
				Line_Draw(&HP_real,"hpr",UI_Graph_Change,0,UI_Color_Pink,28,711,877,711+HP_Now,877);
			else
				Line_Draw(&HP_real,"hpr",UI_Graph_Change,0,UI_Color_Green,28,711,877,711+HP_Now,877);
			Circle_Draw(&Aimbot_Dot,"abd",UI_Graph_Change,0,UI_Color_Pink,10,Aimbot_Message.TargetX,Aimbot_Message.TargetY,10);
		}
		else{
			HP_Now = 0;
			Line_Draw(&HP_real,"hpr",UI_Graph_Change,0,UI_Color_White,28,711,877,711,877);
			Circle_Draw(&Aimbot_Dot,"abd",UI_Graph_Change,0,UI_Color_Pink,10,Aimbot_Message.TargetX,Aimbot_Message.TargetY,0);
		}
		Int_Draw(&HP_real_num,"hrn",UI_Graph_Change,0,UI_Color_Pink,20,2,1216,890,HP_Now);
		/*电容电压*/
		if(CMS_Data.Mode==NORMAL)
			Arc_Draw(&CapData,"cpd",UI_Graph_Change,0,UI_Color_Green,210,(u32)(210+CMS_Data.cms_cap_v*4.07),7,775,540,207,311);
		else if(CMS_Data.Mode==FLY)
			Arc_Draw(&CapData,"cpd",UI_Graph_Change,0,UI_Color_Yellow,210,(u32)(210+CMS_Data.cms_cap_v*4.07),7,775,540,207,311);
		else if(CMS_Data.Mode==HIGH_SPEED)
			Arc_Draw(&CapData,"cpd",UI_Graph_Change,0,UI_Color_Purplish_red,210,(u32)(210+CMS_Data.cms_cap_v*4.07),7,775,540,207,311);
		/*底盘与云台的偏角及电机是否离线*/
		if(Chassis.Mode!=NOFORCE){
			/*电机离线*/
			if(Offline.Motor[4] || Offline.Motor[5] || Offline.Motor[6] || Offline.Motor[7])
			{
				Circle_Draw(&Motor_offline,"mto",UI_Graph_Change,0,UI_Color_Main,10,(uint32_t)(961-50*sin(angle_minus*0.0175)),(uint32_t)(258+50*cos(angle_minus*0.0175)),5);
			}
			else{
				Circle_Draw(&Motor_offline,"mto",UI_Graph_Change,0,UI_Color_Green,10,(uint32_t)(961-50*sin(angle_minus*0.0175)),(uint32_t)(258+50*cos(angle_minus*0.0175)),5);
			}
			/*偏角*/
			if(power_heat_data_t.chassis_voltage<23000)
				Line_Draw(&Posture_line,"pol",UI_Graph_Change,0,UI_Color_Yellow,25,(uint32_t)(961+30*sin(angle_minus*0.0175)),(uint32_t)(258-30*cos(angle_minus*0.0175)),(uint32_t)(961-30*sin(angle_minus*0.0175)),(uint32_t)(258+30*cos(angle_minus*0.0175)));
			else
				Line_Draw(&Posture_line,"pol",UI_Graph_Change,0,UI_Color_Main,25,(uint32_t)(961+30*sin(angle_minus*0.0175)),(uint32_t)(258-30*cos(angle_minus*0.0175)),(uint32_t)(961-30*sin(angle_minus*0.0175)),(uint32_t)(258+30*cos(angle_minus*0.0175)));
		}
		else{
			/*电机离线*/
			if(Offline.Motor[4] || Offline.Motor[5] || Offline.Motor[6] || Offline.Motor[7])
			{
				Circle_Draw(&Motor_offline,"mto",UI_Graph_Change,0,UI_Color_Main,10,(uint32_t)(961),(uint32_t)(308),5);
			}
			else{
				Circle_Draw(&Motor_offline,"mto",UI_Graph_Change,0,UI_Color_Green,10,(uint32_t)(961),(uint32_t)(308),5);
			}
			if(power_heat_data_t.chassis_voltage<22750
				)
				Line_Draw(&Posture_line,"pol",UI_Graph_Change,0,UI_Color_Yellow,25,(uint32_t)(961),(uint32_t)(228),(uint32_t)(961),(uint32_t)(288));
			else
			Line_Draw(&Posture_line,"pol",UI_Graph_Change,0,UI_Color_Main,25,(uint32_t)(961),(uint32_t)(228),(uint32_t)(961),(uint32_t)(288));
		}
		bullet_allowance = bullet_remaining_t.projectile_allowance_17mm<120? bullet_remaining_t.projectile_allowance_17mm : 120;
		Arc_Draw(&Bullet_permission,"bup",UI_Graph_Change,0,UI_Color_Pink,(u32)(150-bullet_allowance),150,7,1145,540,207,311);
		if(!bullet_allowance)
			Arc_Draw(&Bullet_permission,"bup",UI_Graph_Change,0,UI_Color_Pink,(u32)(150-bullet_allowance*0.3),150,7,1145,540,0,0);
		if(count2==0)
		{			
			UI_ReFresh(7,CapData,Aimbot_Dot,Motor_offline,Posture_line,HP_real,HP_real_num,Bullet_permission);
		}
		mode_change_flag=0x00;
		//模式切换
		//底盘模式
		if(Chassis.Mode != mode){
			mode_change_flag |= (uint8_t) (1 << 0);
			mode=Chassis.Mode;
		}
		//开火模式
		if(fire_mode!=(PTZ.PTZStatusInformation&64)){
			mode_change_flag |= (uint8_t) (1<<1);
			fire_mode=(PTZ.PTZStatusInformation&64);
		}
		//击打模式
		if(shoot_mode!=(PTZ.AimTargetRequest&0x02)){
			mode_change_flag |= (uint8_t) (1<<2);
			shoot_mode=(PTZ.AimTargetRequest&0x02);
		}
		//自瞄模式
		if(aim_mode!=(PTZ.AimTargetRequest&0x31)){
			mode_change_flag |= (uint8_t) (1<<4);
			aim_mode=(PTZ.AimTargetRequest&0x31);
		}
		//比赛模式
		if(match_mode!=(PTZ.PTZStatusInformation&0x80))
		{
			mode_change_flag |= (uint8_t) (1<<5);
			match_mode=(PTZ.PTZStatusInformation&0x80);
		}
		//************************************底盘模式**********************************
		if(mode_change_flag&0x01){
			if(Chassis.Mode == NOFORCE)
			{
				Char_Draw(&noforce,"nof",UI_Graph_Change,1,UI_Color_White,16,7,2,1805,855,"noforce");
				Char_ReFresh(noforce);
			}
			else if(Chassis.Mode == ROTING)
			{
				Char_Draw(&noforce,"nof",UI_Graph_Change,1,UI_Color_Green,16,7,2,1805,855,"rotate ");
				Char_ReFresh(noforce);
			}
			else if(Chassis.Mode == FALLOW)
			{ 
				Char_Draw(&noforce,"nof",UI_Graph_Change,1,UI_Color_Orange,16,7,2,1805,855,"follow ");
				Char_ReFresh(noforce);
			}
			else if(Chassis.Mode == STOP)
			{
				Char_Draw(&noforce,"nof",UI_Graph_Change,1,UI_Color_Orange,16,7,2,1805,855,"stop   ");
				Char_ReFresh(noforce);
			}
		}
		//************************************开火模式********************************
		if(mode_change_flag&0x02){
			if(  (PTZ.PTZStatusInformation     &  64) == 64)
			{
				Char_Draw(&open2,"opp",UI_Graph_Change,1,UI_Color_Purplish_red,16,4,2,1805,829,"auto");
				Char_ReFresh(open2);
			}
			else
			{
				Char_Draw(&open2,"opp",UI_Graph_Change,1,UI_Color_White,16,4,2,1805,829,"manu");
				Char_ReFresh(open2);	
			}	
		}
		//*********************************击打模式*********************************
		if(mode_change_flag&0x04){
			if(   (PTZ.AimTargetRequest & 0x02) == 0x02){
				Char_Draw(&open3,"oop",UI_Graph_Change,1,UI_Color_Purplish_red,16,7,2,1805,800,"open  ");
				Char_ReFresh(open3);
			}
			else
			{
				Char_Draw(&open3,"oop",UI_Graph_Change,1,UI_Color_White,16,7,2,1805,800,"close ");
				Char_ReFresh(open3);	
			}	
		}
		/************************rune*******************************/
		if(mode_change_flag&0x10){
			if(PTZ.AimTargetRequest & 0x20)
			{
				Char_Draw(&rune,"run",UI_Graph_Change,1,UI_Color_Pink,16,7,2,1805,771,"BIG    ");
				Char_ReFresh(rune);
			}
			else if(PTZ.AimTargetRequest & 0x10)
			{
				Char_Draw(&rune,"run",UI_Graph_Change,1,UI_Color_Cyan,16,7,2,1805,771,"SMALL  ");
				Char_ReFresh(rune);
			}
			else 
			{
				Char_Draw(&rune,"run",UI_Graph_Change,1,UI_Color_White,16,7,2,1805,771,"NORMAL");
				Char_ReFresh(rune);	
			}
		}
		if(mode_change_flag&40){
			if(PTZ.PTZStatusInformation&0x80){
				Rectangle_Draw(&HP_total,"hpt",UI_Graph_Change,0,UI_Color_Yellow,4,709,863,1210,893);
			}
			else{
				Rectangle_Draw(&HP_total,"hpt",UI_Graph_Change,0,UI_Color_Black,4,709,863,1210,893);
			}
			UI_ReFresh(7,CapData,Aimbot_Dot,Motor_offline,Posture_line,HP_real,HP_total,HP_real_num);
		}
		if(count==0){
			if(num>0)
			{
				while(locked)	osDelay(1);
				locked=1;					
				usart6_tx_dma_enable(UIsend_buffer+head[num-1],head[num]-head[num-1]);
				num--;top=head[num];
				locked=0;		
			}
		}
		count++;
		if(count==100) count=0;
		count2=(count2+1)%200;
        osDelay(1);
    }

 }