#ifndef _INFANTRY4_KEY_MAP_
#define _INFANTRY4_KEY_MAP_

#include "Remote.h"
#define YAW_REMOTE_SENS                         0.25f
#define PITCH_REMOTE_SENS                       0.25f
#define YAW_MOUSE_SENS                          10
#define PITCH_MOUSE_SENS                        30

// 状态机设置
// 云台无力
//#define GIMBAL_WEAK_KEYMAP              SwitchRightDownSide()
// 云台使能
#define GIMBAL_ENABLE_KEYMAP            SwitchRightMidSide() || SwitchRightUpSide()
// 发射机构使能
#define SHOOTER_ENABLE_KEYMAP           SwitchRightUpSide()
// 底盘使能
#define CHASSIS_ENABLE_KEYMAP           SwitchLeftDownSide()

// 云台运动控制指令
// YAW
#define GIMBAL_CMD_YAW_KEYMAP           NormalizedLimit(MouseMoveY()*YAW_MOUSE_SENS + RemoteChannalLeftY()*YAW_REMOTE_SENS)
// PITCH
#define GIMBAL_CMD_PITCH_KEYMAP         -NormalizedLimit(MouseMoveX()*PITCH_MOUSE_SENS + RemoteChannalLeftX()*PITCH_REMOTE_SENS)
// YAW转180度
#define GIMBAL_CMD_YAW_RESERVE__KEYMAP  CheakKeyPressOnce(KEY_PRESSED_OFFSET_X)


// 底盘运动控制指令
// 前后
#define CHASSIS_CMD_X_KEYMAP            NormalizedLimit((RemoteChannalRightX() - CheakKeyPress(KEY_PRESSED_OFFSET_S) + CheakKeyPress(KEY_PRESSED_OFFSET_W)))
// 左右
#define CHASSIS_CMD_Y_KEYMAP            NormalizedLimit((RemoteChannalRightY() - CheakKeyPress(KEY_PRESSED_OFFSET_D) + CheakKeyPress(KEY_PRESSED_OFFSET_A)))
// 高速
#define CHASSIS_HIGH_SPEED_KEYMAP       CheakKeyPress(KEY_PRESSED_OFFSET_C) //|| (RemoteDial() == -1.0f)
// 不动
#define CHASSIS_STOP_KEYMAP             CheakKeyPress(KEY_PRESSED_OFFSET_F)

// 超级电容开关
#define SUPER_CAP_SWITCH_KEYMAP         CheakKeyPress(KEY_PRESSED_OFFSET_C)  

// 小陀螺
#define CHASSIS_ROTATE_SWITCH_KEYMAP    CheakKeyPress(KEY_PRESSED_OFFSET_SHIFT)  /*|| (RemoteDial() == -1.0f)*/ ||+ CheakKeyPress(KEY_PRESSED_OFFSET_CTRL)
//高速小陀螺
#define CHASSIS_HIGH_SPEED_ROTATE       CheakKeyPress(KEY_PRESSED_OFFSET_CTRL)
//反向小陀螺
#define CHASSIS_ROTATE_RESERVE_KEYMAP   RemoteDial() == 1.0f

//自动电容
#define AUTO_CAP_KEYMAP             CheakKeyPressOnce(KEY_PRESSED_OFFSET_R)


// 自瞄补偿
#define AIMBOT_PITCH_BIAS_LOW_KEYMAP	(CheakKeyPress(KEY_PRESSED_OFFSET_SHIFT) && CheakKeyPressOnce(KEY_PRESSED_OFFSET_C))
#define AIMBOT_PITCH_BIAS_HIGH_KEYMAP	(CheakKeyPress(KEY_PRESSED_OFFSET_SHIFT) && CheakKeyPressOnce(KEY_PRESSED_OFFSET_V))
#define AIMBOT_PITCH_BIAS_ZERO_KEYMAP	(CheakKeyPress(KEY_PRESSED_OFFSET_SHIFT) && CheakKeyPressOnce(KEY_PRESSED_OFFSET_Z))




    
// 发弹指令
#define SHOOT_COMMAND_KEYMAP            ((RemoteDial() == 1.0f) || (MousePressLeft()))
// 自瞄指令
#define AIMBOT_COMMAND_KEYMAP           (SwitchLeftUpSide() || MousePressRight())
// 打击模式（手动/自动发弹）
#define FIRE_MODE_KEYMAP                CheakKeyPressOnce(KEY_PRESSED_OFFSET_Q)
// 特殊自瞄任务请求（打符即单发模式）
#define SINGLE_SHOOT_KEMAP              CheakKeyPressOnce(KEY_PRESSED_OFFSET_E)

#define BIG_RUNE_KEYMAP                 CheakKeyPress(KEY_PRESSED_OFFSET_G)

#define SMALL_RUNE_KEYMAP				CheakKeyPress(KEY_PRESSED_OFFSET_B)

// 摩擦轮转速调整
// 转速升高
#define AMMO_SPEED_UP_KEYMAP            !CheakKeyPress(KEY_PRESSED_OFFSET_CTRL) && CheakKeyPressOnce(KEY_PRESSED_OFFSET_V)
// 转速降低
#define AMMO_SPEED_DOWN_KEYMAP          CheakKeyPress(KEY_PRESSED_OFFSET_CTRL) && CheakKeyPressOnce(KEY_PRESSED_OFFSET_V)




#endif
