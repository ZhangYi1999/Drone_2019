#ifndef __FUNC_STATUSMACHINE_H
#define __FUNC_STATUSMACHINE_H

#include "System_Config.h"

typedef enum
{
    chassis_stop = 0,
    chassis_run
}chassis_mode_Enum;
 




//板载用户按键状态
typedef enum{
    ButtonPressedOnce = 0,
    ButtonTaskFinish
} UserButtonStatus_Enum;

//摩擦轮状态
typedef enum
{
    FricStatus_Stop,
    FricStatus_Working_Low,
    FricStatus_Working_High,
    FricStatus_Working_Dynamic,
}FricStatus_Enum;

//拨盘状态
typedef enum
{
    StirStatus_Stop,
    StirStatus_SpeedControl,
    StirStatus_AngleControl,
}StirStatus_Enum;

//控制模式
typedef enum
{
    ControlMode_Protect,            //保护模式，全部停止 
    ControlMode_RC,                 //遥控器控制（增量式）
    ControlMode_Auto,               //自动射击控制
}ControlMode_Enum;

//瞄准模式
typedef enum
{
    NormalAim, 
    AutoAim
}AimModeStatus_Enum;



//状态量
extern UserButtonStatus_Enum    UserButtonStatus;
extern ControlMode_Enum         ControlMode;            //控制模式
extern FricStatus_Enum FricStatus;    //摩擦轮状态
extern StirStatus_Enum          StirStatus;             //拨盘状态
extern AimModeStatus_Enum   AimStatus;//瞄准状态
extern uint8_t ShootStatus;

typedef enum
{
    Comm_Off = 0,
    Comm_On
}BoardToBoardComm_Enum;
extern BoardToBoardComm_Enum BoardToBoardComm;

void StatusMachine_Init(void);
void StatusMachine_Update(void);

#endif
