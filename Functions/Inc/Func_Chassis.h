#ifndef __FUNC_CHASSIS_H
#define __FUNC_CHASSIS_H

#include "System_Config.h"
#include "Func_StatusMachine.h"

#ifdef __FUNC_CHASSIS_GLOBALS
#define __FUNC_CHASSIS_EXT
#else
#define __FUNC_CHASSIS_EXT extern
#endif

//底盘数据发送CANID
#define CHASSISCANID 0x205
//底盘数据接收CANID
#define CHASSISSENDCANID 0x1FF
//M3508电机最大速度
#define M3508SPEEDMAX 8800.0
//C620电调电流最大值
#define C620CURRENTMAX 16000
//电机控制模式
#define CHASSISS_STOP 0
#define CHASSISS_RUN 1

//速度类型枚举
typedef enum
{
    ChassisSpeedLevel_Hight = 0,
    ChassisSpeedLevel_Low
} ChassisSpeedLevel_Enum;

//M3508 PID结构体
typedef struct
{
    float Kp, Ki, Kd;
    int16_t Cur_Error, Last_Error, Sum_Error;
    int32_t Output;
} M3508_PID_Struct;

//单个电机参数
typedef struct
{
    
    int16_t Current_n;              //电机圈数
    int16_t Mechanical_n;           //转子圈数
    uint16_t Mechanical_Angle;      //当前机械角
    uint16_t Last_Mechanical_Angle; //原先机械角
    int16_t TargetSpeed;
    int16_t RealSpeed;
    int16_t NeedCurrent;
    int16_t RealCurrent;
    uint16_t FrameCounter; //帧率计数器
    M3508_PID_Struct PID;
} OneMotorParam_Struct;

//底盘参数
typedef struct
{
    chassis_mode_Enum Chassis_Mode;
    OneMotorParam_Struct Motor;
    ChassisSpeedLevel_Enum SpeedLevel;
    // int16_t TargetABSAngle;
} ChassisParam_Struct;

//底盘状态
extern ChassisParam_Struct ChassisParam;
void M3508_CMD_Trans(int16_t Output);
void Chassis_Ctrl(void);
int8_t If_Pass_Zero(void);
void Chassis_Ctrl_Init(void);
void Chassis_Speed_Set(void);

#endif
