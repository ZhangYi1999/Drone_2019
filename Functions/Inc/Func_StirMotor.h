#ifndef __FUNC_STIRMOTOR_H
#define __FUNC_STIRMOTOR_H

#include "System_Config.h"

#ifdef  __FUNC_STIRMOTOR_GLOBALS
#define __FUNC_STIRMOTOR_EXT
#else
#define __FUNC_STIRMOTOR_EXT    extern
#endif

#define STIRMOTORRECEIVECANID               0x204
#define C610CURRENTMAX                      10000

#define ONECIRCLECOUNTVALUEAFTERDECELERATE  786432       //96*8192
#define ONECIRCLECOUNTVALUEORIGINAL         (uint16_t)8192

#define CROSSZEROUPTHRESHOLD                6000
#define CROSSZERODOWNTHRESHOLD              2000

#define StirMotorChangeToAbsAngle(x)        (float)(360.0f*(x)/ONECIRCLECOUNTVALUEAFTERDECELERATE)   

//过零检测
typedef struct {
    uint8_t  InUpHalfCircle;
    uint8_t  InDownHalfCircle;
    int32_t  ZreoCounter;                   //过零计数器
}CrossZeroCheck_Struct;

//PID
typedef struct{
    float Kp;
    float Ki;
    float Kd;

    float CurrentError;
    float LastError;
    float SumError;

    float Output;
}StirMotorPID_Struct;

//电机参数
typedef struct {
    uint16_t RealAngle;
    uint16_t LastRealAngle;
    uint16_t AngleOffset;                   //角度控制模式时第一次获取的角度
    int16_t  RealSpeed;
    int16_t  TargetSpeed;
    uint16_t FrameCounter;                  //帧率计数器

    float  RealAngle_Decelerate;            //减速后相对角度
    float  TargetAngle_Decelerate;    
    CrossZeroCheck_Struct CrossZero;
    StirMotorPID_Struct   AnglePID;
    StirMotorPID_Struct   SpeedPID;
}StirMotor_Struct;

typedef struct{
		uint16_t LastFrameCounter;
    uint16_t FrameCounter;
    int16_t  RealSpeed;
    int16_t  Mechanical_Angle;
    int16_t  NeedCurrent;
    int16_t  TargetSpeed;
    int8_t  BlockedWarningTimes;
    //820R系列 PID结构体
    struct
    {
        float Kp,Ki,Kd;
        int16_t Cur_Error,Last_Error,Sum_Error;
        int32_t Output;
    } PID;
} ESC_820R;

extern ESC_820R  RM2006_StirMotor;

void StirMotor_Control(void);
void  Cross_Zero_Settle(StirMotor_Struct* RM);
void StirMotor_Get_Angle(StirMotor_Struct* RM);
float StirMotorChangeToPhysicalAngle(int64_t temp);

#endif
