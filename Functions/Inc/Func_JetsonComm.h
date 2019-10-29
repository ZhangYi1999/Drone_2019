#ifndef __FUNC_JETSONCOMM_H
#define __FUNC_JETSONCOMM_H

#include "Task_SystInit.h"
#include <arm_math.h> 


#define mat_init   arm_mat_init_f32
#define mat_add     arm_mat_add_f32 
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32

#define mat         arm_matrix_instance_f32

/*
    镜头右下旋转为正
    镜头右移前进为正
    哨兵向轨道分段标号增加方向运动为正
*/

#define JetsonCommReservedFrameLEN  5

#define JETSONFLAG_LEN      16

//帧头帧尾
#define JetsonCommSOF     0x66
#define JetsonCommEOF     0x88
#define CommSetUp         (uint16_t)0xCCCC
#define RecordAngle       (uint16_t)0xffff
#define RequestTrans      (uint16_t)0xbbbb
//比赛红蓝方
#define BlueTeam          (uint16_t)0xDDDD
#define RedTeam           (uint16_t)0xEEEE
//发射方式
#define NoFire            (uint16_t)(0x00<<8)      //不发射
#define BurstFire         (uint16_t)(0x01<<8)      //点射
#define RunningFire       (uint16_t)(0x02<<8)      //连发
//发射速度（高速低速）
#define HighBulletSpeed   (uint16_t)(0x01)
#define LowBulletSpeed    (uint16_t)(0x02)
//所需控制模式
#define ManualMode        (uint8_t)(0x00)          //手动控制
#define SmallBuffMode     (uint8_t)(0x01)          //小符模式
#define BigBuffMode       (uint8_t)(0x02)          //大符模式
#define AutoShootMode     (uint8_t)(0x03)          //自动射击
//哨兵云台工作模式
#define RotatinPatrol     (uint8_t)(0x01)          //旋转巡逻
#define PatrolArmor0      (uint8_t)(0x02)          //巡逻装甲板0
#define PatrolArmor1      (uint8_t)(0x03)          //巡逻装甲板1
#define ServoMode         (uint8_t)(0x04)          //伺服打击

typedef struct{
    uint8_t  SoF;
    uint8_t  Seq;
    uint16_t ShootMode;            //高八位发射方式 低八位发射速度等级  (0xFFFF-记录当前角度  0xEEEE-红方  0xDDDD-蓝方  0xCCCC-通信建立  0xBBBB-请求数据发送)
    float    TargetPitchAngle;     //Pitch目标角度
    float    TargetYawAngle;       //Yaw目标角度
    /*  哨兵专用   */
    int16_t  TargetSpeedOnRail;    //目标轨道速度（哨兵用）     
    uint8_t  SentryGimbalMode;     //哨兵云台攻击模式
    uint8_t  EoF;
} JetsonToSTM_Struct;

typedef struct
{
	//记录读图时的角度
	float CurAngle_Pitch;
	float CurAngle_Yaw;
	//此次上一次绝对角度
	float Velocity_Pitch;
	float Velocity_Yaw;
	//是否记录过角度
	uint8_t ChangeAngle_flag;
}JetsonFlag_Struct;

typedef struct{
    uint8_t  SoF;
    uint8_t  Seq;
    uint8_t  NeedMode;             //所需控制模式
    uint8_t  ShootSpeed;           //射速
    /*  哨兵专用   */
    uint8_t  RailNum;              //所处轨道标号
    uint8_t  ArmorType;            //被打击装甲板标识
    uint16_t RemainHP;             //剩余血量
    uint8_t  Reserved[11];         //保留字节
    uint8_t  EoF;
} STMToJetson_Struct;

typedef struct{
    uint16_t  team;
    uint8_t   CommSuccess;
} CommStatus_Struct;

extern JetsonFlag_Struct JetsonFlag[JETSONFLAG_LEN];
extern uint8_t Jetson_Seq;

extern  JetsonToSTM_Struct  DataRecFromJetson;
extern  STMToJetson_Struct  DataSendToJetson;
extern  CommStatus_Struct   CommStatus;
extern float Pitch_Desire, Yaw_Desire;

void JetsonCommUart_Config(UART_HandleTypeDef *huart);
void JetsonCommUart_ReConfig_In_IRQHandler(UART_HandleTypeDef *huart);
void JetsonComm_Control(UART_HandleTypeDef *huart);

extern float Pitch_Desire, Yaw_Desire;
extern float Jetson_AnglePitch;
extern float Jetson_AngleYaw;
extern float Jetson_VelocityPitch;
extern float Jetson_VelocityYaw;


typedef struct
{
    float raw_value;
    float filtered_value[2];
    mat xhat; 
    mat xhatminus; 
    mat z; 
    mat A; 
    mat H; 
    mat AT; 
    mat HT; 
    mat Q; 
    mat R; 
    mat P; 
    mat Pminus; 
    mat K;
		mat B;//
		mat u;//
}kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
	float B_data[2];//
	float *u_data;//
}kalman_filter_init_t;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);

float *amended_kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2, float signal3);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2, float signal3);

void KF_Init(void);
void KF_Cal_Desire(void);
void Version_Init(void);
#endif
