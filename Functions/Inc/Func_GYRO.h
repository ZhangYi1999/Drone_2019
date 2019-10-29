#ifndef __FUNC_GYRO_H
#define __FUNC_GYRO_H
#endif
#include "System_Config.h"


#define PersonalGYRO_rx_len 79
#define PersonalGYROFrameLength 50 
//帧头
#define GYROFRAMEHEAD0 0X5A
#define GYROFRAMEHEAD1 0XA5

#define OFFICIALZGYRORESETCANID 0x404   //官方陀螺仪复位CANID
#define OFFICIALZGYRORECEIVECANID 0x401 //官方陀螺仪接收CANID



typedef struct
{
		uint8_t num;
    uint16_t FrameCounter;
    float PitchAngle;
    float YawAngle;
    float RollAngle;
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
} GYRO_Struct;

extern GYRO_Struct OfficialGYRO;
extern GYRO_Struct PersonalGYRO,PersonalGYRO2;
extern uint8_t GYROBuffer[2][PersonalGYRO_rx_len],GYROBuffer2[2][PersonalGYRO_rx_len];
extern uint8_t GYRO_Rx_Mem,GYRO_Rx_Mem2;

void Official_GYRO_Init(void);
void Personal_GYRO_Receive(GYRO_Struct *gyro);
void Uart_Config_In_IRQHandle(UART_HandleTypeDef *huart);
void Uart_Config_In_IRQHandle2(UART_HandleTypeDef *huart);
void PersonalGYRO_Recevie_Enable(void);
