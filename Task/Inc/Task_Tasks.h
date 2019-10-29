#ifndef __TASK_TASKS_H
#define __TASK_TASKS_H

#include "System_Config.h"

/*********  CANSend  *********/
#define CAN_NUMBER_1    ((uint8_t)1)
#define CAN_NUMBER_2    ((uint8_t)2)

typedef struct
{
    uint8_t            CANx;               //CAN编号     1 CAN1      2 CAN2
    CAN_TxHeaderTypeDef    SendCanHeader;       //发送数据
		uint8_t SendCanTxMsg[8];
}CanSend_Type;

extern QueueHandle_t Queue_CANSend;            //CAN发送队列
extern EventGroupHandle_t EG_Wait;

void Task_CANSend(void* parameter);
void Task_Control_Normal(void* Parameter);
void Task_JetsonComm(void *Parameter);
void Task_GYROData_Update(void* Parameter);
void Task_JudgeReceive(void *parameters);
void Task_Imu_Ob(void *parameter);
void Task_LEDBlink(void *parameter);
void Task_Monitor(void *parameter);
void Task_OLEDDisplay(void *parameter);
void Task_RCData_Update(void* Parameter);
void Task_StatusMachineUpdate(void* Parameter);
void Task_Board_To_Board_Comm(void* Parameter);
void Task_InitLED(void *Parameter);
void Task_Pitch_GYRO(void *Parameter);

#endif
