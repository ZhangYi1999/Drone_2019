#define __FUNC_BOARDTOBOARDCOMM_GLOBALS

#include "Func_BoardToBoardComm.h"
#include "Func_StatusMachine.h"
#include "Task_Tasks.h"

#ifdef SLAVE_MODE

void Slave_Send()
{
    static CanSend_Type CANSend;

    CANSend.CANx = CAN_NUMBER_1;

    CANSend.SendCanTxMsg.StdId = SLAVE_SEND_ID;
    CANSend.SendCanTxMsg.IDE = CAN_ID_STD;
    CANSend.SendCanTxMsg.RTR = CAN_RTR_DATA;
    CANSend.SendCanTxMsg.DLC = 0x08;

    CANSend.SendCanTxMsg.Data[0] = (uint8_t)(Slave_Value.location << 4 | Slave_Value.Shoot_Speed);
    CANSend.SendCanTxMsg.Data[1] = (uint8_t)(Slave_Value.Blood_Volume >> 8);
    CANSend.SendCanTxMsg.Data[2] = (uint8_t)Slave_Value.Blood_Volume;
    CANSend.SendCanTxMsg.Data[3] = (uint8_t)Slave_Value.Armored;
    CANSend.SendCanTxMsg.Data[4] = (uint8_t)(Slave_Value.YAW_Angle >> 8);
    CANSend.SendCanTxMsg.Data[5] = (uint8_t)Slave_Value.YAW_Angle;
    CANSend.SendCanTxMsg.Data[6] = (uint8_t)(Slave_Value.Heat >> 8);
    CANSend.SendCanTxMsg.Data[7] = (uint8_t)Slave_Value.Heat;

    xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);
}
#else
#include "Func_RC.h"
#include "Func_JetsonComm.h"
#include "Func_Chassis.h"

void Master_Send()
{
    static CanSend_Type CANSend;

    //发送数据赋值
    switch (ControlMode)
    {
    case ControlMode_RC:
        Master_Value.Chassis_Mode = chassis_run;
        Master_Value.Tar_Speed = M3508SPEEDMAX * Get_Channel_Val(&RC_ReceiveData, RC_CH0) / RC_CH_MAX_RELATIVE;
        break;
    case ControlMode_Auto:
        Master_Value.Chassis_Mode = chassis_run;
        Master_Value.Tar_Speed = DataRecFromJetson.TargetSpeedOnRail;
        break;
    default:
        Master_Value.Chassis_Mode = chassis_stop;
        Master_Value.Tar_Speed = 0;
        break;
    }

    CANSend.CANx = CAN_NUMBER_1;

    CANSend.SendCanHeader.StdId = MASTER_SEND_ID;
    CANSend.SendCanHeader.IDE = CAN_ID_STD;
    CANSend.SendCanHeader.RTR = CAN_RTR_DATA;
    CANSend.SendCanHeader.DLC = 0x08;

    CANSend.SendCanTxMsg[0] = (uint8_t)(Master_Value.Tar_Speed >> 8);
    CANSend.SendCanTxMsg[1] = (uint8_t)(Master_Value.Tar_Speed);
    CANSend.SendCanTxMsg[2] = (uint8_t)Master_Value.Chassis_Mode;
    CANSend.SendCanTxMsg[3] = (uint8_t)0;
    CANSend.SendCanTxMsg[4] = (uint8_t)0;
    CANSend.SendCanTxMsg[5] = (uint8_t)0;
    CANSend.SendCanTxMsg[6] = (uint8_t)0;
    CANSend.SendCanTxMsg[7] = (uint8_t)0;

    xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);
}
#endif
