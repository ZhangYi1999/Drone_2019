#ifndef __FUNC_BOARDTOBOARDCOMM_H
#define __FUNC_BOARDTOBOARDCOMM_H

#include "System_Config.h"
#include "Func_StatusMachine.h"

#ifdef __FUNC_BOARDTOBOARDCOMM_GLOBALS
#define __FUNC_BOARDTOBOARDCOMM_EXT
#else
#define __FUNC_BOARDTOBOARDCOMM_EXT extern
#endif
typedef struct
{
    uint8_t location;      //所处位置(分段)
    uint8_t Shoot_Speed;   //射速
    uint16_t Blood_Volume; //剩余血量
    uint8_t Armored;       //被攻击的装甲板号
    int16_t YAW_Angle;     //YAW轴角度
    uint16_t Heat;         //当前热量值
} Slave_Send_Struct;

typedef struct
{
    int16_t Tar_Speed;
    chassis_mode_Enum Chassis_Mode;
} Master_Send_Struct;

#define SLAVE_SEND_ID 0x66
#define MASTER_SEND_ID 0x67

__FUNC_BOARDTOBOARDCOMM_EXT Slave_Send_Struct Slave_Value;
__FUNC_BOARDTOBOARDCOMM_EXT Master_Send_Struct Master_Value;

void Slave_Send(void);
void Master_Send(void);

#endif
