#ifndef __TASK_SYSYINIT_H
#define __TASK_SYSYINIT_H


#include "System_Config.h"

#ifdef  __TASK_SYSTINIT_GLOBALS
#define __TASK_SYSTINIT_EXT
#else
#define __TASK_SYSTINIT_EXT extern
#endif


/************   ÈÎÎñ¾ä±ú    ************/
__TASK_SYSTINIT_EXT   TaskHandle_t    SystInitTask_Handle;

__TASK_SYSTINIT_EXT   TaskHandle_t    CANSendTask_Handle;
__TASK_SYSTINIT_EXT   TaskHandle_t    NormalControlTask_Handle;
__TASK_SYSTINIT_EXT   TaskHandle_t    LEDTask_Handle;
__TASK_SYSTINIT_EXT   TaskHandle_t    OLEDTask_Handle;
__TASK_SYSTINIT_EXT   TaskHandle_t    RCUpdateTask_Handle;
__TASK_SYSTINIT_EXT   TaskHandle_t    GYROUpdateTask_Handle;
__TASK_SYSTINIT_EXT   TaskHandle_t    PitchGYROTask_Handle;
__TASK_SYSTINIT_EXT   TaskHandle_t    StatusMachineTask_Handle;
__TASK_SYSTINIT_EXT   TaskHandle_t    MonitorTask_Handle;
__TASK_SYSTINIT_EXT   TaskHandle_t    JudgeReceiveTask_Handle;
__TASK_SYSTINIT_EXT   TaskHandle_t    ImuObTask_Handle;
__TASK_SYSTINIT_EXT   TaskHandle_t    JetsonCommTask_Handle;
__TASK_SYSTINIT_EXT   TaskHandle_t    BoardToBoardCommTask_Handle;
__TASK_SYSTINIT_EXT   TaskHandle_t    InitLEDTask_Handle;

__TASK_SYSTINIT_EXT   TaskHandle_t    Test1Task_Handle;
__TASK_SYSTINIT_EXT   TaskHandle_t    Test2Task_Handle;
/**************************************/



#endif

