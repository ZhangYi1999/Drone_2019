
#define __TASK_TEST_GLOBALS

#include "Task_Test.h"
#include "Func_Chassis.h"

#ifdef MASTER_MODE
#include "Func_GimbalMotor.h"
#include "Func_FricMotor.h"
#include "Func_GYRO.h"
#include "Func_JetsonComm.h"
#endif

#include "Func_Imu_OB.h"

/**
  * @brief  功能1测试任务
  * @param  unused
  * @retval void
  */
uint8_t temp = 0;
int probe1 = 0,probe2 = 0,probe3,probe4,probe5;
void Task_Test1(void *parameter)
{
    while (1)
    {
	

    }
}

int16_t test_RELAY = 0;
int16_t test_chassis = 0;

/**
  * @brief  功能2测试任务
  * @param  unused
  * @retval void
  */
void Task_Test2(void *parameter)
{
    while (1)
    {
	#ifdef MASTER_MODE

//        printf("ts %f\t cs %f\t op %d\t",Motor2006_Pitch.TargetSpeed,PersonalGYRO.Gyro_X,Motor2006_Pitch.NeedCurrent);        
//        printf("ta %f\t ca %f\t op %f\r\n",Motor2006_Pitch.TargetAngle,\
//                                                    PersonalGYRO.PitchAngle,\
//                                                    Motor2006_Pitch.PositionPID.Output);        
//        printf("ts %f\t cs %f\t op %d\r\n",Motor2006_Yaw.TargetSpeed,PersonalGYRO.Gyro_Z,Motor2006_Yaw.NeedCurrent);        
//        printf("ta %f\t ca %f\t op %f\r\n",Motor2006_Pitch.TargetAngle,\
//                                                    PersonalGYRO.PitchAngle,\
//                                                    Motor2006_Pitch.PositionPID.Output);        
	#endif
        vTaskDelay(5);
    }
}

#if (RTOS_DEBUG_ENABLE == 1)

/**
  * @brief  获取系统状态（仅调试用）
  * @param  unused
  * @retval void
  */
void PrintSystemState(void)
{
    uint32_t TotalRunTime;
    UBaseType_t ArraySize, x;
    TaskStatus_t *StatusArray;

    printf("/********第一步：函数uxTaskGetSystemState()的使用**********/\r\n");
    ArraySize = uxTaskGetNumberOfTasks();                         //获取系统任务数量
    StatusArray = pvPortMalloc(ArraySize * sizeof(TaskStatus_t)); //申请内存
    if (StatusArray != NULL)                                      //内存申请成功
    {
        ArraySize = uxTaskGetSystemState((TaskStatus_t *)StatusArray, //任务信息存储数组
                                         (UBaseType_t)ArraySize,      //任务信息存储数组大小
                                         (uint32_t *)&TotalRunTime);  //保存系统总的运行时间
        printf("TaskName\t\tPriority\t\tTaskNumber\t\t\r\n");
        for (x = 0; x < ArraySize; x++)
        {
            //通过串口打印出获取到的系统任务的有关信息，比如任务名称、
            //任务优先级和任务编号。
            printf("%s\t\t%d\t\t\t%d\t\t\t\r\n",
                   StatusArray[x].pcTaskName,
                   (int)StatusArray[x].uxCurrentPriority,
                   (int)StatusArray[x].xTaskNumber);
        }
    }
    vPortFree(StatusArray); //释放内存
    printf("/**************************结束***************************/\r\n");
}

/**
  * @brief  获取任务信息（仅调试用）
  * @param  任务名 (char [])
  * @retval void
  */
void PrintTaskInfo(char str[])
{
    TaskHandle_t TaskHandle;
    TaskStatus_t TaskStatus;

    printf("/************第二步：函数vTaskGetInfo()的使用**************/\r\n");
    TaskHandle = xTaskGetHandle(str); //根据任务名获取任务句柄。
    //获取LED0_Task的任务信息
    vTaskGetInfo((TaskHandle_t)TaskHandle,    //任务句柄
                 (TaskStatus_t *)&TaskStatus, //任务信息结构体
                 (BaseType_t)pdTRUE,          //允许统计任务堆栈历史最小剩余大小
                 (eTaskState)eInvalid);       //函数自己获取任务运行状态
    //通过串口打印出指定任务的有关信息。
    printf("任务名:                %s\r\n", TaskStatus.pcTaskName);
    printf("任务编号:              %d\r\n", (int)TaskStatus.xTaskNumber);
    printf("任务状态:              %d\r\n", TaskStatus.eCurrentState);
    printf("任务当前优先级:        %d\r\n", (int)TaskStatus.uxCurrentPriority);
    printf("任务基优先级:          %d\r\n", (int)TaskStatus.uxBasePriority);
    printf("任务堆栈基地址:        %#x\r\n", (int)TaskStatus.pxStackBase);
    printf("任务堆栈历史剩余最小值:%d\r\n", TaskStatus.usStackHighWaterMark);
    printf("/**************************结束***************************/\r\n");
}

/**
  * @brief  获取任务状态（仅调试用）
  * @param  任务名 (char [])
  * @retval void
  */
void PrintTaskState(char str[])
{
    TaskHandle_t TaskHandle;
    eTaskState TaskState;
    char TaskInfo[10];

    printf("/***********第三步：函数eTaskGetState()的使用*************/\r\n");
    TaskHandle = xTaskGetHandle(str);      //根据任务名获取任务句柄。
    TaskState = eTaskGetState(TaskHandle); //获取query_task任务的任务状态
    memset(TaskInfo, 0, 10);
    switch ((int)TaskState)
    {
    case 0:
        sprintf(TaskInfo, "Running");
        break;
    case 1:
        sprintf(TaskInfo, "Ready");
        break;
    case 2:
        sprintf(TaskInfo, "Suspend");
        break;
    case 3:
        sprintf(TaskInfo, "Delete");
        break;
    case 4:
        sprintf(TaskInfo, "Invalid");
        break;
    }
    printf("任务状态值:%d,对应的状态为:%s\r\n", TaskState, TaskInfo);
    printf("/**************************结束**************************/\r\n");
}

/**
  * @brief  打印任务列表（仅调试用）
  * @param  unused
  * @retval void
  * @note   该函数需要分配大量堆栈（500即可）
  */
void PrintTaskListing(void)
{
    char InfoBuffer[1000]; //保存信息的数组

    printf("/*************第三步：函数vTaskList()的使用*************/\r\n");
    vTaskList(InfoBuffer);        //获取所有任务的信息
    printf("%s\r\n", InfoBuffer); //通过串口打印所有任务的信息
    printf("/**************************结束**************************/\r\n");
}

#endif
