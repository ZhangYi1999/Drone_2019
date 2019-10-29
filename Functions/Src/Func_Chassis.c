#define __FUNC_CHASSIS_GLOBALS

#include "Func_Chassis.h"
#include "Func_BoardToBoardComm.h"
#include "Func_StatusMachine.h"
#include "Task_Tasks.h"

#ifdef SLAVE_MODE

#define LEFT_SWITCH_STATUS HAL_GPIO_ReadPin(Left_Switch_GPIO_Port, Left_Switch_Pin)
#define MID_SWITCH_STATUS HAL_GPIO_ReadPin(Mid_Switch_GPIO_Port, Mid_Switch_Pin)
#define RIGHT_SWITCH_STATUS HAL_GPIO_ReadPin(Right_Switch_GPIO_Port, Right_Switch_Pin)

//事件位宏
#define imu_ob_temp_pos 0
#define imu_ob_angle_pos 1
#define chassis_pos 2

extern EventGroupHandle_t EG_Wait;

//调参使用
float i, d = 0;
float p = 0;
int16_t TargetSpeed = 0;

ChassisParam_Struct ChassisParam = {
    .Chassis_Mode = chassis_run,
    .Motor.PID.Kp = 15,   //0.5,
    .Motor.PID.Ki = 0.25, //0.05,
    .Motor.PID.Kd = 2,   //1,
    .Motor.Mechanical_n = 0,
    .Motor.Current_n = 0,
};

static void M3508_PID_Ctrl(ChassisParam_Struct *chassis);

/**
  * @brief  底盘电机总体控制
  * @param  unused
  * @retval void
  */
void Chassis_Ctrl(void)
{
  M3508_PID_Ctrl(&ChassisParam);
}

/**
  * @brief  底盘电机速度闭环 
  * @param  底盘结构体
  * @retval void
  */
void M3508_PID_Ctrl(ChassisParam_Struct *chassis)
{
  /**************  Motor  ***************/
  chassis->Motor.PID.Last_Error = chassis->Motor.PID.Cur_Error;
  chassis->Motor.PID.Cur_Error = chassis->Motor.TargetSpeed - chassis->Motor.RealSpeed;
  chassis->Motor.PID.Sum_Error += chassis->Motor.PID.Cur_Error;
  //积分上限
  chassis->Motor.PID.Sum_Error = chassis->Motor.PID.Sum_Error > 15000 ? 15000 : chassis->Motor.PID.Sum_Error;
  chassis->Motor.PID.Sum_Error = chassis->Motor.PID.Sum_Error < -15000 ? -15000 : chassis->Motor.PID.Sum_Error;

  chassis->Motor.PID.Output = (int32_t)(chassis->Motor.PID.Kp * chassis->Motor.PID.Cur_Error + chassis->Motor.PID.Ki * chassis->Motor.PID.Sum_Error + chassis->Motor.PID.Kd * (chassis->Motor.PID.Cur_Error - chassis->Motor.PID.Last_Error));
  //限制输出电流
  chassis->Motor.NeedCurrent = (chassis->Motor.PID.Output >= C620CURRENTMAX) ? C620CURRENTMAX : (int16_t)chassis->Motor.PID.Output;
  chassis->Motor.NeedCurrent = (chassis->Motor.PID.Output <= -C620CURRENTMAX) ? -C620CURRENTMAX : (int16_t)chassis->Motor.NeedCurrent;
}

/**
  * @brief  底盘电机数据发送  --CAN1
  * @param  地盘电机电流
  * @retval None
  */
void M3508_CMD_Trans(int16_t Output)
{
  static CanSend_Type CANSend;

  CANSend.CANx = CAN_NUMBER_1;

  CANSend.SendCanTxMsg.StdId = CHASSISSENDCANID;
  CANSend.SendCanTxMsg.IDE = CAN_ID_STD;
  CANSend.SendCanTxMsg.RTR = CAN_RTR_DATA;
  CANSend.SendCanTxMsg.DLC = 0x08;

  CANSend.SendCanTxMsg.Data[0] = (uint8_t)(Output >> 8);
  CANSend.SendCanTxMsg.Data[1] = (uint8_t)Output;
  CANSend.SendCanTxMsg.Data[2] = (uint8_t)0;
  CANSend.SendCanTxMsg.Data[3] = (uint8_t)0;
  CANSend.SendCanTxMsg.Data[4] = (uint8_t)0;
  CANSend.SendCanTxMsg.Data[5] = (uint8_t)0;
  CANSend.SendCanTxMsg.Data[6] = (uint8_t)0;
  CANSend.SendCanTxMsg.Data[7] = (uint8_t)0;

  xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);
}

/**
  * @brief  判断电机转子机械角是否过0  --CAN1
  * @param  None
  * @retval 正向过返回1，反向过返回-1，不过返回0
  */
int8_t If_Pass_Zero(void)
{
  if (ChassisParam.Motor.RealSpeed > 0) //正向过0
  {
    if (ChassisParam.Motor.Last_Mechanical_Angle > ChassisParam.Motor.Mechanical_Angle)
      return 1;
    else
      return 0;
  }
  else if (ChassisParam.Motor.RealSpeed < 0)
  {
    if (ChassisParam.Motor.Last_Mechanical_Angle < ChassisParam.Motor.Mechanical_Angle)
      return -1;
    else
      return 0;
  }
  else
    return 0;
}


/**
  * @brief  地盘到初始化位置时的控制
  * @param  None
  * @retval None
  */
void Chassis_Ctrl_Init(void)
{
  ChassisParam.Motor.TargetSpeed = 300; //复位时初速度为300

  if (!LEFT_SWITCH_STATUS)
  {
    xEventGroupSetBits(EG_Wait, 0x01 << chassis_pos);
    ChassisParam.Motor.TargetSpeed = 0;
    ChassisParam.Motor.Current_n = 0;
    ChassisParam.Motor.Mechanical_n = 0;
    LED3_ON();
  }
  Chassis_Ctrl();
  M3508_CMD_Trans(ChassisParam.Motor.NeedCurrent);
}

/**
  * @brief  设置地盘移动速度
  * @param  None
  * @retval None
  */

void Chassis_Speed_Set(void)
{
  if (LEFT_SWITCH_STATUS == 0) //判断左边接近开关
    {
      if (ChassisParam.Motor.TargetSpeed > 0) //判断接收到的目标速度
        ChassisParam.Motor.TargetSpeed = 0;
      if (Master_Value.Tar_Speed > 0) //判断当前移动速度
        ChassisParam.Motor.TargetSpeed = 0;
      else
        ChassisParam.Motor.TargetSpeed = Master_Value.Tar_Speed;
    }
    else if (RIGHT_SWITCH_STATUS == 0) //判断右边接近开关
    {
      if (ChassisParam.Motor.TargetSpeed < 0) //判断接收到的目标速度
        ChassisParam.Motor.TargetSpeed = 0;
      if (Master_Value.Tar_Speed < 0) //判断当前移动速度
        ChassisParam.Motor.TargetSpeed = 0;
      else
        ChassisParam.Motor.TargetSpeed = Master_Value.Tar_Speed;
    }
    else
      ChassisParam.Motor.TargetSpeed = Master_Value.Tar_Speed; //若各个接近开关均未检测到障碍，直接赋值
    
}
#endif
