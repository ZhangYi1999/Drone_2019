/*  该文件包含正常功能的所有任务函数  */

#include "Task_Tasks.h"
#include "Task_SystInit.h"
#include "Func_StatusMachine.h"
#include "Func_BoardToBoardComm.h"
#include "Func_Imu_OB.h"
#include "Func_JudgeReceive.h"
#include "Func_RC.h"
#include "Func_GimbalMotor.h"
#include "Func_FricMotor.h"
#include "Func_StirMotor.h"
#include "Func_GYRO.h"
#include "Func_JetsonComm.h"


/**
  * @brief  CAN发送任务
  * @param  unused
  * @retval void
  */
QueueHandle_t Queue_CANSend; //CAN发送队列
EventGroupHandle_t EG_Wait;

void Task_CANSend(void *parameter)
{
  CanSend_Type CAN_Tx_Msg;
  while (1)
  {
    xQueueReceive(Queue_CANSend, &CAN_Tx_Msg, portMAX_DELAY);
    switch (CAN_Tx_Msg.CANx)
    {
    case CAN_NUMBER_1:
      HAL_CAN_AddTxMessage(&hcan1,&CAN_Tx_Msg.SendCanHeader,CAN_Tx_Msg.SendCanTxMsg,(uint32_t *)CAN_TX_MAILBOX0);
      break;
    case CAN_NUMBER_2:
       HAL_CAN_AddTxMessage(&hcan2,&CAN_Tx_Msg.SendCanHeader,CAN_Tx_Msg.SendCanTxMsg,(uint32_t *)CAN_TX_MAILBOX0);
      break;
    default:
      break;
    }
  }
}

/**
  * @brief  LED闪烁任务
  * @param  unused 
  * @retval void
  */
uint16_t offline=0;
int16_t testoutput_r = 1000;
int16_t testoutput_l = 1000;
uint8_t inittemp=0;
void Task_LEDBlink(void *parameter)
{
  portTickType CurrentControlClock = 0;
  CurrentControlClock = xTaskGetTickCount();
  while (1)
  {
    Green_LED_Blink();
    
		
    FricMotor_Get_Speed();
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,testoutput_l);
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,testoutput_r);
		
//		if(offline==0)
//			FricMotor_PID_Ctrl();
//		if(ControlMode==ControlMode_Protect)
//			FricStatus=FricStatus_Stop;
		//if(aerial_robot_energy.energy_point>0&&aerial_robot_energy.attack_time>0)
		
//		if(ext_game_robot_state.mains_power_shooter_output==1&&inittemp==0)
//		{
//			FricMotor_Init();
//			inittemp=1;
//		}
		//FricMotor_Ctrl();
		//__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,testoutput);
		FricMotor_Ctrl();
		vTaskDelayUntil(&CurrentControlClock, 100);
    //vTaskDelayUntil(&CurrentControlClock, 5);
  }
}

/**
  * @brief  状态机更新任务
  * @param  unused
  * @retval void
  * @note   10ms执行一次   
  */
void Task_StatusMachineUpdate(void *Parameter)
{
  TickType_t xLastWakeUpTime;
  xLastWakeUpTime = xTaskGetTickCount();
  while (1)
  {
    
		StatusMachine_Update();
		vTaskDelayUntil(&xLastWakeUpTime,10);
  }
}


/**
 * @brief  系统初始化LED指示
 * @param  unused
 * @retval none
 */
void Task_InitLED(void *Parameter)
{
  while(1)
  {
    // if(xEventGroupGetBits(EG_Wait) & (0X1 << imu_ob_temp_pos))
    //   LED0_ON();
    // if(xEventGroupGetBits(EG_Wait) & (0X1 << imu_ob_angle_pos))
    //   LED1_ON();
    // if(xEventGroupGetBits(EG_Wait) & (0X1 << imu_rec_pos))
    //   LED2_ON();
    // if(xEventGroupGetBits(EG_Wait) & (0X1 << jetson_pos))
    //   LED3_ON();
    // if(xEventGroupGetBits(EG_Wait) & (0X1 << btb_pos))
    //   LED4_ON();

    // //所有都完成后删除此任务
    // if(xEventGroupGetBits(EG_Wait) &\
    //     (
    //       (0X1 << imu_ob_temp_pos) |\
    //       (0X1 << imu_ob_angle_pos) |\
    //       (0X1 << imu_rec_pos) |\
    //       (0X1 << jetson_pos) |\
    //       (0X1 << btb_pos)
    //     )
    //   )
      vTaskDelete(NULL);

    vTaskDelay(10);
  }
}

/**
  * @brief  控制任务(仅底盘云等CAN1用户普通控制)
  * @param  unused
  * @retval void
  * @note   5ms执行一次   
  * @note   不可在此任务中进行任何打印操作！！！
  */
uint8_t offline_time=0;
void Task_Control_Normal(void *Parameter)
{
	vTaskDelay(1000);
  portTickType CurrentControlClock = 0;
  Gimbal_Init();
	
  CurrentControlClock = xTaskGetTickCount();
  while (1)
  {
		
		monitor_Yaw_RealSpeed=(int32_t)(Motor2006_Yaw.RealSpeed*100);
    
    Gimbal_Ctrl();
		if(RM2006_StirMotor.FrameCounter>RM2006_StirMotor.LastFrameCounter)
		{
			offline_time=0;
			offline=0;
		}
		else
			offline_time++;
		if(RM2006_StirMotor.FrameCounter>10000)
		{
			RM2006_StirMotor.FrameCounter=100;
			RM2006_StirMotor.LastFrameCounter=0;
		}
		
		if(offline_time>20)
			offline=1;
		if(offline==0)
			StirMotor_Control();
		else
		{
			RM2006_StirMotor.PID.Sum_Error=0;
			StirStatus=StirStatus_Stop;
			FricStatus=FricStatus_Stop;
		}
		RM2006_StirMotor.LastFrameCounter=RM2006_StirMotor.FrameCounter;
		if(RM2006_StirMotor.FrameCounter>10000)
		{
			RM2006_StirMotor.FrameCounter=100;
			RM2006_StirMotor.LastFrameCounter=10;
		}
		
		
    if (ControlMode == ControlMode_Protect)
    {
      Gimbal_CMD_Trans(0, 0, 0);
    }
    else
    {
      //Gimbal_CMD_Trans(Motor2006_Yaw.NeedCurrent, Motor6020_Pitch.NeedCurrent, RM2006_StirMotor.NeedCurrent);
			Gimbal_CMD_Trans(Motor2006_Yaw.NeedCurrent, Motor6020_Pitch.NeedCurrent, RM2006_StirMotor.NeedCurrent);
			//Gimbal_CMD_Trans(Motor2006_Yaw.NeedCurrent,0,RM2006_StirMotor.NeedCurrent);
    }
    vTaskDelayUntil(&CurrentControlClock, 5);
  }
}

/**
  * @brief  自用陀螺仪数据更新任务
  * @param  unused
  * @retval void
  * @note   一旦接收到就触发任务
  */
void Task_GYROData_Update(void *Parameter)
{
	
  while (1)
  {
		Personal_GYRO_Receive(&PersonalGYRO);
		//Motor2006_Yaw.RealAngle=PersonalGYRO.YawAngle;
		//monitor_Yaw_RealAngle=(int32_t)(Motor2006_Yaw.RealAngle*100);
//		if(Motor2006_Yaw.RealAngle>0)
//			Motor2006_Yaw.RealAngle-=180.0f;
//		else if(Motor2006_Yaw.RealAngle<0)
//			Motor2006_Yaw.RealAngle+=180.0f;
		//Motor2006_Yaw.RealSpeed=PersonalGYRO.Gyro_Z;
		
//		Motor6020_Pitch.last_RealAngle=Motor6020_Pitch.RealAngle;
//		if(Motor6020_Pitch.MechanicalAngle>1340)
//			Motor6020_Pitch.RealAngle=(7490-Motor6020_Pitch.MechanicalAngle)/22.75f;
//		else
//			Motor6020_Pitch.RealAngle=(-701-Motor6020_Pitch.MechanicalAngle)/22.75f;
//		
//		Motor6020_Pitch.RealSpeed=(Motor6020_Pitch.RealAngle-Motor6020_Pitch.last_RealAngle)/0.01f;
		
//		if((Motor6020_Yaw.MechanicalAngle+6236)%8191<4095)
//			Motor6020_Yaw.RealAngle=((Motor6020_Yaw.MechanicalAngle+6236)%8191)/22.75f;
//		else
//			Motor6020_Yaw.RealAngle=((Motor6020_Yaw.MechanicalAngle+6236)%8191-8191)/22.75f;
//		
//		Motor6020_Yaw.RealSpeed=(Motor6020_Yaw.RealAngle-Motor6020_Yaw.last_RealAngle)/0.1f;
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
}


/**
  * @brief  Pitch轴陀螺仪数据更新任务
  * @param  unused
  * @retval void
  * @note   一旦接收到就触发任务
  */
void Task_Pitch_GYRO(void *Parameter)
{
//	portTickType CurrentControlClock = 0;
//  CurrentControlClock = xTaskGetTickCount();
  while (1)
  {
		Personal_GYRO_Receive(&PersonalGYRO2);
//		Motor6020_Pitch.RealAngle=PersonalGYRO2.RollAngle-180.0f;
//		if(Motor6020_Pitch.RealAngle<=-180.0f)
//			Motor6020_Pitch.RealAngle=Motor6020_Pitch.RealAngle+360.0f;
//		Motor6020_Pitch.RealSpeed=PersonalGYRO2.Gyro_X;
//		
				//Motor6020_Pitch.RealSpeed=PersonalGYRO2.Gyro_X;
				if(PersonalGYRO2.RollAngle>0)
					Motor6020_Pitch.RealAngle=PersonalGYRO2.RollAngle-180;
				else
					Motor6020_Pitch.RealAngle=180+PersonalGYRO2.RollAngle;
				Motor2006_Yaw.RealAngle=PersonalGYRO2.YawAngle;
//		Motor6020_Pitch.last_RealAngle=Motor6020_Pitch.RealAngle;
//		if(Motor6020_Pitch.MechanicalAngle>1991)
//			Motor6020_Pitch.RealAngle=(1991-Motor6020_Pitch.MechanicalAngle)/22.75f;
//		else
//			Motor6020_Pitch.RealAngle=(1991-Motor6020_Pitch.MechanicalAngle)/22.75f;
//		
//		Motor6020_Pitch.RealSpeed=(Motor6020_Pitch.RealAngle-Motor6020_Pitch.last_RealAngle)/0.01f;
		
//		if((Motor6020_Yaw.MechanicalAngle+6236)%8191<4095)
//			Motor6020_Yaw.RealAngle=((Motor6020_Yaw.MechanicalAngle+6236)%8191)/22.75f;
//		else
//			Motor6020_Yaw.RealAngle=((Motor6020_Yaw.MechanicalAngle+6236)%8191-8191)/22.75f;
//		
//		Motor6020_Yaw.RealSpeed=(Motor6020_Yaw.RealAngle-Motor6020_Yaw.last_RealAngle)/0.1f;
//		vTaskDelayUntil(&CurrentControlClock, 10);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
}



/**
  * @brief  遥控器数据更新任务
  * @param  unused
  * @retval void
  * @note   有数据就更新
  */
void Task_RCData_Update(void *Parameter)
{
  while (1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    RC_Data_Update();
  }
}

/**
  * @brief  与Jetson通讯数据接收处理
  * @param  unused
  * @retval void
  * @note   收到通知激活 
  */
void Task_JetsonComm(void *Parameter)
{
  while (1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  //pdTRUE让通知值为0，使其进入阻塞;pdFALSE让通知值减一，第二个参数为等待通知的最大时间，单位ms
    JetsonComm_Control(&huart7);
  }
}

void Task_Imu_Ob(void *paramter)
{

  //PWM加热初始化
	

  TickType_t xPreviousWakeTime;
	xPreviousWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		mpu_get_data();
    imu_ahrs_update();
    imu_attitude_update();
    imu_temp_ctrl();
	//温度闭环
		
		vTaskDelayUntil(&xPreviousWakeTime, 2);
	}
}



void Task_JudgeReceive(void *paramter)
{
	while(1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    JudgeReceive();
	}
}


