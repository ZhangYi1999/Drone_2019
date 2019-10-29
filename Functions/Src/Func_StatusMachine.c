#include "Func_JetsonComm.h"
#include "Func_StatusMachine.h"
#include "Func_RC.h"
#include "Func_FricMotor.h"
#include "Task_Tasks.h"
#include "Func_JudgeReceive.h"


BoardToBoardComm_Enum BoardToBoardComm = Comm_Off;


ControlMode_Enum ControlMode = ControlMode_Protect;
FricStatus_Enum FricStatus = FricStatus_Stop;
StirStatus_Enum StirStatus = StirStatus_Stop;
uint8_t ShootStatus;

/**
  * @brief  状态机更新
  * @param  void
  * @retval void
  * @note   右拨码开关上：保护模式，左上电机全部关闭
            右拨码开关中：遥控器控制：左中正常，左上开关摩擦轮，左下发射
            右拨码开关下：自动控制（接收上层命令）
  */
uint8_t AutoShootEnable = pdFALSE;
void StatusMachine_Update(void)
{

    static uint8_t FricCtrlRelease = 1;
//    static uint8_t StirKBCtrlRelease = 1;
    //帧率过低保护
    //基本模式切换
    switch (Get_Switch_Val(&RC_ReceiveData, RC_SW_Right))
    {
    case RC_SW_MID:
    {
        ControlMode = ControlMode_RC;
			
        break;
    }
    case RC_SW_DOWN:
    {
        ControlMode = ControlMode_Auto;
        break;
    }
    default:
    {
        ControlMode = ControlMode_Protect;
        break;
    }
    }

    //子模式切换
    switch (ControlMode)
    {
    case ControlMode_RC: //遥控器模式
    {
        /*******************************************  ↓  摩擦轮  ↓  *******************************************/
//				if(aerial_robot_energy.energy_point>0&&aerial_robot_energy.attack_time>0)
//				{
				
        if (Get_Switch_Val(&RC_ReceiveData, RC_SW_Left) == RC_SW_UP)
        {
            if (FricCtrlRelease)
            {
								if(FricStatus == FricStatus_Stop)
								{
									FricStatus = FricStatus_Working_High;
								}
                else if (FricStatus == FricStatus_Working_High)
                {
                    FricStatus = FricStatus_Stop;
                }
                FricCtrlRelease = 0;
            }
        }
        else
        {
            FricCtrlRelease = 1;
        }
        /*******************************************  ↑  摩擦轮  ↑  *******************************************/
        /*******************************************  ↓   拨盘   ↓  *******************************************/
        if ((Get_Switch_Val(&RC_ReceiveData, RC_SW_Left) == RC_SW_DOWN))
				{
            StirStatus = StirStatus_SpeedControl;
        }
        else
        {
            StirStatus = StirStatus_Stop;
        }
        /*******************************************  ↑   拨盘   ↑  *******************************************/
				break;
			}
        
    case ControlMode_Auto: //自瞄模式
    {
				
				//	if(aerial_robot_energy.attack_time==0||aerial_robot_energy.energy_point==0)
//		FricStatus=FricStatus_Stop;

				if(Get_Keyboard_Val(&RC_ReceiveData,KEY_G)&&!Get_Keyboard_Val(&LastRC_ReceiveData,KEY_G))
				{
					FricStatus=FricStatus_Stop;
					FricCtrlRelease = 1;
				}
				if(Get_Keyboard_Val(&RC_ReceiveData,KEY_F)&&!Get_Keyboard_Val(&LastRC_ReceiveData,KEY_F))
        {
          
						FricStatus = FricStatus_Working_High;
            FricCtrlRelease = 0;
          
        }
//				else
//				{
//						FricStatus = FricStatus_Stop;
//						FricCtrlRelease = 1;
//				}
            

        if(Get_Mouse_Pressed(&RC_ReceiveData,MOUSE_LEFT)&&(FricCtrlRelease == 0))
            StirStatus = StirStatus_SpeedControl;
        else
            StirStatus = StirStatus_Stop;
        break;
//				 
    }
    case ControlMode_Protect: //保护模式
    {
        
       FricStatus = FricStatus_Stop;
       StirStatus = StirStatus_Stop;
        
        break;
    }
    }

}
