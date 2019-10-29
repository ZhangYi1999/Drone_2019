#define __FUNC_STIRMOTOR_GLOBALS

#include "Func_StirMotor.h"
#include "Func_StatusMachine.h"



/*
给定电流为正——角度增加——角速度为正——逆时针（从轴看）
过零检测：
  正转时过零计数器++，反转过零计数器--
为保证数据尽量准确，发射关闭时清零
*/

void Stir_Motor_Speed_Control(ESC_820R* motor);
void StirMotor_Blocked_Detect(ESC_820R* motor);


ESC_820R  RM2006_StirMotor = {
  .PID.Kp = 5.0f,
  .PID.Ki = 0.25f, 
  .PID.Kd = 0.0f,
  .TargetSpeed = 0
};

/**
  * @brief  拨盘电机控制
  * @param  void
  * @retval void
  * @note   此函数要调用Stir_Motor_Speed_Control函数
  */
uint16_t stirspeed = 5500;
uint16_t itemp=1000;

void StirMotor_Control(void)
{

  if(StirStatus == StirStatus_SpeedControl)
    RM2006_StirMotor.TargetSpeed = -1*stirspeed;//5300
  if(StirStatus == StirStatus_Stop)
    RM2006_StirMotor.TargetSpeed = 0;
  
  StirMotor_Blocked_Detect(&RM2006_StirMotor);
  
  //如果检测到赌转则反转3个控制周期
  if(RM2006_StirMotor.BlockedWarningTimes > 0)
  {
    RM2006_StirMotor.TargetSpeed = 2000;
    RM2006_StirMotor.BlockedWarningTimes--;
  }
  
  Stir_Motor_Speed_Control(&RM2006_StirMotor);
	
	if(ControlMode == ControlMode_Protect)
		RM2006_StirMotor.NeedCurrent = 0;
//  if(StirStatus == StirStatus_SpeedControl)
//    RM2006_StirMotor.NeedCurrent = itemp;//5300
//  if(StirStatus == StirStatus_Stop)
//    RM2006_StirMotor.NeedCurrent = 0;
}

/**
  * @brief  拨盘电机速度闭环
  * @param  RM：拨盘电机结构体
  * @retval void
  */
void Stir_Motor_Speed_Control(ESC_820R* motor)
{
  motor->PID.Last_Error = motor->PID.Cur_Error;
  motor->PID.Cur_Error = motor->TargetSpeed - motor->RealSpeed;

  motor->PID.Sum_Error += motor->PID.Cur_Error;
  //积分上限
  motor->PID.Sum_Error = motor->PID.Sum_Error > 10000 ? 10000 : motor->PID.Sum_Error;
  motor->PID.Sum_Error = motor->PID.Sum_Error < -10000 ? -10000 : motor->PID.Sum_Error;

  motor->PID.Output = (motor->PID.Kp * motor->PID.Cur_Error\
                                    + motor->PID.Ki * motor->PID.Sum_Error\
                                    + motor->PID.Kd * (motor->PID.Cur_Error - motor->PID.Last_Error));
  //限制输出电流
  motor->NeedCurrent = motor->PID.Output > C610CURRENTMAX ? C610CURRENTMAX : motor->PID.Output;
  motor->NeedCurrent = motor->PID.Output < -C610CURRENTMAX ? -C610CURRENTMAX : motor->NeedCurrent;
}

/**
  * @brief  拨盘电机赌转检测
  * @param  RM：拨盘电机结构体
  * @retval void
  * @note   100ms检测 200ms反转
  */
void StirMotor_Blocked_Detect(ESC_820R* motor)
{
  static uint8_t BlockedTimes = 0;
  //反转控制完成后再次检测
  if(motor->BlockedWarningTimes <= 0)
  {
    if(abs(motor->RealSpeed) < 100 && abs(motor->PID.Sum_Error) == 10000) 
      BlockedTimes++;
    else
      BlockedTimes = 0;
    //连续100ms检测到赌转
    if(BlockedTimes >= 20)
    {
      motor->BlockedWarningTimes = 40;
      BlockedTimes = 0;
    }
  }
}



/**
  * @brief  获取减速后电机转过的角度（96:1）
  * @param  RM：拨盘电机结构体
  * @retval none
  * @note   需要过零处理！！！ 角度值相对offset
	* @note		temp与ZreoCounter有溢出风险!!!!!!
  */
void StirMotor_Get_Angle(StirMotor_Struct* RM)
{
  static int64_t temp = 0;
  temp = (int64_t)(RM->RealAngle + (RM->CrossZero.ZreoCounter * ONECIRCLECOUNTVALUEORIGINAL) - RM->AngleOffset);
  if(temp > ONECIRCLECOUNTVALUEAFTERDECELERATE || temp <- ONECIRCLECOUNTVALUEAFTERDECELERATE)
  {
    while(temp > ONECIRCLECOUNTVALUEAFTERDECELERATE)  temp -= ONECIRCLECOUNTVALUEAFTERDECELERATE;
    while(temp < -ONECIRCLECOUNTVALUEAFTERDECELERATE) temp += ONECIRCLECOUNTVALUEAFTERDECELERATE;
  }
  RM->RealAngle_Decelerate = StirMotorChangeToAbsAngle(temp);
}
/**
  * @brief  角度的过零处理
  * @param  RM：拨盘电机结构体
  * @retval void
  * @note   只有在拨盘进行角度控制时才进行过零处理
  */
void  Cross_Zero_Settle(StirMotor_Struct* RM)
{
  static  uint8_t OffsetValueSetNumberOfTimes = 0;

	if(StirStatus == StirStatus_AngleControl)  
  {
    //只赋值一次
    if(OffsetValueSetNumberOfTimes == 0)
		{
			RM->AngleOffset = RM->RealAngle;
			OffsetValueSetNumberOfTimes++;
		}
    //判断过零
    if(RM->RealSpeed > 10)           //正向过零
    {
      //if(RM->LastRealAngle < RM->RealAngle) RM->CrossZero.ZreoCounter++;
      if(RM->RealAngle > CROSSZEROUPTHRESHOLD)  RM->CrossZero.InUpHalfCircle = 1;
      if(RM->CrossZero.InUpHalfCircle && RM->RealAngle < CROSSZERODOWNTHRESHOLD)
      {
        RM->CrossZero.ZreoCounter++;
        RM->CrossZero.InUpHalfCircle = 0;
      }
    }
    else if(RM->RealSpeed < -10)     //反向过零
    {
      //if(RM->RealAngle > RM->LastRealAngle) RM->CrossZero.ZreoCounter--;
      if(RM->RealAngle < CROSSZERODOWNTHRESHOLD)  RM->CrossZero.InDownHalfCircle = 1;
      if(RM->CrossZero.InDownHalfCircle && RM->RealAngle > CROSSZEROUPTHRESHOLD)
      {
        RM->CrossZero.ZreoCounter--;
        RM->CrossZero.InDownHalfCircle = 0;
      }
    }
  }
  else 
  {
    RM->CrossZero.ZreoCounter = 0;
    OffsetValueSetNumberOfTimes = 0;
  }
}

