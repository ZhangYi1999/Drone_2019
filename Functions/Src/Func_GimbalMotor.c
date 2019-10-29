#define __FUNC_GimbalMOTOR_GLOBALS

#include "Func_GimbalMotor.h"
#include "Func_GYRO.h"
#include "Func_RC.h"
#include "Func_StatusMachine.h"
#include "Func_JetsonComm.h"
#include "Func_Imu_OB.h"
#include "Task_Tasks.h"


float PITCH_ANGLE=0;
float YAW_ANGLE=0;

//调参使用
// float i, d = 0;
// float p = 0;
// float angle = 0;

GM6020 Motor6020_Pitch = {
    .SpeedPID.Kp = 60, //100
    .SpeedPID.Ki = 0.3, //0.3
    .SpeedPID.Kd = 0, //0,
		.SpeedPID.DeadBand=0,
    .PositionPID.Kp = 8,//10
    .PositionPID.Ki = 0,
    .PositionPID.Kd = 0,//0,
		.PositionPID.DeadBand=0,
    .TargetAngle = 0,
    .TargetSpeed = 0,
		.NeedCurrent=0};

//GM6020 Motor6020_Yaw = {
//    .SpeedPID.Kp = 55,//450
//    .SpeedPID.Ki = 0.12,//0.7
//    .SpeedPID.Kd = 10,
//		.PositionPID.Kp = 5,//1
//    .PositionPID.Ki = 0,
//    .PositionPID.Kd = 0,//12
//    .TargetAngle = 0,
//    .TargetSpeed = 0,
//		.NeedCurrent=0};
		
M2006 Motor2006_Yaw={
	  .SpeedPID.Kp = 8, 
    .SpeedPID.Ki = 0.2, 
    .SpeedPID.Kd = 20,
		.SpeedPID.DeadBand=0,
    .PositionPID.Kp =	12,//20,
    .PositionPID.Ki = 0,
    .PositionPID.Kd = 0,//,
		.PositionPID.DeadBand=0.5,
    .TargetAngle = 0,
    .TargetSpeed = 0,
		.LastTargetSpeed=0,
		.NeedCurrent=0
};

int monitor_Pitch_RealSpeed=0;		
int monitor_Pitch_TargetSpeed=0;
int monitor_Pitch_RealAngle=0;
int monitor_Pitch_TargetAngle=0;
int monitor_Yaw_TargetSpeed=0;
int monitor_Yaw_RealSpeed=0;
int monitor_Yaw_TargetAngle=0;
int monitor_Yaw_RealAngle=0;
static int16_t PitchGravityCompensation(float Angle);
static void GimbalMotor_AngleSet(GM6020 *Pitch, M2006 *Yaw);
void GimbalMotor_PID(GM6020 *Pitch, M2006 *Yaw);
void GimbalMotor_PID_Singleloop(GM6020 *Pitch, M2006 *YAW);

/**
  * @brief  云台上电初始化
  * @param  void
  * @retval None
  * @note   此函数调用在控制任务while（1）进入之前
  */
#define PitchInitAngle 0.0f
extern EventGroupHandle_t EG_Wait;
float Kptemp=0;
float Yawinitangle;
void Gimbal_Init(void)
{
			
		Motor2006_Yaw.TargetAngle=Motor2006_Yaw.RealAngle;
    Yawinitangle = Motor2006_Yaw.RealAngle;
		//Motor2006_Yaw.TargetSpeed=Motor2006_Yaw.RealSpeed;
		Motor6020_Pitch.TargetAngle=Motor6020_Pitch.RealAngle;
//    for(int i=0;i<20;i++)
//		{
//			Motor6020_Pitch.TargetAngle=-43.75-2*(20-i);
//			GimbalMotor_PID(&Motor6020_Pitch,NULL);
//			Gimbal_CMD_Trans(0, Motor6020_Pitch.NeedCurrent,0);
//			vTaskDelay(10);
//		}
	  
}
/**
  * @brief  云台控制
  * @param  void
  * @retval None
  */
int16_t ReverseOffset=0;
void Gimbal_Ctrl(void)
{

    GimbalMotor_AngleSet(&Motor6020_Pitch, &Motor2006_Yaw);
		GimbalMotor_PID(&Motor6020_Pitch, &Motor2006_Yaw);
		//GimbalMotor_PID_Singleloop(&Motor6020_Pitch,NULL);
		
		
		if(ReverseOffset>0)
			ReverseOffset--;
}
/**
  * @brief  云台目标角度设定
  * @param  两电机的结构体
  * @retval None
  * @note   取云台向右下为正
  */

int16_t MAXSPEED=80;
double step=0.96;
void GimbalMotor_AngleSet(GM6020 *Pitch, M2006 *Yaw)
{

//    static float temp = 0;
	/*if (ControlMode == ControlMode_RC)
		Yaw->TargetAngle = 0;
	else if(ControlMode == ControlMode_Keyboard)
		Yaw->TargetAngle = MAXSPEED;
	*/
		if	(ControlMode == ControlMode_Protect)
		{
			Pitch->TargetAngle=Pitch->RealAngle;
			Yaw->TargetAngle=Yaw->RealAngle;
		}
    if (ControlMode == ControlMode_RC)
    {
        
        //此处是减号是测得的结果
				Yaw->TargetAngle -= -PhysicalAngleAddStep * Get_Channel_Val(&RC_ReceiveData, RC_CH0) / RC_CH_MAX_RELATIVE;
         //Yaw->TargetSpeed =MAXSPEED * Get_Channel_Val(&RC_ReceiveData, RC_CH0) / RC_CH_MAX_RELATIVE;  
				 Pitch->TargetAngle += step * Get_Channel_Val(&RC_ReceiveData, RC_CH3) / RC_CH_MAX_RELATIVE;
				//Pitch->TargetSpeed = MAXSPEED*Get_Channel_Val(&RC_ReceiveData, RC_CH3) / RC_CH_MAX_RELATIVE;
//				if(Yaw->TargetSpeed>0)
//				{
//					Yaw->TargetSpeed=MAXSPEED;
//				}
//				else if(Yaw->TargetSpeed<0)
//				{
//					Yaw->TargetSpeed=-1*MAXSPEED;
//				}
//				if(Yaw->RealAngle>170&&Yaw->TargetSpeed>0)
//					Yaw->TargetSpeed=0;
//				else if(Yaw->RealAngle<-170&&Yaw->TargetSpeed<0)
//					Yaw->TargetSpeed=0;
				
				if(Yaw->TargetSpeed*Yaw->RealSpeed<0)
					ReverseOffset=200;
				
				Pitch->TargetAngle = Pitch->TargetAngle > 10 ? 10 : Pitch->TargetAngle;
				Pitch->TargetAngle = Pitch->TargetAngle < -60 ? -60 : Pitch->TargetAngle;
				Yaw->TargetAngle=Yaw->TargetAngle>Yawinitangle + 150? Yawinitangle + 150:Yaw->TargetAngle;
				Yaw->TargetAngle=Yaw->TargetAngle < Yawinitangle - 80 ? Yawinitangle - 80:Yaw->TargetAngle;
				monitor_Pitch_TargetAngle=(int32_t)(Pitch->TargetAngle*100);
				monitor_Yaw_TargetAngle=(int32_t)(Yaw->TargetAngle*100);
				Jetson_AnglePitch=Pitch->RealAngle;
				Jetson_AngleYaw=Yaw->RealAngle;
    }
    else if(ControlMode == ControlMode_Auto)
    {
//				Pitch->TargetSpeed = MAXSPEED*Get_Channel_Val(&RC_ReceiveData, RC_CH3) / RC_CH_MAX_RELATIVE;
//				Yaw->TargetSpeed =MAXSPEED * Get_Channel_Val(&RC_ReceiveData, RC_CH0) / RC_CH_MAX_RELATIVE;  
//				if(Pitch->TargetSpeed>0)
//				{
//					Pitch->TargetSpeed=MAXSPEED;
//					
//				}
//				else if(Pitch->TargetSpeed<0)
//				{
//					Pitch->TargetSpeed=-1*MAXSPEED;
//				}
//				if(Yaw->TargetSpeed>0)
//				{
//					Yaw->TargetSpeed=MAXSPEED;
//					
//				}
//				else if(Yaw->TargetSpeed<0)
//				{
//					Yaw->TargetSpeed=-1*MAXSPEED;
//				}
//			if(AutoGimbal.ControlMode==DRONE_SPEED_CONTROL)
//			{
//        Yaw->TargetSpeed = (*(int16_t*)(&AutoGimbal.Yawdata));
//				Yaw->TargetSpeed /= 100.0f;
//				Pitch->TargetSpeed =(*(int16_t*)(&AutoGimbal.Pitchdata));
//				Pitch->TargetSpeed /=100.0f;
//			}
//				if(Yaw->RealAngle>170&&Yaw->TargetSpeed>0)
//						Yaw->TargetSpeed = 0;
//				else if(Yaw->RealAngle<-170&&Yaw->TargetSpeed<0)
//						Yaw->TargetSpeed = 0;
//				if(Pitch->RealAngle>10&&Pitch->TargetSpeed>0)
//						Pitch->TargetSpeed = 0;
//				else if(Pitch->RealAngle<-80&&Pitch->TargetSpeed<0)
//						Pitch->TargetSpeed = 0;
//        Yaw->TargetAngle=Yaw->RealAngle;
//				Pitch->TargetAngle=Pitch->RealAngle;
//				
//				monitor_Pitch_TargetSpeed=(int32_t)(Pitch->TargetSpeed*100);
				/************** YAW **************/
        //此处是减号是测得的结果
//        temp = 1.3 * YawSpeedMax * Get_Mouse_Speed(&RC_ReceiveData, MOUSE_X) / 200;
//        /************** PITCH **************/
        //Pitch->TargetAngle -= 8 * PhysicalAngleAddStep * Get_Mouse_Speed(&RC_ReceiveData, MOUSE_Y) / 200;
				//Yaw->TargetAngle += 8 * PhysicalAngleAddStep * Get_Mouse_Speed(&RC_ReceiveData, MOUSE_X) / 200;
				Yaw->TargetAngle -= -PhysicalAngleAddStep * Get_Channel_Val(&RC_ReceiveData, RC_CH0) / RC_CH_MAX_RELATIVE;
				Pitch->TargetAngle += step * Get_Channel_Val(&RC_ReceiveData, RC_CH3) / RC_CH_MAX_RELATIVE;
//        if(Get_Keyboard_Val(&RC_ReceiveData,KEY_Q))
//        {
//            temp = 0.5 * YawSpeedMax;
//        }
//        else if(Get_Keyboard_Val(&RC_ReceiveData,KEY_E))
//        {
//            temp = -0.5 * YawSpeedMax;
//        }
//        Yaw->TargetSpeed = SmoothFilter(temp);
//				if(Yaw->RealAngle>170&&Yaw->TargetSpeed>0)
//						Yaw->TargetSpeed = 0;
//				else if(Yaw->RealAngle<-170&&Yaw->TargetSpeed<0)
//						Yaw->TargetSpeed = 0;
//				Yaw->TargetAngle=Yaw->RealAngle;
				//目标角度必须处在限位幅度内
//				Pitch->TargetAngle = Jetson_AnglePitch;
//				Yaw->TargetAngle = Jetson_AngleYaw;
				if(Get_Keyboard_Val(&RC_ReceiveData,KEY_W))
				{
					Pitch->TargetAngle += 0.08f;					
				}
				
				if(Get_Keyboard_Val(&RC_ReceiveData,KEY_S))
				{
					Pitch->TargetAngle -= 0.08f;		
				}
				Pitch->TargetAngle = Pitch->TargetAngle < -60 ? -60 : Pitch->TargetAngle;
				Pitch->TargetAngle = Pitch->TargetAngle > 10 ? 10 : Pitch->TargetAngle;
        Yaw->TargetAngle=Yaw->TargetAngle>Yawinitangle + 170? Yawinitangle + 170:Yaw->TargetAngle;
				Yaw->TargetAngle=Yaw->TargetAngle < Yawinitangle - 90 ? Yawinitangle - 90:Yaw->TargetAngle;
    }
}

int16_t Gcurrent;
/**
  * @brief  云台单环PID控制  --CAN1
  * @param  Pitch轴电机结构体     Yaw轴电机结构体
  * @retval None
*/	
void GimbalMotor_PID_Singleloop(GM6020 *Pitch, M2006 *YAW)
{
    /************** PITCH **************/
    if (Pitch != NULL)
    {
        Pitch->SpeedPID.Last_Error = Pitch->SpeedPID.Cur_Error;
        Pitch->SpeedPID.Cur_Error = Pitch->TargetSpeed - Pitch->RealSpeed;
					
				if(fabs(Pitch->SpeedPID.Cur_Error)<Pitch->SpeedPID.DeadBand)
					Pitch->SpeedPID.Cur_Error=0;
		
				Pitch->SpeedPID.Sum_Error += Pitch->SpeedPID.Cur_Error;
				
        Pitch->SpeedPID.Sum_Error = Pitch->SpeedPID.Sum_Error > 10000 ? 10000 : Pitch->SpeedPID.Sum_Error;
        Pitch->SpeedPID.Sum_Error = Pitch->SpeedPID.Sum_Error < -10000 ? -10000 : Pitch->SpeedPID.Sum_Error;
				Gcurrent=PitchGravityCompensation(Pitch->RealAngle);
        Pitch->SpeedPID.Output = Pitch->SpeedPID.Kp * Pitch->SpeedPID.Cur_Error \
                                    + Pitch->SpeedPID.Ki * Pitch->SpeedPID.Sum_Error \
                                    + Pitch->SpeedPID.Kd * (Pitch->SpeedPID.Cur_Error - Pitch->SpeedPID.Last_Error)\
																		+ PitchGravityCompensation(Pitch->RealAngle);

        //?T??ê?3?μ?á÷
        Pitch->NeedCurrent = (Pitch->SpeedPID.Output >  GM6020CURRENTMAX ? GM6020CURRENTMAX : Pitch->SpeedPID.Output);
        Pitch->NeedCurrent = Pitch->SpeedPID.Output < -GM6020CURRENTMAX ? -GM6020CURRENTMAX : Pitch->NeedCurrent;
				Pitch->NeedCurrent = Pitch->NeedCurrent*-1;
    }
    /************** YAW **************/
    if (YAW != NULL)
    {
				
        YAW->SpeedPID.Last_Error = YAW->SpeedPID.Cur_Error;
        YAW->SpeedPID.Cur_Error = YAW->TargetSpeed - YAW->RealSpeed;
				
				
				YAW->SpeedPID.Sum_Error +=YAW->SpeedPID.Cur_Error;

        YAW->SpeedPID.Sum_Error = YAW->SpeedPID.Sum_Error > 10000 ? 10000 : YAW->SpeedPID.Sum_Error;
        YAW->SpeedPID.Sum_Error = YAW->SpeedPID.Sum_Error < -10000 ? -10000 : YAW->SpeedPID.Sum_Error;
				
					YAW->SpeedPID.Output = YAW->SpeedPID.Kp * YAW->SpeedPID.Cur_Error \
                                + YAW->SpeedPID.Ki * YAW->SpeedPID.Sum_Error \
                                + YAW->SpeedPID.Kd * (YAW->SpeedPID.Cur_Error - YAW->SpeedPID.Last_Error)+ReverseOffset;
        //?T??ê?3?μ?á÷
        YAW->NeedCurrent = (YAW->SpeedPID.Output > M2006CURRENTMAX ? M2006CURRENTMAX : YAW->SpeedPID.Output);
        YAW->NeedCurrent = (YAW->SpeedPID.Output < -M2006CURRENTMAX ? -M2006CURRENTMAX : YAW->NeedCurrent);
				YAW->NeedCurrent = -1*YAW->NeedCurrent;
    }
}

/**
  * @brief  云台双环PID控制  --CAN1
  * @param  Pitch轴电机结构体     Yaw轴电机结构体
  * @retval None
*/	
void GimbalMotor_PID(GM6020 *Pitch, M2006 *YAW)
{
   /************** PITCH **************/
    if (Pitch != NULL)
    {
        
        Pitch->PositionPID.Last_Error = Pitch->PositionPID.Cur_Error;
        Pitch->PositionPID.Cur_Error = Pitch->TargetAngle-Pitch->RealAngle;
				if(fabs(Pitch->PositionPID.Cur_Error)<Pitch->PositionPID.DeadBand)
					Pitch->PositionPID.Cur_Error=0;
        
        Pitch->PositionPID.Cur_Error = Pitch->PositionPID.Cur_Error > 180 ? Pitch->PositionPID.Cur_Error - 360 : Pitch->PositionPID.Cur_Error;
        Pitch->PositionPID.Cur_Error = Pitch->PositionPID.Cur_Error < -180 ? Pitch->PositionPID.Cur_Error + 360 : Pitch->PositionPID.Cur_Error;

        Pitch->PositionPID.Sum_Error += (Pitch->PositionPID.Cur_Error+Pitch->PositionPID.Last_Error)/2.0f;
			

        Pitch->PositionPID.Sum_Error = Pitch->PositionPID.Sum_Error > 5000 ? 5000 : Pitch->PositionPID.Sum_Error;
        Pitch->PositionPID.Sum_Error = Pitch->PositionPID.Sum_Error < -5000 ? -5000 : Pitch->PositionPID.Sum_Error;

        Pitch->PositionPID.Output = Pitch->PositionPID.Kp * Pitch->PositionPID.Cur_Error + Pitch->PositionPID.Ki * Pitch->PositionPID.Sum_Error + Pitch->PositionPID.Kd * (Pitch->PositionPID.Cur_Error - Pitch->PositionPID.Last_Error);

        Pitch->TargetSpeed = Pitch->PositionPID.Output;
				
        //?ù?è?·
				
        Pitch->SpeedPID.Last_Error = Pitch->SpeedPID.Cur_Error;
        Pitch->SpeedPID.Cur_Error = Pitch->TargetSpeed-Pitch->RealSpeed;

        if(fabs(Pitch->SpeedPID.Cur_Error)<Pitch->SpeedPID.DeadBand)
					Pitch->SpeedPID.Cur_Error=0;
					
					Pitch->SpeedPID.Sum_Error += (Pitch->SpeedPID.Cur_Error+Pitch->SpeedPID.Last_Error)/2.0f;


        Pitch->SpeedPID.Sum_Error = Pitch->SpeedPID.Sum_Error > 10000 ? 10000 : Pitch->SpeedPID.Sum_Error;
        Pitch->SpeedPID.Sum_Error = Pitch->SpeedPID.Sum_Error < -10000 ? -10000 : Pitch->SpeedPID.Sum_Error;

        Pitch->SpeedPID.Output = Pitch->SpeedPID.Kp * Pitch->SpeedPID.Cur_Error \
                                + Pitch->SpeedPID.Ki * Pitch->SpeedPID.Sum_Error \
                                + Pitch->SpeedPID.Kd * (Pitch->SpeedPID.Cur_Error - Pitch->SpeedPID.Last_Error) \
                                + PitchGravityCompensation(Pitch->RealAngle);
				
        Pitch->NeedCurrent = (Pitch->SpeedPID.Output >  GM6020CURRENTMAX ? GM6020CURRENTMAX : Pitch->SpeedPID.Output);
        Pitch->NeedCurrent = Pitch->SpeedPID.Output < -GM6020CURRENTMAX ? -GM6020CURRENTMAX : Pitch->NeedCurrent;
				Pitch->NeedCurrent = Pitch->NeedCurrent*-1;
    }
    /************** YAW **************/
    if (YAW != NULL)
    {

			  
        YAW->PositionPID.Last_Error = YAW->PositionPID.Cur_Error;
        YAW->PositionPID.Cur_Error = YAW->TargetAngle - YAW->RealAngle;
				
				if(fabs(YAW->PositionPID.Cur_Error)<YAW->PositionPID.DeadBand)
					YAW->PositionPID.Cur_Error=0;
			
        YAW->PositionPID.Cur_Error = YAW->PositionPID.Cur_Error > 180 ? YAW->PositionPID.Cur_Error - 360 : YAW->PositionPID.Cur_Error;
        YAW->PositionPID.Cur_Error = YAW->PositionPID.Cur_Error < -180 ? YAW->PositionPID.Cur_Error + 360 : YAW->PositionPID.Cur_Error;

        
        YAW->PositionPID.Sum_Error += YAW->PositionPID.Cur_Error;
        

        YAW->PositionPID.Sum_Error = YAW->PositionPID.Sum_Error > 5000 ? 5000 : YAW->PositionPID.Sum_Error;
        YAW->PositionPID.Sum_Error = YAW->PositionPID.Sum_Error < -5000 ? -5000 : YAW->PositionPID.Sum_Error;

        YAW->PositionPID.Output = YAW->PositionPID.Kp * YAW->PositionPID.Cur_Error + YAW->PositionPID.Ki * YAW->PositionPID.Sum_Error + YAW->PositionPID.Kd * (YAW->PositionPID.Cur_Error - YAW->PositionPID.Last_Error);

        YAW->TargetSpeed = YAW->PositionPID.Output;
//				if(ControlMode == ControlMode_Auto)
//				{
//					YAW->TargetSpeed = 800 * PhysicalAngleAddStep * Get_Mouse_Speed(&RC_ReceiveData, MOUSE_X) / 200;
//					if(Get_Keyboard_Val(&RC_ReceiveData,KEY_A))
//					{
//						YAW->TargetSpeed = -20;					
//					}
//					else if(Get_Keyboard_Val(&RC_ReceiveData,KEY_D))
//					{
//						YAW->TargetSpeed = 20;		
//					}
//					// float rightanglemax = Yawinitangle + 150;
//          // float leftanglemax = Yawinitangle - 80;

//					// if(YAW->RealAngle > Yawinitangle + 150 && YAW->TargetSpeed > 0)
//					// 	YAW->TargetSpeed = 0;
//					// else if(YAW->RealAngle < Yawinitangle - 80 && YAW->TargetSpeed < 0)
//					// 	YAW->TargetSpeed = 0;
//				}
//				else if(ControlMode == ControlMode_RC)
//				{
//					YAW->TargetSpeed = 200 * Get_Channel_Val(&RC_ReceiveData, RC_CH0) / RC_CH_MAX_RELATIVE;

//				}
        //?ù?è?·
        YAW->SpeedPID.Last_Error = YAW->SpeedPID.Cur_Error;
        YAW->SpeedPID.Cur_Error = YAW->TargetSpeed - YAW->RealSpeed;
				
				if(fabs(YAW->SpeedPID.Cur_Error)<YAW->SpeedPID.DeadBand)
					YAW->SpeedPID.Cur_Error=0;
				
				YAW->SpeedPID.Sum_Error +=YAW->SpeedPID.Cur_Error;

        YAW->SpeedPID.Sum_Error = YAW->SpeedPID.Sum_Error > 10000 ? 10000 : YAW->SpeedPID.Sum_Error;
        YAW->SpeedPID.Sum_Error = YAW->SpeedPID.Sum_Error < -10000 ? -10000 : YAW->SpeedPID.Sum_Error;
				
				YAW->SpeedPID.Output = YAW->SpeedPID.Kp * YAW->SpeedPID.Cur_Error \
                                + YAW->SpeedPID.Ki * YAW->SpeedPID.Sum_Error \
                                + YAW->SpeedPID.Kd * (YAW->SpeedPID.Cur_Error - YAW->SpeedPID.Last_Error)+ReverseOffset;
        //?T??ê?3?μ?á÷
        YAW->NeedCurrent = (YAW->SpeedPID.Output > M2006CURRENTMAX ? M2006CURRENTMAX : YAW->SpeedPID.Output);
        YAW->NeedCurrent = (YAW->SpeedPID.Output < -M2006CURRENTMAX ? -M2006CURRENTMAX : YAW->NeedCurrent);
				YAW->NeedCurrent = -1*YAW->NeedCurrent;
    }
}

/**
  * @brief  云台电机数据发送  --CAN1
  * @param  Yaw轴电流值     Pitch轴电流值
  * @retval None
  */
void Gimbal_CMD_Trans(int16_t Current_Yaw, int16_t Current_Pitch, int16_t Current_StirMotor)
{
    static CanSend_Type CANSend;

    CANSend.CANx = CAN_NUMBER_1;

    CANSend.SendCanHeader.StdId = 0x200;
    CANSend.SendCanHeader.IDE = CAN_ID_STD;
    CANSend.SendCanHeader.RTR = CAN_RTR_DATA;
    CANSend.SendCanHeader.DLC = 0x08;

    CANSend.SendCanTxMsg[0] = (uint8_t)(Current_Yaw >> 8);
    CANSend.SendCanTxMsg[1] = (uint8_t)(Current_Yaw);
    CANSend.SendCanTxMsg[2] = 0;
    CANSend.SendCanTxMsg[3] = 0;
    CANSend.SendCanTxMsg[4] = 0;
    CANSend.SendCanTxMsg[5] = 0;
    CANSend.SendCanTxMsg[6] = (uint8_t)(Current_StirMotor >> 8);
    CANSend.SendCanTxMsg[7] = (uint8_t)(Current_StirMotor);
    xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);
    static CanSend_Type CANSend2;

    CANSend2.CANx = CAN_NUMBER_1;

    CANSend2.SendCanHeader.StdId = 0x1FF;
    CANSend2.SendCanHeader.IDE = CAN_ID_STD;
    CANSend2.SendCanHeader.RTR = CAN_RTR_DATA;
    CANSend2.SendCanHeader.DLC = 0x08;

    
		CANSend2.SendCanTxMsg[0] = 0;
		CANSend2.SendCanTxMsg[1] = 0;
    CANSend2.SendCanTxMsg[2] = 0;
    CANSend2.SendCanTxMsg[3] = 0;
		CANSend2.SendCanTxMsg[4] = 0;
		CANSend2.SendCanTxMsg[5] = 0;
    CANSend2.SendCanTxMsg[6] = (uint8_t)(Current_Pitch >> 8);
    CANSend2.SendCanTxMsg[7] = (uint8_t)(Current_Pitch);
    xQueueSend(Queue_CANSend, &CANSend2, 3 / portTICK_RATE_MS);
}

/**
  * @brief  滤波器——用于平滑处理
  * @param  当前数据
  * @retval 滤波后处理数据
  */
float SmoothFilter(float data)
{

    static float lastoutput = 0;
    static float alpha = 0.6;
    float temp;

    temp = alpha * lastoutput + (1 - alpha) * data;
    lastoutput = temp;

    return temp;
}

/**
  * @brief  云台Pitch轴重力补偿
  * @param  Pitch轴当前机械角度
  * @retval 对应补偿电流
  */
int16_t Gravity = 3000;
int16_t offset;
int16_t PitchGravityCompensation(float Angle)
{
//	Gravity=(int16_t)(0.0017f*Angle*Angle*Angle-0.1726f*Angle*Angle+7.4049f*Angle+3442.4f);
//	offset=(int16_t)(Gravity * cos((Angle) * Pi / 180 ));
//	offset = offset > 0? offset :0;
	return (int16_t)(Gravity * cos((Angle) * Pi / 180 ));
}

