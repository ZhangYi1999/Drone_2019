#define  __FRICMOTOR_GLOBALS
#include "Func_JudgeReceive.h"
#include "Func_FricMotor.h"
#include "Func_StatusMachine.h"
uint16_t ARR_MAX=1000;
uint16_t ARR_MIN=1000; 
uint8_t Firc_times=0;
FricMotor_Struct Snail2305;
uint8_t Fric_Init_State=0;

uint16_t arr = 1000;
uint16_t HighArr = 1260;
uint16_t LowArr = 1000;
uint16_t LeftLowArr =800;
uint16_t LeftHighArr =1080;
uint16_t MidArr=1175;
uint16_t LeftMidArr=1050;
uint16_t tempLeft =800;

float Speed_High=30.0f;
float Speed_Mid=20.0f;
float Speed_Low=0.0f;
uint16_t Right_tempspeed=1500;
uint16_t Left_tempspeed=1500;
//????
#define BASE_DUTY 1000
//????PWM??
//#define FRIC_PWM_MID 1160
#define FRIC_PWM_HIGH 1350
//???????3
#define SPEEDHIG 211
#define SPEEDLOW 147
#define sum_Error_Boundary 1000

FricMotor_Struct Snail2305_Left=
{
	.Kp=0,
	.Ki=0,
	.Kd=0,
	.Cur_Error=0,
	.Last_Error=0,
	.Sum_Error=0,
	.Target_Speed=0,
	.Real_Speed=0,
	.Speed_ARR=1000,
	.Last_Speed_ARR=1000,
	.SpeedUpFinish=0,
  .Speedupcnt=0,
};

FricMotor_Struct Snail2305_Right=
{
	.Kp=0,
	.Ki=0,
	.Kd=0,
	.Cur_Error=0,
	.Last_Error=0,
	.Sum_Error=0,
	.Target_Speed=0,
	.Real_Speed=0,
	.Speed_ARR=1000,
	.Last_Speed_ARR=1000,
	.SpeedUpFinish=0,
  .Speedupcnt=0,
};

/**
  * @brief ??????
  * @param  void
  * @retval void
  */
void FricMotor_Init(void)
{
		HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,2000);           //Fric2305_Left
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2000);           //Fric2305_Right
    
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);

    HAL_Delay(2000);

    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000);           
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);           

    HAL_Delay(1000);
    Snail2305.Speed_ARR = ARR_MIN;
}
  
/**
  * @brief  ???????
  * @param  void
  * @retval void
  */
TickType_t L_curtime,L_pretime,L_deitatime;
int16_t L_Capturenum;
	
TickType_t R_curtime,R_pretime,R_deitatime;
int16_t R_Capturenum;
void FricMotor_Get_Speed(void)
{
	L_curtime = xTaskGetTickCount();
	L_Capturenum = (int16_t) __HAL_TIM_GET_COUNTER(&htim4);//?????
	L_deitatime = (L_curtime - L_pretime);
	if(L_deitatime != 0)
		Snail2305_Left.Real_Speed = fabs((float)L_Capturenum/(float)L_deitatime) / 0.3f;
	L_pretime = L_curtime;
	__HAL_TIM_SET_COUNTER(&htim4,0);
	
	R_curtime = xTaskGetTickCount();
	R_Capturenum = (int32_t) __HAL_TIM_GET_COUNTER(&htim8);//?????p
	R_deitatime = (R_curtime - R_pretime);
	if(R_deitatime != 0)
		Snail2305_Right.Real_Speed = fabs((float)R_Capturenum/(float)R_deitatime) / 0.3f;
	R_pretime = R_curtime;
	__HAL_TIM_SET_COUNTER(&htim8,0);
}

/**
  * @brief  ???????
  * @param  void
  * @retval void
  */

void FricMotor_Ctrl(void)
{
	if(FricStatus == FricStatus_Working_High)
	{
		if(Snail2305.Speed_ARR < 1280)
			Snail2305.Speed_ARR++;
		else
		{
			Snail2305.SpeedUpFinish = 1;
		}
		
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Snail2305.Speed_ARR);           
    	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Snail2305.Speed_ARR);
		Snail2305.SpeedUpFinish = 1;
	}
	else if(FricStatus == FricStatus_Working_Low)
	{
		Snail2305.SpeedUpFinish = 1;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1200);           
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1200);
	}
	else
	{
		Snail2305.Speed_ARR = 1000;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Snail2305.Speed_ARR);           
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Snail2305.Speed_ARR);
		Snail2305.SpeedUpFinish = 0;
	}
//            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,LowArr);           
//            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,HighArr);
//    if(FricStatus == FricStatus_Working_High)
//			{     
//        while(Snail2305.Speed_ARR < HighArr)
//				{
//            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,++Snail2305.Speed_ARR);           
//            
//            //???????
//            vTaskDelay(1);
//				}
////				while(tempLeft<LeftHighArr)
////				{
////					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,++tempLeft);           
////            
////            //???????
////            vTaskDelay(1);
////				}
//				Laser_On();
//        Snail2305.SpeedUpFinish = pdTRUE;
//		}
//		else if(FricStatus == FricStatus_Working_Low) 
//		{
//			while(Snail2305.Speed_ARR < MidArr)
//				{
//            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,++Snail2305.Speed_ARR);           
//            
//            //???????
//            vTaskDelay(1);
//				}
////				while(tempLeft<LeftMidArr)
////				{
////					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,++tempLeft);           
////            
////            //???????
////            vTaskDelay(1);
////				}
//				Laser_On();
//        Snail2305.SpeedUpFinish = pdTRUE;
//		}
//		else
//    {
//        Snail2305.Speed_ARR = LowArr;
//				tempLeft=LeftLowArr;
//        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Snail2305.Speed_ARR);           
//        //__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,LeftLowArr);     
//        Laser_Off();  
//        Snail2305.SpeedUpFinish = pdFALSE;
//    }
}
     
//    else if(FricStatus == FricStatus_Working_Low)     
//    {
//        while(Snail2305.Speed_ARR < LowArr)
//        {
//            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,++Snail2305.Speed_ARR);           
//            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,++Snail2305.Speed_ARR);          
//            //???????
//            vTaskDelay(5);      
//        }    
//        Laser_On();
//        Snail2305.SpeedUpFinish = pdTRUE;
//    }
//    else
//    {
//        Snail2305.Speed_ARR = ARR_MIN;
//        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Snail2305.Speed_ARR);           
//        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Snail2305.Speed_ARR);     
//        Laser_Off();  
//        Snail2305.SpeedUpFinish = pdFALSE;
//    }


/**
  * @brief  ???PID????
  * @param  void
  * @retval void
  */

//void FricMotor_PID_Ctrl(void)
//{
//	if(FricStatus == FricStatus_Stop)     
//   {
//		Snail2305_Left.Target_Speed=Speed_Low;
//		Snail2305_Right.Target_Speed=Speed_Low;
//		Snail2305_Left.Last_Speed_ARR =BASE_DUTY;
//		Snail2305_Right.Last_Speed_ARR =BASE_DUTY;
//		Snail2305_Left.Speed_ARR = BASE_DUTY;
//		Snail2305_Right.Speed_ARR = BASE_DUTY;
//		Snail2305_Left.Cur_Error = 0;
//		Snail2305_Right.Cur_Error = 0;
//		Snail2305_Left.Last_Error = 0;
//		Snail2305_Right.Last_Error = 0;
//		Snail2305_Left.Sum_Error = 0;
//		Snail2305_Right.Sum_Error = 0;
//		Snail2305_Left.SpeedUpFinish = 0;
//		Snail2305_Right.SpeedUpFinish = 0;
//		Laser_Off();
//	}
//	else
//	{
//		if (FricStatus == FricStatus_Working_High)
//		{
//			if (Snail2305_Left.Speed_ARR >= FRIC_PWM_HIGH||Snail2305_Right.Speed_ARR >= FRIC_PWM_HIGH)
//			{
//				Snail2305_Left.SpeedUpFinish = 1;
//				Snail2305_Right.SpeedUpFinish = 1;
//				Laser_On();
//			}
//		}
//		else if(FricStatus == FricStatus_Working_Low)
//		{
//			if (Snail2305_Left.Speed_ARR >= FRIC_PWM_MID||Snail2305_Right.Speed_ARR >= FRIC_PWM_MID)
//			{
//				Snail2305_Left.SpeedUpFinish = 1;
//				Snail2305_Right.SpeedUpFinish = 1;
//				Laser_On();
//			}
//		}
//		//????????
//		if (Snail2305_Left.SpeedUpFinish == 0 && Snail2305_Right.SpeedUpFinish == 0)
//		{
//			if ((Snail2305_Left.Speedupcnt % 5) == 0 && (Snail2305_Right.Speedupcnt % 5) == 0 )
//			{
//				Snail2305_Left.Speedupcnt++;
//				Snail2305_Right.Speedupcnt++;
//			}
//			else
//			{
//				Snail2305_Left.Speed_ARR++;
//				Snail2305_Right.Speed_ARR++;
//			}
//		}
//		else
//		{
//			if (FricStatus == FricStatus_Working_High)
//			{
////				if(Snail2305_Left.Target_Speed < Speed_High && Snail2305_Right.Target_Speed < Speed_High)
////				{
////					Snail2305_Left.Target_Speed += 0.1f;
////				  Snail2305_Right.Target_Speed += 0.1f;
////				}
////				else
////				{
//				   Snail2305_Left.Target_Speed= Speed_High;
//				   Snail2305_Right.Target_Speed= Speed_High;
////				}
//			}
//			else if (FricStatus == FricStatus_Working_Low)
//			{
////				if(Snail2305_Left.Target_Speed < Speed_Low && Snail2305_Right.Target_Speed < Speed_Low)
////				{
////					Snail2305_Left.Target_Speed  += 0.1f;
////					Snail2305_Right.Target_Speed += 0.1f;
////				}
////				else
////				{
//				Snail2305_Left.Target_Speed= Speed_Low;
//				Snail2305_Right.Target_Speed= Speed_Low;
////				}
//			}
//			Snail2305_Left.Last_Error=Snail2305_Left.Cur_Error;
//			Snail2305_Left.Last_Speed_ARR=Snail2305_Left.Speed_ARR;
//			Snail2305_Left.Cur_Error=Snail2305_Left.Target_Speed-Snail2305_Left.Real_Speed;
//			Snail2305_Left.Sum_Error+=Snail2305_Left.Cur_Error;
//			Snail2305_Left.Sum_Error=Snail2305_Left.Sum_Error<sum_Error_Boundary?Snail2305_Left.Sum_Error:sum_Error_Boundary;
//			Snail2305_Left.Sum_Error=Snail2305_Left.Sum_Error>-sum_Error_Boundary?Snail2305_Left.Sum_Error:-sum_Error_Boundary;
//			
//			int32_t output=Snail2305_Left.Kp*Snail2305_Left.Cur_Error\
//															+Snail2305_Left.Ki*Snail2305_Left.Sum_Error\
//															+Snail2305_Left.Kd*(Snail2305_Left.Cur_Error-Snail2305_Left.Last_Error)+1500;
//			output = output > 800 ? 800 : output;
//			output = output < -50 ? -50 : output;
//			if (FricStatus == FricStatus_Working_High)
//			{
//				output += Speed_High;
//			}
//			else if (FricStatus == FricStatus_Working_Low)
//			{
//				output += Speed_Mid;
//			}
//			output=output>1000?output:1000;
//			output=output<1900?output:1900;

//			if(output>Snail2305_Left.Last_Speed_ARR+20)
//				output=Snail2305_Left.Last_Speed_ARR+20;
//			else if(output<Snail2305_Left.Last_Speed_ARR-20)
//				output=Snail2305_Left.Last_Speed_ARR-20;
//			Snail2305_Left.Speed_ARR=output;
//			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Snail2305_Left.Speed_ARR);
//			Snail2305_Right.Last_Error=Snail2305_Right.Cur_Error;
//			Snail2305_Right.Last_Speed_ARR=Snail2305_Right.Speed_ARR;
//			Snail2305_Right.Cur_Error=Snail2305_Right.Target_Speed-Snail2305_Right.Real_Speed;
//			Snail2305_Right.Sum_Error+=Snail2305_Right.Cur_Error;
//			Snail2305_Right.Sum_Error=Snail2305_Right.Sum_Error<sum_Error_Boundary?Snail2305_Right.Sum_Error:sum_Error_Boundary;
//			Snail2305_Right.Sum_Error=Snail2305_Right.Sum_Error>-sum_Error_Boundary?Snail2305_Right.Sum_Error:-sum_Error_Boundary;
//			
//			output=Snail2305_Right.Kp*Snail2305_Right.Cur_Error\
//															+Snail2305_Right.Ki*Snail2305_Right.Sum_Error\
//															+Snail2305_Right.Kd*(Snail2305_Right.Cur_Error-Snail2305_Right.Last_Error)+1500;
//			output = output > 800 ? 800 : output;
//			output = output < -50 ? -50 : output;
//			if (FricStatus == FricStatus_Working_High)
//			{
//				output += FRIC_PWM_HIGH;
//			}
//			else if (FricStatus == FricStatus_Working_Low)
//			{
//				output += FRIC_PWM_MID;
//			}
//			output=output>1000?output:1000;
//			output=output<1900?output:1900;
//			if(output>Snail2305_Right.Last_Speed_ARR+20)
//				output=Snail2305_Right.Last_Speed_ARR+20;
//			else if(output<Snail2305_Right.Last_Speed_ARR-20)
//				output=Snail2305_Right.Last_Speed_ARR-20;
//			Snail2305_Right.Speed_ARR=output;
//			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Snail2305_Right.Speed_ARR);
//		}
// 	}
//}
void FricMotor_pwm_control()
{
	static int leftcount = 0;
	static int rightcount = 0;
	if (FricStatus == FricStatus_Stop)
	{
		Snail2305_Left.Speed_ARR=1000;
		Snail2305_Right.Speed_ARR=1000;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Snail2305_Left.Speed_ARR);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Snail2305_Right.Speed_ARR);
    Laser_Off();  
	} 
	else if (FricStatus == FricStatus_Working_High)
	{
		/*?????*/
		if ( Snail2305_Left.Speed_ARR < FRIC_PWM_HIGH)
		{
			if (Snail2305_Left.Speed_ARR < 1100)
			{
				if (leftcount == 3)
				{
					Snail2305_Left.Speed_ARR++;
					leftcount = 0;
				}
				else
					leftcount++;
			}
			else
			{
				Snail2305_Left.Speed_ARR++;
				leftcount = 0;
			}
		}
		else
			Snail2305_Left.Speed_ARR = FRIC_PWM_HIGH;
		/*?????*/
		if (Snail2305_Right.Speed_ARR < FRIC_PWM_HIGH)
		{
			if (Snail2305_Right.Speed_ARR < 1100)
			{
				if (rightcount == 3)
				{
					Snail2305_Right.Speed_ARR++;
					rightcount = 0;
				}
				else
					rightcount++;
			}
			else
			{
				Snail2305_Right.Speed_ARR++;
				rightcount = 0;
			}
		}
		else
			Snail2305_Right.Speed_ARR = FRIC_PWM_HIGH;
		Laser_On();
	}
 	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Snail2305_Left.Speed_ARR);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Snail2305_Right.Speed_ARR);
	vTaskDelay(10);
}
