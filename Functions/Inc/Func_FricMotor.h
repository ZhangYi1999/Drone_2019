#ifndef __FRICMOTOR_H
#define __FRICMOTOR_H

#include "System_Config.h"

#ifdef  __FRICMOTOR_GLOBALS
#define __FRICMOTOR_EXT
#else
#define __FRICMOTOR_EXT extern
#endif



#define Laser_On()      HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_SET)
#define Laser_Off()     HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_RESET)

//Ä¦²ÁÂÖµç»ú
typedef struct
{
		float Kp, Ki, Kd;
		float Cur_Error, Last_Error, Sum_Error;
		float Real_Speed;
		float Target_Speed;
    uint16_t Speed_ARR;
		uint16_t Last_Speed_ARR;
    uint8_t  SpeedUpFinish;
	  int16_t Speedupcnt;
}FricMotor_Struct;

extern FricMotor_Struct Snail2305;
extern FricMotor_Struct Snail2305_Left;
extern FricMotor_Struct Snail2305_Right;
extern uint8_t Firc_times;
extern uint8_t Fric_Init_State;

void FricMotor_Init(void);
void FricMotor_Get_Speed(void);
void FricMotor_Ctrl(void);
void FricMotor_PID_Ctrl(void);
#endif
