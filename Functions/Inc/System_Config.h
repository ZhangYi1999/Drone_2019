#ifndef __SYSCONFIG_H
#define __SYSCONFIG_H

#include <stdlib.h>
#include <math.h>
#include <string.h> 
/**********   BSP   *********/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "spi.h"
#include "math.h"
/**********  OS   *********/
#include "cmsis_os.h"


//RTOS任务资源监视
#define RTOS_DEBUG_ENABLE  1

#define Pi  3.1415926535f

//开发板模式 (MASTER_MODE SLAVE_MODE)
#define MASTER_MODE

//用户按键是否按下
#define UserButtonPressed !HAL_GPIO_ReadPin(Key_GPIO_Port,Key_Pin)
//LED闪烁
#define Green_LED_Blink() HAL_GPIO_TogglePin(Green_Led_GPIO_Port,Green_Led_Pin)
#define Red_LED_Blink()   HAL_GPIO_TogglePin(Red_Led_GPIO_Port,Red_Led_Pin)

#define LED0_ON() HAL_GPIO_WritePin(STATUS_LED1_GPIO_Port,STATUS_LED1_Pin,0);
#define LED1_ON() HAL_GPIO_WritePin(STATUS_LED2_GPIO_Port,STATUS_LED2_Pin,0);
#define LED2_ON() HAL_GPIO_WritePin(STATUS_LED3_GPIO_Port,STATUS_LED3_Pin,0);
#define LED3_ON() HAL_GPIO_WritePin(STATUS_LED4_GPIO_Port,STATUS_LED4_Pin,0);
#define LED4_ON() HAL_GPIO_WritePin(STATUS_LED5_GPIO_Port,STATUS_LED5_Pin,0);

//红蓝阵营选择
#define WeAreRedTeam    HAL_GPIO_ReadPin(CampInput_GPIO_Port,CampInput_Pin)      //高电平为红方（不插）
#define WeAreBlueTeam    !HAL_GPIO_ReadPin(CampInput_GPIO_Port,CampInput_Pin)     //低电平为蓝方（插）

#define imu_ob_temp_pos  0
#define imu_ob_angle_pos 1
#define chassis_pos      2
#define imu_rec_pos      2
#define jetson_pos       3
#define btb_pos          4

#endif
