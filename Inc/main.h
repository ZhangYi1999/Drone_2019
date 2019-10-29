/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Judge_Pin GPIO_PIN_0
#define Judge_GPIO_Port GPIOE
#define Laser_Pin GPIO_PIN_13
#define Laser_GPIO_Port GPIOG
#define CampInput_Pin GPIO_PIN_0
#define CampInput_GPIO_Port GPIOF
#define Power1_Pin GPIO_PIN_2
#define Power1_GPIO_Port GPIOH
#define Power2_Pin GPIO_PIN_3
#define Power2_GPIO_Port GPIOH
#define CampOutput_Pin GPIO_PIN_1
#define CampOutput_GPIO_Port GPIOF
#define Power3_Pin GPIO_PIN_4
#define Power3_GPIO_Port GPIOH
#define Power4_Pin GPIO_PIN_5
#define Power4_GPIO_Port GPIOH
#define Right_Switch_Pin GPIO_PIN_12
#define Right_Switch_GPIO_Port GPIOH
#define STATUS_LED5_Pin GPIO_PIN_5
#define STATUS_LED5_GPIO_Port GPIOG
#define STATUS_LED4_Pin GPIO_PIN_4
#define STATUS_LED4_GPIO_Port GPIOG
#define STATUS_LED3_Pin GPIO_PIN_3
#define STATUS_LED3_GPIO_Port GPIOG
#define Mid_Switch_Pin GPIO_PIN_11
#define Mid_Switch_GPIO_Port GPIOH
#define Left_Switch_Pin GPIO_PIN_10
#define Left_Switch_GPIO_Port GPIOH
#define STATUS_LED2_Pin GPIO_PIN_2
#define STATUS_LED2_GPIO_Port GPIOG
#define Key_Pin GPIO_PIN_2
#define Key_GPIO_Port GPIOB
#define STATUS_LED1_Pin GPIO_PIN_1
#define STATUS_LED1_GPIO_Port GPIOG
#define JetsonComm_Tx_Pin GPIO_PIN_8
#define JetsonComm_Tx_GPIO_Port GPIOE
#define Red_Led_Pin GPIO_PIN_11
#define Red_Led_GPIO_Port GPIOE
#define Green_Led_Pin GPIO_PIN_14
#define Green_Led_GPIO_Port GPIOF
#define JetsonComm_Rx_Pin GPIO_PIN_7
#define JetsonComm_Rx_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
