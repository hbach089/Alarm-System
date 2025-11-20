/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLUE_PUSH_BTN_Pin GPIO_PIN_13
#define BLUE_PUSH_BTN_GPIO_Port GPIOC
#define BLUE_PUSH_BTN_EXTI_IRQn EXTI15_10_IRQn
#define RED_LED_Pin GPIO_PIN_0
#define RED_LED_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define GREEN_LED_Pin GPIO_PIN_4
#define GREEN_LED_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define BUZZER_TIM3_CH3_Pin GPIO_PIN_0
#define BUZZER_TIM3_CH3_GPIO_Port GPIOB
#define C1_OPT_Pin GPIO_PIN_10
#define C1_OPT_GPIO_Port GPIOB
#define R2_IPT_Pin GPIO_PIN_7
#define R2_IPT_GPIO_Port GPIOC
#define R4_IPT_Pin GPIO_PIN_8
#define R4_IPT_GPIO_Port GPIOA
#define R3_IPT_Pin GPIO_PIN_9
#define R3_IPT_GPIO_Port GPIOA
#define PIR_Sensor_Pin GPIO_PIN_10
#define PIR_Sensor_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define C2_OPT_Pin GPIO_PIN_4
#define C2_OPT_GPIO_Port GPIOB
#define C3_OPT_Pin GPIO_PIN_5
#define C3_OPT_GPIO_Port GPIOB
#define R1_IPT_Pin GPIO_PIN_6
#define R1_IPT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
