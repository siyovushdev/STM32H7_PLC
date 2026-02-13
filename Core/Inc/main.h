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
#include "stm32h7xx_hal.h"

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
#define DO2_Pin GPIO_PIN_2
#define DO2_GPIO_Port GPIOE
#define DO4_Pin GPIO_PIN_4
#define DO4_GPIO_Port GPIOE
#define DO5_Pin GPIO_PIN_5
#define DO5_GPIO_Port GPIOE
#define DO6_Pin GPIO_PIN_6
#define DO6_GPIO_Port GPIOE
#define ADC_CS_Pin GPIO_PIN_0
#define ADC_CS_GPIO_Port GPIOB
#define ADC_CVA_Pin GPIO_PIN_1
#define ADC_CVA_GPIO_Port GPIOB
#define ADC_CVB_Pin GPIO_PIN_2
#define ADC_CVB_GPIO_Port GPIOB
#define ADC_RST_Pin GPIO_PIN_7
#define ADC_RST_GPIO_Port GPIOE
#define ADC_BUSY_Pin GPIO_PIN_8
#define ADC_BUSY_GPIO_Port GPIOE
#define ADC_BUSY_EXTI_IRQn EXTI9_5_IRQn
#define DAC_CLR_Pin GPIO_PIN_14
#define DAC_CLR_GPIO_Port GPIOB
#define DAC_LDAC_Pin GPIO_PIN_15
#define DAC_LDAC_GPIO_Port GPIOB
#define DAC_SYNC_Pin GPIO_PIN_8
#define DAC_SYNC_GPIO_Port GPIOD
#define DO7_Pin GPIO_PIN_10
#define DO7_GPIO_Port GPIOD
#define DI0_Pin GPIO_PIN_0
#define DI0_GPIO_Port GPIOD
#define DI1_Pin GPIO_PIN_1
#define DI1_GPIO_Port GPIOD
#define DI2_Pin GPIO_PIN_2
#define DI2_GPIO_Port GPIOD
#define DI3_Pin GPIO_PIN_3
#define DI3_GPIO_Port GPIOD
#define DI4_Pin GPIO_PIN_4
#define DI4_GPIO_Port GPIOD
#define DI6_Pin GPIO_PIN_6
#define DI6_GPIO_Port GPIOD
#define DI7_Pin GPIO_PIN_7
#define DI7_GPIO_Port GPIOD
#define DO0_Pin GPIO_PIN_0
#define DO0_GPIO_Port GPIOE
#define DO1_Pin GPIO_PIN_1
#define DO1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
