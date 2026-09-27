/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ARM_R_Pin GPIO_PIN_0
#define ARM_R_GPIO_Port GPIOF
#define ARM_G_Pin GPIO_PIN_1
#define ARM_G_GPIO_Port GPIOF
#define ARM_IN_Pin GPIO_PIN_3
#define ARM_IN_GPIO_Port GPIOF
#define ARM_IN_EXTI_IRQn EXTI3_IRQn
#define CONN_R_Pin GPIO_PIN_4
#define CONN_R_GPIO_Port GPIOF
#define CONN_G_Pin GPIO_PIN_5
#define CONN_G_GPIO_Port GPIOF
#define NFZ_R_Pin GPIO_PIN_6
#define NFZ_R_GPIO_Port GPIOF
#define NFZ_G_Pin GPIO_PIN_7
#define NFZ_G_GPIO_Port GPIOF
#define KILL_Pin GPIO_PIN_9
#define KILL_GPIO_Port GPIOF
#define KILL_EXTI_IRQn EXTI9_5_IRQn
#define VOLTAGE_Pin GPIO_PIN_10
#define VOLTAGE_GPIO_Port GPIOF
#define GPS_R_Pin GPIO_PIN_0
#define GPS_R_GPIO_Port GPIOA
#define GPS_G_Pin GPIO_PIN_1
#define GPS_G_GPIO_Port GPIOA
#define THROTTLE_Pin GPIO_PIN_6
#define THROTTLE_GPIO_Port GPIOA
#define PITCH_Pin GPIO_PIN_4
#define PITCH_GPIO_Port GPIOC
#define LPUSH_Pin GPIO_PIN_1
#define LPUSH_GPIO_Port GPIOB
#define LPUSH_EXTI_IRQn EXTI1_IRQn
#define RPUSH_Pin GPIO_PIN_2
#define RPUSH_GPIO_Port GPIOB
#define RPUSH_EXTI_IRQn EXTI2_IRQn
#define YAW_Pin GPIO_PIN_11
#define YAW_GPIO_Port GPIOF
#define ROLL_Pin GPIO_PIN_13
#define ROLL_GPIO_Port GPIOF
#define RADIO_EN_Pin GPIO_PIN_10
#define RADIO_EN_GPIO_Port GPIOB
#define WIFI_EN_Pin GPIO_PIN_11
#define WIFI_EN_GPIO_Port GPIOB
#define WIFI_TX_Pin GPIO_PIN_14
#define WIFI_TX_GPIO_Port GPIOB
#define WIFI_RX_Pin GPIO_PIN_15
#define WIFI_RX_GPIO_Port GPIOB
#define SD_DET_Pin GPIO_PIN_7
#define SD_DET_GPIO_Port GPIOC
#define SD_DET_EXTI_IRQn EXTI9_5_IRQn
#define TP_INT_Pin GPIO_PIN_6
#define TP_INT_GPIO_Port GPIOD
#define TP_INT_EXTI_IRQn EXTI9_5_IRQn
#define DISPLAY_RST_Pin GPIO_PIN_7
#define DISPLAY_RST_GPIO_Port GPIOD
#define LCD_RS_Pin GPIO_PIN_9
#define LCD_RS_GPIO_Port GPIOG
#define TP_CS_Pin GPIO_PIN_10
#define TP_CS_GPIO_Port GPIOG
#define LCD_CS_Pin GPIO_PIN_11
#define LCD_CS_GPIO_Port GPIOG
#define SCREEN_PWM_Pin GPIO_PIN_6
#define SCREEN_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
