/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define BLUE_LED_Pin GPIO_PIN_13
#define BLUE_LED_GPIO_Port GPIOC
#define BATV_SNS_Pin GPIO_PIN_0
#define BATV_SNS_GPIO_Port GPIOA
#define GNSS_TX_Pin GPIO_PIN_2
#define GNSS_TX_GPIO_Port GPIOA
#define GNSS_RX_Pin GPIO_PIN_3
#define GNSS_RX_GPIO_Port GPIOA
#define PWM_MOTOR_Pin GPIO_PIN_0
#define PWM_MOTOR_GPIO_Port GPIOB
#define MODEM_LED_Pin GPIO_PIN_14
#define MODEM_LED_GPIO_Port GPIOB
#define MODEM_TX_Pin GPIO_PIN_9
#define MODEM_TX_GPIO_Port GPIOA
#define MODEM_RX_Pin GPIO_PIN_10
#define MODEM_RX_GPIO_Port GPIOA
#define MODEM_AUX_Pin GPIO_PIN_11
#define MODEM_AUX_GPIO_Port GPIOA
#define MODEM_M0_Pin GPIO_PIN_12
#define MODEM_M0_GPIO_Port GPIOA
#define MODEM_M1_Pin GPIO_PIN_15
#define MODEM_M1_GPIO_Port GPIOA
#define PWM_ROLL_Pin GPIO_PIN_4
#define PWM_ROLL_GPIO_Port GPIOB
#define PWM_PITCH_Pin GPIO_PIN_5
#define PWM_PITCH_GPIO_Port GPIOB
#define MEMS_I2C_SCL_Pin GPIO_PIN_6
#define MEMS_I2C_SCL_GPIO_Port GPIOB
#define MEMS_I2C_SDA_Pin GPIO_PIN_7
#define MEMS_I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define __ramfunc  __attribute__ ((section (".RamFunc")))
extern UART_HandleTypeDef huart1;
#define HAL_MODEM_UART	huart1
extern UART_HandleTypeDef huart2;
#define HAL_GNSS_UART	huart2
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
#define HAL_MEMS_I2C	hi2c1

//static void MX_USART1_UART_Init(void);
extern void MX_UARTReInit();
extern void MX_USART1_UART_Init115200();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
