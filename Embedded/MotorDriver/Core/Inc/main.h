/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "stm32f3xx_hal.h"

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
#define PWM_FREQUENCY 22000
#define TIM1_RELOAD (TIM1_CLOCK_FREQUENCY / PWM_FREQUENCY)
#define SAMPLES_PER_PERIOD 8
#define SAMPLING_FREQUENCY (SAMPLES_PER_PERIOD * PWM_FREQUENCY)
#define TIM2_RELOAD (TIM2_CLOCK_FREQUENCY / SAMPLING_FREQUENCY)
#define TIM2_CLOCK_FREQUENCY 72000000
#define TIM1_CLOCK_FREQUENCY 144000000
#define CONTROL_FREQUENCY 10000
#define TIM6_CLOCK_FREQUENCY 72000000
#define TIM6_RELOAD (TIM6_CLOCK_FREQUENCY / CONTROL_FREQUENCY)
#define ISENS_A_Pin GPIO_PIN_0
#define ISENS_A_GPIO_Port GPIOA
#define ISENS_B_Pin GPIO_PIN_1
#define ISENS_B_GPIO_Port GPIOA
#define VSENS_Pin GPIO_PIN_2
#define VSENS_GPIO_Port GPIOA
#define CALAD_Pin GPIO_PIN_3
#define CALAD_GPIO_Port GPIOA
#define CALBC_Pin GPIO_PIN_4
#define CALBC_GPIO_Port GPIOA
#define nRESET_Pin GPIO_PIN_5
#define nRESET_GPIO_Port GPIOA
#define nSLEEP_Pin GPIO_PIN_6
#define nSLEEP_GPIO_Port GPIOA
#define PWM_A_Pin GPIO_PIN_7
#define PWM_A_GPIO_Port GPIOA
#define nDRV8881_SLEEP_Pin GPIO_PIN_0
#define nDRV8881_SLEEP_GPIO_Port GPIOB
#define PWM_nA_Pin GPIO_PIN_8
#define PWM_nA_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_9
#define PWM_B_GPIO_Port GPIOA
#define STEP_Pin GPIO_PIN_10
#define STEP_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_11
#define DIR_GPIO_Port GPIOA
#define PWM_nB_Pin GPIO_PIN_12
#define PWM_nB_GPIO_Port GPIOA
#define nDRV8881_FAULT_Pin GPIO_PIN_3
#define nDRV8881_FAULT_GPIO_Port GPIOB
#define DRIVER_TX_Pin GPIO_PIN_6
#define DRIVER_TX_GPIO_Port GPIOB
#define DRIVER_RX_Pin GPIO_PIN_7
#define DRIVER_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
