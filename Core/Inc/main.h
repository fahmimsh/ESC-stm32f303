/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
typedef struct
{
    volatile uint8_t flag;     /* Timeout event flag */
    uint16_t timer;             /* Timeout duration in msec */
    uint16_t prevCNDTR;         /* Holds previous value of DMA_CNDTR */
} DMA_Event_t;
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
#define ADC_CS_Pin GPIO_PIN_0
#define ADC_CS_GPIO_Port GPIOA
#define COMP_PHASEA_Pin GPIO_PIN_1
#define COMP_PHASEA_GPIO_Port GPIOA
#define PWMIN_Pin GPIO_PIN_2
#define PWMIN_GPIO_Port GPIOA
#define COMP_PHASEB_Pin GPIO_PIN_3
#define COMP_PHASEB_GPIO_Port GPIOA
#define ADC_PHASEA_Pin GPIO_PIN_5
#define ADC_PHASEA_GPIO_Port GPIOA
#define ADC_PHASEB_Pin GPIO_PIN_6
#define ADC_PHASEB_GPIO_Port GPIOA
#define ADC_PHASEC_Pin GPIO_PIN_7
#define ADC_PHASEC_GPIO_Port GPIOA
#define OUT_A_Pin GPIO_PIN_0
#define OUT_A_GPIO_Port GPIOB
#define OUT_B_Pin GPIO_PIN_1
#define OUT_B_GPIO_Port GPIOB
#define OUT_C_Pin GPIO_PIN_10
#define OUT_C_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOB
#define COMP_PHASEC_Pin GPIO_PIN_14
#define COMP_PHASEC_GPIO_Port GPIOB
#define PWMOUT_Pin GPIO_PIN_8
#define PWMOUT_GPIO_Port GPIOA
#define IN_A_Pin GPIO_PIN_3
#define IN_A_GPIO_Port GPIOB
#define INH_A_Pin GPIO_PIN_4
#define INH_A_GPIO_Port GPIOB
#define IN_B_Pin GPIO_PIN_5
#define IN_B_GPIO_Port GPIOB
#define INH_B_Pin GPIO_PIN_6
#define INH_B_GPIO_Port GPIOB
#define IN_C_Pin GPIO_PIN_7
#define IN_C_GPIO_Port GPIOB
#define INH_C_Pin GPIO_PIN_8
#define INH_C_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define map(x,in_min,in_max,out_min,out_max) ( (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min )
#define constrain(nilaix,bawah,atas) ( (nilaix)<(bawah) ? (bawah) : ( (nilaix)>(atas) ? (atas) : (nilaix) ) )
//#define getRPM(cnt) (float)((360 / ((float)cnt / 18000000)) /  6) / 12
//#define geRPM(T) 60 / (T * 6 * 6/12000000)
#define getRPM(T) 3333333.33333/(float)T
#define PWM_MAX 1600
#define PWM_MIN 0
#define FREQ_INPUT_PWM_MAX 51.0
#define FREQ_INPUT_PWM_MIN 48.0
#define NEXT 9
#define STEP_0 0
#define STEP_1 1
#define STEP_2 2
#define STEP_3 3
#define STEP_4 4
#define STEP_5 5
#define COMPDELAY for(uint32_t i = 0; i < 250; i++) asm("nop")
#define DMA_BUF_SIZE 50
#define DMA_TIMEOUT_MS 10      /* DMA Timeout duration in msec */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
