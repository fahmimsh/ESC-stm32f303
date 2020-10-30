/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
bool PWMHigh = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;
extern COMP_HandleTypeDef hcomp3;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim15;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim2;
extern DMA_Event_t dma_uart_rx;
extern uint8_t compState;
extern uint16_t commutationTimerCounter;
extern uint16_t commutationTimerCounterArray[6];
extern uint8_t commutationStepCounter;
extern void commutationPattern(uint8_t step);
extern uint8_t waitForCommutation;
extern uint16_t commutationTimerCounterAverage;
extern uint16_t commutationTimerOffset;
extern uint16_t counterTIM3Values;
extern uint16_t oc5ValueOld;

extern uint16_t oc5Value;
extern uint16_t setPWM,newPWM;
extern uint8_t pwmState;
extern int16_t compWindowOffset;
extern DAC_HandleTypeDef hdac;

extern ADC_ChannelConfTypeDef sConfig;
extern uint32_t adcIntegral;
extern uint32_t adcOffset;
extern uint8_t adcCounter;

extern uint32_t tim2cnt;
extern bool readRotation;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  if(dma_uart_rx.timer == 1)
     {
         /* DMA Timeout event: set Timeout Flag and call DMA Rx Complete Callback */
         dma_uart_rx.flag = 1;
         hdma_usart1_rx.XferCpltCallback(&hdma_usart1_rx);
     }
     if(dma_uart_rx.timer) { --dma_uart_rx.timer; }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
	if(__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_EOC) == SET){
		adcIntegral += HAL_ADC_GetValue(&hadc2);
	}
	if(adcIntegral >= adcOffset){
		HAL_ADC_Stop_IT(&hadc2);
		adcIntegral = 0;
		commutationPattern(NEXT);
	}
  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break and TIM15 interrupts.
  */
void TIM1_BRK_TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim15);
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
	/*if(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC2) != RESET){
		if(__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_EOC) != RESET){
			adcIntegral += HAL_ADC_GetValue(&hadc2);
		}
		if(PWMHigh){
			//while(TIM1->CNT < TIM1->CCR2){
			//for(int i = 0; i < setPWM / 100; i++);

			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		} else if(!PWMHigh){

		}

		PWMHigh = !PWMHigh;
	}*/
	/*if(adcIntegral >= adcOffset){
		adcIntegral = 0;
		__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC2);
		//HAL_ADC_Stop(&hadc2);
		commutationPattern(NEXT);
	}*/

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	//if((__HAL_TIM_GET_FLAG(&htim3,TIM_FLAG_CC1)== SET) && waitForCommutation == 1)
	//{
		//commutationPattern(NEXT);
	//}
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
    if((USART1->ISR & USART_ISR_IDLE) != RESET)
    {
        USART1->ICR = UART_CLEAR_IDLEF;
        /* Start DMA timer */
        dma_uart_rx.timer = DMA_TIMEOUT_MS;
    }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles Timer 6 interrupt and DAC underrun interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  HAL_DAC_IRQHandler(&hdac);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles COMP1, COMP2 and COMP3 interrupts through EXTI lines 21, 22 and 29.
  */
void COMP1_2_3_IRQHandler(void)
{
  /* USER CODE BEGIN COMP1_2_3_IRQn 0 */
	TIM1->CCR5 = setPWM + compWindowOffset;

	if(__HAL_COMP_COMP1_EXTI_GET_FLAG() && waitForCommutation == 0){
		HAL_COMP_Stop_IT(&hcomp1);
		sConfig.Channel = ADC_CHANNEL_2;

		/*commutationTimerCounterArray[5] = __HAL_TIM_GET_COUNTER(&htim3);
		commutationTimerCounterAverage = (commutationTimerCounterArray[0]+commutationTimerCounterArray[1]+commutationTimerCounterArray[2]+commutationTimerCounterArray[3]+commutationTimerCounterArray[4]+commutationTimerCounterArray[5])/12;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, commutationTimerCounterAverage + commutationTimerOffset);
		__HAL_TIM_SET_COUNTER(&htim3, 0);

		for(uint8_t i = 0; i<5; i++) commutationTimerCounterArray[i] = commutationTimerCounterArray[i+1];*/

		waitForCommutation = 1;

	}

	else if(__HAL_COMP_COMP2_EXTI_GET_FLAG() && waitForCommutation == 0){
		HAL_COMP_Stop_IT(&hcomp2);
		sConfig.Channel = ADC_CHANNEL_3;

		/*commutationTimerCounterArray[5] = __HAL_TIM_GET_COUNTER(&htim3);
		commutationTimerCounterAverage = (commutationTimerCounterArray[0]+commutationTimerCounterArray[1]+commutationTimerCounterArray[2]+commutationTimerCounterArray[3]+commutationTimerCounterArray[4]+commutationTimerCounterArray[5])/12;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, commutationTimerCounterAverage + commutationTimerOffset);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		for(uint8_t i = 0; i<5; i++) commutationTimerCounterArray[i] = commutationTimerCounterArray[i+1];*/

		waitForCommutation = 1;
	}

	else if(__HAL_COMP_COMP3_EXTI_GET_FLAG() && waitForCommutation == 0){
		HAL_COMP_Stop_IT(&hcomp3);
		sConfig.Channel = ADC_CHANNEL_4;

		/*commutationTimerCounterArray[5] = __HAL_TIM_GET_COUNTER(&htim3);
		commutationTimerCounterAverage = (commutationTimerCounterArray[0]+commutationTimerCounterArray[1]+commutationTimerCounterArray[2]+commutationTimerCounterArray[3]+commutationTimerCounterArray[4]+commutationTimerCounterArray[5])/12;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, commutationTimerCounterAverage + commutationTimerOffset);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		for(uint8_t i = 0; i<5; i++) commutationTimerCounterArray[i] = commutationTimerCounterArray[i+1];*/

		waitForCommutation = 1;

		/*if(setPWM > 300 && pwmState == 0){
			HAL_GPIO_WritePin(GPIOB, OUT_A_Pin | OUT_B_Pin | OUT_C_Pin, GPIO_PIN_SET);

			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_5);
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1900 + oc5Value);

			 TIM_OC_InitTypeDef sConfigOC5A;
			 sConfigOC5A.OCMode = TIM_OCMODE_PWM1;
			 compWindowOffset = -80;
			 sConfigOC5A.OCFastMode = TIM_OCFAST_DISABLE; //DISABLE
			 sConfigOC5A.OCIdleState = TIM_OCIDLESTATE_RESET;
			 sConfigOC5A.OCPolarity = TIM_OCPOLARITY_LOW; //LOW for PWM high detection
			 HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC5A, TIM_CHANNEL_5);
			 TIM1->CCR5 = setPWM + compWindowOffset;
			 pwmState = 1;
			 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_5);
		}*/

		/*if(setPWM > 300 && pwmState == 1 && oc5Value != oc5ValueOld){
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1900 + oc5Value);
			oc5ValueOld = oc5Value;
		}*/

		/*if(setPWM <= 250 && pwmState == 1){
			HAL_GPIO_WritePin(GPIOB, OUT_A_Pin | OUT_B_Pin | OUT_C_Pin, GPIO_PIN_RESET);

			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_5);
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);

			TIM_OC_InitTypeDef sConfigOC5B;
			sConfigOC5B.OCMode = TIM_OCMODE_PWM1;
			compWindowOffset = 300;
			sConfigOC5B.OCFastMode = TIM_OCFAST_DISABLE;
			sConfigOC5B.OCIdleState = TIM_OCIDLESTATE_RESET;
			sConfigOC5B.OCPolarity = TIM_OCPOLARITY_HIGH; //HIGH for PWM low detection
			HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC5B, TIM_CHANNEL_5);
			TIM1->CCR5 = setPWM + compWindowOffset;
			pwmState = 0;
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_5);*/
		//}
	}

	TIM1->CCR5 = setPWM + compWindowOffset;
	TIM1->CCR2 = setPWM;

	adcOffset = 1000;
	adcIntegral = 0;
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);
	HAL_ADC_Start_IT(&hadc2);
	//HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);

/*	adcOffset = 100;
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);
	HAL_ADC_Start(&hadc2);
	//for(int i=0;i<50;i++) asm("nop");

	while(adcIntegral <= adcOffset){
		if(__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_EOC) == SET){
			adcIntegral += HAL_ADC_GetValue(&hadc2);
			for(int i=0; i<50; i++);
			//__HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_EOC);
		}
	}

	HAL_ADC_Stop(&hadc2);
	adcIntegral = 0;
	commutationPattern(NEXT);*/


  /* USER CODE END COMP1_2_3_IRQn 0 */
  HAL_COMP_IRQHandler(&hcomp1);
  HAL_COMP_IRQHandler(&hcomp2);
  HAL_COMP_IRQHandler(&hcomp3);
  /* USER CODE BEGIN COMP1_2_3_IRQn 1 */

  /* USER CODE END COMP1_2_3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
