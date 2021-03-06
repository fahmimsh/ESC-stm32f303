/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "dwt_delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	MODE_MOTOR_STOP,
	MODE_MOTOR_CALIBRATION,
	MODE_MOTOR_READY,
	MODE_MOTOR_START,
	MODE_MOTOR_RUN
} MODE_MOTOR_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp3;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t buffer[200];
volatile uint16_t strSize;
volatile int commutationTimerCounter = 2000;
volatile uint16_t commutationTimerCounterArray[6] = { 20000, 20000, 2000, 20000, 20000, 20000 };
volatile uint8_t commutationStepCounter;
volatile uint8_t waitForCommutation = 1;
volatile uint16_t myADCvalue;
volatile uint16_t commutationTimerCounterAverage;
volatile uint16_t counterTIM3Values;
volatile int16_t commutationTimerOffset;

uint32_t tim2cnt;
uint8_t RXBuffer[200];
bool USART1DataFlag = false;
float adcIntegral = 0;

volatile int16_t newPWM = 100;
volatile int16_t setPWM = 100;
volatile int16_t compWindowOffset = 250; //this opens a comparator window to block overshooting signals from triggering (adjust to Vbat)
volatile uint16_t oc5Value = 200;  //preset offset-value (for fine tuning) of the DAC voltage output as the reference voltage for the compartors (about 0V or Vbat/2)
volatile uint8_t pwmState = 0; //indicates whether a zero crossing (pwm low) or a Vbat/2 crossing (pwm high) will be detected; 0 = PWM low detection, 1 = PWM high detection
volatile uint8_t bTransferRequest = 0;
volatile uint8_t motorGotStarted = 0;

ADC_ChannelConfTypeDef sConfig;
TIM_OC_InitTypeDef sConfigOC5B;
TIM_OC_InitTypeDef sConfigOC5A;
//float adcOffset = 2500;
float adcOffset = 2000;
uint8_t adcCounter = 0;
uint16_t csADC[12];
uint16_t adcBuffer[10];
uint16_t csSize;
uint8_t csString[100];
uint32_t tim7cnt;

volatile bool readRotation = false;

float measuredRPM = 0;
float inputRPM = 0;
uint32_t rpm_cnt = 0;

uint16_t input_pwm_min = 1000;
uint16_t input_pwm_max = 1900;

uint32_t inputDutyCycle = 0;
float inputFrequency = 0;
uint16_t oc5ValueOld = 0;

MODE_MOTOR_t mode_motor = MODE_MOTOR_STOP;
DMA_Event_t dma_uart_rx = {0, 0, DMA_BUF_SIZE};
uint8_t dma_rx_buf[DMA_BUF_SIZE];
uint8_t data[DMA_BUF_SIZE] = {'\0'};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_COMP3_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void Controller(){
	float kp = 0.005f, ki = 0.0025f, kd = 0.00f;
	float error = (float)(inputRPM - measuredRPM);

	static float sumIntegral;
	sumIntegral += error * 0.00001;
	sumIntegral = constrain(sumIntegral, -5000, 5000);

	static float preverror;
	float derivative = (error - preverror) * 0.00001;
	preverror = error;

	float output = kp * error + ki * sumIntegral + kd * derivative;
	int16_t pwmError;
	if(output < 0){
		pwmError = (int16_t)map(output, -5000, 0,-PWM_MAX, PWM_MIN);
	} else if(output >= 0){
		pwmError = (int16_t)map(output, 0, 5000, PWM_MIN, PWM_MAX);
	}
	newPWM += pwmError;
	setPWM = constrain(newPWM, PWM_MIN, PWM_MAX);

	TIM1->CCR1 = setPWM;
	TIM1->CCR5 = setPWM + compWindowOffset;

}

void getUSARTData(){
	 if(USART1DataFlag){
		 USART1DataFlag = false;
		 if(data[0] == '#'){
			 if(memcmp(data + 1, "PWM=", 4) == 0 || memcmp(data + 1, "pwm=", 4) == 0){
				uint8_t value[4];
				memcpy(value, data + 5, 4);
				uint16_t x = atoi((char*)value);
				if(x){
					x = constrain(x, 1000, 2000);
					newPWM = map(x, 1000, 2000, PWM_MIN, PWM_MAX);
					setPWM = newPWM;

					TIM1->CCR1 = setPWM;
					TIM1->CCR5 = setPWM + compWindowOffset;

					strSize = sprintf((char*)buffer, "PWM: %d\r\n", newPWM);
					HAL_UART_Transmit(&huart1, buffer, strSize, 50);
				}
			}
			else if(memcmp(data + 1, "OC5=", 4) == 0 || memcmp(data + 1, "oc5=", 4) == 0){
				//uint8_t value[3];
				//memcpy(value, data + 5, 3);
				oc5Value = (uint16_t)strtoul((char*)data + 5, '\0',10);
				strSize = sprintf((char*)buffer, "oc5: %d\r\n", oc5Value);
				HAL_UART_Transmit(&huart1, buffer, strSize, 50);
			}
			else if(memcmp(data + 1, "STA", 3) == 0 || memcmp(data + 1, "sta", 3) == 0){
				motorGotStarted = 1;
				strSize = sprintf((char*)buffer, "Start Motor\r\n");
				HAL_UART_Transmit(&huart1, buffer, strSize, 50);
			}
			else if(memcmp(data + 1, "STO", 3) == 0 || memcmp(data + 1, "sto", 3) == 0){
				newPWM = setPWM = 0;
				TIM1->CCR1 = setPWM;
				TIM1->CCR5 = setPWM + compWindowOffset;
				motorGotStarted = 0;
				strSize = sprintf((char*)buffer, "Stop Motor\r\n");
				HAL_UART_Transmit(&huart1, buffer, strSize, 50);
			}
			else if(memcmp(data + 1, "ADO", 3) == 0 || memcmp(data +1, "ado", 3) == 0){
				adcOffset = (float)strtof((char*)data + 5,'\0');
				strSize = sprintf((char*)buffer, "ADC Offset: %f\r\n", adcOffset);
				HAL_UART_Transmit(&huart1, buffer, strSize, 20);
			}
			else if(memcmp(data + 1, "STS", 3) == 0 || memcmp(data + 1, "sts", 3) == 0){
				strSize = sprintf((char*)buffer, "adcOffset: %f\r\noc5Value: %d\r\n", adcOffset, oc5Value);
				HAL_UART_Transmit_IT(&huart1, buffer, strSize);
			}
		}
	 }
}
void commutateNow_0(void){
	HAL_GPIO_WritePin(INH_B_GPIO_Port, INH_B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_C_GPIO_Port, IN_C_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_SET);
}

void commutateNow_1(void){
	HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_B_GPIO_Port, IN_B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INH_B_GPIO_Port, INH_B_Pin, GPIO_PIN_SET);
}

void commutateNow_2(void){
	HAL_GPIO_WritePin(IN_A_GPIO_Port, IN_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_RESET);
}

void commutateNow_3(void){
	HAL_GPIO_WritePin(INH_B_GPIO_Port, INH_B_Pin, GPIO_PIN_RESET); //INH_B connected to ground, IC B Sleep Mode
	HAL_GPIO_WritePin(IN_C_GPIO_Port, IN_C_Pin, GPIO_PIN_SET); //IN_C floating, pwm mode
	HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_SET); //INH_C is high, IC C Active Mode

}

void commutateNow_4(void){
	HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_RESET); //INH_A connected to ground, IC A Sleep Mode
	HAL_GPIO_WritePin(IN_B_GPIO_Port, IN_B_Pin, GPIO_PIN_RESET); //IN_B connected to ground
	HAL_GPIO_WritePin(INH_B_GPIO_Port, INH_B_Pin, GPIO_PIN_SET); //INH_B is high, IC B Active Mode

}
void commutateNow_5(void){
	HAL_GPIO_WritePin(IN_A_GPIO_Port, IN_A_Pin, GPIO_PIN_SET); //IN_A connected to ground, PWM Mode
	HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_SET); //INH_A is HIGH, IC A Active Mode
	HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_RESET); //INH_C connected to ground, IC C Sleep Mode
}

void commutationPattern(uint8_t step){
	if (step == NEXT && waitForCommutation == 1) {
		if (commutationStepCounter < STEP_5)
			commutationStepCounter++;
		else{
			commutationStepCounter = STEP_0;
		}
		switch (commutationStepCounter) {
		case STEP_0:
			commutateNow_0();
			COMPDELAY;
			waitForCommutation = 0;
			HAL_COMP_Start_IT(&hcomp2);
			break;
		case STEP_1:
			commutateNow_1();
			COMPDELAY;
			waitForCommutation = 0;
			HAL_COMP_Start_IT(&hcomp1);
			break;
		case STEP_2:
			commutateNow_2();
			COMPDELAY;
			waitForCommutation = 0;
			HAL_COMP_Start_IT(&hcomp3);
			break;
		case STEP_3:
			commutateNow_3();
			COMPDELAY;
			waitForCommutation = 0;
			HAL_COMP_Start_IT(&hcomp2);
			break;
		case STEP_4:
			commutateNow_4();
			COMPDELAY;
			waitForCommutation = 0;
			HAL_COMP_Start_IT(&hcomp1);
			break;
		case STEP_5:
			commutateNow_5();
			COMPDELAY;
			waitForCommutation = 0;
			HAL_COMP_Start_IT(&hcomp3);
			break;
		}
	} else {
		waitForCommutation = 0;
		switch (step) {
		case STEP_0:
			commutateNow_0();
			break;
		case STEP_1:
			commutateNow_1();
			break;
		case STEP_2:
			commutateNow_2();
			break;
		case STEP_3:
			commutateNow_3();
			break;
		case STEP_4:
			commutateNow_4();
			break;
		case STEP_5:
			commutateNow_5();
			break;
		}
	}
}

/*void startMotor(void){
	newPWM = setPWM = 300;
	compWindowOffset = 200;
	TIM1->CCR1 = setPWM;
	TIM1->CCR5 = setPWM + compWindowOffset;

	commutateNow_0();
	HAL_Delay(150);
	commutationStepCounter = STEP_0;
	waitForCommutation = 1;

	for (uint16_t i = 0; i <1; i++) {
		commutationPattern(NEXT);
		waitForCommutation = 1;
		HAL_Delay(150);
	}
	HAL_Delay(150);
	motorGotStarted = 2;
}*/

/*void startMotor(){
	TIM1->CCR1 = 350;
	TIM1->CCR5 = 350 + compWindowOffset;
	waitForCommutation = 0;

	commutateNow_0();
	HAL_Delay(10);
	commutateNow_2();
	HAL_Delay(10);

	commutateNow_4();
	COMPDELAY;
	HAL_COMP_Start(&hcomp1);
	while(HAL_COMP_GetOutputLevel(&hcomp1) != COMP_OUTPUTLEVEL_LOW);
	HAL_COMP_Stop(&hcomp1);

	commutateNow_5();
	COMPDELAY;
	HAL_COMP_Start(&hcomp3);
	while(HAL_COMP_GetOutputLevel(&hcomp3) != COMP_OUTPUTLEVEL_HIGH);
	HAL_COMP_Stop(&hcomp3);

	__HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_EOC);
	__HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_EOC);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)csADC, 12);
	mode_motor = MODE_MOTOR_RUN;

	commutationStepCounter = STEP_5;
	waitForCommutation = 1;
	HAL_TIM_Base_Start(&htim7);
	commutationPattern(NEXT);
	//HAL_ADC_Start_IT(&hadc1);

	motorGotStarted = 2;
}*/

void startMotor(){
	adcIntegral = 0;
	setPWM =  newPWM = 350;

	TIM1->CCR1 = setPWM;
	TIM1->CCR5 = setPWM + compWindowOffset;

	uint8_t step = 0;
	uint16_t i = 2020;

	while(i > 1300){
		DWT_Delay(i);
		commutationPattern(step);
		step += 1;
		step %= 6;
		i -= 10;
		//DWT_Delay(10);
	}

	//HAL_ADC_Start_IT(&hadc1);
	__HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_EOC);
	__HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_EOS);
	__HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_EOC);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)csADC, 12);

	__HAL_TIM_SET_COUNTER(&htim2, 0);
	commutationStepCounter = STEP_5;
	waitForCommutation = 1;
	HAL_TIM_Base_Start(&htim7);
	commutationPattern(NEXT);

	mode_motor = MODE_MOTOR_RUN;
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_COMP3_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_TIM15_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  DWT_Init();
  mode_motor = MODE_MOTOR_STOP;

  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  sConfigOC5B.OCMode = TIM_OCMODE_PWM1;
  sConfigOC5B.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC5B.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC5B.OCPolarity = TIM_OCPOLARITY_HIGH; //HIGH for PWM low detection

  sConfigOC5A.OCMode = TIM_OCMODE_PWM1;
  sConfigOC5A.OCFastMode = TIM_OCFAST_DISABLE; //DISABLE
  sConfigOC5A.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC5A.OCPolarity = TIM_OCPOLARITY_LOW; //LOW for PWM high detection

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_5);
  HAL_TIM_Base_Start(&htim2);
  //HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
  newPWM = setPWM = TIM1->CCR1 = TIM1->CCR5 = 0;

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  if(HAL_UART_Receive_DMA(&huart1, dma_rx_buf, DMA_BUF_SIZE) != HAL_OK){
	  Error_Handler();
  }

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim16);

  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

  HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2);

  strSize = sprintf((char*)buffer, "Test\r\n");
  HAL_UART_Transmit(&huart1, buffer, strSize, 10);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(mode_motor){
	  case MODE_MOTOR_STOP:
		  break;
	  case MODE_MOTOR_CALIBRATION:
		  while(mode_motor == MODE_MOTOR_CALIBRATION){
			  static int calibration_counter;
			  if(inputDutyCycle < input_pwm_min) input_pwm_min = inputDutyCycle;
			  if(inputDutyCycle > input_pwm_max) input_pwm_max = inputDutyCycle;
			  if(inputDutyCycle < 1050){
				  if(calibration_counter > 1000){
					  strSize = sprintf((char*)buffer, "motor ready\r\n");
					  HAL_UART_Transmit(&huart1, buffer, strSize, 10);
					  mode_motor = MODE_MOTOR_READY;
				  }
				  calibration_counter++;
				  HAL_Delay(1);
			  }
		  }
		  break;
	  case MODE_MOTOR_READY:
		  TIM1->CCR1 = 0;
		  TIM1->CCR5 = 0;
		  tim2cnt = 0;
		  break;
	  case MODE_MOTOR_START:
		  startMotor();
		  break;
	  case MODE_MOTOR_RUN:
			newPWM = map(inputDutyCycle, input_pwm_min, input_pwm_max, PWM_MIN, PWM_MAX);
			setPWM = newPWM;

		  	if(inputDutyCycle < 1050){
				measuredRPM = 0;
				inputRPM = 0;
				mode_motor = MODE_MOTOR_READY;
				strSize = sprintf((char*)buffer, "motor ready\r\n");
				HAL_UART_Transmit(&huart1, buffer, strSize, 10);
				HAL_ADC_Stop_DMA(&hadc1);
		  	} else {
				TIM1->CCR1 = setPWM;
				TIM1->CCR5 = setPWM + compWindowOffset;
		  	}
	  	  break;
	  }

	  getUSARTData();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV6;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH1;
  hcomp1.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp1.Init.Output = COMP_OUTPUT_NONE;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRCE_TIM1OC5;
  hcomp1.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH1;
  hcomp2.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO2;
  hcomp2.Init.Output = COMP_OUTPUT_NONE;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRCE_TIM1OC5;
  hcomp2.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}

/**
  * @brief COMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP3_Init(void)
{

  /* USER CODE BEGIN COMP3_Init 0 */

  /* USER CODE END COMP3_Init 0 */

  /* USER CODE BEGIN COMP3_Init 1 */

  /* USER CODE END COMP3_Init 1 */
  hcomp3.Instance = COMP3;
  hcomp3.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH1;
  hcomp3.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp3.Init.Output = COMP_OUTPUT_NONE;
  hcomp3.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp3.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp3.Init.BlankingSrce = COMP_BLANKINGSRCE_TIM1OC5;
  hcomp3.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp3.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp3.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  if (HAL_COMP_Init(&hcomp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP3_Init 2 */

  /* USER CODE END COMP3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = 2000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC5REF_RISING_OC6REF_RISING;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 120;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 72 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10 ;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 6 - 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 72 - 1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 72 - 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT_A_Pin|OUT_B_Pin|OUT_C_Pin|LED1_Pin
                          |LED2_Pin|IN_A_Pin|INH_A_Pin|IN_B_Pin
                          |INH_B_Pin|IN_C_Pin|INH_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADC_PHASEA_Pin */
  GPIO_InitStruct.Pin = ADC_PHASEA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_PHASEA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_A_Pin OUT_B_Pin OUT_C_Pin IN_A_Pin
                           IN_B_Pin IN_C_Pin */
  GPIO_InitStruct.Pin = OUT_A_Pin|OUT_B_Pin|OUT_C_Pin|IN_A_Pin
                          |IN_B_Pin|IN_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin INH_A_Pin INH_B_Pin
                           INH_C_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|INH_A_Pin|INH_B_Pin
                          |INH_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    uint16_t i, pos, start, length;
    uint16_t currCNDTR = __HAL_DMA_GET_COUNTER(huart->hdmarx);

    /* Ignore IDLE Timeout when the received characters exactly filled up the DMA buffer and DMA Rx Complete IT is generated, but there is no new character during timeout */
    if(dma_uart_rx.flag && currCNDTR == DMA_BUF_SIZE)
    {
        dma_uart_rx.flag = 0;
        return;
    }

    /* Determine start position in DMA buffer based on previous CNDTR value */
    start = (dma_uart_rx.prevCNDTR < DMA_BUF_SIZE) ? (DMA_BUF_SIZE - dma_uart_rx.prevCNDTR) : 0;

    if(dma_uart_rx.flag)    /* Timeout event */
    {
        /* Determine new data length based on previous DMA_CNDTR value:
         *  If previous CNDTR is less than DMA buffer size: there is old data in DMA buffer (from previous timeout) that has to be ignored.
         *  If CNDTR == DMA buffer size: entire buffer content is new and has to be processed.
        */
        length = (dma_uart_rx.prevCNDTR < DMA_BUF_SIZE) ? (dma_uart_rx.prevCNDTR - currCNDTR) : (DMA_BUF_SIZE - currCNDTR);
        dma_uart_rx.prevCNDTR = currCNDTR;
        dma_uart_rx.flag = 0;
    }
    else                /* DMA Rx Complete event */
    {
        length = DMA_BUF_SIZE - start;
        dma_uart_rx.prevCNDTR = DMA_BUF_SIZE;
    }

    /* Copy and Process new data */
    for(i=0,pos=start; i<length; ++i,++pos)
    {
        data[i] = dma_rx_buf[pos];
    }
    USART1DataFlag = true;

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	static uint32_t Count_RisingEdge;
	static uint32_t Count_FallingEdge;
	static uint32_t Count_Freq1;
	static uint32_t Count_Freq2;
	static bool Freq_State;

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
		Count_RisingEdge = TIM15->CCR1;

		if(Freq_State == 0){
			Freq_State = 1;
			Count_Freq1 = Count_RisingEdge;

		}
		else if(Freq_State == 1){
			Freq_State = 0;
			Count_Freq2 = Count_RisingEdge;
			if(Count_Freq2 > Count_Freq1) inputFrequency = 1/((float)Count_Freq2 - (float)Count_Freq1) * 1000000; //in kHz;
		}
	}

	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		Count_FallingEdge = TIM15->CCR2;

		if(Count_RisingEdge < Count_FallingEdge){
			inputDutyCycle = Count_FallingEdge - Count_RisingEdge;
			//input_pwm_min = inputDutyCycle < input_pwm_min ? inputDutyCycle : input_pwm_min;
			//input_pwm_max = inputDutyCycle > input_pwm_max ? inputDutyCycle : input_pwm_max;

			//strSize = sprintf((char*)buffer, "DutyCycle: %lu, Frequency: %f\r\n", inputDutyCycle, inputFrequency);
			//HAL_UART_Transmit(&huart1, buffer, strSize, 10);

			//inputRPM = map(inputDutyCycle, 1000, 2000, 0, 5000);

			Count_RisingEdge = 0;
			Count_FallingEdge = 0;
		}
	}

	if(inputFrequency >= FREQ_INPUT_PWM_MIN && inputFrequency <= FREQ_INPUT_PWM_MAX){
		//strSize = sprintf((char*)buffer, "Frequency %f\t PWM: %d\r\n", inputFrequency, setPWM);
		//HAL_UART_Transmit(&huart1, buffer, strSize, 10);
		if(inputDutyCycle >= 1050 && mode_motor == MODE_MOTOR_STOP){
			mode_motor = MODE_MOTOR_CALIBRATION;
			strSize = sprintf((char*)buffer, "motor calibration\r\n");
			HAL_UART_Transmit(&huart1, buffer, strSize, 10);
		} else if(inputDutyCycle < 1050 && mode_motor == MODE_MOTOR_STOP){
			mode_motor = MODE_MOTOR_READY;
			strSize = sprintf((char*)buffer, "motor ready\r\n");
			HAL_UART_Transmit(&huart1, buffer, strSize, 10);
		} else if(inputDutyCycle >= 1050 && mode_motor == MODE_MOTOR_READY){
			mode_motor = MODE_MOTOR_START;
			strSize = sprintf((char*)buffer, "motor start\r\n");
			HAL_UART_Transmit(&huart1, buffer, strSize, 10);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM6){
		static int count;
		count++;
		//measuredRPM = getRPM(tim2cnt);
		//strSize = sprintf((char*)buffer, "%f\r\n", measuredRPM);
		//HAL_UART_Transmit_IT(&huart1, buffer, strSize);
		if(count % 100 == 0 && mode_motor == MODE_MOTOR_RUN){
			strSize = sprintf((char*)buffer, "%lu\r\n", tim2cnt);
			HAL_UART_Transmit_IT(&huart1, buffer, strSize);
		}

	}
	else if(htim->Instance == TIM16){
		//Controller();
		static bool flash;
		static uint64_t led_counter;
		led_counter++;
		switch(mode_motor){
		case MODE_MOTOR_STOP:
			if(led_counter % 1000 == 0) HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			break;
		case MODE_MOTOR_CALIBRATION:
			if(led_counter % 100 == 0) HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			break;
		case MODE_MOTOR_READY:
			if(led_counter % 2000 == 0) flash = true;
			if(flash){
				static uint8_t flash_count;
				if(led_counter % 100 == 0){
					flash_count++;
					HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
					if(flash_count >= 10) {
						flash_count = 0;
						flash = false;
					}
				}
			} else if(HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin) != GPIO_PIN_SET && !flash)
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			break;
		default:
			if(HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin) != GPIO_PIN_SET) 	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			break;

		}
	}
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
	static uint8_t count;
	TIM1->CCR5 = setPWM + compWindowOffset;

	if(hcomp->Instance == COMP1 && waitForCommutation == 0){
		count++;
		if(count >= 0){
			count = 0;

			tim2cnt = __HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim2, 0);


			HAL_COMP_Stop_IT(&hcomp1);
			sConfig.Channel = ADC_CHANNEL_2;
			HAL_ADC_ConfigChannel(&hadc2, &sConfig);
			HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adcBuffer, 10);
			//HAL_ADC_Start_IT(&hadc2);

			waitForCommutation = 1;
		}


	}

	else if(hcomp->Instance == COMP2 && waitForCommutation == 0){
		count++;
		if(count >= 0){
			count = 0;

			tim2cnt = __HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim2, 0);

			HAL_COMP_Stop_IT(&hcomp2);
			sConfig.Channel = ADC_CHANNEL_3;
			HAL_ADC_ConfigChannel(&hadc2, &sConfig);
			HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adcBuffer, 10);
			//HAL_ADC_Start_IT(&hadc2);

			waitForCommutation = 1;
		}

	}

	else if(hcomp->Instance == COMP3 && waitForCommutation == 0){
		count++;
		if(count >= 0){
			count = 0;

			tim2cnt = __HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim2, 0);

			HAL_COMP_Stop_IT(&hcomp3);
			sConfig.Channel = ADC_CHANNEL_4;
			HAL_ADC_ConfigChannel(&hadc2, &sConfig);
			HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adcBuffer, 10);
			//HAL_ADC_Start_IT(&hadc2);

			waitForCommutation = 1;
		}

	}

	if(setPWM > 300 && !pwmState){
		HAL_GPIO_WritePin(GPIOB, OUT_A_Pin | OUT_B_Pin | OUT_C_Pin, GPIO_PIN_RESET);

		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_5);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1800 + oc5Value);
		compWindowOffset = -80;

		HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC5A, TIM_CHANNEL_5);
		TIM1->CCR5 = setPWM + compWindowOffset;
		pwmState = 1;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_5);
	}

	if(setPWM < 250 && pwmState){
		HAL_GPIO_WritePin(GPIOB, OUT_A_Pin | OUT_B_Pin | OUT_C_Pin, GPIO_PIN_SET);

		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_5);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
		compWindowOffset = 250;

		HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC5B, TIM_CHANNEL_5);
		TIM1->CCR5 = setPWM + compWindowOffset;
		pwmState = 0;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_5);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	static float csADCAverage;
	static uint32_t csADCTotal;
	static float adcAverage;
	static float adcTotal;
	/*if(hadc->Instance == ADC2){
		adcIntegral += (float)HAL_ADC_GetValue(&hadc2);

		if(adcIntegral >= adcOffset){
			HAL_ADC_Stop_IT(&hadc2);
			adcIntegral = 0;
			commutationPattern(NEXT);
		} else{
			HAL_ADC_Start_IT(&hadc2);
		}
	}*/
	if(hadc->Instance == ADC2){
		for(int i = 0; i < 10; i++){
			adcTotal += (float)adcBuffer[i];
		}
		adcAverage = adcTotal / 10;
		adcIntegral += adcAverage;

		adcTotal = 0;

		if(adcIntegral > adcOffset){
			HAL_ADC_Stop_DMA(&hadc2);
			adcIntegral = 0;

			commutationPattern(NEXT);
		}
	}

	if(hadc->Instance == ADC1){
		for(int i = 0; i < 12; i++){
			csADCTotal += csADC[i];
		}
		csADCAverage = (float)csADCTotal / 12;
		csADCTotal = 0;
		//csSize = sprintf((char*)csString, "%f\r\n", csADCAverage);
		//HAL_UART_Transmit_IT(&huart1, csString, csSize);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
