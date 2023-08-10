/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spi.h"
#include "gpio.h"
#include "tim.h"
#include <stdio.h>
#include "led.h"
#define ARM_MATH_CM7
#include "arm_math.h"
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFFER_SIZE 16384 // Buffer size to store data for processing, higher buffer creates larger latency
#define FFT_BUFFER_SIZE 4096
#define GPIOC_CLK_EN	(1U<<2)
#define PIN13	(1U<<13)
#define BTN_PIN	PIN13


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

//SPI_HandleTypeDef hspi5;

//TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
const float INT16_TO_FLOAT = 1.0f/(32768.0f);
const float FLOAT_TO_INT16 = 32768.0f;

float real_fsample = 49350; // MEASURED WITH OSCILLOSCOPE
float fftBufIn[FFT_BUFFER_SIZE];
float fftBufOut[FFT_BUFFER_SIZE];
float freq_prev;
float fft_in_buf[FFT_BUFFER_SIZE];
float fft_out_buf[FFT_BUFFER_SIZE];

uint8_t fftFlag = 0; // When FFT is complete
uint8_t outarray[14];
uint8_t button = 0;
uint8_t dataReadyFlag = 0;

uint16_t leftOut = 0;
uint16_t rightOut = 0;
uint16_t inBufPtr; // Processing ptr
uint16_t freqs_prev[FFT_BUFFER_SIZE/2];

int16_t rxBuf[ADC_BUFFER_SIZE];  // DMA stream for data
int16_t txBuf[ADC_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2S2_Init(void);
//static void MX_SPI5_Init(void);
//static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
arm_rfft_fast_instance_f32 fft_handler;
void PeriphCommonClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI5_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PeriphCommonClock_Config(void) //Disappears when I2S1 disabled
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 100;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sClockSelection = RCC_I2SCLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}



void DoFFT() {
	//Do FFT
	arm_rfft_fast_f32(&fft_handler, fft_in_buf, fft_out_buf,0);

	uint16_t peakVal = 0;
	int peak = 0;
	int freqs[FFT_BUFFER_SIZE/2];
	int index = 0;
	int freqpoint = 0;
	float freq = 0;
	char counter_buff[10];



	if (GPIOC->IDR & BTN_PIN) // If button is pressed
	{
		button = 1 - button;
		ILI9341_Fill_Screen(BLACK);
		HAL_Delay(50);
	}

	for (int i = 0; i < FFT_BUFFER_SIZE; i += 2)
	{
		freqs[freqpoint] = (int)(20*log(sqrtf(((fft_out_buf[i])*(fft_out_buf[i])) + ((fft_out_buf[i+1])*fft_out_buf[i+1]))));
		freqpoint ++;
	}

	for (int j = 0; j < FFT_BUFFER_SIZE/2; j +=1)
	{
		if(freqs[j] > peak)
		{
			peak = freqs[j];
			peakVal = j;

		}

		freq = (peakVal)/((FFT_BUFFER_SIZE)/real_fsample);

	}


	if(button == 0)
	{
		for (int j = 2; j < 322; j +=1)
		{
			ILI9341_Draw_Pixel(320-(j-2),(freqs_prev[j]+100), BLACK);
		}


		for (int j = 2; j < 322; j +=1)
		{
			ILI9341_Draw_Pixel(320-(j-2),(freqs[j]+100), WHITE);

		}
	}

	else
	{
		for (int j = 0; j < 320; j +=1)
		{
			index = index + 5;
			ILI9341_Draw_Pixel(320-(j),(freqs_prev[(index+322)]+100), BLACK);
		}
		index = 0;

		for (int j = 0; j < 320; j +=1)
		{
			index = index + 5;
			ILI9341_Draw_Pixel(320-(j),(freqs[(index+322)]+100), WHITE);
		}
	}

	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
	sprintf(counter_buff, "PEAK: %0.1f", freq_prev);
	ILI9341_Draw_Text(counter_buff, 80, 200, WHITE, 2, WHITE);
	sprintf(counter_buff, "PEAK: %0.1f", freq);
	ILI9341_Draw_Text(counter_buff, 80, 200, BLACK, 2, WHITE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
	for (int i = 0; i < FFT_BUFFER_SIZE/2; i += 1)
	{
		freqs_prev[i] = freqs[i];
	}
	freq_prev = freq;
	printf("%d %0.1f %d \r\n",peakVal,freq,peak);

	dataReadyFlag = 0;
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	dataReadyFlag = 1;
	led_toggle(BLUE_LED);
}


void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	dataReadyFlag = 2;
	led_toggle(RED_LED);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  PeriphCommonClock_Config(); // Disappears when I2S1 disabled
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_I2S2_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();//initial driver setup to drive ili9341
  /*Initialise LEDs*/
  user_leds_init();

  /*Set PC13 as user button*/
  GPIOC->MODER &=~ (1U<<26);
  GPIOC->MODER &=~ (1U<<27);

  /*Initialise FFT*/
  arm_rfft_fast_init_f32(&fft_handler, FFT_BUFFER_SIZE);

  // Start I2S DMA streams
  HAL_StatusTypeDef status = HAL_I2S_Receive_DMA(&hi2s2, rxBuf, ADC_BUFFER_SIZE); // Start DMA stream
  if(status != HAL_OK)
  {
	  led_on(RED_LED);
  }
  else
  {
	  led_on(GREEN_LED);
  }

  ILI9341_Fill_Screen(BLACK);
  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //float peakVal
  //= 0.0f;
  //uint16_t peakHz = 0;

  while (1)
  {


	  int fft_in_ptr = 0;
	  if (dataReadyFlag == 1)
	  {
		  //processData();
		  //dataReadyFlag = 0;
		  for (int i = 0; i < (ADC_BUFFER_SIZE/2); i += 2)
		  {
//			  fft_in_buf[fft_in_ptr] = (float) ((uint32_t) (rxBuf[i]<<16)|rxBuf[i+1]);
//			  fft_in_buf[fft_in_ptr] += (float) ((uint32_t) (rxBuf[i+2]<<16)|rxBuf[i+3]);

			  //fft_in_buf[fft_in_ptr] = Moving_Average_Compute(rxBuf[i], &filterStruct);
			  fft_in_buf[fft_in_ptr] = INT16_TO_FLOAT*((float)rxBuf[i]);

			  txBuf[i] = rxBuf[i];
			  //txBuf[i+1] = rxBuf[i+1];
			  //txBuf[i+2] = rxBuf[i+2];
			  //txBuf[i+3] = rxBuf[i+3];
			  fft_in_ptr++;


		  }
		  DoFFT();
	  }

	  if (dataReadyFlag == 2)
	  {
		  //processData();
		  //dataReadyFlag = 0;
		  for (int i = (ADC_BUFFER_SIZE/2); i < ADC_BUFFER_SIZE; i += 2)
		  {
			  //fft_in_buf[fft_in_ptr] = (float) ((int) (rxBuf[i]<<16)|rxBuf[i+1]);
			  //fft_in_buf[fft_in_ptr] += (float) ((int) (rxBuf[i+2]<<16)|rxBuf[i+3]);
			  fft_in_buf[fft_in_ptr] = INT16_TO_FLOAT*((float)rxBuf[i]) ;
			  txBuf[i] = rxBuf[i];
			  //txBuf[i+1] = rxBuf[i+1];
			  //txBuf[i+2] = rxBuf[i+2];
			  //txBuf[i+3] = rxBuf[i+3];
			  fft_in_ptr++;
			  //printf("%d\r\n",txBuf[i]);


		  }
		  DoFFT();
	  }



//	  if (fftFlag)
//	  {
//		   peakVal = 0.0f;
//		   peakHz = 0.0f;
//
//		   uint16_t freqIndex = 0;
//
//		   for (uint16_t index = 0; index < FFT_BUFFER_SIZE; index += 2)
//		   {
//			   float curVal = sqrtf((fftBufOut[index] * fftBufOut[index])+(fftBufOut[index + 1] * fftBufOut[index + 1]));
//
//			   if (curVal > peakVal)
//			   {
//				   peakVal = curVal;
//				   peakHz = (uint16_t)(freqIndex * SAMPLE_RATE_HZ / ((float) FFT_BUFFER_SIZE));
//			   }
//
//			   freqIndex++;
//		   }
//
//		   fftFlag = 0;
//	  }


//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	  		ILI9341_Draw_Text("FPS TEST, 40 loop 2 screens", 10, 10, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		uint32_t Timer_Counter = 0;
//	  		for(uint32_t j = 0; j < 2; j++)
//	  		{
//	  			HAL_TIM_Base_Start(&htim1);
//	  			for(uint16_t i = 0; i < 10; i++)
//	  			{
//	  				ILI9341_Fill_Screen(WHITE);
//	  				ILI9341_Fill_Screen(BLACK);
//	  			}
//
//	  			//20.000 per second!
//	  			HAL_TIM_Base_Stop(&htim1);
//	  			Timer_Counter += __HAL_TIM_GET_COUNTER(&htim1);
//	  			__HAL_TIM_SET_COUNTER(&htim1, 0);
//	  		}
//	  		Timer_Counter /= 2;
//
//	  		char counter_buff[30];
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	  		sprintf(counter_buff, "Timer counter value: %d", Timer_Counter*2);
//	  		ILI9341_Draw_Text(counter_buff, 10, 10, BLACK, 1, WHITE);
//
//	  		double seconds_passed = 2*((float)Timer_Counter / 20000);
//	  		sprintf(counter_buff, "Time: %.3f Sec", seconds_passed);
//	  		ILI9341_Draw_Text(counter_buff, 10, 30, BLACK, 2, WHITE);
//
//	  		double timer_float = 20/(((float)Timer_Counter)/20000);	//Frames per sec
//
//	  		sprintf(counter_buff, "FPS:  %.2f", timer_float);
//	  		ILI9341_Draw_Text(counter_buff, 10, 50, BLACK, 2, WHITE);
//	  		double MB_PS = timer_float*240*320*2/1000000;
//	  		sprintf(counter_buff, "MB/S: %.2f", MB_PS);
//	  		ILI9341_Draw_Text(counter_buff, 10, 70, BLACK, 2, WHITE);
//	  		double SPI_utilized_percentage = (MB_PS/(6.25 ))*100;		//50mbits / 8 bits
//	  		sprintf(counter_buff, "SPI Utilized: %.2f", SPI_utilized_percentage);
//	  		ILI9341_Draw_Text(counter_buff, 10, 90, BLACK, 2, WHITE);
//	  		HAL_Delay(10000);





//	  		ILI9341_Fill_Screen(BLACK);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, WHITE, 1, BLACK);
//	  		ILI9341_Draw_Text("Filled Rectangles", 10, 20, RED, 1, BLACK);
//
//	  		HAL_Delay(500);
//	  		ILI9341_Fill_Screen(BLACK);
	  //HAL_Delay(100);



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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_SPI5_Init(void)
//{
//
//  /* USER CODE BEGIN SPI5_Init 0 */
//////////
//  /* USER CODE END SPI5_Init 0 */
//
//  /* USER CODE BEGIN SPI5_Init 1 */
//////////
//  /* USER CODE END SPI5_Init 1 */
//  /* SPI5 parameter configuration*/
//  hspi5.Instance = SPI5;
//  hspi5.Init.Mode = SPI_MODE_MASTER;
//  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi5.Init.NSS = SPI_NSS_SOFT;
//  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi5.Init.CRCPolynomial = 7;
//  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
//  hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
//  if (HAL_SPI_Init(&hspi5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI5_Init 2 */
//////////
//  /* USER CODE END SPI5_Init 2 */
//
//}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM1_Init(void)
//{
//
//  /* USER CODE BEGIN TIM1_Init 0 */
//
//  /* USER CODE END TIM1_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM1_Init 1 */
//
//  /* USER CODE END TIM1_Init 1 */
//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 10000;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = 65535;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
//  htim1.Init.RepetitionCounter = 0;
//  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM1_Init 2 */
//
//  /* USER CODE END TIM1_Init 2 */
//
//}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
///* USER CODE BEGIN MX_GPIO_Init_1 */
///* USER CODE END MX_GPIO_Init_1 */
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOF_CLK_ENABLE();
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOC, CS_Pin|DC_Pin|RST_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pins : CS_Pin DC_Pin RST_Pin */
//  GPIO_InitStruct.Pin = CS_Pin|DC_Pin|RST_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
///* USER CODE BEGIN MX_GPIO_Init_2 */
///* USER CODE END MX_GPIO_Init_2 */
//}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart3, (uint8_t *) ptr, len, HAL_MAX_DELAY);
	return len;
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
