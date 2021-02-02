/* USER CODE BEGIN Header */
/**
 *******************************************************************************

                   main.c module of the program ARM_Radio

                                                  Copyright 2015 by Alberto I2PHD, June 2015

    This file is part of ARM_Radio.

    ARM_Radio is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ARM_Radio is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ARM_Radio.  It is contained in the file Copying.txt in the
    same ZIP file where this file was extracted from.

 *******************************************************************************
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
  FAQ about cache https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices
  FPU performance https://community.arm.com/developer/ip-products/processors/f/cortex-m-forum/5567/what-is-the-advantage-of-floating-point-of-cm7-versus-cm4
  CIC filter decomposition: https://www.embedded.com/reducing-power-consumption-in-cic-filter-algorithm-designs/
  NCO https://www.embedded.com/configuring-a-well-behaved-digital-quadrature-oscillator/
  DMA, Cache https://hackaday.io/project/117555-network-analyzer-on-an-stm32h7/details
  FFT Filters https://www.danvillesignal.com/images/pdfs/compdsp_Borgerding_FFT_slides2.pdf http://dsp-book.narod.ru/DSPMW/08.PDF
  Measuring inductance https://daycounter.com/Articles/How-To-Measure-Inductance.phtml
  RF Filters https://rf-tools.com/lc-filter/


  MCO2 output is pin PC9, CN8 pin 4.
 */

#define IN_MAIN

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "Globals.h"
#include "FIRcoefs.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/**
 * @brief  Computation of ADC master conversion result
 *         from ADC dual mode conversion result (ADC master and ADC slave
 *         results concatenated on data register of ADC master).
 * @param  DATA: ADC dual mode conversion result
 * @retval None
 */
#define COMPUTATION_DUALMODEINTERLEAVED_ADCMASTER_RESULT(DATA)                 \
		((DATA) & 0x0000FFFF)

/**
 * @brief  Computation of ADC slave conversion result
 *         from ADC dual mode conversion result (ADC master and ADC slave
 *         results concatenated on data register of ADC master).
 * @param  DATA: ADC dual mode conversion result
 * @retval None
 */
#define COMPUTATION_DUALMODEINTERLEAVED_ADCSLAVE_RESULT(DATA)                  \
		((DATA) >> 16)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

LPTIM_HandleTypeDef hlptim2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* Variable containing ADC conversions results */
ALIGN_32BYTES(__IO uint32_t   aADCDualConvertedValues[BSIZE]);    /* ADC dual mode interleaved conversion results (ADC master and ADC slave results concatenated on data register 32 bits of ADC master). */
ALIGN_32BYTES(__IO uint16_t   aADCxConvertedValues[BSIZE]);       /* For the purpose of this example, dispatch dual conversion values into arrays corresponding to each ADC conversion values. */
ALIGN_32BYTES(__IO uint16_t   aADCyConvertedValues[BSIZE]);       /* For the purpose of this example, dispatch dual conversion values into arrays corresponding to each ADC conversion values. */
ALIGN_32BYTES(__IO short AudioOut[BSIZE*2]);			  // A single array because we are using the half/full complete irq to switch between the two halves

uint8_t         ubADCDualConversionComplete = RESET;                        /* Set into ADC conversion complete callback */

const uint16_t sine_wave_array[32] = {2047, 1648, 1264, 910, 600,  345,
		156, 39,  0,  39,  156,  345,
		600, 910, 1264, 1648, 2048, 2447,
		2831, 3185, 3495, 3750, 3939, 4056,
		4095, 4056, 3939, 3750, 3495, 3185,
		2831, 2447};

char UartTXString[256];
char UartRXString[256];
__IO ITStatus UartRXDataReady = RESET;
__IO ITStatus UartTXFinished = RESET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC1_Init(void);
static void MX_LPTIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

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
	//We use our clock init function and we prevent the use of the STM32Cube tool generated one. Not possible to overclock with it.
#ifdef EXECUTE_CUBE_CLOCK_INIT
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
#endif

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();

	/* Enable D-Cache---------------------------------------------------------*/
#ifdef USE_DCACHE
	SCB_EnableDCache();
#endif

	// Don't forget to set a different SamplingRate in main.c
	SystemClock_Config_For_OC();


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_LPTIM2_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	// Initialise again so we can change divider with a #define in main.h
	MX_TIM6_Init_Custom_Rate();

	/* Run the ADC calibration in single-ended mode */
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
	{
		/* Calibration Error */
		Error_Handler();
	}

	if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
	{
		/* Calibration Error */
		Error_Handler();
	}

	volume= 0.1;


	// Set now the default values for some variables
	SetFstep(2);
	cwpitch = CWPITCH;
	os_time = 0;
	meanavg = 0.f;
	Qfactor = 0.987f;         // Q factor for the CW peak filter
	Muted   = false;
	AMindex  = LSBindex = 1;
	USBindex = CWindex  = 1;
	bw[AM]   = bw[LSB]  = Wide;
	bw[USB]  = bw[CW]   = Wide;
	agc[AM]  = agc[LSB] = Slow;
	agc[USB] = Slow;
	agc[CW]  = Fast;
	AGC_decay[Fast] = 0.9995f;
	AGC_decay[Slow] = 0.99995f;
	Hangcount[Fast] = 2;
	Hangcount[Slow] = 30;
	AgcThreshold    = 1.92e-4f;

#ifdef CLK_600M_CPU_150M_ADC
	SamplingRate = ((150000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_500M_CPU_120M_ADC
	SamplingRate = ((120000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_500M_CPU_128M_ADC
	SamplingRate = ((128000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_480M_CPU_120M_ADC
	SamplingRate = ((120000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_600M_CPU_60M_ADC
	SamplingRate = ((60000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_600M_CPU_160M_ADC
	SamplingRate = ((160000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif

	SetFOutVHF(10000000);

	SamplingRate = SamplingRate * 4000000.f / 3999300.f; // Correct Xtal error

	AudioRate = SamplingRate / 4 /16.f / 4.f; //First decimation was 16, now is 64
	SDR_compute_IIR_parms();  // compute the IIR parms for the CW peak filter

	// init the decimating FIR control blocks
	arc = arm_fir_decimate_init_f32(&SfirR, NUMFIRCOEFS, 4, FIRcoefs, FIRstate1R, BSIZE*4);
	while(arc != ARM_MATH_SUCCESS)
	{};   // spin loop if error
	arc = arm_fir_decimate_init_f32(&SfirI, NUMFIRCOEFS, 4, FIRcoefs, FIRstate1I, BSIZE*4);
	while(arc != ARM_MATH_SUCCESS)
	{};   // spin loop if error

	Load_Presets();
	Tune_Preset(1);      // Set the initial tuning to Preset 1

	DisplayStatus();    // Display status, it would not be shown until a user input was given
	if (HAL_ADCEx_MultiModeStart_DMA(&hadc1,
			(uint32_t *)aADCDualConvertedValues,
			BSIZE   //Source code says transfer size is in bytes, but it is in number of transfers
	) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

	///////////////
	HAL_TIM_Base_Start(&htim6);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)AudioOut, BSIZE * 2, DAC_ALIGN_12B_R);
	///////

	/*		 __HAL_RCC_PLL2_DISABLE();
		__HAL_RCC_PLL2_CONFIG(4, 144, 24, 2, 2);
		 __HAL_RCC_PLL2_ENABLE();
		 __HAL_RCC_PLL2FRACN_CONFIG(1013); // 0-8191, can be issued at any time
	 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//	 SetFOut(20700000);

	/* Enable PLL2FRACN . */
	__HAL_RCC_PLL2FRACN_ENABLE();


	/*

while (1)
{
 __HAL_RCC_PLL2FRACN_DISABLE();
	 __HAL_RCC_PLL2FRACN_CONFIG(8191); // 0-8191, can be issued at any time
	 __HAL_RCC_PLL2FRACN_ENABLE();
	  HAL_Delay(20);
	 __HAL_RCC_PLL2FRACN_DISABLE();
	 __HAL_RCC_PLL2FRACN_CONFIG(0); // 0-8191, can be issued at any time
	 __HAL_RCC_PLL2FRACN_ENABLE();
	  HAL_Delay(20);
}

	 */

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* Turn-on/off LED1 in function of ADC conversion result */
		/*  - Turn-off if ADC conversions buffer is not complete */
		/*  - Turn-on if ADC conversions buffer is complete */

		/* ADC conversion buffer complete variable is updated into ADC conversions*/
		/* complete callback.*/

		UserInput();
		//	  HAL_Delay(10);
		if (ubADCDualConversionComplete == RESET)
		{
			//	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		}
		else
		{
			//	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		}


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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 480;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_LPTIM2
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 144;
  PeriphClkInitStruct.PLL2.PLL2P = 19;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3M = 4;
  PeriphClkInitStruct.PLL3.PLL3N = 240;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_1;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Lptim2ClockSelection = RCC_LPTIM2CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLL2PCLK, RCC_MCODIV_1);
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_INTERL;
  multimode.DualModeData = ADC_DUALMODEDATAFORMAT_32_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_2CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPTIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM2_Init(void)
{

  /* USER CODE BEGIN LPTIM2_Init 0 */

  /* USER CODE END LPTIM2_Init 0 */

  /* USER CODE BEGIN LPTIM2_Init 1 */

  /* USER CODE END LPTIM2_Init 1 */
  hlptim2.Instance = LPTIM2;
  hlptim2.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim2.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim2.Init.UltraLowPowerClock.Polarity = LPTIM_CLOCKPOLARITY_RISING;
  hlptim2.Init.UltraLowPowerClock.SampleTime = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;
  hlptim2.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim2.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim2.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim2.Init.CounterSource = LPTIM_COUNTERSOURCE_EXTERNAL;
  hlptim2.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim2.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM2_Init 2 */

  /* USER CODE END LPTIM2_Init 2 */

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
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 8191;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

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
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
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
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SwInt1_Pin */
  GPIO_InitStruct.Pin = SwInt1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SwInt1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */


/**
 * @brief  Conversion complete callback in non blocking mode
 * @param  AdcHandle : ADC handle

 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
	/* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes */
#ifdef USE_DCACHE
	SCB_InvalidateDCache_by_Addr((uint32_t *) &aADCDualConvertedValues[BSIZE/2], 4*BSIZE/2);
#endif
	ADC_Stream0_Handler(1);
	/* Set variable to report DMA transfer status to main program */
	ubADCDualConversionComplete = SET;
}

/**
 * @brief  Conversion DMA half-transfer callback in non blocking mode
 * @param  hadc: ADC handle
 * */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	/* Invalidate Data Cache to get the updated content of the SRAM on the first half of the ADC converted data buffer: 32 bytes */
#ifdef USE_DCACHE
	SCB_InvalidateDCache_by_Addr((uint32_t *) &aADCDualConvertedValues[0], 4*BSIZE/2);
#endif
	ADC_Stream0_Handler(0);
	/* Reset variable to report DMA transfer status to main program */
	ubADCDualConversionComplete = RESET;
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	ValidAudioHalf = &AudioOut[BSIZE];
	LED_RED_ON;

	//	 __HAL_RCC_PLL2_DISABLE();
	//	__HAL_RCC_PLL2_CONFIG(4, 240, 16, 2, 2);
	//	 __HAL_RCC_PLL2_ENABLE();

}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	ValidAudioHalf = &AudioOut[0];
	LED_RED_OFF;
	//	 __HAL_RCC_PLL2_DISABLE();
	//	 __HAL_RCC_PLL2_CONFIG(4, 120, 16, 2, 2);
	//	 __HAL_RCC_PLL2_ENABLE();
}

void SystemClock_Config_For_OC(void)

{
	/* MCO2 is not correctly initialized by CubeMX if LPTIM or another peripheral is not enabled with the
	 * clock from the same PLL as MCO2.
	 */


	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
	/** Macro to configure the PLL clock source
	 */
	__HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
	/** Initializes the CPU, AHB and APB busses clocks
	 */



	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 480;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;


#ifdef USE_EXTERNAL_OSCILLATOR
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS; // External clock on pin 29 CN 11 (PF0/PH0)
#else
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
#endif
	RCC_OscInitStruct.HSEState = RCC_HSE_ON; //Xtal oscillator
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 2;
#ifdef CLK_600M_CPU_150M_ADC
	RCC_OscInitStruct.PLL.PLLN = 300;
#endif
#ifdef CLK_500M_CPU_120M_ADC
	RCC_OscInitStruct.PLL.PLLN = 250;
#endif
#ifdef CLK_500M_CPU_128M_ADC
	RCC_OscInitStruct.PLL.PLLN = 250;
#endif
#ifdef CLK_480M_CPU_120M_ADC
	RCC_OscInitStruct.PLL.PLLN = 240;
#endif
#ifdef CLK_600M_CPU_60M_ADC
	RCC_OscInitStruct.PLL.PLLN = 300;
#endif
#ifdef CLK_600M_CPU_160M_ADC
	RCC_OscInitStruct.PLL.PLLN = 300;
#endif
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_LPTIM2
			|RCC_PERIPHCLK_ADC;
	PeriphClkInitStruct.PLL3.PLL3M = 4;
#ifdef CLK_600M_CPU_150M_ADC
	PeriphClkInitStruct.PLL3.PLL3N = 300;
#endif
#ifdef CLK_500M_CPU_120M_ADC
	PeriphClkInitStruct.PLL3.PLL3N = 240;
#endif
#ifdef CLK_500M_CPU_128M_ADC
	PeriphClkInitStruct.PLL3.PLL3N = 256;
#endif
#ifdef CLK_480M_CPU_120M_ADC
	PeriphClkInitStruct.PLL3.PLL3N = 120;
#endif
#ifdef CLK_600M_CPU_60M_ADC
	PeriphClkInitStruct.PLL3.PLL3N = 120;
#endif
#ifdef CLK_600M_CPU_160M_ADC
	PeriphClkInitStruct.PLL3.PLL3N = 320;
#endif
	PeriphClkInitStruct.PLL2.PLL2M = 4;
	PeriphClkInitStruct.PLL2.PLL2N = 240;
	PeriphClkInitStruct.PLL2.PLL2P = 16;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
	PeriphClkInitStruct.PLL2.PLL2FRACN = 6000;
	PeriphClkInitStruct.PLL3.PLL3M = 4;
	//	PeriphClkInitStruct.PLL3.PLL3N = 240;
	PeriphClkInitStruct.PLL3.PLL3P = 2;
	PeriphClkInitStruct.PLL3.PLL3Q = 2;
	PeriphClkInitStruct.PLL3.PLL3R = 4;
	PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_1;
	PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
	PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
	PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;

	PeriphClkInitStruct.Lptim2ClockSelection = RCC_LPTIM2CLKSOURCE_PLL2;
	PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLL2PCLK, RCC_MCODIV_1);

	//	  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_HSE, RCC_MCODIV_3);
	//	  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_10);
}

void UserInput(void)
{
	volatile HAL_StatusTypeDef result;


	__HAL_UART_SEND_REQ (&huart3, UART_RXDATA_FLUSH_REQUEST);
	__HAL_UART_CLEAR_OREFLAG (&huart3);
	//result = (HAL_UART_Receive(&huart3, (uint8_t *) UartRXString, 1, 1 )) ;
	result = (HAL_UART_Receive_IT(&huart3, (uint8_t *) UartRXString, 1 )) ;

	if (result == HAL_OK)
	{
		UartRXDataReady = RESET;
		switch (UartRXString[0])
		{
		case 49: //1
			FminusClicked(); break;
		case 50: //2
			FplusClicked(); break;
		case 51: //3
			SetFstep(5);  break;
		case 52: //4
			SetFstep(4);  break;
		case 53: //5
			SetFstep(3);  break;
		case 54: //6
			SetFstep(2);  break;
		case 55: //7
			SetFstep(1); break;
		case 56: //8
			SetFstep(0); break;
		case 108: //L
			SetMode((Mode)LSB); break;
		case 117: //U
			SetMode((Mode)USB); break;
		case 97: //A
			SetMode((Mode)AM); break;
		case 99: //C
			SetMode((Mode)CW); break;
		case 102: //F
			SetAGC((Agctype)Fast);  break;
		case 115: //S
			SetAGC((Agctype)Slow);  break;
		case 110: //N
			SetBW((Bwidth)Narrow);  break;
		case 119: //W
			SetBW((Bwidth)Wide);  break;
		case 45: //-
			volume -= 0.1;
			if (volume < 0)
				volume = 0;
			break;
		case 43: //+
			volume += 0.1;
			if (volume > 1.0)
				volume = 1.0;
			break;
		}

		DisplayStatus();
	}

	SValue = 10 / 3.01 * log10(PeakAudioValue * 2000.0);
	sprintf((char*)UartTXString, "S %-4.1f\r", SValue);
	HAL_UART_Transmit(&huart3, (uint8_t *) UartTXString, strlen(UartTXString), 100);
}

void DisplayStatus(void)
{
	static char StringMode[8];
	static char StringWidth[8];
	static char StringAGC[8];
	static char StringStep[8];


	switch(Fstep)
	{
	case 1: strcpy(StringStep,"   1"); break;
	case 10: strcpy(StringStep,"  10"); break;
	case 100: strcpy(StringStep," 100"); break;
	case 1000: strcpy(StringStep,"  1K"); break;
	case 10000: strcpy(StringStep," 10K"); break;
	case 100000: strcpy(StringStep,"100K"); break;
	}

	switch(CurrentMode)
	{
	case LSB: strcpy(StringMode,"LSB"); break;
	case USB: strcpy(StringMode,"USB"); break;
	case AM: strcpy(StringMode,"AM"); break;
	case CW: strcpy(StringMode,"CW"); break;
	}
	switch (CurrentAGC)
	{
	case Fast: strcpy(StringAGC,"Fast"); break;
	case Slow: strcpy(StringAGC,"Slow"); break;
	}
	switch (CurrentBW)
	{
	case Narrow: strcpy(StringWidth,"Narrow"); break;
	case Wide: strcpy(StringWidth,"Wide"); break;
	}
	sprintf(UartTXString, "          Freq %.0f Step %s Mode %s BW %s AGG %s Volume %1.1f   \r", LOfreq, StringStep, StringMode, StringWidth, StringAGC, volume);
	HAL_UART_Transmit(&huart3, (uint8_t *) UartTXString, strlen(UartTXString), 100);
}


/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
void MX_TIM6_Init_Custom_Rate(void)
{

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 0;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;


#ifdef CLK_600M_CPU_150M_ADC
	htim6.Init.Period = 8191;
#endif
#ifdef CLK_500M_CPU_120M_ADC
	htim6.Init.Period = 8541;
#endif
#ifdef CLK_500M_CPU_128M_ADC
	htim6.Init.Period = 7999;
#endif
#ifdef CLK_480M_CPU_120M_ADC
	htim6.Init.Period = 8191;
#endif
#ifdef CLK_600M_CPU_60M_ADC
	htim6.Init.Period = 20479;
#endif
#ifdef CLK_600M_CPU_160M_ADC
	htim6.Init.Period = 7679;
#endif
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}



/**
 * @brief  Rx Transfer completed callback
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report end of DMA Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{

	/* Turn LED2 on: Transfer in reception process is correct */
	LED_RED_OFF;
}

void SetFOut(uint32_t FHz)
{
	volatile uint32_t DivN2, FracN2;


	/* FOut = FXtal * M / N / P / MCODIV
FXtal = 8 MHz

Local oscillator is 10.7 - 40.7 MHz

MCODIV = 1
M = 4
P = 24
MCODIV = 1
	 */
	DivN2 = (((uint64_t)FHz * 24 * 4 * 0x2000) / (uint64_t)8000000) >> 13;
	FracN2 = (((uint64_t)FHz * 24 * 4* 0x2000) / (uint64_t)8000000) & 0x1FFF;

	__HAL_RCC_PLL2_DISABLE();
	__HAL_RCC_PLL2_CONFIG(4, DivN2, 24, 2, 2);
	__HAL_RCC_PLL2_ENABLE();
}

void SetFracPLL(uint32_t Coeff)
{
	/* Do not use as signal generator. PLL output voltage is not filtered and the sigma delta
	 * modulation results infrequency hopping between main integer dividers
	 */
	__HAL_RCC_PLL2FRACN_DISABLE();
	__HAL_RCC_PLL2FRACN_CONFIG(Coeff); // 0-8191, can be issued at any time
	__HAL_RCC_PLL2FRACN_ENABLE();
}

void SetFOutVHF(uint32_t FHz)
{
	volatile uint32_t DivN2, FracN2;


	/* FOut = FXtal * M / N / P / MCODIV
FXtal = 8 MHz

Local oscillator is 10.7 - 40.7 MHz

MCODIV = 1
M = 4
P = 24
MCODIV = 1
	 */
	DivN2 = 291;
	FracN2 = 0;

	__HAL_RCC_PLL2_DISABLE();
	__HAL_RCC_PLL2_CONFIG(8, DivN2, 2, 2, 2);
	__HAL_RCC_PLL2_ENABLE();


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

	while(1)
	{
		if((os_time % 50) == 0)  // blink fast the two leds in case of errors
			LED_switch();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
