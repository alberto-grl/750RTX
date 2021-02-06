/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include <arm_math.h>
#include "SDR_math.h"
//#include "Globals.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

//typedef uint8_t    u8;
typedef enum {AM=0, LSB=1, USB=2, CW=3} Mode;
typedef enum {Fast, Slow}   Agctype;
typedef enum {Narrow, Wide} Bwidth;
typedef struct
        {
          char   name[16];
          float  freq;
          Mode   mode;
          Bwidth bw;
        } Presets;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define LED_YELLOW_OFF    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET)
#define LED_YELLOW_ON   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET)
#define LED_RED_OFF  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define LED_RED_ON  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

//void SDR_ADC_StructInit(SDR_ADC_InitTypeDef* ADC_InitStruct);

//extern void Set_NVIC_PriorityGroupConfig(uint32_t PriorityGroup);
//extern void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
//extern void SDR_ADC_Init(ADC_TypeDef* ADCx, SDR_ADC_InitTypeDef* ADC_InitStruct);

//extern void SDR_InitGPIO(void);
//extern void SDR_InitEXTI(void);
//extern void SDR_InitDAC(void);
//extern void SDR_InitADC(void);
//extern void SDR_StartADC(void);
//extern void SDR_InitEXTI(void);
extern void Tune_Preset(uint8_t);
extern void Load_Presets(void);
extern void DisplayFrequency(void);
extern void SetFstep(int);
extern void FminusClicked(void);
extern void FplusClicked(void);
extern void SetAGC(Agctype);
extern void SetBW(Bwidth);
extern void SetMode(Mode);

extern void DisplayFrequency(void);
extern void Touch(void);
extern void Display(void);
extern void Display_Init(void);
extern void ADC_Stream0_Handler(uint8_t);
extern void LED_switch(void);

extern void DisplayStatus(void);
extern void UserInput(void);
void SystemClock_Config_For_OC(void);
void MX_TIM6_Init_Custom_Rate(void);

extern void SetFOut(uint32_t);



/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SwInt1_Pin GPIO_PIN_14
#define SwInt1_GPIO_Port GPIOC
#define SwInt1_EXTI_IRQn EXTI15_10_IRQn
#define LED_GREEN_Pin GPIO_PIN_8
#define LED_GREEN_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

#define USE_DCACHE

// Uncomment for external 8 MHz clock instead of internal xtal oscillator
//#define USE_EXTERNAL_OSCILLATOR

//#define USE_EXTERNAL_MIXER

// Select CPU and ADC Clock. Uncomment only one at a time
//#define CLK_600M_CPU_150M_ADC
//TODO 500 128 seems to be bad
//#define CLK_500M_CPU_120M_ADC
//#define CLK_500M_CPU_128M_ADC
//#define CLK_480M_CPU_120M_ADC /*old board*/
//#define CLK_600M_CPU_60M_ADC
#define CLK_600M_CPU_160M_ADC /* new board */

//#define TEST_NO_SDR
//#define TEST_FRAC_DIV

//#define UART_UI
#define USB_UI


#define BSIZE        (512)
#define FFTLEN       (BSIZE*2)
#define NUMFIRCOEFS  64
#define DIRECTFFT    0
#define INVERSEFFT   1
#define NOREVERSE    1
#define MAXPRESETS   14
#define true  1
#define false 0
#define __FPU_PRESENT 1
#define myPI         3.14159265358979f
#define TWOPI        (2.f*myPI)
#define CWPITCH      650.f




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
