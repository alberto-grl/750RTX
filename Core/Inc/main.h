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


typedef struct OutData_t
{
  uint32_t   OutCodes[100];
  uint32_t   OutLength;
} OutData_t;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* Board 1
#define LED_YELLOW_OFF    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET)
#define LED_YELLOW_ON   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET)
#define LED_RED_OFF  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define LED_RED_ON  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define LED_GREEN_OFF  	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define LED_GREEN_ON  	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
*/

#define TX_DELAY
#define SEMI_QSK

#define LED_YELLOW_OFF    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define LED_YELLOW_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define LED_RED_OFF  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define LED_RED_ON  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define LED_GREEN_OFF  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_GREEN_ON  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)


#define RELAY_TX_OFF  	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET)
#define RELAY_TX_ON  	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET)
#define SW01_IN          !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)
#define KEYER_DASH    	 !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)
#define KEYER_DOT    	 !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)
#define ENC_BUTTON		 !HAL_GPIO_ReadPin(ENC_BUTTON_GPIO_Port, ENC_BUTTON_Pin)
// keyerControl bit definitions
#define HIGH 1
#define LOW 0
#define DIT_L    0x01     // Dit latch
#define DAH_L    0x02     // Dah latch
#define DIT_PROC 0x04     // Dit is being processed
#define PDLSWAP  0x08     // 0 for normal, 1 for swap
#define IAMBICB  0x10     // 0 for Iambic A, 1 for Iambic B
#define IAMBICA  0x00     // 0 for Iambic A, 1 for Iambic B
#define SINGLE   2        // Keyer Mode 0 1 -> Iambic2  2 ->SINGLE



extern void loadWPM (int);
extern void update_PaddleLatch(void);
extern void DoKeyer(void);
extern void PrepareBits(uint8_t *, OutData_t *);

extern int cw_tx(char*);


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
extern void FminusClicked(uint16_t);
extern void FplusClicked(uint16_t);
extern void SetAGC(Agctype);
extern void SetBW(Bwidth);
extern void SetMode(Mode);
extern void HAL_GPIO_EXTI_Callback(uint16_t);
extern void EXTI1_IRQHandler(void);

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
extern void DecodeCW(void);
extern void keyIsDown(void);
extern void keyIsUp(void);
extern void printPunctuation(void);
extern void printSpace(void);
extern void sendToLCD(void);
extern void printCharacter(void);
extern void shiftBits(void);
extern void PrintUI(uint8_t*);
extern void SDR_demodAM_AGC(float32_t *, float32_t *);

extern void CarrierEnable(uint8_t);
extern void TXSwitch(uint8_t);

extern void SetTXPLL(float);
extern void SetFracPLL(uint32_t);

extern void SendWSPR(void);
extern void SetWSPRPLLCoeff(double, uint16_t *, uint16_t *);

extern
void SendCWMessage(uint8_t);

extern int get_wspr_channel_symbols(char *, uint8_t *);
extern void DoDCF77(uint16_t);
extern void DCF77StatusDisplay(void);
extern void My_arm_cfft_f32(
				const arm_cfft_instance_f32 *,
		        float32_t *,
		        uint8_t,
		        uint8_t);

extern void SetMask(float, float);
extern int make_kaiser(float *,unsigned int, float);
extern float i0(float);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN_SW01_Pin GPIO_PIN_13
#define IN_SW01_GPIO_Port GPIOC
#define SwInt1_Pin GPIO_PIN_14
#define SwInt1_GPIO_Port GPIOC
#define SwInt1_EXTI_IRQn EXTI15_10_IRQn
#define KEYER_DASH_Pin GPIO_PIN_6
#define KEYER_DASH_GPIO_Port GPIOA
#define KEYER_DOT_Pin GPIO_PIN_7
#define KEYER_DOT_GPIO_Port GPIOA
#define LedYellow_Pin GPIO_PIN_12
#define LedYellow_GPIO_Port GPIOB
#define LedGreen_Pin GPIO_PIN_13
#define LedGreen_GPIO_Port GPIOB
#define LedRed_Pin GPIO_PIN_14
#define LedRed_GPIO_Port GPIOB
#define RXTX_Pin GPIO_PIN_10
#define RXTX_GPIO_Port GPIOD
#define TX_ENA_Pin GPIO_PIN_11
#define TX_ENA_GPIO_Port GPIOD
#define ENC_BUTTON_Pin GPIO_PIN_14
#define ENC_BUTTON_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

#define USE_DCACHE

// Uncomment for external 8 MHz clock instead of internal xtal oscillator
#define USE_EXTERNAL_OSCILLATOR

//#define USE_EXTERNAL_MIXER

//Example: actual signal F: 7000000. Tuned at 7000050. XTAL_F_ERROR = -1 + (7000000 / 7000050) = -7.142806e-6

//#define XTAL_F_ERROR -10.142931e-6
//#define XTAL_F_ERROR -12.142931e-6
#define XTAL_F_ERROR -8.142931e-6


//#define SCAMP_OOK
//#define SCAMP_FSK



//CIC decimation, followed by a 4 times FIR decimator, total will be 256 or 64
#define CIC_DECIMATE_64
//#define CIC_DECIMATE_16

//TODO CLK_600M_CPU_150M_ADC_XTAL25 should be best but needs more optimization of code. 128 MHz is OK but 40 m band is too close to sampling F of 8 MHz.

// Select CPU and ADC Clock. Uncomment only one at a time
//#define CLK_600M_CPU_150M_ADC
//TODO 500 128 seems to be bad
//#define CLK_500M_CPU_120M_ADC /*Works but check the tim6 DAC setting*/
//#define CLK_500M_CPU_128M_ADC
//#define CLK_480M_CPU_120M_ADC /*old board*/
//#define CLK_600M_CPU_60M_ADC
//#define CLK_600M_CPU_160M_ADC /* new board */
//#define CLK_600M_CPU_160M_ADC_XTAL25 /* new board */
//#define CLK_600M_CPU_150M_ADC_XTAL25 /* ADC sample rate too high for CPU to consume data. Popping noise  */
//#define CLK_600M_CPU_120M_ADC_XTAL25 /* new board */
#define CLK_600M_CPU_128M_ADC_XTAL25 /* new board USE THIS*/
//#define CLK_600M_CPU_96M_ADC_XTAL25 /* new board */
//#define CLK_620M_CPU_160M_ADC_XTAL25 /* new board Motorboat noise */
//#define CLK_640M_CPU_160M_ADC_XTAL25 /*CPU Hangs - DO NOT USE. new board */

//#define TEST_NO_SDR
#define TEST_FRAC_DIV

//#define UART_UI
#define USB_UI

//FFT masks are precalculated with Remez exchange in Octave and Matlab, or on demand with window fast convolution
//Shape of the filter from the former method is somewhat better but uses more flash storage, about 23 KB for three filters.
//It is possible to mantain Remez and save memory by dropping AM filter (disable RECEIVE_AM)
//FFT masks are calculated with the scripts in the "FFT Mask generators" folder
//
//#define PRECALC_MASKS
//save memory by commenting out
#define RECEIVE_AM
//#define TEST_SINGLE_ADC
//#define AG_TEST_AUDIO

//#define FAKE_SINE_RF_SIGNAL
//#define FAKE_SQUARE_RF_SIGNAL
//#define FAKE_NO_RF_SIGNAL

//#define DEBUG_TX_CW
//#define TEST_WF

// Select only one of the following
//#define ASCII_BANDSCOPE
//#define COLOR_BANDSCOPE

#define CW_TX_SIDETONE
#define SIDETONE_VOLUME (0.2f)

//#define CW_DECODER
//#define DCF77_DECODER

//for debugging with STMStudio
//#define SNAPSHOT_ACQUISITION_DBG

//At power on listens to DCF77, waits for two fixes with reasonable content, starts WSPR beacon
//#define WSPR_BEACON_MODE
#define WSPR_FREQ 7040135.f

//this is the frequency for RX in USB
#define WSPR_RX_FREQ 7038600.f
#define FT8_FREQ 7078000.f
#define FT8_USB_MODE
#define DCF77_FREQ 77500.f

#ifdef WSPR_BEACON_MODE
#define DCF77_DECODER
#endif

//TODO make KEYER disappear. Leave it always defined for now
#define KEYER
#define USE_KEYER
//#define USE_SCAMP
#define SIGNAL_AVERAGE_T_CONST 0.2

#define CW_LEVEL_AVERAGE_T_CONST 0.001
#define MEDIUM_LEVEL_AVERAGE_T_CONST 0.00001
#define BASE_NOISE_AVERAGE_T_CONST 0.001


//approx bias vs. VRMS 50 Ohm out vs power
		//4095 17.1  5.8
		//2048 13.1  3.4
		//1024 7.5	 1.1
		// 256 3.8   0.3

#define LOW_POWER_OUT (1024)
#define MID_POWER_OUT (2048)
#define MAX_POWER_OUT (4095)



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
#define CWPITCH      650.f /* was 650.f*/

#define OVF_TIMEOUT 2

#define DIGITAL_MODES




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
