/*
 * FT8.c
 *
 *  Created on: Aug 29, 2023
 *      Author: alberto
 */

#include "stdio.h"
#include "main.h"
#include "Globals.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>



void SendFT8(void)
{
	volatile uint32_t FracDiv;
	uint8_t result;



//	LOfreq = (double)FT8_FREQ;
	LastTXFreq = LOfreq;
	/*
	DMA interrupt must be active during transmission
	because PLL dithering is performed in its isr.
	It should be called by a timer, but for now leave it this way.
	 */
#if 0
	if (HAL_ADCEx_MultiModeStart_DMA(HAdc1,
			(uint32_t *)aADCDualConvertedValues,
			BSIZE   //Source code says transfer size is in bytes, but it is in number of transfers
			//We transfer BSIZE * 4 bytes, it equals 2 * BSIZE samples (1024) because we have full and half interrupts
	) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
#endif

#if 1
	HAL_ADCEx_MultiModeStop_DMA(HAdc1);
	do
		{
		result = HAL_ADCEx_MultiModeStart_DMA(HAdc1,
			(uint32_t *)aADCDualConvertedValues,
			BSIZE);   //Source code says transfer size is in bytes, but it is in number of transfers
			//We transfer BSIZE * 4 bytes, it equals 2 * BSIZE samples (1024) because we have full and half interrupts
		}
	while (result != HAL_OK);

#endif

	TXSwitch(1);
	CarrierEnable(1);
	TransmittingFT8 = 1;

	/* send FT8 */
	while (FSKAudioPresent)
	{
		if(KEYER_DASH || KEYER_DOT)
		{
			break;  // stop when button/key pressed;
		}
		FracDiv = (uint32_t) ((FT8_FREQ + USBFreqFiltered - FT8_OutF) / (FT8_OutFHigherStep - FT8_OutF)
					* 8192 * 8); //FracDiv PWM has 8 levels

		FT8FracPWMCoeff = FracDiv & 0x07;
		FracDiv >>= 0x03;
		FT8FracDivCoeff = FracDiv;
	}

	TransmittingWSPR = 0;
	TXSwitch(0);
	CarrierEnable(0);
}
