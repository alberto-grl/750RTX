/*
 * DCF77.c
 *
 *  Created on: 30 nov 2022
 *      Author: Alberto I4NZX
 *

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
 *
 *
 *
 */

#include "stdio.h"
#include "stdlib.h"
#include "main.h"
#include "Globals.h"



#ifdef DCF77_DECODER



//#pragma GCC push_options
//#pragma GCC optimize ("O0")

volatile static int16_t DecodedTestBuffer[16384], Test_i;
static uint8_t RisingEdge, FallingEdge, DCF77Message[60];
static uint32_t DCF77HighSampleCounter, DCF77LowSampleCounter, DCF77BitCounter;

void DecodeDCF77(void)
{
	uint8_t i;
	static uint8_t MinParity, HourParity;
	MinParity = HourParity = 0;
	for (i = 21 ; i < 28; i++)
		if (DCF77Message[i])
			MinParity ^= 1;
	for (i = 29 ; i < 35; i++)
		if (DCF77Message[i])
			HourParity ^= 1;

	if (MinParity != DCF77Message[28] && HourParity != DCF77Message[35])
		return;
	DCF77Min = 0;
	for (i = 0 ; i < 4; i++)
		DCF77Min += DCF77Message[21+i] << i;
	for (i = 0 ; i < 3; i++)
		DCF77Min += 10 * (DCF77Message[25+i] << i);

	DCF77Hour = 0;
	for (i = 0 ; i < 4; i++)
		DCF77Hour += (DCF77Message[29+i] << i);
	for (i = 0 ; i < 2; i++)
		DCF77Hour += 10 * (DCF77Message[33+i] << i);

	switch (WSPRBeaconState)
	{
	case NO_FIX:
		SystemMinutes = DCF77Min;
		SystemSeconds = 0;
		WSPRBeaconState = FIRST_FIX;
		break;
	case FIRST_FIX:
		if (SystemMinutes == DCF77Min)
		{
			SystemSeconds = 0;
			srand((unsigned) HAL_GetTick());
			TransmittingWSPR = 1;
			WSPRBeaconState = SEND_WSPR;
		}
		else
		{
			SystemMinutes = DCF77Min;
			SystemSeconds = 0;
			WSPRBeaconState = FIRST_FIX;
		}
		break;
	case SEND_WSPR:
		break;
	}
}

void DoDCF77(uint16_t DCF77In)
{
//This routine is called eg (128 MHz ADC); 128000000 / 16 / 64 / 4 = 32000 Hz
//100 mSec (DCF77 0) is 3200 samples
//200 mSec (DCF77 1) is 6400 samples
//2 Sec (DCF77 Sync) is 64000 samples


	if (DCF77In && !LastDCF77In)
		RisingEdge = 1;
	else
		RisingEdge = 0;

	if (!DCF77In && LastDCF77In)
		FallingEdge = 1;
	else
		FallingEdge = 0;

	if (FallingEdge)
	{
		if (DCF77HighSampleCounter > 40000 && DCF77HighSampleCounter < 90000)
		{
			if (DCF77BitCounter == 59)
				DecodeDCF77();
			DCF77BitCounter = 0;
		}
		DCF77LowSampleCounter = 0;
	}

	if (RisingEdge)
	{
		if (DCF77LowSampleCounter > 3000 && DCF77LowSampleCounter < 6000)
			DCF77Message[DCF77BitCounter++] = 0;
		else
			if (DCF77LowSampleCounter > 6000 && DCF77LowSampleCounter < 12000)
				DCF77Message[DCF77BitCounter++] = 1;
		DCF77HighSampleCounter = 0;
	}
	if (DCF77In && DCF77HighSampleCounter < 100000)
		DCF77HighSampleCounter++;

	if (!DCF77In && DCF77LowSampleCounter < 100000)
		DCF77LowSampleCounter++;

	if (DCF77BitCounter > 59)
		DCF77BitCounter = 59;
	LastDCF77In = DCF77In;
}

void DCF77StatusDisplay(void)
{
	static uint16_t DCF77DisplayCounter, DCF77DisplayPrescaler;

	if (DCF77DisplayPrescaler++ == 1)
	{
		DCF77DisplayPrescaler = 0;
		DCF77DisplayCounter++;
	}
	if (DCF77DisplayCounter % 2 == 0)
		LED_GREEN_OFF;
	else
	{
		if (DCF77DisplayCounter <= (WSPRBeaconState + 1) * 2 )
			LED_GREEN_ON;
		else
			LED_GREEN_OFF;
		if (DCF77DisplayCounter > (WSPRBeaconState + 3) * 2 )
			DCF77DisplayCounter = 0;
	}

}

//#pragma GCC pop_options
#endif
