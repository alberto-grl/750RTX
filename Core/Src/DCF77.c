/*
 * DCF77.c
 *
 *  Created on: 30 nov 2022
 *      Author: alberto
 */

#include "stdio.h"
#include "main.h"
#include "Globals.h"



#ifdef DCF77_DECODER



#pragma GCC push_options
#pragma GCC optimize ("O0")

volatile static int16_t DecodedTestBuffer[16384], Test_i;
static uint8_t LastCWIn, RisingEdge, FallingEdge, DCF77Message[60];
volatile static uint32_t DCF77HighSampleCounter, DCF77LowSampleCounter, DCF77BitCounter;
static uint8_t MinParity, HourParity;

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
		SystemMinutes = DCF77Min;
		SystemSeconds = 0;
}

void DoDCF77(uint16_t DCF77In)
{

	DecodedTestBuffer[Test_i++] = DCF77In;
	if (Test_i == 16384)
		Test_i = 0;

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
#pragma GCC pop_options
#endif
