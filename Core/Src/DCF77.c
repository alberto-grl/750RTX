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

volatile static int16_t DecodedTestBuffer[256], Test_i;
static uint8_t LastCWIn, RisingEdge, FallingEdge, DCF77Message[60];
volatile static uint16_t DCF77HighSampleCounter, DCF77LowSampleCounter, DCF77BitCounter;
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
}

void DoDCF77(uint8_t CWIn)
{

	DecodedTestBuffer[Test_i++] = CWIn;
	if (Test_i == 256)
		Test_i = 0;

	if (CWIn && !LastCWIn)
		RisingEdge = 1;
	else
		RisingEdge = 0;

	if (!CWIn && LastCWIn)
		FallingEdge = 1;
	else
		FallingEdge = 0;

	if (FallingEdge)
	{
		if (DCF77HighSampleCounter > 90 && DCF77HighSampleCounter < 120)
		{
			if (DCF77BitCounter == 59)
				DecodeDCF77();
			DCF77BitCounter = 0;
		}
		DCF77LowSampleCounter = 0;
	}

	if (RisingEdge)
	{
		if (DCF77LowSampleCounter > 2 && DCF77LowSampleCounter < 8)
			DCF77Message[DCF77BitCounter++] = 0;
		else
			if (DCF77LowSampleCounter > 8 && DCF77LowSampleCounter < 15)
				DCF77Message[DCF77BitCounter++] = 1;
		DCF77HighSampleCounter = 0;
	}
	if (CWIn && DCF77HighSampleCounter < 100)
		DCF77HighSampleCounter++;

	if (!CWIn && DCF77LowSampleCounter < 100)
		DCF77LowSampleCounter++;

	if (DCF77BitCounter > 59)
		DCF77BitCounter = 59;
	LastCWIn = CWIn;
}
#pragma GCC pop_options
#endif
