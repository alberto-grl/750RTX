/*******************************************************************************

                   SDR_func.c module of the program ARM_Radio

						                          Copyright 2015 by Alberto I2PHD, June 2015
					Heavy remix by Alberto I4NZX
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

 *******************************************************************************/

#include "main.h"
#include <arm_const_structs.h>
#include "fftmask.h"
#include "Presets.h"
#include "Globals.h"
#include "dataAcq.h"

#include "tusb.h"
#include "usb_descriptors.h"

extern uint16_t FracDivPWM;
extern uint16_t LowestWSPRToneFracDivPWM;


#ifdef TEST_FRAC_DIV
static int16_t IntCounter;
#endif

//#include "Presets.h"

//-----------------------------------------------------------------------------
uint32_t bouncing(void)
{	
	int delta;
	static int last_time = 0;

	delta = os_time - last_time;
	last_time = os_time; 
	// if less than 100 time units have elapsed from the previous
	// invokation, then bouncing...
	return delta < 100;  
}
//-----------------------------------------------------------------------------



// Load from the Presets table
void Load_Presets(void)
{
	int k;

	for(k=0; k<MAXPRESETS; k++)
	{
		strcpy(psets[k].name, pNames[k]);
		psets[k].freq = pFreqs[k];
		psets[k].mode = pModes[k];
		psets[k].bw   = pBws[k];
	}
}
//-----------------------------------------------------------------------------
// Tune to the chosen preset
void Tune_Preset(uint8_t Idx)
{
	int kHz, Hz;

	LOfreq = psets[Idx].freq;
	SetMode( psets[Idx].mode);
	SetBW( psets[Idx].bw);
	strcpy(msg, psets[Idx].name);

	kHz = LOfreq / 1000.f;  Hz = LOfreq - kHz*1000;


}


//-----------------------------------------------------------------------------
// Load the FFT mask according to the mode and the bandwidth chosen,
// and change the color of the buttons to indicate the active bandwidth
void SetBW(/*WM_HWIN ptr,*/ Bwidth newbw)
{
	if (newbw == CurrentBW)
		return;

	CurrentBW = newbw;
	switch(CurrentMode)
	{
	case AM :
#ifdef RECEIVE_AM
		bw[AM] = newbw;
		AMindex = (newbw == Narrow) ? 0 : 1;
		AMindex = 0; // TODO toglimi
#ifdef PRECALC_MASKS
		SDR_2R_toC_f32((float *)FFTmaskAM_R[AMindex],
				(float *)FFTmaskAM_I[AMindex], FFTmask, FFTLEN);
#else

		SetMask(-3000.0f, 3000.0f);
#endif
		break;
#endif
	case LSB :

		bw[LSB] = newbw;
		LSBindex = (newbw == Narrow) ? 0 : 1;
		AMindex = (newbw == Narrow) ? 0 : 1;
		LSBindex = 0; // TODO toglimi
#ifdef PRECALC_MASKS
		SDR_2R_toC_f32((float *)FFTmaskSSB_R[LSBindex],
				(float *)FFTmaskSSB_I[LSBindex], FFTmask, FFTLEN);
#else
		SetMask(300.0f, 2500.0f);
#endif

		break;

	case USB :

		bw[USB] = newbw;
		USBindex = (newbw == Narrow) ? 0 : 1;
		AMindex = (newbw == Narrow) ? 0 : 1;
		USBindex = 0; // TODO toglimi
#ifdef PRECALC_MASKS
		SDR_2R_toC_f32((float *)FFTmaskSSB_R[USBindex],
				(float *)FFTmaskSSB_I[USBindex], FFTmask, FFTLEN);
#else
		SetMask(300.0f, 2500.0f);
#endif
		break;

	case CW  :

		bw[CW] = newbw;
		CWindex = (newbw == Narrow) ? 0 : 1;
		CWindex = 0; // TODO toglimi
#ifdef PRECALC_MASKS
		SDR_2R_toC_f32((float *)FFTmaskCW_R[CWindex],
				(float *)FFTmaskCW_I[CWindex], FFTmask, FFTLEN);
#else
		SetMask(500.0f, 800.0f); //CWPITCH is 650
#endif
		break;

	default :
		break;
	}
}	
//-----------------------------------------------------------------------------
// Change the AGC constants according to the mode and the AGC chosen,
// and change the color of the buttons to indicate the active AGC speed
void SetAGC(/*WM_HWIN ptr,*/ Agctype newAGC)
{
	CurrentAGC =newAGC;
	switch(CurrentMode)
	{
	case AM :       agc[AM] = newAGC;
	Decay[AM]   = AGC_decay[newAGC];
	Hcount[AM]  = Hangcount[newAGC]; break;

	case LSB :      agc[LSB] = newAGC;
	Decay[LSB]  = AGC_decay[newAGC];
	Hcount[LSB] = Hangcount[newAGC]; break;

	case USB :      agc[USB] = newAGC;
	Decay[USB]  = AGC_decay[newAGC];
	Hcount[USB] = Hangcount[newAGC]; break;

	case CW :       agc[CW] = newAGC;
	Decay[CW]   = AGC_decay[newAGC];
	Hcount[CW]  = Hangcount[newAGC]; break;
	}
	//  ChangeColor(ptr, hFAST, (newAGC == Fast) ? GUI_RED   : GUI_BLACK);
	//  ChangeColor(ptr, hSLOW, (newAGC == Slow) ? GUI_RED   : GUI_BLACK);
}	
//-----------------------------------------------------------------------------
// Set the new demodulation mode chosen by the user, and change the color
// of the buttons to indicate the active mode

void SetMode(/*WM_HWIN ptr,*/ Mode newmode)
{
	if (CurrentMode == newmode)
		return;

	CurrentMode = newmode;

	switch(CurrentMode)
	{
	case AM :
		SetBW(/*ptr,*/ bw[AM]); SetAGC(/*ptr,*/ agc[AM]);
		break;

	case LSB :
		SetBW(/*ptr,*/ bw[LSB]);  SetAGC(/*ptr,*/ agc[LSB]);
		break;

	case USB :
		SetBW(/*ptr,*/ bw[USB]);  SetAGC(/*ptr,*/ agc[USB]);
		break;

	case CW  :
		SetBW(/*ptr,*/ bw[CW]);  SetAGC(/*ptr,*/ agc[CW]);
		break;

	default :
		break;
	}
}	

//-----------------------------------------------------------------------------
// Set the frequency step according to the radio button pressed by the user
void SetFstep(int idx)
{
	if (idx == 9)
		Fstep = 9000;  // MW Channel for Europe
	else
		Fstep = pow(10, 5 - idx);
}	
//-----------------------------------------------------------------------------
// Increase the frequency by the value of the current step
void FplusClicked(uint16_t Nsteps)
{	
	LOfreq += Fstep * (float)Nsteps / 2.0;
	LOfreq  = min(LOfreq, 50000000.f);
	psets[0].freq = LOfreq; psets[0].mode = CurrentMode;
	psets[0].bw = bw[CurrentMode];

#ifdef USE_EXTERNAL_MIXER
	SetFOut((uint32_t)(LOfreq + 10698000.0));
	LOfreq = 10698000.0;
#endif

	Tune_Preset(0);  // preset 0 means "User tuning"
}	
//-----------------------------------------------------------------------------
// Decrease the frequency by the value of the current step
void FminusClicked(uint16_t Nsteps)
{	
	LOfreq -= Fstep * (float)Nsteps / 2.0;
	LOfreq  = max(LOfreq, 8000.f);
	psets[0].freq = LOfreq; psets[0].mode = CurrentMode;
	psets[0].bw = bw[CurrentMode];

#ifdef USE_EXTERNAL_MIXER
	SetFOut((uint32_t)(LOfreq + 10698000.0));
	LOfreq = 10698000.0;
#endif


	Tune_Preset(0);  // preset 0 means "User tuning"
}



//-----------------------------------------------------------------------------
// Toggle the ON/OFF status of the two on-board Leds
void LED_switch()
{	


	//if (++timer_cnt & 1) {LED_On(1); LED_Off(0);}
	//else                 {LED_On(0); LED_Off(1);}	
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// This is the handler of the software interrupt generated by the highest
// priority task that handles the interrupts generated by DMA2 Stream 0,
// when an ADC buffer is filled
//void EXTI1_IRQHandler()
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
	if (pin == GPIO_PIN_0) {
		tud_task();
		audio_task(); //not needed anymore
//		MainLoopCounter++;  //used with debugger to check frequency of main loop
	cdc_task();
	}

	if (pin == GPIO_PIN_14) {

#ifdef CW_DECODER
		volatile uint16_t WFSample;
		volatile float tmp;
		float BinValue;
		int16_t i;
#endif



#ifdef TEST_NO_SDR
		return;
#endif


#ifdef DEBUG_TX_CW
		static int TX;
		if (TX++ == 1)
		{
			TXEnable(0);
		}
		else
		{
			if (TX== 4)
			{
				TX = 0;
				TXEnable(1);
			}
		}
#endif





		// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // set bit 8 of GPIOF high, to be observed with an oscilloscope


		// copy into work buffers the data received by CIC decimator
		SDR_memcpy_f32(Rbase, Rbasedata, BSIZE*4);
		SDR_memcpy_f32(Ibase, Ibasedata, BSIZE*4);




		// inverse sync filtering and decimation by 4
		arm_fir_decimate_f32(&SfirR, Rbase, Rdata, BSIZE*4);
		arm_fir_decimate_f32(&SfirI, Ibase, Idata, BSIZE*4);

		// filter now with fast convolution
		//---------------------------------
		// shift the FFT buffer to the left
		SDR_memcpy_f32(fCbase, fCbase + FFTLEN, FFTLEN);




		// insert at the right edge the latest data
		SDR_2R_toC_f32(Rdata, Idata, fCbase + FFTLEN, BSIZE);



		// copy into the (in place...) FFT buffer
		SDR_memcpy_f32(FFTbuf, fCbase, FFTLEN*2);




		//FFT returns BUFLEN/2 bins, 1024. Each bin size is FADC/ decimation /FFT size
		//ADC requires 16 clock cycles per sample, as it is set now
		// eg (160 MHz ADC) 10000000 / 64 / 4 / 1024 = 38.14697 Hz
		// eg (150 MHz ADC) 9375000 / 64 / 4 / 1024 = 36.62109375 Hz
		// eg (128 MHz ADC) 9375000 / 64 / 4 / 1024 = 31.250 Hz
		// when NCO F is higher than signal then the highest elements are filled
		//eg NCO 625381, signal 625000, FFTbuf elements filled are 2028 and 2029
		//FFTBuf 1023 and 1025 are the farest from NCO freq.

		// compute the direct FFT
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFTbuf, DIRECTFFT, NOREVERSE);
		/*
	// if LSB, copy the LSB in the lower half (USB)
	if(CurrentMode == LSB) SDR_mirror_LSB(FFTbuf, FFTLEN);
		 */

		// TODO: check why with the original code above LSB and USB are swapped

		//if USB, copy the USB in the lower half (LSB)
		if(CurrentMode == USB) SDR_mirror_LSB(FFTbuf, FFTLEN);

#ifdef TEST_WF
		if (ShowWF) {
			for (WFSample=0; WFSample<(FFTLEN * 2); WFSample += 2)
			{
				tmp = FFTbuf[WFSample] * FFTbuf[WFSample] + FFTbuf[WFSample+1] * FFTbuf[WFSample+1];
				arm_sqrt_f32(tmp, &WFBuffer[WFSample >> 1]);
			}
		}
#endif


#ifdef CW_DECODER
		volatile static int16_t DecodedTestBuffer[256], Test_i;
		CWLevel = 0;
		BaseNoiseLevel = 9999.f;
		for (WFSample=22; WFSample<44; WFSample += 2)
			//	for (WFSample=64; WFSample<84; WFSample += 2)
			//		for (WFSample=2*FFTLEN -50; WFSample<(2*FFTLEN - 40); WFSample += 2)
			//for (WFSample=46; WFSample<52; WFSample += 2)
		{
			tmp = FFTbuf[WFSample] * FFTbuf[WFSample] + FFTbuf[WFSample+1] * FFTbuf[WFSample+1];
			arm_sqrt_f32(tmp, &BinValue);
			if (CWLevel < BinValue)
				CWLevel = BinValue;
			if (BaseNoiseLevel > BinValue)
				BaseNoiseLevel = BinValue;
		}
		SignalAverage = SIGNAL_AVERAGE_T_CONST * CWLevel + (1 - SIGNAL_AVERAGE_T_CONST) * OldSignalAverage;
		OldSignalAverage = SignalAverage;

		//We shorten the pulse by filtering out the first sample at attack.
		// This gives a 50% duty cycle for a square wave.
		// Without filter a square wave would have an higher on time than off time.

		//		if (CWLevel > (SignalAverage * CWThreshold))
		if (CWLevel - BaseNoiseLevel > (CWThreshold))
			//			if (CWLevel / BaseNoiseLevel > (CWThreshold))
			//			if (!SW01_IN)
			CWIn += 1; //TODO limit CW increase
		else
			CWIn = 0;


		DecodeCW();

#endif

		/*
#ifdef CW_DECODER

	CWLevel = 0;
	for (WFSample=2*FFTLEN -42; WFSample<(2*FFTLEN - 40); WFSample += 2)
	//for (WFSample=46; WFSample<52; WFSample += 2)
	{
		tmp = FFTbuf[WFSample] * FFTbuf[WFSample] + FFTbuf[WFSample+1] * FFTbuf[WFSample+1];
		arm_sqrt_f32(tmp, &BinValue);
		if (CWLevel < BinValue);
			CWLevel = BinValue;
	}
	BaseNoiseLevel = 0;
	for (WFSample=2*FFTLEN -62; WFSample<(2*FFTLEN - 50); WFSample += 2)
	{
		tmp = FFTbuf[WFSample] * FFTbuf[WFSample] + FFTbuf[WFSample+1] * FFTbuf[WFSample+1];
		arm_sqrt_f32(tmp, &BinValue);
		BaseNoiseLevel += BinValue;
	}
	SignalAverage = SIGNAL_AVERAGE_T_CONST * CWLevel + (1 - SIGNAL_AVERAGE_T_CONST) * OldSignalAverage;
	OldSignalAverage = SignalAverage;

	if (CWLevel > (SignalAverage + CW_THRESHOLD))
//	if (CWLevel - BaseNoiseLevel > (CW_THRESHOLD))
//	if (SW01_IN)

		CWIn = 1;
	else
		CWIn = 0;

	DecodeCW();

#endif
		 */
		/*
#ifdef CW_DECODER

	CWLevel = 0;
	for (WFSample=2*FFTLEN -42; WFSample<(2*FFTLEN - 40); WFSample += 2)
	//for (WFSample=46; WFSample<52; WFSample += 2)
	{
		tmp = FFTbuf[WFSample] * FFTbuf[WFSample] + FFTbuf[WFSample+1] * FFTbuf[WFSample+1];
		arm_sqrt_f32(tmp, &BinValue);
		CWLevel += BinValue;
	}
	BaseNoiseLevel = 0;
	for (WFSample=2*FFTLEN -62; WFSample<(2*FFTLEN - 50); WFSample += 2)
	{
		tmp = FFTbuf[WFSample] * FFTbuf[WFSample] + FFTbuf[WFSample+1] * FFTbuf[WFSample+1];
		arm_sqrt_f32(tmp, &BinValue);
		BaseNoiseLevel += BinValue;
	}
	SignalAverage = SIGNAL_AVERAGE_T_CONST * CWLevel + (1 - SIGNAL_AVERAGE_T_CONST) * OldSignalAverage;
	OldSignalAverage = SignalAverage;

	if (CWLevel > (SignalAverage + CW_THRESHOLD))
//	if (CWLevel - BaseNoiseLevel > (CW_THRESHOLD))
//	if (SW01_IN)

		CWIn = 1;
	else
		CWIn = 0;

	DecodeCW();

#endif
		 */

		// mult. by the fast convolution mask
		arm_cmplx_mult_cmplx_f32(FFTbuf, FFTmask, FFTbuf2, FFTLEN);

		// compute now the inverse FFT
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFTbuf2, INVERSEFFT, NOREVERSE);
		// then do the overlap-discard
		SDR_memcpy_f32(tmpSamp, FFTbuf2 + 2*FFTLEN - 2*BSIZE, 2*BSIZE);


		// we have now the bandpass filtered I/Q, demodulate the signal
		switch(CurrentMode)
		{
		case AM :
#ifdef RECEIVE_AM
			SDR_demodAM_AGC(tmpSamp, fAudio);  break;
#endif
		case LSB :
		case USB :
			SDR_demodSSB_CW_AGC(tmpSamp, fAudio); break;

		case  CW :
			SDR_demodSSB_CW_AGC(tmpSamp, fAudio);
			if(bw[CW] == Narrow)
				SDR_CWPeak(fAudio, BSIZE);
			break;

		default:
			break;
		}


#ifdef DCF77_DECODER

		if (WSPRBeaconMode == 1)
		{
			CWLevel = 0;
			BaseNoiseLevel = 9999.f;
			//	uint16_t i;

			for (int i = 0; i < BSIZE; i++)
			{
				CWLevel = fabs(fAudio[i]);
				CWLevelFiltered = CW_LEVEL_AVERAGE_T_CONST * CWLevel + (1 - CW_LEVEL_AVERAGE_T_CONST) * OldCWLevelAverage;
				OldCWLevelAverage = CWLevelFiltered;

				MediumLevelFiltered = MEDIUM_LEVEL_AVERAGE_T_CONST * CWLevel + (1 - MEDIUM_LEVEL_AVERAGE_T_CONST) * OldMediumLevelAverage;
				OldMediumLevelAverage = MediumLevelFiltered;
				//		if (CWLevel > (SignalAverage * CWThreshold))
				if ( MediumLevelFiltered - CWLevelFiltered  > CWThreshold)
				{
					DCF77In = 0;
					LED_RED_OFF;
				}
				else
				{
					DCF77In += 1; //TODO limit CW increase
					LED_RED_ON;
				}
#ifdef SNAPSHOT_ACQUISITION_DBG
				DumpTrace();
#endif
				DoDCF77(DCF77In);
			}
		}
#endif


#ifdef AG_TEST_AUDIO
		//TODO correct comment
		// Sample rate of DAC is 27901.786 Hz set from timer 6 (TIM_TimeBaseInit)
		// Soft interrupt handler is at 27901.786 / 512 = 54.496 Hz
		// Generate test square wave with a period of 64 samples = 436 Hz. (32 On 32 Off)
		for (i=0; i<BSIZE; i++)
		{
			if (i % 64 > 31)
				fAudio[i] = 0.1; //Volume
			else
				fAudio[i] = -0.1;
		}

#endif

#ifdef CW_TX_SIDETONE
		// CW tone while keying
		//TODO: make it sine and with attack/decay
		if (TXCarrierEnabled)
			for (int i=0; i<BSIZE; i++)
			{
				if (i % 64 > 31)
					fAudio[i] = RXVolume * SIDETONE_VOLUME; //Volume
				else
					fAudio[i] = -RXVolume * SIDETONE_VOLUME;
			}
		else
		{
			if (TransmissionEnabled)
				for (int i=0; i<BSIZE; i++)
				{
					fAudio[i] = 0.;
				}

		}
#endif

		// send the demodulated audio to the DMA buffer just emptied

		//LED_YELLOW_ON;
		SDR_float_to_DAC_audio(fAudio, ValidAudioHalf, BSIZE);
		//LED_YELLOW_OFF;


		// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // set bit 8 of GPIOF low, to be observed with an oscilloscope
	}
}





//-----------------------------------------------------------------------------  
// This the handler of the highest priority task interrupts, those generated
// by DMA2 Stream when a new ADC buffer is just filled
// Frequency is FADC / bitsPerSampleADC / BSIZE/2
// 150000000 /16 /512 = 18310,54688
// 128000000 / 16 / 512 = 15625

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

void ADC_Stream0_Handler(uint8_t FullConversion)
{
	static int16_t k, idx = 0;


	float sum;

	register float *ptDataR, *ptDataI, inER, inOR, inEI, inOI, outR, outI,
	tmp1R, tmp1I, tmp2R, tmp2I, tmp3R, tmp3I,
	tmp4R, tmp4I, tmp5R, tmp5I;

	static   float inE1Rold,  inE2Rold,  inE1Iold,   inE2Iold,  inO1Rold, 
	inO2Rold,  inO1Iold,  inO2Iold,   inO1Rold2, inO1Iold2,
	inO2Rold2, inO2Iold2,             inE3Rold,  inE3Iold,
	inO3Rold,  inO3Iold,  inO3Rold2,  inO3Iold2, inE4Rold,
	inE4Iold,  inO4Rold2, inO4Iold2,  inO4Rold,  inO4Iold,
	inE5Rold,  inE5Iold,  inO5Rold2,  inO5Iold2,  inO5Rold,  inO5Iold,
	inE6Rold,  inE6Iold,  inO6Rold2,  inO6Iold2,  inO6Rold,  inO6Iold;



	volatile uint16_t *pR;


	//LED_YELLOW_ON;

	/*
		if (PLLDitherCounter==3)
		{
			SetFracPLL(16);
			PLLDitherCounter = 0;
		}
		else
		{
				SetFracPLL(15);
				PLLDitherCounter++;
		}
	 */

	/* It needs a delay between fract div disable and parameter setting.
	 * Reference manual says otherwise.
	 * So we disable at the top of the ISR and set the parameter near the bottom.
	 */
	if (TransmittingWSPR)
	{
		__HAL_RCC_PLL2FRACN_DISABLE();
	}




	// process the data contained in the just filled buffer
	if(FullConversion)
		pR =(uint16_t *) &aADCDualConvertedValues[BSIZE/2];
	else
		pR = (uint16_t *) &aADCDualConvertedValues[0];



#ifdef FAKE_SINE_RF_SIGNAL
	pR=TestSignalData;
#endif

#ifdef FAKE_SQUARE_RF_SIGNAL
	pR=TestSignalData;
#endif


#ifdef FAKE_NO_RF_SIGNAL
	pR=TestSignalData;
#endif


	// compute the new NCO buffer, with the CWpitch offset if receiving CW
	if(CurrentMode == CW)
		SDR_ComputeLO(LOfreq + cwpitch);  // prepare next LO buffer
	else
		SDR_ComputeLO(LOfreq);          // prepare next LO buffer

	// compute the smoothed average value of the buffer, to be used as offset
	// in the short words to floating point conversion routine

	//TODO Check if it should be BSIZE/2

	sum = 0; k = BSIZE;
	while(k)
	{
		sum += pR[k-1];
		sum += pR[k-2];
		sum += pR[k-3];
		sum += pR[k-4];
		k-=4;
	}

	TestSampledValue=pR[BSIZE/2];

	meanavg = sum/(float)BSIZE; //TODO was "mean". Seems to be a bug from original ArmRadio

	// downconvert to zero IF, by multiplication by the exp(-jwt) signal
	// generated by the NCO, and at the same time convert to floating point
	SDR_downconvert_f32((uint16_t *)pR, meanavg, ADC_Rdata, ADC_Idata);



	ptDataR = ADC_Rdata;  ptDataI = ADC_Idata;


	if (TransmittingWSPR)
	{
		if (IntCounter++ < FracPWMCoeff[WSPRTone])
		{
			__HAL_RCC_PLL2FRACN_CONFIG(FracDivCoeff[WSPRTone] + 1);
		}
		else
		{
			__HAL_RCC_PLL2FRACN_CONFIG(FracDivCoeff[WSPRTone] );
		}
		if (IntCounter == 8)
		{
			IntCounter = 0;
		}
		__HAL_RCC_PLL2FRACN_ENABLE();
	}

#ifdef CIC_DECIMATE_64

	//-------------------------------------------------------------------------
	// Now we decimate by 16 or 64 the input samples, using the CIC polyphase decomposition
	// technique, which has the advantage of eliminating the recursive
	// component, allowing the use of floating point, rather fast on a Cortex M4F
	//
	// A dividing by 16, order 4, CIC is used. Then a 4096-entry buffer is filled, and
	// passed to the baseband interrupt routine, where it is additionally filtered with a
	// sync-compensating FIR, which also adds further stop band rejection and a decimation by 4
	//-------------------------------------------------------------------------

	k=BSIZE/2;  // BSIZE/2 to process BSIZE entries, two at a time
	asm("nop");
	while(k--)
	{
		// CIC, R=16, M=4, computed in four div_by_2 sections, using the polyphase decomposition
		// H(z) = (1 + z^-1)(1 + z^-1)(1 + z^-1)(1 + z^-1) each section, which can be decomposed as follows :
		// H(z) = 1 + 4z^-1 + 6z^-2 + 4z^-3 + z^-4
		//        which being separated in even and odd samples in advance becomes
		// (1 + 6z^-1 + z^-2) for odd samples and (4 + 4z^-1) for even samples, which, when summed, give :
		// odd + 6odd_old + odd_old2 + 4even + 4even_old =	odd + 6odd_old + odd_old2 + 4(even + even_old)


		inER=*ptDataR++; inOR=*ptDataR++;          inEI=*ptDataI++; inOI=*ptDataI++;
		outR=(inOR+6.f*inO1Rold+inO1Rold2+4.f*(inER+inE1Rold)); outI=(inOI+6.f*inO1Iold+inO1Iold2+4.f*(inEI+inE1Iold));

		inE1Rold = inER;                           inE1Iold = inEI;
		inO1Rold2 = inO1Rold; inO1Rold = inOR;     inO1Iold2 = inO1Iold; inO1Iold = inOI;

		if((k & 0x1))  // skip the if-block for k multiple of 2 (in base zero),
			// else save the even element just produced and cycle the while loop...
		{
			tmp1R = outR; tmp1I = outI;  // save the even element produced
			continue;
		}

		// at this point we have two elem. (tmp1R[even] and outR[odd] and also the I counterparts)
		// produced using 4 input samples, totalling a decimation by 2
		// now compute the couple of elements for the next step

		inER=tmp1R;  inOR=outR;                    inEI=tmp1I;  inOI=outI;
		outR=(inOR+6.f*inO2Rold+inO2Rold2+4.f*(inER+inE2Rold)); outI=(inOI+6.f*inO2Iold+inO2Iold2+4.f*(inEI+inE2Iold));

		inE2Rold = inER;                           inE2Iold = inEI;
		inO2Rold2 = inO2Rold; inO2Rold = inOR;     inO2Iold2 = inO2Iold; inO2Iold = inOI;

		if((k & 0x2)) // skip the if block for k multiple of 4 (in base zero),
			// else save the even element just produced and cycle the while loop...
		{
			tmp2R = outR; tmp2I = outI;  // save the even element produced
			continue;
		}

		// now we have the input samples decimated by 4, even element in tmp2R, tmp2I,
		// and the odd element in outR, outI
		// now compute the couple of elements for the next step

		inER=tmp2R;  inOR=outR;                    inEI=tmp2I;  inOI=outI;
		outR=(inOR+6.f*inO3Rold+inO3Rold2+4.f*(inER+inE3Rold)); outI=(inOI+6.f*inO3Iold+inO3Iold2+4.f*(inEI+inE3Iold));

		inE3Rold  = inER;                          inE3Iold  = inEI;
		inO3Rold2 = inO3Rold; inO3Rold = inOR;     inO3Iold2 = inO3Iold; inO3Iold = inOI;

		if((k & 0x4)) // skip the if block for k multiple of 8 (in base zero),
			// else save the even element just produced and cycle the while loop...
		{
			tmp3R = outR; tmp3I = outI;  // save the even element produced
			continue;
		}
		/////////////////////////// Added two more sections of filter: decimation is now 2^6 = 64

		// at this point we have two elem. (tmp1R[even] and outR[odd] and also the I counterparts)
		// produced using 4 input samples, totalling a decimation by 8
		// now compute the couple of elements for the next step

		inER=tmp3R;  inOR=outR;                    inEI=tmp3I;  inOI=outI;
		outR=(inOR+6.f*inO4Rold+inO4Rold2+4.f*(inER+inE4Rold)); outI=(inOI+6.f*inO4Iold+inO4Iold2+4.f*(inEI+inE4Iold));

		inE4Rold = inER;                           inE4Iold = inEI;
		inO4Rold2 = inO4Rold; inO4Rold = inOR;     inO4Iold2 = inO4Iold; inO4Iold = inOI;

		if((k & 0x8)) // skip the if block for k multiple of 8 (in base zero),
			// else save the even element just produced and cycle the while loop...
		{
			tmp4R = outR; tmp4I = outI;  // save the even element produced
			continue;
		}

		// now we have the input samples decimated by 8, even element in tmp2R, tmp2I,
		// and the odd element in outR, outI
		// now compute the couple of elements for the next step

		inER=tmp4R;  inOR=outR;                    inEI=tmp4I;  inOI=outI;
		outR=(inOR+6.f*inO5Rold+inO5Rold2+4.f*(inER+inE5Rold)); outI=(inOI+6.f*inO5Iold+inO5Iold2+4.f*(inEI+inE5Iold));

		inE5Rold  = inER;                          inE5Iold  = inEI;
		inO5Rold2 = inO5Rold; inO5Rold = inOR;     inO5Iold2 = inO5Iold; inO5Iold = inOI;



		if((k & 0x10)) // skip the if block for k multiple of 10 (in base zero),
			// else save the even element just produced and cycle the while loop...
		{
			tmp5R = outR; tmp5I = outI;  // save the even element produced
			continue;
		}

		///////////////////////////




		// at this point we have two elem. (tmp3R[even] and outR[odd] and also the I counterparts)
		// produced with 4 of the previous elem, i.e. with 16 input samples, totalling
		// a decimation by 16. Now compute the couple of elements for the next step

		inER=tmp5R;  inOR=outR;                    inEI=tmp5I;  inOI=outI;
		outR=(inOR+6.f*inO6Rold+inO6Rold2+4.f*(inER+inE6Rold)); outI=(inOI+6.f*inO6Iold+inO6Iold2+4.f*(inEI+inE6Iold));

		inE6Rold = inER;                           inE6Iold = inEI;
		inO6Rold2 = inO6Rold; inO6Rold = inOR;     inO6Iold2 = inO6Iold; inO6Iold = inOI;





		// at this point we have a single element (outR and its counterpart outI), produced
		// with 2 of the previous element, i.e. with 16 input samples, totalling a decimation by 64
		// we downscale it with a factor of 8388608, i.e. the gain of the CIC, i.e.	R^M = 64^4 = 16777216
		// divided by two, to compensate for the 3 dB loss caused by keeping just half of the band

		// create a block of BSIZE*4 entries, which will be then decimated by 4

		Rbasedata[idx] = outR/8388608.f;    Ibasedata[idx++] = outI/8388608.f;  //decimate by 64
		//	  Rbasedata[idx] = outR/65536.f;    Ibasedata[idx++] = outI/65536.f; //decimate by 16

		if(idx < BSIZE*4)
			continue;
		idx = 0;

#endif

#ifdef CIC_DECIMATE_16

		//-------------------------------------------------------------------------
		// Now we decimate by 16 or 64 the input samples, using the CIC polyphase decomposition
		// technique, which has the advantage of eliminating the recursive
		// component, allowing the use of floating point, rather fast on a Cortex M4F
		//
		// A dividing by 16, order 4, CIC is used. Then a 4096-entry buffer is filled, and
		// passed to the baseband interrupt routine, where it is additionally filtered with a
		// sync-compensating FIR, which also adds further stop band rejection and a decimation by 4
		//-------------------------------------------------------------------------

		k=BSIZE/2;  // BSIZE/2 to process BSIZE entries, two at a time
		while(k--)
		{
			// CIC, R=16, M=4, computed in four div_by_2 sections, using the polyphase decomposition
			// H(z) = (1 + z^-1)(1 + z^-1)(1 + z^-1)(1 + z^-1) each section, which can be decomposed as follows :
			// H(z) = 1 + 4z^-1 + 6z^-2 + 4z^-3 + z^-4
			//        which being separated in even and odd samples in advance becomes
			// (1 + 6z^-1 + z^-2) for odd samples and (4 + 4z^-1) for even samples, which, when summed, give :
			// odd + 6odd_old + odd_old2 + 4even + 4even_old =	odd + 6odd_old + odd_old2 + 4(even + even_old)

			inER=*ptDataR++; inOR=*ptDataR++;          inEI=*ptDataI++; inOI=*ptDataI++;
			outR=(inOR+6.f*inO1Rold+inO1Rold2+4.f*(inER+inE1Rold)); outI=(inOI+6.f*inO1Iold+inO1Iold2+4.f*(inEI+inE1Iold));

			inE1Rold = inER;                           inE1Iold = inEI;
			inO1Rold2 = inO1Rold; inO1Rold = inOR;     inO1Iold2 = inO1Iold; inO1Iold = inOI;

			if((k & 0x1))  // skip the if-block for k multiple of 2 (in base zero),
				// else save the even element just produced and cycle the while loop...
			{
				tmp1R = outR; tmp1I = outI;  // save the even element produced
				continue;
			}

			// at this point we have two elem. (tmp1R[even] and outR[odd] and also the I counterparts)
			// produced using 4 input samples, totalling a decimation by 2
			// now compute the couple of elements for the next step

			inER=tmp1R;  inOR=outR;                    inEI=tmp1I;  inOI=outI;
			outR=(inOR+6.f*inO2Rold+inO2Rold2+4.f*(inER+inE2Rold)); outI=(inOI+6.f*inO2Iold+inO2Iold2+4.f*(inEI+inE2Iold));

			inE2Rold = inER;                           inE2Iold = inEI;
			inO2Rold2 = inO2Rold; inO2Rold = inOR;     inO2Iold2 = inO2Iold; inO2Iold = inOI;

			if((k & 0x2)) // skip the if block for k multiple of 4 (in base zero),
				// else save the even element just produced and cycle the while loop...
			{
				tmp2R = outR; tmp2I = outI;  // save the even element produced
				continue;
			}

			// now we have the input samples decimated by 4, even element in tmp2R, tmp2I,
			// and the odd element in outR, outI
			// now compute the couple of elements for the next step

			inER=tmp2R;  inOR=outR;                    inEI=tmp2I;  inOI=outI;
			outR=(inOR+6.f*inO3Rold+inO3Rold2+4.f*(inER+inE3Rold)); outI=(inOI+6.f*inO3Iold+inO3Iold2+4.f*(inEI+inE3Iold));

			inE3Rold  = inER;                          inE3Iold  = inEI;
			inO3Rold2 = inO3Rold; inO3Rold = inOR;     inO3Iold2 = inO3Iold; inO3Iold = inOI;

			if((k & 0x4)) // skip the if block for k multiple of 8 (in base zero),
				// else save the even element just produced and cycle the while loop...
			{
				tmp3R = outR; tmp3I = outI;  // save the even element produced
				continue;
			}

			// at this point we have two elem. (tmp3R[even] and outR[odd] and also the I counterparts)
			// produced with 4 of the previous elem, i.e. with 16 input samples, totalling
			// a decimation by 8. Now compute the couple of elements for the next step

			inER=tmp3R;  inOR=outR;                    inEI=tmp3I;  inOI=outI;
			outR=(inOR+6.f*inO4Rold+inO4Rold2+4.f*(inER+inE4Rold)); outI=(inOI+6.f*inO4Iold+inO4Iold2+4.f*(inEI+inE4Iold));

			inE4Rold = inER;                           inE4Iold = inEI;
			inO4Rold2 = inO4Rold; inO4Rold = inOR;     inO4Iold2 = inO4Iold; inO4Iold = inOI;

			// at this point we have a single element (outR and its counterpart outI), produced
			// with 2 of the previous element, i.e. with 16 input samples, totalling a decimation by 16
			// we downscale it with a factor of 32768, i.e. the gain of the CIC, i.e.	R^M = 16^4 = 65536
			// divided by two, to compensate for the 3 dB loss caused by keeping just half of the band

			// create a block of BSIZE*4 entries, which will be then decimated by 4

			Rbasedata[idx] = outR/32768.f;    Ibasedata[idx++] = outI/32768.f;

			if(idx < BSIZE*4)
				continue;
			idx = 0;


#endif


			// generate now an interrupt to signal the base band processing routine that it has a new buffer

		EXTI->SWIER1 |= GPIO_PIN_14;
		}

		// LED_YELLOW_OFF;

	}
	//-----------------------------------------------------------------------------

	//#pragma GCC pop_options
