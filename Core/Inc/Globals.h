/*******************************************************************************

                   Globals.h module of the program ARM_Radio
						                          
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBALS_H
#define __GLOBALS_H

// allocate actual memory only when invoked from the main.c module
#ifdef IN_MAIN
 #define EXTERN
#else
 #define EXTERN extern
#endif  
 
#include "stdint.h" 
//TODO #include "stm32f4xx.h"
#include "main.h"
#include "arm_math.h"



 
// place the following variables in the dtcm section of RAM ------------------
// ****AG CCM should be defined in LinkerScript.ld
EXTERN float Rbasedata[BSIZE*4] __attribute__ ((section (".dtcm")));
EXTERN float Ibasedata[BSIZE*4] __attribute__ ((section (".dtcm")));
EXTERN float Rbase[BSIZE*4]     __attribute__ ((section (".dtcm")));
EXTERN float Ibase[BSIZE*4]     __attribute__ ((section (".dtcm")));
EXTERN float Rdata[BSIZE]       __attribute__ ((section (".dtcm")));
EXTERN float Idata[BSIZE]       __attribute__ ((section (".dtcm")));
EXTERN float FFTmask[FFTLEN*2]  __attribute__ ((section (".dtcm")));
EXTERN float FFTbuf[FFTLEN*2]   __attribute__ ((section (".dtcm")));
EXTERN float FFTbuf2[FFTLEN*2]  __attribute__ ((section (".dtcm")));
// end of dtcm placing --------------------------------------------------------

EXTERN volatile uint16_t ADC_Data0[BSIZE] __attribute__ ((section (".dtcm")));
EXTERN volatile uint16_t ADC_Data1[BSIZE] __attribute__ ((section (".dtcm")));

#pragma pack(16)
EXTERN float ADC_Rdata[BSIZE] __attribute__ ((section (".dtcm")));
EXTERN float ADC_Idata[BSIZE] __attribute__ ((section (".dtcm")));
EXTERN float IQdata[BSIZE*2]  __attribute__ ((section (".dtcm")));    // IQdata  is a complex signal
EXTERN float fCbase[FFTLEN*2] ;//__attribute__ ((section (".dtcm")));   //TODO: when in DTCIM RX works only after a debugger reset. No working after power up           // fCbase  is a complex signal
EXTERN float tmpSamp[BSIZE*2+12] ;//__attribute__ ((section (".dtcm")));         // tmpSamp is a complex signal
EXTERN float LO_R[BSIZE] ;//__attribute__ ((section (".dtcm")));
EXTERN float LO_I[BSIZE] ;//__attribute__ ((section (".dtcm")));    // LO is a complex signal
EXTERN float fAudio[BSIZE];
EXTERN __IO short* ValidAudioHalf;

EXTERN  ADC_HandleTypeDef *HAdc1;
EXTERN ALIGN_32BYTES(__IO uint32_t   aADCDualConvertedValues[BSIZE]);    /* ADC dual mode interleaved conversion results (ADC master and ADC slave results concatenated on data register 32 bits of ADC master). */


EXTERN float FIRstate1R[NUMFIRCOEFS + BSIZE*4 - 1];
EXTERN float FIRstate1I[NUMFIRCOEFS + BSIZE*4 - 1];
EXTERN Agctype agc[4];
EXTERN Bwidth  bw[4];
EXTERN Presets psets[MAXPRESETS];

EXTERN Agctype CurrentAGC;
EXTERN Bwidth CurrentBW;

EXTERN arm_status arc;
EXTERN arm_fir_decimate_instance_f32 SfirR;
EXTERN arm_fir_decimate_instance_f32 SfirI;

EXTERN float TestSampledValue;

EXTERN float     RXVolume, Qfactor, a1, a2, b0, cwpitch, audiotmp,
	               AgcThreshold, AGC_decay[2], LOfreq, mean, meanavg, Decay[4];
EXTERN uint16_t  Hangcount[2], AMindex, LSBindex, USBindex, CWindex, Hcount[4];    
EXTERN Mode      CurrentMode;
EXTERN float32_t SamplingRate, AudioRate;

EXTERN uint32_t  os_time, Fstep;
//EXTERN __IO uint32_t uwTick;

//EXTERN WM_HWIN   hWin, hItem;
EXTERN uint8_t        Muted;
EXTERN char      msg[32];
EXTERN float SValue;
EXTERN uint16_t OVFDetected;
EXTERN uint16_t EncVal, LastEncVal ;

EXTERN float WFBuffer[FFTLEN];
EXTERN uint8_t ShowWF;

#ifdef FAKE_SINE_RF_SIGNAL
EXTERN uint16_t TestSignalData[BSIZE];
#endif

#ifdef FAKE_SQUARE_RF_SIGNAL
EXTERN uint16_t TestSignalData[BSIZE];
#endif

#ifdef FAKE_NO_RF_SIGNAL
EXTERN uint16_t TestSignalData[BSIZE];
#endif

#ifdef CW_DECODER
EXTERN float CWLevel, SignalAverage, OldSignalAverage, BaseNoiseLevel;
EXTERN uint8_t CWIn;
EXTERN char DecodedCWChar;
EXTERN int NCharReceived;
EXTERN int CurrentAverageDah;
EXTERN float LastPulsesRatio;
EXTERN float LastDownTime;
#endif


#ifdef DCF77_DECODER
EXTERN float CWLevel, SignalAverage, OldSignalAverage, BaseNoiseLevel, CWLevelAverage, OldCWLevelAverage, BaseNoiseLevelAverage;
EXTERN float CWLevelFiltered, BaseNoiseLevelFiltered, OldCWLevelAverage, OldBaseNoiseAverage, MediumLevelFiltered, OldMediumLevelAverage;
EXTERN uint8_t CWIn;
EXTERN uint16_t DCF77In, LastDCF77In;
EXTERN char DecodedCWChar;
EXTERN int NCharReceived;
EXTERN int CurrentAverageDah;
EXTERN float LastPulsesRatio;
EXTERN float LastDownTime;
#endif

#ifdef USE_SCAMP
EXTERN uint8_t TXString[255];
EXTERN OutData_t TXMessage;

#endif

EXTERN uint8_t TXCarrierEnabled;
EXTERN uint8_t TransmissionEnabled;

EXTERN float XTalFreq;
EXTERN float CWThreshold;

EXTERN int TXFreqError;

EXTERN int keyer_speed;
EXTERN unsigned long ditTime;                    // No. nseconds per dit
EXTERN uint8_t keyerControl;
EXTERN uint8_t keyerState;
EXTERN uint8_t keyer_mode; //->  SINGLE
EXTERN uint8_t keyer_swap; //->  DI/DAH

EXTERN uint32_t ktimer;
EXTERN int Key_state;
EXTERN int debounce;
EXTERN uint32_t semi_qsk_timeout;
EXTERN uint8_t tx;
EXTERN uint8_t txdelay;

EXTERN uint32_t SpaceFracDiv, MarkFracDiv;

EXTERN enum KSTYPE {IDLE, CHK_DIT, CHK_DAH, KEYED_PREP, KEYED, INTER_ELEMENT } KSType; // State machine states
EXTERN int hangcnt, Saved_hangcnt;
EXTERN float pk, Saved_pk;

EXTERN uint32_t TxPowerOut;

EXTERN uint32_t USBRXLength;

EXTERN uint8_t UartTXString[4096];
EXTERN uint8_t UartRXString[256];

EXTERN uint16_t FracDivPWM;
EXTERN uint16_t LowestWSPRToneFracDivPWM;

EXTERN uint16_t FracDivCoeff[4];
EXTERN uint16_t FracPWMCoeff[4];
EXTERN uint32_t SystemSeconds, SystemMinutes;

EXTERN uint8_t TransmittingWSPR;
EXTERN uint8_t WSPRTone, WSPRTXFraction;
EXTERN float LastTXFreq;

EXTERN uint8_t DCF77Min, DCF77Hour;

EXTERN enum WSPR_BEACON_STATE {NO_FIX, FIRST_FIX, SEND_WSPR} WSPRBeaconState;

EXTERN uint8_t DisableDisplay;
EXTERN uint8_t WSPRBeaconMode;
// TODO EXTERN NVIC_InitTypeDef      NVIC_InitStructure;
//TODO EXTERN EXTI_InitTypeDef      EXTI_InitStructure;

EXTERN float PeakAudioValue;

//EXTERN WM_MESSAGE *GlobalMsgPtr;

#endif /* __GLOBALS_H */
