/*******************************************************************************

                   SDR_math.c module of the program ARM_Radio

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

/******************************************************************************
         Some mathematical routines needed by the program ARM_Radio
 ******************************************************************************/

#include "Globals.h"

#include <arm_const_structs.h>

//------------------------------------------------------------------------------
void SDR_ComputeLO(float32_t freq)
{
	/*------------------------------------------------------------------------------
  ComputeLO,  fill the LO complex vector with the new samples
	 *------------------------------------------------------------------------------*/
	uint16_t        k;
	float           *pBufR=LO_R, *pBufI=LO_I;
	static float    costheta, sintheta, oldfreq = 1.e9f, ym1i=1.f, ym1q=0.f,
			ypi, ypq, tmpi, gain=1.f;

	if (oldfreq != freq)
	{	
		oldfreq  =  freq;
		costheta =  cos(TWOPI * freq / SamplingRate);
		sintheta = -sin(TWOPI * freq / SamplingRate);
	}

	k=BSIZE/4;   // BSIZE/4, as the loop is unrolled by 4 and we
	// have to fill a buffer of BSIZE complex entries

	// Coupled Quadrature Oscillator with level stabilization
	while(k)
	{                    
		// loop partially unrolled for performance

		ypi  = ym1i * costheta - ym1q * sintheta; *pBufR++ = tmpi = ypi * gain;
		ypq  = ym1i * sintheta + ym1q * costheta; *pBufI++ = ym1q = ypq * gain;  
		ym1i = tmpi;

		ypi  = ym1i * costheta - ym1q * sintheta; *pBufR++ = tmpi = ypi * gain;
		ypq  = ym1i * sintheta + ym1q * costheta; *pBufI++ = ym1q = ypq * gain;  
		ym1i = tmpi;

		ypi  = ym1i * costheta - ym1q * sintheta; *pBufR++ = tmpi = ypi * gain;
		ypq  = ym1i * sintheta + ym1q * costheta; *pBufI++ = ym1q = ypq * gain;  
		ym1i = tmpi;

		ypi  = ym1i * costheta - ym1q * sintheta; *pBufR++ = tmpi = ypi * gain;
		ypq  = ym1i * sintheta + ym1q * costheta; *pBufI++ = ym1q = ypq * gain;  
		ym1i = tmpi;

		k--;
	}
	// compute the gain to be applied to stabilize the level
	gain = (8192.5f - (ypi * ypi + ypq * ypq))/8192.f; //was (8192.5f - (ypi * ypi + ypq * ypq))/8192.f;
}	
//------------------------------------------------------------------------------
// Combine two floating point real vectors into one complex vector 
void SDR_2R_toC_f32(float * pSrcA, float * pSrcB, float * pDst, uint32_t blockSize)
{
	uint32_t blkCnt;           /* loop counter */

	// loop Unrolling
	blkCnt = blockSize >> 2u;

	// Compute 4 outputs at a time
	while(blkCnt)
	{
		*pDst++ = *pSrcA++;  *pDst++ = *pSrcB++;
		*pDst++ = *pSrcA++;  *pDst++ = *pSrcB++;
		*pDst++ = *pSrcA++;  *pDst++ = *pSrcB++;
		*pDst++ = *pSrcA++;  *pDst++ = *pSrcB++;

		blkCnt--;
	}
}
//---------------------------------------------------------------------------------------
// Multiply the real signal vector by the complex NCO vector producing the zeroIF
// complex vector, and at the same time convert to floating point also using
// the smoothed average ADC offset computed by the DMA2_Stream0_IRQHandler routine
void SDR_downconvert_f32(uint16_t* signal, float offset, float* zeroIF_R, float* zeroIF_I)
{
	uint32_t blkCnt;            // loop counter
	float  tmp1, tmp2, tmp3, tmp4, *LOI=LO_R, *LOR=LO_I;
	uint16_t *pt = signal;

	// loop Unrolling
	blkCnt = BSIZE >> 2u;   // loop unrolling.  Compute 4 outputs at a time
	while(blkCnt)
	{  // DMA Mode2, first ADC2 then ADC1...
		/*
     tmp1=((*(pt+1)-offset)) / 2048.f;
		 tmp2=((*(pt)  -offset)) / 2048.f;
		 tmp3=((*(pt+3)-offset)) / 2048.f;
		 tmp4=((*(pt+2)-offset)) / 2048.f;
		 */
#ifdef TEST_SINGLE_ADC
		tmp2=((*(pt+1)-offset)) / 2048.f;
		tmp1 = tmp2;
		tmp4=((*(pt+3)-offset)) / 2048.f;
		tmp3 = tmp4;
#else
		tmp2=((*(pt+1)-offset)) / 2048.f;
		tmp1=((*(pt)  -offset)) / 2048.f;
		tmp4=((*(pt+3)-offset)) / 2048.f;
		tmp3=((*(pt+2)-offset)) / 2048.f;
#endif


		*zeroIF_R++ = *LOR++ * tmp1;  *zeroIF_I++ = *LOI++ * tmp1;
		*zeroIF_R++ = *LOR++ * tmp2;  *zeroIF_I++ = *LOI++ * tmp2;
		*zeroIF_R++ = *LOR++ * tmp3;  *zeroIF_I++ = *LOI++ * tmp3;
		*zeroIF_R++ = *LOR++ * tmp4;  *zeroIF_I++ = *LOI++ * tmp4;
		pt += 4;
		blkCnt--;
	}	
}
//---------------------------------------------------------------------------------------
// Convert back from floating point to short words, applying the volume setting
void SDR_float_to_DAC_audio(float *pSrc, short *pDst, uint16_t blockSize)
{
	uint16_t blkCnt;            // loop counter
	short *AudioBuffer;

	AudioBuffer = pDst;

	/* loop Unrolling */
	blkCnt = blockSize >> 2u;   // loop unrolling.  Compute 4 outputs at a time
	while(blkCnt--)
	{
		*pDst++ = (short)(2048.f + *pSrc++ * 2048.f * RXVolume);
		*pDst++ = (short)(2048.f + *pSrc++ * 2048.f * RXVolume);
		*pDst++ = (short)(2048.f + *pSrc++ * 2048.f * RXVolume);
		*pDst++ = (short)(2048.f + *pSrc++ * 2048.f * RXVolume);
	}	

	// SCB_Clean because is from RAM to DMA. Invalidate is for DMA to RAM
#ifdef USE_DCACHE
	SCB_CleanDCache_by_Addr((uint32_t *) AudioBuffer, 4 * blockSize);
#endif
	return;
}	
//---------------------------------------------------------------------------------------
// Optimized version of the memcpy routine
void SDR_memcpy_f32(float* pDst, float* pSrc, uint32_t blockSize)
{
	uint32_t blkCnt;            /* loop counter */

	float32_t in1, in2, in3, in4;

	// loop Unrolling
	blkCnt = blockSize >> 2u;

	// Compute 4 outputs at a time.
	while(blkCnt > 0u)
	{
		/* Copy and then store the results in the destination buffer */
		in1 = *pSrc++;  *pDst++ = in1;
		in2 = *pSrc++;  *pDst++ = in2;
		in3 = *pSrc++;  *pDst++ = in3;
		in4 = *pSrc++;  *pDst++ = in4;

		// Decrement the loop counter
		blkCnt--;
	}
}
//---------------------------------------------------------------------------------------
// Mirror the upper half (LSB) into the lower half (USB) of the complex buffer	
void  SDR_mirror_LSB(float* buf, uint32_t blockSize)
{
	uint32_t blkCnt;            /* loop counter */
	float *pbR, *pbI, *peR, *peI;

	// loop Unrolling */
	blkCnt = blockSize >> 3u;  // divide by 8, as the mirroring stops at half the buffer...
	blkCnt--;                  // minus 1, as the DC term is skipped

	pbR = buf+2;  pbI = buf+3; peR = buf + blockSize*2 - 2; peI = buf + blockSize*2 - 1;

	//  Compute 4 outputs at a time.
	while(blkCnt--)
	{
		*pbR = *peR; *pbI = -*peI; pbR+=2; pbI+=2; peR-=2; peI-=2;
		*pbR = *peR; *pbI = -*peI; pbR+=2; pbI+=2; peR-=2; peI-=2;
		*pbR = *peR; *pbI = -*peI; pbR+=2; pbI+=2; peR-=2; peI-=2;
		*pbR = *peR; *pbI = -*peI; pbR+=2; pbI+=2; peR-=2; peI-=2;
	}
}
// ------------------------------------------------------
// Compute the parameters for the double IIR filter used for the narrow CW mode
void SDR_compute_IIR_parms(void)
{
	float r, r2, wr, cosw0;

#ifdef CIC_DECIMATE_64
	float rate = SamplingRate/256; //SamplingRate / decimation
#endif
#ifdef CIC_DECIMATE_16
	float rate = SamplingRate/64; //SamplingRate / decimation
#endif
	r = Qfactor;

	a1 = a2 = b0 = 0.f;
	r2 = r*r;
	wr = 2.f * cwpitch / rate * myPI;
	cosw0 = (2.f * r / (1.f + r2)) * cos(wr); // resonance frequency correction
	// (see the Proakis & Manolakis book)
	a1 = -2.f * r * cosw0;
	a2 = r2;
	// b0 is normalized for gain ~ 2dB on all the band
	b0 = 1.2f * (1.f - r) * sqrt(1.f + r2 - 2.f * r * cos(2.f*wr));
}
// ------------------------------------------------------
// Double IIR resonator with two poles at wr e -wr. Used for the narrow CW mode
void SDR_CWPeak(float *buf, uint32_t blockSize)
{
	static float y1a=0.f, y2a=0.f, y1b=0.f, y2b=0.f;
	register float x0, y0;
	uint32_t blkCnt = blockSize >> 2u;       /* loop counter */

	// Compute 4 outputs at a time, loop unrolled for performance
	while(blkCnt--)
	{
		x0 = *buf;
		y0 = -a1 * y1a - a2 * y2a + b0 * x0;
		y2a = y1a;
		y1a = y0;
		y0 = -a1 * y1b - a2 * y2b + b0 * y0;
		y2b = y1b;
		y1b = y0;
		*buf++ = y0;

		x0 = *buf;
		y0 = -a1 * y1a - a2 * y2a + b0 * x0;
		y2a = y1a;
		y1a = y0;
		y0 = -a1 * y1b - a2 * y2b + b0 * y0;
		y2b = y1b;
		y1b = y0;
		*buf++ = y0;

		x0 = *buf;
		y0 = -a1 * y1a - a2 * y2a + b0 * x0;
		y2a = y1a;
		y1a = y0;
		y0 = -a1 * y1b - a2 * y2b + b0 * y0;
		y2b = y1b;
		y1b = y0;
		*buf++ = y0;

		x0 = *buf;
		y0 = -a1 * y1a - a2 * y2a + b0 * x0;
		y2a = y1a;
		y1a = y0;
		y0 = -a1 * y1b - a2 * y2b + b0 * y0;
		y2b = y1b;
		y1b = y0;
		*buf++ = y0;
	}
}
// ------------------------------------------------------

#ifdef RECEIVE_AM

// AM demodulation with AGC
void SDR_demodAM_AGC(float32_t * tmpSamp, float32_t * fAudio)
{
	static float wold = 0.f;
	volatile 	float        w, tmp;
	int          k, j;

	// AM demodulation, compute the magnitude taking the data from the right edge
	// of the buffer after the inverse FFT and the overlap-and-discard algorithm

	for(k=j=0; k<BSIZE*2; k+=2)
	{
		tmp = tmpSamp[k]*tmpSamp[k] + tmpSamp[k+1]*tmpSamp[k+1];
		arm_sqrt_f32(tmp, &audiotmp);      // implement also the AM demod

		if(pk < audiotmp)
		{
			pk = audiotmp;
			hangcnt = Hcount[AM];
		}

		audiotmp /= max(pk, AgcThreshold);

		if(hangcnt == 0)
			pk  *= Decay[AM];

		// DC removal filter -----------------------
		w = audiotmp + wold * 0.96f; // increasing this constant gives more bass response...
		fAudio[j++] = w - wold;      // .... but keep it ALWAYS less than 1 ....  !
		wold = w;
		// -----------------------------------------
	}
	PeakAudioValue=pk;
	if(hangcnt > 0)  hangcnt--;
}

#endif

//---------------------------------------------------------------------------
// SSB and CW demodulation with AGC
void SDR_demodSSB_CW_AGC(float32_t * tmpSamp, float32_t * fAudio)
{
	static float sav;
	float        tmp;
	int          k, j;


	// SSB or CW demodulation, compute the audio taking the data from the real part of
	// the right edge of the buffer after the inverse FFT and the overlap-and-discard algorithm

	for(k=j=0; k<BSIZE*2; k+=2)
	{
		tmp = tmpSamp[k]*tmpSamp[k] + tmpSamp[k+1]*tmpSamp[k+1];
		arm_sqrt_f32(tmp, &sav);

#ifdef CW_DECODER
		if ((sav) > CWLevel)
			CWLevel= (sav);
#endif


		if(pk < sav)
		{
			pk = sav;
			if(CurrentMode == CW) hangcnt = Hcount[CW];
			else
				hangcnt = (CurrentMode == LSB) ? Hcount[LSB] : Hcount[USB];
		}

		// take just the real part and that is the demodulated audio
		fAudio[j++] = tmpSamp[k] / max(pk, AgcThreshold);

		if(hangcnt == 0)
		{
			if(CurrentMode == CW) pk  *= Decay[CW];
			else
				pk  *= (CurrentMode == LSB) ? Decay[LSB] : Decay[USB];
		}
	}
	PeakAudioValue=pk;
	if(hangcnt > 0)  hangcnt--;
}


// Modified Bessel function of the 0th kind, used by the Kaiser window
float i0(float const x){
	const float t = 0.25 * x * x;
	float sum = 1 + t;
	float term = t;
	for(int k=2;k<40;k++){
		term *= t/(k*k);
		sum += term;
		if(term < 1e-12 * sum)
			break;
	}
	return sum;
}

// Compute an entire Kaiser window
// More efficient than repeatedly calling kaiser(n,M,beta)
int make_kaiser(float * const window,unsigned int const M,float const beta){
	assert(window != NULL);
	if(window == NULL)
		return -1;
	// Precompute unchanging partial values
	float const numc = M_PI * beta;
	float const inv_denom = 1. / i0(numc); // Inverse of denominator
	float const pc = 2.0 / (M-1);

	// The window is symmetrical, so compute only half of it and mirror
	// this won't compute the middle value in an odd-length sequence
	for(int n = 0; n < M/2; n++){
		float const p = pc * n  - 1;
		window[M-1-n] = window[n] = i0(numc * sqrtf(1-p*p)) * inv_denom;
	}
	// If sequence length is odd, middle value is unity
	if(M & 1)
		window[(M-1)/2] = 1; // The -1 is actually unnecessary

	return 0;
}



void SetMask(float low, float high)
{

	int const L = BSIZE;
	int const M = BSIZE + 1;

	int const N = L + M - 1;
	// Window shape factor for Kaiser window
	// Transition region is approx sqrt(1+Beta^2)
	float Kaiser_beta = 3.0;
	float kaiser_window[M];

	//TODO controllare
	for(int n=0;n<N;n++){
		float f;
		if(n <= N/2)
			f = (float)n / N;
		else
			f = (float)(n-N) / N;
		if(f >= low / AudioRate && f <= high /AudioRate)
			FFTmask[n * 2] = 1024;
		else
			FFTmask[n * 2] = 0;
		FFTmask[n * 2 + 1] = 0; // Imaginary part
	}


	// compute now the inverse FFT
	arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFTmask, INVERSEFFT, NOREVERSE);
	make_kaiser(kaiser_window,M,Kaiser_beta); //TODO: integrate coefficient in window shaping loop



	// Round trip through FFT/IFFT scales by N
	float const gain = 1./N;
	// Shift to beginning of buffer to make causal; apply window and gain
	for(int n = M - 1; n >= 0; n--){
		FFTmask[n * 2] = FFTmask[((n-M/2+N)%N) * 2] * kaiser_window[n] * gain;
	//	FFTmask[n * 2 + 1] = FFTmask[((n-M/2+N)%N) * 2 + 1];
		FFTmask[n * 2 + 1] = FFTmask[((n-M/2+N)%N) * 2 + 1] * kaiser_window[n] * gain;
	}
	// Pad with zeroes on right side
	memset(FFTmask + M * 2,0,(N-M)* 2 * sizeof(*FFTmask));


	// Now back to frequency domain
	arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFTmask, DIRECTFFT, NOREVERSE);


#if 0
	void SDR_2R_toC_f32(float * pSrcA, float * pSrcB, float * pDst, uint32_t blockSize)
	{
		uint32_t blkCnt;           /* loop counter */

		// loop Unrolling
		blkCnt = blockSize >> 2u;

		// Compute 4 outputs at a time
		while(blkCnt)
		{
			*pDst++ = *pSrcA++;  *pDst++ = *pSrcB++;
			*pDst++ = *pSrcA++;  *pDst++ = *pSrcB++;
			*pDst++ = *pSrcA++;  *pDst++ = *pSrcB++;
			*pDst++ = *pSrcA++;  *pDst++ = *pSrcB++;

			blkCnt--;
		}
	}
#endif
}
//---------------------------------------------------------------------------
