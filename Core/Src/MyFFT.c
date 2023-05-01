/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_cfft_f32.c
 * Description:  Combined Radix Decimation in Frequency CFFT Floating point processing function
 *
 * $Date:        18. March 2019
 * $Revision:    V1.6.0
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2019 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "arm_math.h"
#include "arm_common_tables.h"

extern void arm_radix8_butterfly_f32(
        float32_t * pSrc,
        uint16_t fftLen,
  const float32_t * pCoef,
        uint16_t twidCoefModifier);

extern void arm_bitreversal_32(
        uint32_t * pSrc,
  const uint16_t bitRevLen,
  const uint16_t * pBitRevTable);


/**
  @brief         Processing function for the floating-point complex FFT.
  @param[in]     S              points to an instance of the floating-point CFFT structure
  @param[in,out] p1             points to the complex data buffer of size <code>2*fftLen</code>. Processing occurs in-place
  @param[in]     ifftFlag       flag that selects transform direction
                   - value = 0: forward transform
                   - value = 1: inverse transform
  @param[in]     bitReverseFlag flag that enables / disables bit reversal of output
                   - value = 0: disables bit reversal of output
                   - value = 1: enables bit reversal of output
  @return        none
 */

void My_arm_cfft_f32(
  const arm_cfft_instance_f32 * S,
        float32_t * p1,
        uint8_t ifftFlag,
        uint8_t bitReverseFlag)
{
  uint32_t  L = S->fftLen, l;
  float32_t invL, * pSrc;

  if (ifftFlag == 1U)
  {
    /* Conjugate input data */
    pSrc = p1 + 1;
    for (l = 0; l < L; l++)
    {
      *pSrc = -*pSrc;
      pSrc += 2;
    }
  }

  switch (L)
  {
  case 16:
  case 128:
  case 1024:
    arm_cfft_radix8by2_f32 ( (arm_cfft_instance_f32 *) S, p1);
    break;
  case 32:
  case 256:
  case 2048:
    arm_cfft_radix8by4_f32 ( (arm_cfft_instance_f32 *) S, p1);
    break;
  case 64:
  case 512:
  case 4096:
    arm_radix8_butterfly_f32 ( p1, L, (float32_t *) S->pTwiddle, 1);
    break;
  }

  if ( bitReverseFlag )
    arm_bitreversal_32 ((uint32_t*) p1, S->bitRevLength, S->pBitRevTable);

  if (ifftFlag == 1U)
  {
    invL = 1.0f / (float32_t)L;

    /* Conjugate and scale output data */
    pSrc = p1;
    for (l= 0; l < L; l++)
    {
      *pSrc++ *=   invL ;
      *pSrc    = -(*pSrc) * invL;
      pSrc++;
    }
  }
}



