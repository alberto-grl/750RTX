/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : dataAcqPerf.h
* Author             : MCD Tools Development
* Version            : V1.0
* Date               : 13/04/2011
* Description        : This file provides a mechanism for software data trace
*                      performance measurement.
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef __DATAACQ_PERF_H__
#define __DATAACQ_PERF_H__

#ifdef MEASURE_PERFORMANCE

void InitTraceGpio(void);
void SetTraceGpio(void);
void ResetTraceGpio(void);

#endif
#endif
