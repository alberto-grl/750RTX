/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : acqTrigger.h
* Author             : MCD Tools Development
* Version            : V1.0
* Date               : 03/06/2011
* Description        : This file provides trigger mechanism for software data
*                      trace from the application.
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef __ACQTRIGGER_H__
#define __ACQTRIGGER_H__

#include "dataAcq.h"

#ifdef USING_TRIGGER // Modify dataAcq.h in order to get rid of trigger

// Return values for AcqEvaluateTrigger
#define TRIG_EVAL_NO_REC   0  // Do not record
#define TRIG_EVAL_POST_REC 1  // Post-trigger record (committed)
#define TRIG_EVAL_EVENT    2  // Trig event just occurred
#define TRIG_EVAL_PRE1_REC 3  // Waiting for empty buffer for pre-trigger start
#define TRIG_EVAL_PRE2_REC 4  // Pre-trigger record (not committed before trig event)

// Interpret the trigger part of the snapshot trace header.
// To call at least once by recording session
void AcqConfigureTrigger(volatile TraceHeaderTriggerT *pTrigConfig);

// Main trigger state machine, to call periodically. Returns 1 when:
// - no trigger is configured
// - a trigger was configured, started and the condition is hit.trigger
// Returns 0 otherwise.
int AcqEvaluateTrigger(void);

// Inform the trigger module about internal buffer overflow, that may be used
// as stop condition (0: no overflow; !=0: overflow)
void SetTriggerOverflow(int overflow);

// Inform the trigger module that a new record is being traced (useful for stop after N)
void AddingNewRecord(void);

// Inform the trigger module that the host has read some data
void HostReadEvent(void);

#endif // USING_TRIGGER

#endif
