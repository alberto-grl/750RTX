/**
  ******************************************************************************
  * @file    acqTrigger.h
  * @author  MCD Tools Development
  * @brief   This file provides trigger mechanism for software data trace from
  *          the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#ifndef ACQTRIGGER_H
#define ACQTRIGGER_H

#include "dataAcq.h"

#ifdef USING_TRIGGER /* Modify dataAcq.h in order to get rid of trigger */

/* Return values for AcqEvaluateTrigger */
#define TRIG_EVAL_NO_REC   0  /* Do not record */
#define TRIG_EVAL_POST_REC 1  /* Post-trigger record (committed) */
#define TRIG_EVAL_EVENT    2  /* Trig event just occurred */
#define TRIG_EVAL_PRE1_REC 3  /* Waiting for empty buffer for pre-trigger start */
#define TRIG_EVAL_PRE2_REC 4  /* Pre-trigger record (not committed before trig event) */

/* Interpret the trigger part of the snapshot trace header. */
/* To call at least once by recording session */
void AcqConfigureTrigger(__IO TraceHeaderTrigger_t *pTrigConfig);

/* Main trigger state machine, to call periodically. Returns 1 when:
 - no trigger is configured
 - a trigger was configured, started and the condition is hit.trigger
 Returns 0 otherwise. */
int32_t AcqEvaluateTrigger(void);

/* Inform the trigger module about internal buffer overflow, that may be used
 as stop condition (0: no overflow; !=0: overflow) */
void SetTriggerOverflow(int32_t overflow);

/* Inform the trigger module that a new record is being traced (useful for stop after N) */
void AddingNewRecord(void);

/* Inform the trigger module that the host has read some data */
void HostReadEvent(void);

/* Rearm the trigger */
void AutoRearmTrigger(void);

#endif /* USING_TRIGGER */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#endif /* ACQTRIGGER_H */
