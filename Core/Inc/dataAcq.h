/**
  ******************************************************************************
  * @file    dataAcq.h
  * @author  MCD Tools Development
  * @brief   This file provides a mechanism for software data trace from the
  *          application.
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


#ifndef DATAACQ_H
#define DATAACQ_H

#include <stdint.h>
#include <math.h>

/* Put the following line under comment in order to get rid of triggers */
#define USING_TRIGGER

/* Following line activates the code managing the trigger evaluation on a float
   variable. Uses a floating point library. Remove the comment in order to activate. */
#define USING_TRIGGER_ON_FLOAT

/* Following line activates the code managing the trigger evaluation on a double
   variable (STM32 only). Uses a floating point library. Remove the comment in
   order to activate. */
#define USING_TRIGGER_ON_DOUBLE

/* Maximum number of variables in one record.
   It defines the size of the variable description header (statically allocated),
   not the real number of traced variables (that may be less).
   Note also that a variable of type "double" takes 2 TraceHeaderField_t elements
   in the data buffer. */
#define SNP_TRC_NB_MAX_WORD_VAR 10

#define     __IO    volatile             /*!< Defines 'read / write' volatile permissions */

/* Type for trace header alignment: each field of the trace header must have this
   type. Note that "double" type variables are made with 2 TraceHeaderField_t. */
typedef uint32_t TraceHeaderField_t;

/* Type for the trace buffer contents (timestamp+var values) */
typedef uint32_t TraceBufferField_t;


/******************************************************************************/
/* Description of the variables to be traced */

#define SNP_TRC_DATA_ACCESS_8BIT  0 /* signed or unsigned for acquisition; signed only for trigger */
#define SNP_TRC_DATA_ACCESS_16BIT 1 /* signed or unsigned for acquisition; signed only for trigger */
#define SNP_TRC_DATA_ACCESS_32BIT 2 /* signed or unsigned for acquisition; signed only for trigger */
#define SNP_TRC_DATA_ACCESS_64BIT 3 /* For management of double */
#define SNP_TRC_DATA_ACCESS_TRIG_U8  4 /* For trigger only: unsigned; note that bit2 discriminates */
#define SNP_TRC_DATA_ACCESS_TRIG_U16 5 /* For trigger only: unsigned; note that bit2 discriminates */
#define SNP_TRC_DATA_ACCESS_TRIG_U32 6 /* For trigger only: unsigned; note that bit2 discriminates */
#define SNP_TRC_DATA_ACCESS_TRIG_FLOAT  7 /* For trigger only */
#define SNP_TRC_DATA_ACCESS_TRIG_DOUBLE 8 /* For trigger only */

typedef TraceHeaderField_t DataTraceAccessType_t;
typedef TraceHeaderField_t DataTraceAddress_t;
typedef struct
{
  DataTraceAccessType_t accessType;
  DataTraceAddress_t address;
} DataTraceVarMap_t;


/******************************************************************************/
/* Trace header structure */

/* Flag definitions for DataTraceHeaderT.headerVersion */
#define SNP_TRC_HEADER_VERSION_MASK               (0x00FF)
#define SNP_TRC_HEADER_SUPPORTS_TRIGGER           (0x0100)
#define SNP_TRC_HEADER_SUPPORTS_TRIGGER_ON_FLOAT  (0x0200)
#define SNP_TRC_HEADER_SUPPORTS_TRIGGER_ON_DOUBLE (0x0400)

/* Flag definitions for DataTraceHeaderT.flags */
#define SNP_TRC_FLAG_OVERFLOW      1
#define SNP_TRC_START_STOP_ACK     2
#define SNP_TRC_RECORD_SKIPPED     4

/* Flag definitions for DataTraceHeaderT.TraceHeaderTriggerT.cmd */
#define SNP_TRC_TRIGGER_STOP     0
#define SNP_TRC_TRIGGER_START    1

/* Flag definitions for DataTraceHeaderT.TraceHeaderTriggerT.mode
   bit[3:0]: start condition
   bit[7:4]: stop condition */
#define SNP_TRC_TRIGGER_START_MASK       0x0F
#define SNP_TRC_TRIGGER_START_IMMEDIATE  0x00
#define SNP_TRC_TRIGGER_RISING           0x01
#define SNP_TRC_TRIGGER_FALLING          0x02
#define SNP_TRC_TRIGGER_STOP_MASK        0x70
#define SNP_TRC_TRIGGER_AUTO_RESTART     0x10
#define SNP_TRC_TRIGGER_STOP_OVF         0x20
#define SNP_TRC_TRIGGER_STOP_N_REC       0x40
#define SNP_TRC_TRIGGER_RST_TIMESTAMP    0x80 /* Restart timestamp on each trig event */

/* Flag definitions for DataTraceHeaderT.TraceHeaderTriggerT.state */
#define SNP_TRC_TRIGGER_STOPPED     0 /* Init state, waiting for start cmd */
#define SNP_TRC_TRIGGER_STARTED     1 /* Trigger started, waiting for the configured event */
#define SNP_TRC_TRIGGER_TRIGGED     2 /* Event hit; waiting for next cmd (start or stop) */

/* Predefined values for timestamp base unit */
#define SNP_TRC_SECONDS       15
#define SNP_TRC_MILLI_SECONDS 14
#define SNP_TRC_MICRO_SECONDS 13
#define SNP_TRC_NANO_SECONDS  12

typedef struct
{
  /* Fields that may be modified by the host are declared volatile */
  __IO TraceHeaderField_t address;     /* Address of variable for trigger */
  __IO TraceHeaderField_t accessType;  /* Type of variable for trigger */
  __IO TraceHeaderField_t threshold;   /* Trigger threshold */
  __IO TraceHeaderField_t mode;        /* Trigger mode */
  __IO TraceHeaderField_t cmd;         /* Trigger command */
  __IO TraceHeaderField_t stopParam;   /* Optional parameter for end condition */
  TraceHeaderField_t state;                /* Trigger state */
  __IO TraceHeaderField_t nPreTrig;    /* Number of records to keep before the trigger. 0 by default (no pretrig) */
  __IO TraceHeaderField_t bIgnoreTrig;  /* Ignore trigger when buffer not empty */
#ifdef USING_TRIGGER_ON_DOUBLE
  __IO TraceHeaderField_t thresholdDouble;  /* First word (low address part) of the threshold expressed as double */
  __IO TraceHeaderField_t thresholdDouble2; /* Second word (high address part) of the threshold expressed as double */
#endif /* USING_TRIGGER_ON_DOUBLE */
} TraceHeaderTrigger_t;

typedef struct
{
  /* Static fields (affected at compilation time) */
  uint8_t startMark[3];                 /* Unchanging marker used for robustness */
  uint8_t headerFieldSize;              /* Size in bytes of following fields in the header */
  TraceHeaderField_t headerVersion;   /* Identifier of the header format */
  TraceHeaderField_t bufferStartAddr; /* First (low) address of memory area reserved for trace data */
  TraceHeaderField_t bufferEndAddr;   /* Last (high) address of memory area reserved for trace data */
  TraceHeaderField_t nbVarMax;        /* Maximum number of variables (32bits words) in one record */
  TraceHeaderField_t bufferFormat;    /* Version identifier of buffer format */
  TraceHeaderField_t timestamp_base_unit;    /* Period between 2 consecutive calls to Dumptrace */
  /* Control fields (used at run time) - volatile because some may be written by the host PC */
  __IO TraceHeaderField_t writePointer;     /* Position in the trace buffer where to put the next record */
  __IO TraceHeaderField_t writePointerCopy; /* Do a copy as basic security for shared area */
  __IO TraceHeaderField_t readPointer;      /* Position in the trace buffer of the last fully read record */
  __IO TraceHeaderField_t readPointerCopy;  /* Do a copy as basic security for shared area */
  __IO TraceHeaderField_t flags;            /* Shared status flags (overflow) */
  __IO TraceHeaderField_t subSamplingRate;  /* Recording rate 1=always; 2 = 50%, 3=33%, ... */
  __IO TraceHeaderField_t nbVar;            /* Number of variables (32bits words) currently being traced */
  /* Variables description - volatile because written by the host PC */
  __IO DataTraceVarMap_t g_varList[SNP_TRC_NB_MAX_WORD_VAR];
#ifdef USING_TRIGGER
  /* Trigger description */
  __IO TraceHeaderTrigger_t trigger;
#endif /* USING_TRIGGER */
} DataTraceHeader_t;


/* Defines for variable placement out of the 256 bytes 0 page */
#if defined(__CSMC__) /* Cosmic STM8 */
#define _NEAR_DATA_ @near
#define _FAR_DATA_ @far
#elif defined(__RAISONANCE__) /* Raisonance STM8 */
#define _NEAR_DATA_ data
#define _FAR_DATA_
#elif defined(__IAR_SYSTEMS_ICC__) /* IAR STM8 and STM32 */
#define _NEAR_DATA_
#if __ICCARM__ /* STM32 */
#define _FAR_DATA_
#else /* STM8 */
#define _FAR_DATA_ __far
#endif /* STM32 STM8 */
#elif defined(__ARMCC_VERSION) /* Keil STM32 */
#define _NEAR_DATA_
#define _FAR_DATA_
#elif defined ( __GNUC__ ) /* STM32CubeIDE / GNU Compiler */
#define _NEAR_DATA_
#define _FAR_DATA_
#endif


/******************************************************************************/
/* Exported routine for trigger implementation */
int32_t IsBufferEmpty(void);

/******************************************************************************/
/* Exported routines for acquisition in the embedded */

/* Reset the data trace buffer to a clean state. Reset timestamp counters */
void ClearBuffer(void);

#if (defined(__CSMC__) || defined(__RAISONANCE__) )
/* Workaround against bad initialization of pointers into a structure by some
   compiler versions */
void InitHeader(void);
#endif /* (defined(__CSMC__) || defined(__RAISONANCE__) ) */

void DumpTrace(void);

#endif /* DATAACQ_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
