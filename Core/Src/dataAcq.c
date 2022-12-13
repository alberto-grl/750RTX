/**
  ******************************************************************************
  * @file    dataAcq.c
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


#include "string.h" /* For memset */
#include "dataAcq.h"

#ifdef USING_TRIGGER
#include "acqtrigger.h"
#endif /* USING_TRIGGER */

#ifdef MEASURE_PERFORMANCE
#include "dataAcqPerf.h"
#endif /* MEASURE_PERFORMANCE */

/* Define the period between consecutive calls to DumpTrace */
/* Select the unit ... */
#define SNP_TRC_TIMESTAMP_BASE_UNIT SNP_TRC_MILLI_SECONDS  /* Either SNP_TRC_SECONDS, SNP_TRC_MILLI_SECONDS, 
SNP_TRC_MICRO_SECONDS, SNP_TRC_NANO_SECONDS */
/* ... and value in range [1;1000] */
#define SNP_TRC_TIMESTAMP_VALUE 25

#if (SNP_TRC_TIMESTAMP_VALUE<1) || (SNP_TRC_TIMESTAMP_VALUE>1000)
#error "Bad value for timestamp base"
#endif /* (SNP_TRC_TIMESTAMP_VALUE<1) || (SNP_TRC_TIMESTAMP_VALUE>1000) */

/* Trace buffer size in TraceBufferField_t words; recommended to be a multiple of the trace
 record size for optimal use of the memory (but this depends on nbVar ...).
 Note that if the buffer is able to contain N records, the implementation below considers
 that the buffer is full (SNP_TRC_FLAG_OVERFLOW) before writing the last record [in order
 to reduce the code complexity]. As a result the overflow occurs with (N-1) records, and
 the buffer has to contain at least 2 records. [defining more than 2 is strongly recommended]. */
#define SNP_TRC_BUFFER_SIZE (50*(SNP_TRC_NB_MAX_WORD_VAR+1)) /* +1 for timestamp when SNP_TRC_HEADER_VERSION==1 */


/* Trace header format
 V1: - Starts with a 3-bytes marker + 1 byte giving the size of following fields
     - 11 description fields, including the max number of variables (N)
     - 2*N variable description fields
 V2: Added support for triggers:
     - 6 additional fields at the end
 V3: Evolution from V1 (no trigger):
     - Added subSamplingRate
 V4: Added support for preTrigger (evolution from V2+V3):
     - V2 + V3 + 1 additional field at the end
 V5: - V4 + 1 additional field at the end to ignore trigger
 V6: Evolution from V3 (no trigger):
     - Added timestamp_base_unit field
 V7: Evolution from V5 (trigger):
     - Added timestamp_base_unit field
 V8: Evolution from V6 (no trigger) and V7 (trigger):
     - Changed DataTraceHeaderT.nbVar field format for management of double
     - Changed DataTraceHeaderT.headerVersion field format: version is limited
       to the 8 LSbits. 8 next bits are dedicated to optionnal functionalities;
       16 remaining bits are equals to 0 (used for endianess verification from
       the host) */

#define BASIC_VER1 8

#ifdef USING_TRIGGER  /* Modify dataAcq.h in order to get rid of trigger */
#define BASIC_VER2 (BASIC_VER1+SNP_TRC_HEADER_SUPPORTS_TRIGGER)
#else
#define BASIC_VER2 (BASIC_VER1)
#endif /* USING_TRIGGER */

#ifdef USING_TRIGGER_ON_FLOAT  /* Modify dataAcq.h in order to get rid of trigger on float */
#define BASIC_VER3 (BASIC_VER2+SNP_TRC_HEADER_SUPPORTS_TRIGGER_ON_FLOAT)
#else
#define BASIC_VER3 (BASIC_VER2)
#endif /* USING_TRIGGER_ON_FLOAT */

#ifdef USING_TRIGGER_ON_DOUBLE  /* Modify dataAcq.h in order to get rid of trigger on double */
#define SNP_TRC_HEADER_VERSION (BASIC_VER3+SNP_TRC_HEADER_SUPPORTS_TRIGGER_ON_DOUBLE)
#else
#define SNP_TRC_HEADER_VERSION (BASIC_VER3)
#endif /* USING_TRIGGER_ON_DOUBLE */

/* Trace buffer format:
 V1: A trace record is made with:
      - one start-timestamp (32bits) as first element (= simple index currently).
      - N values of variables (user defined addresses). Each variable own a 32bits
        wide element in the record. 8 or 16 bits variables are stored in LSB.
    The trace buffer contains contiguous trace records. */
#define SNP_TRC_BUFFER_FORMAT 1

/* The data trace buffer that will be read by the host */
_NEAR_DATA_ static TraceBufferField_t g_dataTraceBuffer[SNP_TRC_BUFFER_SIZE];
_NEAR_DATA_ static __IO DataTraceHeader_t g_traceHeader =
{
  /* Static fields (affected at compilation time) */
  { 0xA7, 0x25, 0x8F},                          /* startMark[3]: Unchanging marker used for robustness */
  sizeof(TraceHeaderField_t),                 /* Size in bytes of following fields in the header */
  SNP_TRC_HEADER_VERSION,                    /* headerVersion:    Identifier of the header format */
#if (defined(__CSMC__) || defined(__RAISONANCE__) )
  /* Non constant initializers are pushing compilers to their limit (for instance
   Cosmic 4.3.4 that wrongly optimizes the structure field, or Raisonance 2.30.10
   that refuses some expressions. Initializing at runtime is more secure. */
  0, /* Correct value will be initialized at runtime */
  0, /* Correct value will be initialized at runtime */
#else
  (TraceHeaderField_t) &g_dataTraceBuffer,   /* bufferStartAddr:  First (low) address of memory area reserved for
                                               trace data */
  (TraceHeaderField_t)(&g_dataTraceBuffer[SNP_TRC_BUFFER_SIZE]) - 1, /* bufferEndAddr:    Last (high) address of memory
                                                                       area reserved for trace data */
#endif /* (defined(__CSMC__) || defined(__RAISONANCE__) ) */
  SNP_TRC_NB_MAX_WORD_VAR, /* nbVarMax:         Maximum number of variables (32 or 64bits words) in one record */
  SNP_TRC_BUFFER_FORMAT,   /* bufferFormat:     Version identifier of buffer format */
  (1 << SNP_TRC_TIMESTAMP_BASE_UNIT) + SNP_TRC_TIMESTAMP_VALUE,

  /* Control fields (used at run time)*/
#if (defined(__CSMC__) || defined(__RAISONANCE__) )
  /* Non constant initializers are pushing compilers to their limit (for instance
   Cosmic 4.3.4 that wrongly optimizes the structure field, or Raisonance 2.30.10
   that refuses some expressions. Initializing at runtime is more secure. */
  0, /* Correct value will be initialized at runtime */
  0, /* Correct value will be initialized at runtime */
  0, /* Correct value will be initialized at runtime */
  0, /* Correct value will be initialized at runtime */
#else
  (TraceHeaderField_t) &g_dataTraceBuffer, /* writePointer: Position in the trace buffer where to put the next record */
  (TraceHeaderField_t) &g_dataTraceBuffer, /* writePointerCopy: Do a copy as basic security for shared area */
  (TraceHeaderField_t)(&g_dataTraceBuffer[SNP_TRC_BUFFER_SIZE] - 1), /* readPointer:   Position in the trace buffer of
                                                                       the last fully read record */
  (TraceHeaderField_t)(&g_dataTraceBuffer[SNP_TRC_BUFFER_SIZE] - 1), /* readPointerCopy:  Do a copy as basic security
                                                                       for shared area */
  /* CAUTION: this initial value is not aligned with a start of record address; the effective value has to be computed
      later in order to correctly detect the first buffer overflow */
#endif /* (defined(__CSMC__) || defined(__RAISONANCE__) ) */
  0,    /* flags:            Shared status flags (overflow) */
  1,    /* subSamplingRate:  Recording rate 1=always; 2 = 50%, 3=33%, ... */
  0,    /* nbVar.LSW:        Number of variables (32 or 64 bits words) currently being traced */
  /* nbVar.MSW:        Number of double variables (64 bits words) currently being traced */

  /* Variables description */
  {     /* g_varList[SNP_TRC_NB_MAX_WORD_VAR]; */
    {SNP_TRC_DATA_ACCESS_32BIT, 0},
    {SNP_TRC_DATA_ACCESS_32BIT, 0},
    {SNP_TRC_DATA_ACCESS_32BIT, 0},
    {SNP_TRC_DATA_ACCESS_32BIT, 0},
    {SNP_TRC_DATA_ACCESS_32BIT, 0},
    {SNP_TRC_DATA_ACCESS_32BIT, 0},
    {SNP_TRC_DATA_ACCESS_32BIT, 0},
    {SNP_TRC_DATA_ACCESS_32BIT, 0},
    {SNP_TRC_DATA_ACCESS_32BIT, 0},
    {SNP_TRC_DATA_ACCESS_32BIT, 0}
  }
#ifdef USING_TRIGGER
  /* Trigger description */
  , {
    0,                               /* Address of variable for trigger */
    SNP_TRC_DATA_ACCESS_TRIG_U32,    /* Type of variable for trigger */
    0,                               /* Trigger threshold */
    0,                               /* Trigger mode default: immediate start, never stop */
    0,                               /* Trigger command */
    0,                               /* stopParam */
    SNP_TRC_TRIGGER_STOPPED,         /* Trigger state */
    0,                               /* pretrig */
    0                                /* ignore trigger when buffer not empty */
  }
#endif /* USING_TRIGGER */
};

/* Global variables shared between several routines */
static uint32_t g_recordSize = 0; /* In bytes */
static TraceHeaderField_t g_lastReadPointer = 0;

#ifdef STM32F0XX
/* Align the variables addresses */
void alignAddresses(uint32_t nbVar);
#endif /* STM32F0XX */

#if (defined(__CSMC__) || defined(__RAISONANCE__) )
/**
  * @brief  Workaround against bad initialization of pointers into a structure
  *  by some compiler versions
  * @note   < OPTIONAL: add here global note >
  * @param  None
  * @retval None
 */
void InitHeader(void)
{
  g_traceHeader.bufferStartAddr = (TraceHeaderField_t) &g_dataTraceBuffer;
  g_traceHeader.bufferEndAddr = (TraceHeaderField_t)(&g_dataTraceBuffer[SNP_TRC_BUFFER_SIZE]) - 1;
  g_traceHeader.writePointer = (TraceHeaderField_t) &g_dataTraceBuffer;
  g_traceHeader.writePointerCopy = (TraceHeaderField_t) &g_dataTraceBuffer;
  g_traceHeader.readPointer = (TraceHeaderField_t)(&g_dataTraceBuffer[SNP_TRC_BUFFER_SIZE] - 1);
  g_traceHeader.readPointerCopy = (TraceHeaderField_t)(&g_dataTraceBuffer[SNP_TRC_BUFFER_SIZE] - 1);
}
#endif /* (defined(__CSMC__) || defined(__RAISONANCE__) ) */

typedef TraceBufferField_t(*pReadFunc)(uint32_t);

/* Prototypes of private functions */
TraceBufferField_t readVar8bits(uint32_t addr);
TraceBufferField_t readVar16bits(uint32_t addr);
TraceBufferField_t readVar32bits(uint32_t addr);

/**
  * @brief  Memory read function for 8 bits variables
  * @param  addr : variable address
  * @retval Data in buffer
  */
TraceBufferField_t readVar8bits(uint32_t addr)
{
  return (TraceBufferField_t) * (_FAR_DATA_ uint8_t *)addr;
}

/**
  * @brief  Memory read function for 16 bits variables
  * @param  addr : variable address
  * @retval Data in buffer
  */
TraceBufferField_t readVar16bits(uint32_t addr)
{
  /* Caution: if TraceBufferField_t is 8 bits, there is a truncature here */
  return (TraceBufferField_t) * (_FAR_DATA_ int16_t *)addr;
}

/**
  * @brief  Memory read functions for 32 bits variables
  * @param  addr : variable address
  * @retval Data in buffer
  */
TraceBufferField_t readVar32bits(uint32_t addr)
{
  /* Caution: if TraceBufferField_t is 8 or 16 bits, there is a truncature here */
  return (TraceBufferField_t) * (_FAR_DATA_ uint32_t *)addr;
}

/* Read function lookup table */
pReadFunc g_readFunc[3] = {readVar8bits, readVar16bits, readVar32bits};

/* Timestamp incremented at each call to DumpTrace. Starts from 0. */
static TraceBufferField_t g_timestamp = 0;
static TraceBufferField_t g_timestampOverflow = 0;

/**
  * @brief  Reset the data trace buffer to a clean state. Reset timestamp counter
  * @param  None
  * @retval None
  */
void ClearBuffer(void)
{
  memset((void *)g_dataTraceBuffer, 0, sizeof(g_dataTraceBuffer));
  g_timestamp = 0;
}

/**
  * @brief  check that buffer is empty
  * @note   This routine is called by AcqEvaluateTrigger; as a result we can use some
            global variables that are correctly set up at this stage.
            Think about it before using somewhere else.
  * @param  None
  * @retval None
  */

int32_t IsBufferEmpty(void)
{
  int32_t nextReadAddr;
  if (g_lastReadPointer + 2 * g_recordSize - 1 > g_traceHeader.bufferEndAddr)
  {
    /* There is a place for only 1 record after g_lastReadPointer; the next
     one makes the buffer loopback */
    nextReadAddr = g_traceHeader.bufferStartAddr;
  }
  else
  {
    nextReadAddr = g_lastReadPointer + g_recordSize;
  }
  if (nextReadAddr == g_traceHeader.writePointer)
  {
    /* This means the buffer is empty */
    return 1;
  }
  return 0;
}

/**
  * @brief  The data trace dump function that must be called by the application where a
            snapshot is expected. Each snapshot increments a "timestamp" that is visible
            from the host.
  * @note   Caution for the provided STM8 example: ensure that the STM8S_FAMILY or STM8L_FAMILY
            flag in stm8/common/mcuregs.h matches the connected microcontroller, in order to
             get DumpTrace properly called from the timer event.
  * @param  None
  * @retval None
  */

/* */
void DumpTrace(void)
{
  static uint8_t bOverflow = 0;
  static uint8_t bStarted = 0;
  static TraceBufferField_t *pWrite = 0;
#ifdef USING_TRIGGER
  static uint8_t bPreTrigBufOvf = 0;
#endif /* USING_TRIGGER */
  static TraceBufferField_t g_subSamplingCount = 0;
  uint32_t varIdx;
  DataTraceAccessType_t varAccessType;
  TraceHeaderField_t readPointer, readPointerCopy;
  TraceHeaderField_t recordLastAddr = (TraceHeaderField_t)(-1); /* Init value used as signal of new record */
  /* with triggers in subsampling mode */
  /*  uint32_t timeout; */
  static uint32_t nbRecords = 0; /* static because reused (instead of recomputed) by triggers */
  TraceHeaderField_t expectedInitialReadPointer;
  TraceHeaderField_t flags, writePointer;
  uint16_t nbVar, nbDoubleVar;
#ifdef USING_TRIGGER
  static uint8_t bHasTrigged = 0;
  uint32_t trigEval;
#endif /* USING_TRIGGER */

#ifdef MEASURE_PERFORMANCE
  SetTraceGpio();
#endif /* MEASURE_PERFORMANCE */

  /* Get the trace header critical section contents once and for all */
  nbVar = g_traceHeader.nbVar & 0x0000FFFF;
  nbDoubleVar = (g_traceHeader.nbVar & 0xFFFF0000) >> 16;
  flags = g_traceHeader.flags;
  writePointer = g_traceHeader.writePointer;

  if (nbVar == 0)
  {
    /* Nothing to trace */
    if (bStarted == 1)
    {
      /* Acquisition stop: acknowledge to the host for synchronization */
      bStarted = 0;
      flags = SNP_TRC_START_STOP_ACK;
      g_traceHeader.flags = flags;
    }
  }
  else
  {
    /* There is something to be traced
    The very critical field for read from the embedded is "readPointer"; to
    ensure the value read is not a transistory value, a copy is done. */
    readPointer = g_traceHeader.readPointer;
    readPointerCopy = g_traceHeader.readPointerCopy;
    if (readPointer != readPointerCopy)
    {
      /* The read pointer is being moved by the host; use the latest stable value
      instead; otherwise we may use an unpredictable value. A waiting loop may
      impact too much the user application. The worst case will lead to
      overflow (no deadlock). */
      readPointer = g_lastReadPointer;
    }
    else
    {
      /* This becomes the most recently known stable value */
#ifdef USING_TRIGGER
      if ((g_lastReadPointer != readPointer) && (bStarted == 1))
      {
        /* Inform the trigger module that the host has read some data */
        HostReadEvent();
      }
#endif /* USING_TRIGGER */
      g_lastReadPointer = readPointer;
    }

    if (bStarted == 0)
    {
      /* This is the very first acquisition: let s compute the record size and
      check that the correct value for the readPointer has been set by the host
      (mandatory for correct management of overflow) */
      g_recordSize = (1 + nbVar + nbDoubleVar) * sizeof(TraceBufferField_t); /* SNP_TRC_BUFFER_FORMAT==1 contains a
                                                                               timestamp => 1+nbVar */
      /* Each double takes 2 places */
      nbRecords = SNP_TRC_BUFFER_SIZE * sizeof(TraceBufferField_t) / g_recordSize;
      writePointer = (TraceHeaderField_t)&g_dataTraceBuffer;
      pWrite = (TraceHeaderField_t *)writePointer;
      expectedInitialReadPointer = writePointer + (nbRecords - 1) * g_recordSize;
      if (expectedInitialReadPointer == readPointer)
      {
        /* All is OK for tracing: let s start */
#ifdef STM32F0XX
        /* On Cortex M0, the LDRx instructions require to be aligned: do it now for robustness (no hard fault) */
        alignAddresses(nbVar);
#endif /* STM32F0XX */
        bStarted = 1;
        /* Reset static variables */
        bOverflow = 0;
        g_timestamp = 0;
        g_timestampOverflow = 0;
        g_subSamplingCount = 0;
#ifdef USING_TRIGGER
        bPreTrigBufOvf = 0;
        bHasTrigged = 0;
        /* Inform the trigger module */
        SetTriggerOverflow(0);
#endif /* USING_TRIGGER */
        /* Reset write pointer in header critical section */
        g_traceHeader.writePointer = writePointer;
        g_traceHeader.writePointerCopy = writePointer;
        /* Initialize the most recently stable value for read pointer */
        g_lastReadPointer = expectedInitialReadPointer;
#ifdef USING_TRIGGER
        AcqConfigureTrigger(&(g_traceHeader.trigger));
#endif /* USING_TRIGGER */
        /* Acknowledge to the host for synchronization */
        flags |= SNP_TRC_START_STOP_ACK;
        g_traceHeader.flags = flags;
      }
      else
      {
        /* Something wrong with the host: flag it */
        flags |= SNP_TRC_RECORD_SKIPPED;
        g_traceHeader.flags = flags;
#ifdef MEASURE_PERFORMANCE
        ResetTraceGpio();
#endif /* MEASURE_PERFORMANCE */
        return;
      }
    }

    if (bOverflow == 1)
    {
      /* We were in overflow mode; check if the readPointer moved */
      if (writePointer != readPointer)
      {
        /* Yes */
        bOverflow = 0;
#ifdef USING_TRIGGER
        /* Inform the trigger module */
        SetTriggerOverflow(0);
#endif /* USING_TRIGGER */
      }
    }

#ifdef USING_TRIGGER
    trigEval = AcqEvaluateTrigger();
    if ((trigEval != TRIG_EVAL_NO_REC) && (trigEval != TRIG_EVAL_PRE1_REC))
    {
      /* There is something to do with triggers */
#endif /* USING_TRIGGER */
      g_subSamplingCount++;
#ifdef USING_TRIGGER
      if ((trigEval == TRIG_EVAL_EVENT) || ((trigEval == TRIG_EVAL_POST_REC) && (bHasTrigged == 0)))
      {
        /* Ensure the trigger event is traced, for easier management */
        g_subSamplingCount = g_traceHeader.subSamplingRate;
      }
      bHasTrigged = 1;
#endif /* USING_TRIGGER */
      /* Do not systematically trace at each call; allow sub-sampling (added from version 3) */
      if (g_subSamplingCount >= g_traceHeader.subSamplingRate)
      {
        /* Reset the counter for next time */
        g_subSamplingCount = 0;

        if (bOverflow == 0)
        {
          if (writePointer == readPointer)
          {
            /* We are about to write the last available area into the buffer. For easiest
            overflow management, consider this as the overflow signal (do not write
            to it, do not increment writePointer) */
            bOverflow = 1; /* For internal processing of the overflow */
            flags |= SNP_TRC_FLAG_OVERFLOW;
            /* Immediately modify the header critical section in this case (nothing else to do) */
            g_traceHeader.flags = flags;
#ifdef USING_TRIGGER
            /* Inform the trigger module */
            SetTriggerOverflow(1);
#endif /* USING_TRIGGER */
          }
          else
          {
            /* Let s add a new record */
#ifdef USING_TRIGGER
            /* Inform the trigger module of new records (real ones, not pretrig ones) */
            if (trigEval != TRIG_EVAL_PRE2_REC)
            {
              AddingNewRecord();
            }
#endif /* USING_TRIGGER */
            recordLastAddr = (TraceHeaderField_t)pWrite + g_recordSize - 1;

            /* Begin with timestamp */
            *pWrite = g_timestamp;
            pWrite++;
            varIdx = 0;
            /* Continue with variables values */
            while ((TraceHeaderField_t)pWrite <= recordLastAddr)
            {
              uint32_t tmpAddr = g_traceHeader.g_varList[varIdx].address;
              varAccessType = g_traceHeader.g_varList[varIdx].accessType;
              if (varAccessType == SNP_TRC_DATA_ACCESS_64BIT)
              {
                /* Special case for doubles: do it in 2x32bits */
                varAccessType = SNP_TRC_DATA_ACCESS_32BIT;
                *pWrite = g_readFunc[varAccessType](tmpAddr);
                pWrite++;
                tmpAddr += sizeof(TraceHeaderField_t);
              }
              *pWrite = g_readFunc[varAccessType](tmpAddr);
              varIdx++;
              pWrite++;
            }
            /* At the end, increment the pointer shared with the host */
            recordLastAddr++;
            /* Manage trace buffer loopback */
            if (recordLastAddr + g_recordSize - 1 > g_traceHeader.bufferEndAddr)
            {
              /* There is not enough space in the buffer for a new record: restart from
              the beginning */
              recordLastAddr = g_traceHeader.bufferStartAddr;
              pWrite = (TraceHeaderField_t *)g_traceHeader.bufferStartAddr;
            }
#ifndef USING_TRIGGER
            g_traceHeader.writePointer = recordLastAddr;
            /* Duplicate for the host to be ensured the value is stable */
            g_traceHeader.writePointerCopy = recordLastAddr;
#endif /* USING_TRIGGER */
          }
        }
      }  /* Note that g_timestamp increments even when record is skipped by subsampling (may change in the future ?) */
#ifdef USING_TRIGGER
      if (trigEval == TRIG_EVAL_EVENT)
      {
        /* We just hit the trigger after PRETRIG state: let s compute the index */
        /* of the first record to commit to the host */
        int32_t nbPreTrigRecords;
        if (bPreTrigBufOvf == 1)
        {
          /* The buffer is full with PRETRIG records */
          nbPreTrigRecords = nbRecords - 1; /* -1 for trig event */
          /* Clear the flag for next time */
          bPreTrigBufOvf = 0;
        }
        else
        {
          /* Pretrig records are between g_traceHeader.writePointer (last record
          committed to the host) and pWrite (last record stored in buffer).
          One must also deduce the trig event record => -1 */
          if ((TraceBufferField_t)pWrite > g_traceHeader.writePointer)
          {
            nbPreTrigRecords = ((TraceBufferField_t)pWrite - g_traceHeader.writePointer) / g_recordSize - 1;
          }
          else
          {
            nbPreTrigRecords = nbRecords - (g_traceHeader.writePointer - (TraceBufferField_t)pWrite) / g_recordSize - 1;
          }
        }
        /* Makes the host read pointer points to the first valid pretrig record */
        if (nbPreTrigRecords > g_traceHeader.trigger.nPreTrig)
        {
          /* We recorded more than expected; truncate. */
          nbPreTrigRecords = g_traceHeader.trigger.nPreTrig;
        }
        /* readPointer is the first address of the last record already read by the host. The next record to be
        read is at address readPointer+g_recordSize. Moreover, we must also add the trig event record => +2 */
        if ((TraceBufferField_t)pWrite >= (TraceHeaderField_t)&g_dataTraceBuffer + (nbPreTrigRecords + 2)*g_recordSize)
        {
          g_traceHeader.readPointer = (TraceBufferField_t)pWrite - (nbPreTrigRecords + 2) * g_recordSize;
        }
        else
        {
          /* Buffer loopback: read pointer must be after pWrite */
          if (nbRecords > nbPreTrigRecords + 2)
          {
            g_traceHeader.readPointer = (TraceBufferField_t)pWrite + (nbRecords - nbPreTrigRecords - 2) * g_recordSize;
          }
          else
          {
            /* The full buffer will have to be displayed: point immediately after pWrite for easier
               overflow management */
            g_traceHeader.readPointer = (TraceBufferField_t)pWrite + g_recordSize;
          }
        }
        g_traceHeader.readPointerCopy = g_traceHeader.readPointer;
      }
      if (trigEval == TRIG_EVAL_PRE2_REC)
      {
        /* In PRETRIG state, do not commit records to the host, but simply manage
        the (internal) overflow */
        if (((TraceBufferField_t)pWrite == g_traceHeader.writePointer) && (recordLastAddr != (TraceHeaderField_t)(-1)))
        {
          /* At least one record was added into the buffer (because recordLastAddr != -1) then pWrite looped back to
             writePointer */
          bPreTrigBufOvf = 1;
        }
      }
      else
      {
        if (recordLastAddr != (TraceHeaderField_t)(-1))
        {
          /* A new record was added; increment write pointer now for host synchro */
          g_traceHeader.writePointer = recordLastAddr;
          /* Duplicate for the host to be ensured the value is stable */
          g_traceHeader.writePointerCopy = recordLastAddr;
        }
        if (trigEval == TRIG_EVAL_EVENT)
        {
          /* Signal the TRIG event to the host in pre-trig mode
          (because in this mode, using only read and write pointers to check against new records
          may lead to deadlock condition in case nPreTrig+nbRecordsAfterTrig==nbRecords).
          This MUST be done lastly for correct host synchronization */
          g_traceHeader.trigger.state = SNP_TRC_TRIGGER_TRIGGED;
        }
      }
    }
    else
    {
      if ((trigEval == TRIG_EVAL_NO_REC) && (bHasTrigged != 0))
      {
        /* A trigger stop condition was hit. If the user expected it, the timestamp
        should restart from 0 the next time */
        if (g_traceHeader.trigger.mode & SNP_TRC_TRIGGER_RST_TIMESTAMP)
        {
          g_timestamp = 0;
        }
        /* Note that otherwise g_timestamp continues to increment */
      }
    }
#endif /* USING_TRIGGER */

    /* Always increment the timestamp for timestamp synchro with the host according to SNP_TRC_TIMESTAMP_BASE_UNIT */
    g_timestamp++;
    if (g_timestamp == 0)
    {
      /* Flag the overflow */
      g_timestampOverflow++;
    }
  }
#ifdef MEASURE_PERFORMANCE
  ResetTraceGpio();
#endif /* MEASURE_PERFORMANCE */
}

#ifdef STM32F0XX
/**
  * @brief  align varaibale addresses
  * @note   On Cortex M0, the LDRx instructions require to be aligned: do it now
  *         for robustness (no hard fault)
  * @param  nbVar : number of variable to align
  * @retval None
  */
void alignAddresses(uint32_t nbVar)
{
  DataTraceAccessTypeT varAccessType;
  uint32_t varIdx;
  for (varIdx = 0; varIdx < nbVar; varIdx++)
  {
    varAccessType = g_traceHeader.g_varList[varIdx].accessType;
    if ((varAccessType == SNP_TRC_DATA_ACCESS_64BIT) || (varAccessType == SNP_TRC_DATA_ACCESS_32BIT))
    {
      /* 32-bits aligned */
      g_traceHeader.g_varList[varIdx].address &= 0xFFFFFFFCUL;
    }
    else if (varAccessType == SNP_TRC_DATA_ACCESS_16BIT)
    {
      /* 16-bits aligned */
      g_traceHeader.g_varList[varIdx].address &= 0xFFFFFFFEUL;
    }
  }
}
#endif /* STM32F0XX */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
