/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : dataAcq.h
* Author             : MCD Tools Development
* Version            : V1.1
* Date               : 23/03/2011
* Description        : This file provides a mechanism for software data trace
*                      from the application.
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef __DATAACQ_H__
#define __DATAACQ_H__

// Put the following line under comment in order to get rid of triggers
#define USING_TRIGGER

// Following line activates the code managing the trigger evaluation on a float
// variable. Uses a floating point library. Remove the comment in order to activate.
#define USING_TRIGGER_ON_FLOAT

// Following line activates the code managing the trigger evaluation on a double
// variable (STM32 only). Uses a floating point library. Remove the comment in
// order to activate.
#define USING_TRIGGER_ON_DOUBLE

// Maximum number of variables in one record.
// It defines the size of the variable description header (statically allocated),
// not the real number of traced variables (that may be less).
// Note also that a variable of type "double" takes 2 TraceHeaderFieldT elements
// in the data buffer.
#define SNP_TRC_NB_MAX_WORD_VAR 10

// Type for trace header alignment: each field of the trace header must have this
// type. Note that "double" type variables are made with 2 TraceHeaderFieldT.
typedef unsigned long TraceHeaderFieldT;

// Type for the trace buffer contents (timestamp+var values)
typedef unsigned long TraceBufferFieldT;


////////////////////////////////////////////////////////////////////////////////
// Description of the variables to be traced

#define SNP_TRC_DATA_ACCESS_8BIT  0 // signed or unsigned for acquisition; signed only for trigger
#define SNP_TRC_DATA_ACCESS_16BIT 1 // signed or unsigned for acquisition; signed only for trigger
#define SNP_TRC_DATA_ACCESS_32BIT 2 // signed or unsigned for acquisition; signed only for trigger
#define SNP_TRC_DATA_ACCESS_64BIT 3 // For management of double
#define SNP_TRC_DATA_ACCESS_TRIG_U8  4 // For trigger only: unsigned; note that bit2 discriminates
#define SNP_TRC_DATA_ACCESS_TRIG_U16 5 // For trigger only: unsigned; note that bit2 discriminates
#define SNP_TRC_DATA_ACCESS_TRIG_U32 6 // For trigger only: unsigned; note that bit2 discriminates
#define SNP_TRC_DATA_ACCESS_TRIG_FLOAT  7 // For trigger only
#define SNP_TRC_DATA_ACCESS_TRIG_DOUBLE 8 // For trigger only

typedef TraceHeaderFieldT DataTraceAccessTypeT;
typedef TraceHeaderFieldT DataTraceAddressT;
typedef struct
{
	DataTraceAccessTypeT accessType;
	DataTraceAddressT address;
} DataTraceVarMapT;


////////////////////////////////////////////////////////////////////////////////
// Trace header structure

// Flag definitions for DataTraceHeaderT.headerVersion
#define SNP_TRC_HEADER_VERSION_MASK               (0x00FF)
#define SNP_TRC_HEADER_SUPPORTS_TRIGGER           (0x0100)
#define SNP_TRC_HEADER_SUPPORTS_TRIGGER_ON_FLOAT  (0x0200)
#define SNP_TRC_HEADER_SUPPORTS_TRIGGER_ON_DOUBLE (0x0400)

// Flag definitions for DataTraceHeaderT.flags
#define SNP_TRC_FLAG_OVERFLOW      1
#define SNP_TRC_START_STOP_ACK     2
#define SNP_TRC_RECORD_SKIPPED     4

// Flag definitions for DataTraceHeaderT.TraceHeaderTriggerT.cmd
#define SNP_TRC_TRIGGER_STOP     0
#define SNP_TRC_TRIGGER_START    1

// Flag definitions for DataTraceHeaderT.TraceHeaderTriggerT.mode
// bit[3:0]: start condition
// bit[7:4]: stop condition
#define SNP_TRC_TRIGGER_START_MASK       0x0F
#define SNP_TRC_TRIGGER_START_IMMEDIATE  0x00
#define SNP_TRC_TRIGGER_RISING           0x01
#define SNP_TRC_TRIGGER_FALLING          0x02
#define SNP_TRC_TRIGGER_STOP_MASK        0x70
#define SNP_TRC_TRIGGER_AUTO_RESTART     0x10
#define SNP_TRC_TRIGGER_STOP_OVF         0x20
#define SNP_TRC_TRIGGER_STOP_N_REC       0x40
#define SNP_TRC_TRIGGER_RST_TIMESTAMP    0x80 // Restart timestamp on each trig event

// Flag definitions for DataTraceHeaderT.TraceHeaderTriggerT.state
#define SNP_TRC_TRIGGER_STOPPED     0 // Init state, waiting for start cmd
#define SNP_TRC_TRIGGER_STARTED     1 // Trigger started, waiting for the configured event
#define SNP_TRC_TRIGGER_TRIGGED     2 // Event hit; waiting for next cmd (start or stop)

// Predefined values for timestamp base unit
#define SNP_TRC_SECONDS       15 
#define SNP_TRC_MILLI_SECONDS 14 
#define SNP_TRC_MICRO_SECONDS 13 
#define SNP_TRC_NANO_SECONDS  12 

typedef struct
{
	// Fields that may be modified by the host are declared volatile
	volatile TraceHeaderFieldT address;     // Address of variable for trigger
	volatile TraceHeaderFieldT accessType;  // Type of variable for trigger
	volatile TraceHeaderFieldT threshold;   // Trigger threshold
	volatile TraceHeaderFieldT mode;        // Trigger mode
	volatile TraceHeaderFieldT cmd;         // Trigger command
	volatile TraceHeaderFieldT stopParam;   // Optional parameter for end condition
	TraceHeaderFieldT state;                // Trigger state
	volatile TraceHeaderFieldT nPreTrig;    // Number of records to keep before the trigger. 0 by default (no pretrig)
	volatile TraceHeaderFieldT bIgnoreTrig;  // Ignore trigger when buffer not empty
#ifdef USING_TRIGGER_ON_DOUBLE
	volatile TraceHeaderFieldT thresholdDouble;  // First word (low address part) of the threshold expressed as double
	volatile TraceHeaderFieldT thresholdDouble2; // Second word (high address part) of the threshold expressed as double
#endif
} TraceHeaderTriggerT;

typedef struct
{
	// Static fields (affected at compilation time)
	char startMark[3];                 // Unchanging marker used for robustness
	char headerFieldSize;              // Size in bytes of following fields in the header
	TraceHeaderFieldT headerVersion;   // Identifier of the header format
	TraceHeaderFieldT bufferStartAddr; // First (low) address of memory area reserved for trace data
	TraceHeaderFieldT bufferEndAddr;   // Last (high) address of memory area reserved for trace data
	TraceHeaderFieldT nbVarMax;        // Maximum number of variables (32bits words) in one record
	TraceHeaderFieldT bufferFormat;    // Version identifier of buffer format
	TraceHeaderFieldT timestamp_base_unit;    // Period between 2 consecutive calls to Dumptrace
	// Control fields (used at run time) - volatile because some may be written by the host PC
	volatile TraceHeaderFieldT writePointer;     // Position in the trace buffer where to put the next record
	volatile TraceHeaderFieldT writePointerCopy; // Do a copy as basic security for shared area
	volatile TraceHeaderFieldT readPointer;      // Position in the trace buffer of the last fully read record
	volatile TraceHeaderFieldT readPointerCopy;  // Do a copy as basic security for shared area
	volatile TraceHeaderFieldT flags;            // Shared status flags (overflow)
	volatile TraceHeaderFieldT subSamplingRate;  // Recording rate 1=always; 2 = 50%, 3=33%, ...
	volatile TraceHeaderFieldT nbVar;            // Number of variables (32bits words) currently being traced
	// Variables description - volatile because written by the host PC
	volatile DataTraceVarMapT g_varList[SNP_TRC_NB_MAX_WORD_VAR];
#ifdef USING_TRIGGER
	// Trigger description
	volatile TraceHeaderTriggerT trigger;
#endif
} DataTraceHeaderT;


// Defines for variable placement out of the 256 bytes 0 page
#if defined(__CSMC__) // Cosmic STM8
	#define _NEAR_DATA_ @near
	#define _FAR_DATA_ @far
#elif defined(__RAISONANCE__) // Raisonance STM8
	#define _NEAR_DATA_ data
	#define _FAR_DATA_
#elif defined(__IAR_SYSTEMS_ICC__) // IAR STM8 and STM32
	#define _NEAR_DATA_
	#if __ICCARM__ // STM32
		#define _FAR_DATA_
	#else // STM8
		#define _FAR_DATA_ __far
	#endif
#elif defined(__ARMCC_VERSION) // Keil STM32
	#define _NEAR_DATA_
	#define _FAR_DATA_
#endif


////////////////////////////////////////////////////////////////////////////////
// Exported routine for trigger implementation
int IsBufferEmpty(void);

////////////////////////////////////////////////////////////////////////////////
// Exported routines for acquisition in the embedded

// Reset the data trace buffer to a clean state. Reset timestamp counters
void ClearBuffer(void);

#if (defined(__CSMC__) || defined(__RAISONANCE__) )
// Workaround against bad initialization of pointers into a structure by some
// compiler versions
void InitHeader(void);
#endif

void DumpTrace(void);

#endif
