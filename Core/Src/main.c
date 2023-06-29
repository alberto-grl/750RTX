/* USER CODE BEGIN Header */
/**
 *******************************************************************************

                   main.c module of the program ARM_Radio

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

 *******************************************************************************
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 * CW Decoder based on WB7FHC's Morse Code Decoder v1.1
 * https://github.com/kareiva/wb7fhc-cw-decoder
 *
 * On-demand filter generator based on Phil Karn KA9Q code, https://github.com/ka9q/ka9q-radio
 * Simplified Linux test project is at https://github.com/alberto-grl/ka9q-filters
 *
 * SCAMP protocol by Daniel Marks, KW4TI https://github.com/profdc9/RFBitBanger/blob/main/Docs/SCAMP-Digital-Mode-Proposal-v0.6.pdf
 * https://github.com/profdc9/RFBitBanger/blob/main/Code/RFBitBanger/scamp.c
 * Linux SCAMP terminal and test code at https://github.com/alberto-grl/SCAMP_TERM
 *
 *
 ******************************************************************************
  FAQ about cache https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices
  FPU performance https://community.arm.com/developer/ip-products/processors/f/cortex-m-forum/5567/what-is-the-advantage-of-floating-point-of-cm7-versus-cm4
  CIC filter decomposition: https://www.embedded.com/reducing-power-consumption-in-cic-filter-algorithm-designs/
  NCO https://www.embedded.com/configuring-a-well-behaved-digital-quadrature-oscillator/
  DMA, Cache https://hackaday.io/project/117555-network-analyzer-on-an-stm32h7/details
  FFT Filters https://www.danvillesignal.com/images/pdfs/compdsp_Borgerding_FFT_slides2.pdf http://dsp-book.narod.ru/DSPMW/08.PDF
  Measuring inductance https://daycounter.com/Articles/How-To-Measure-Inductance.phtml
  RF Filters https://rf-tools.com/lc-filter/
  Only terminal for Android that supports escape codes is DroidTerm, AFAIK.
  Capacitors ESR: https://ds.murata.co.jp/simsurfing/index.html?lcid=en-us
  DSP, FFT filters: https://www.analog.com/en/education/education-library/scientist_engineers_guide.html
  FT8 FSK: 8 different tones spaced at 5.86 Hz (6.25 is reported elsewhere)

  MCO2 output is pin PC9, CN8 pin 4.


  TIM4 - Encoder
  TIM6 - DMA trigger for audio DAC
  TIM7 - CW Keyer
  TIM13 - TinyUSB periodic call
  TIM7 - CW Keyer

  PA6 - Keyer Dash
  PA7 - Keyer Dot
 */

#define IN_MAIN

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "Globals.h"
#include "FIRcoefs.h"
#include "tusb.h"
//#include "board.h"
#include "usb_descriptors.h"
//#include "usbd_cdc_if.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// List of supported sample rates
#if defined(__RX__)
const uint32_t sample_rates[] = {44100, 48000};
#else
const uint32_t sample_rates[] = {44100, 48000, 88200, 96000};
#endif

uint32_t current_sample_rate  = 44100;

#define N_SAMPLE_RATES  TU_ARRAY_SIZE(sample_rates)

/* Blink pattern
 * - 25 ms   : streaming data
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum
{
	BLINK_STREAMING = 25,
	BLINK_NOT_MOUNTED = 250,
	BLINK_MOUNTED = 1000,
	BLINK_SUSPENDED = 2500,
};

enum
{
	VOLUME_CTRL_0_DB = 0,
	VOLUME_CTRL_10_DB = 2560,
	VOLUME_CTRL_20_DB = 5120,
	VOLUME_CTRL_30_DB = 7680,
	VOLUME_CTRL_40_DB = 10240,
	VOLUME_CTRL_50_DB = 12800,
	VOLUME_CTRL_60_DB = 15360,
	VOLUME_CTRL_70_DB = 17920,
	VOLUME_CTRL_80_DB = 20480,
	VOLUME_CTRL_90_DB = 23040,
	VOLUME_CTRL_100_DB = 25600,
	VOLUME_CTRL_SILENCE = 0x8000,
};


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/**
 * @brief  Computation of ADC master conversion result
 *         from ADC dual mode conversion result (ADC master and ADC slave
 *         results concatenated on data register of ADC master).
 * @param  DATA: ADC dual mode conversion result
 * @retval None
 */
#define COMPUTATION_DUALMODEINTERLEAVED_ADCMASTER_RESULT(DATA)                 \
		((DATA) & 0x0000FFFF)

/**
 * @brief  Computation of ADC slave conversion result
 *         from ADC dual mode conversion result (ADC master and ADC slave
 *         results concatenated on data register of ADC master).
 * @param  DATA: ADC dual mode conversion result
 * @retval None
 */
#define COMPUTATION_DUALMODEINTERLEAVED_ADCSLAVE_RESULT(DATA)                  \
		((DATA) >> 16)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

LPTIM_HandleTypeDef hlptim2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* Variable containing ADC conversions results */
ALIGN_32BYTES(__IO short AudioOut[BSIZE*2]);			  // A single array because we are using the half/full complete irq to switch between the two halves

uint8_t         ubADCDualConversionComplete = RESET;                        /* Set into ADC conversion complete callback */

const uint16_t sine_wave_array[32] = {2047, 1648, 1264, 910, 600,  345,
		156, 39,  0,  39,  156,  345,
		600, 910, 1264, 1648, 2048, 2447,
		2831, 3185, 3495, 3750, 3939, 4056,
		4095, 4056, 3939, 3750, 3495, 3185,
		2831, 2447};

extern uint8_t UartTXString[4096];
extern uint8_t UartRXString[256];
__IO ITStatus UartRXDataReady = RESET;
__IO ITStatus UartTXFinished = RESET;


volatile uint32_t MainLoopCounter;
static uint32_t blink_interval_ms = BLINK_STREAMING;
static uint32_t StopwatchStart, StopwatchValue;
// Audio controls
// Current states
int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];       // +1 for master channel 0
int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];    // +1 for master channel 0

// Buffer for microphone data
int32_t mic_buf[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4];
// Buffer for speaker data
int32_t spk_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4];
// Speaker data size received in the last frame
int spk_data_size;
// Resolution per format
const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX,
		CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_RX};
// Current resolution, update on format change
uint8_t current_resolution;
volatile uint32_t AudioCounter;
uint8_t BufferFillRequired = 1;
uint8_t SendGreetings = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC1_Init(void);
static void MX_LPTIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM13_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t board_millis(void);
//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
	blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void)remote_wakeup_en;
	blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	blink_interval_ms = BLINK_MOUNTED;
}

// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request)
{
	TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

	if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
	{
		if (request->bRequest == AUDIO_CS_REQ_CUR)
		{
			TU_LOG1("Clock get current freq %lu\r\n", current_sample_rate);

			audio_control_cur_4_t curf = { (int32_t) tu_htole32(current_sample_rate) };
			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
		}
		else if (request->bRequest == AUDIO_CS_REQ_RANGE)
		{
			audio_control_range_4_n_t(N_SAMPLE_RATES) rangef =
			{
					.wNumSubRanges = tu_htole16(N_SAMPLE_RATES)
			};
			TU_LOG1("Clock get %d freq ranges\r\n", N_SAMPLE_RATES);
			for(uint8_t i = 0; i < N_SAMPLE_RATES; i++)
			{
				rangef.subrange[i].bMin = (int32_t) sample_rates[i];
				rangef.subrange[i].bMax = (int32_t) sample_rates[i];
				rangef.subrange[i].bRes = 0;
				TU_LOG1("Range %d (%d, %d, %d)\r\n", i, (int)rangef.subrange[i].bMin, (int)rangef.subrange[i].bMax, (int)rangef.subrange[i].bRes);
			}

			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
		}
	}
	else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
			request->bRequest == AUDIO_CS_REQ_CUR)
	{
		audio_control_cur_1_t cur_valid = { .bCur = 1 };
		TU_LOG1("Clock get is valid %u\r\n", cur_valid.bCur);
		return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
	}
	TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n",
			request->bEntityID, request->bControlSelector, request->bRequest);
	return false;
}

// Helper for clock set requests
static bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
	(void)rhport;

	TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
	TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

	if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
	{
		TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));

		current_sample_rate = (uint32_t) ((audio_control_cur_4_t const *)buf)->bCur;

		TU_LOG1("Clock set current freq: %ld\r\n", current_sample_rate);

		return true;
	}
	else
	{
		TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n",
				request->bEntityID, request->bControlSelector, request->bRequest);
		return false;
	}
}

// Helper for feature unit get requests
static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request)
{
	TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);

	if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && request->bRequest == AUDIO_CS_REQ_CUR)
	{
		audio_control_cur_1_t mute1 = { .bCur = mute[request->bChannelNumber] };
		TU_LOG1("Get channel %u mute %d\r\n", request->bChannelNumber, mute1.bCur);
		return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &mute1, sizeof(mute1));
	}
	else if (UAC2_ENTITY_SPK_FEATURE_UNIT && request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
	{
		if (request->bRequest == AUDIO_CS_REQ_RANGE)
		{
			audio_control_range_2_n_t(1) range_vol = {
					.wNumSubRanges = tu_htole16(1),
					.subrange[0] = { .bMin = tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(256) }
			};
			TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\r\n", request->bChannelNumber,
					range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
		}
		else if (request->bRequest == AUDIO_CS_REQ_CUR)
		{
			audio_control_cur_2_t cur_vol = { .bCur = tu_htole16(volume[request->bChannelNumber]) };
			TU_LOG1("Get channel %u volume %d dB\r\n", request->bChannelNumber, cur_vol.bCur / 256);
			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
		}
	}
	TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n",
			request->bEntityID, request->bControlSelector, request->bRequest);

	return false;
}

// Helper for feature unit set requests
static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
	(void)rhport;

	TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
	TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

	if (request->bControlSelector == AUDIO_FU_CTRL_MUTE)
	{
		TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));

		mute[request->bChannelNumber] = ((audio_control_cur_1_t const *)buf)->bCur;

		TU_LOG1("Set channel %d Mute: %d\r\n", request->bChannelNumber, mute[request->bChannelNumber]);

		return true;
	}
	else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
	{
		TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));

		volume[request->bChannelNumber] = ((audio_control_cur_2_t const *)buf)->bCur;

		TU_LOG1("Set channel %d volume: %d dB\r\n", request->bChannelNumber, volume[request->bChannelNumber] / 256);

		return true;
	}
	else
	{
		TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n",
				request->bEntityID, request->bControlSelector, request->bRequest);
		return false;
	}
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
	audio_control_request_t const *request = (audio_control_request_t const *)p_request;

	if (request->bEntityID == UAC2_ENTITY_CLOCK)
		return tud_audio_clock_get_request(rhport, request);
	if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
		return tud_audio_feature_unit_get_request(rhport, request);
	else
	{
		TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n",
				request->bEntityID, request->bControlSelector, request->bRequest);
	}
	return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
	audio_control_request_t const *request = (audio_control_request_t const *)p_request;

	if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
		return tud_audio_feature_unit_set_request(rhport, request, buf);
	if (request->bEntityID == UAC2_ENTITY_CLOCK)
		return tud_audio_clock_set_request(rhport, request, buf);
	TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n",
			request->bEntityID, request->bControlSelector, request->bRequest);

	return false;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
	(void)rhport;

	uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
	uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

	if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt == 0)
		blink_interval_ms = BLINK_MOUNTED;

	return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
	(void)rhport;
	uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
	uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

	TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
	if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt != 0)
		blink_interval_ms = BLINK_STREAMING;

	// Clear buffer when streaming format is changed
	spk_data_size = 0;
	if(alt != 0)
	{
		current_resolution = resolutions_per_format[alt-1];
	}

	return true;
}

bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting)
{
	(void)rhport;
	(void)func_id;
	(void)ep_out;
	(void)cur_alt_setting;

	spk_data_size = tud_audio_read(spk_buf, n_bytes_received);
	return true;
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
	(void) rhport;
	(void) itf;
	(void) ep_in;
	(void) cur_alt_setting;


	return true;
}


bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
	(void)rhport;
	(void)itf;
	(void)ep_in;
	(void)cur_alt_setting;
	int16_t *dst = (int16_t*)mic_buf;

#if 0
	static uint16_t LastBytesCopied; //used to check for partial writes, but it seems this never happens
	if (n_bytes_copied != 96 && n_bytes_copied != 0)
	{
		LastBytesCopied = n_bytes_copied;
	}
#endif
#if 0
	for (uint16_t i = 0; i < 48000/1000; i++ )
	{
		//		*dst ++ = (int16_t)(20000.0f * arm_sinf32((float)(432.0f * 6.28f * AudioCounter++ / 48000)));
		*dst ++ = (int16_t)(20000.0f * sinf((float)(432.0f * 6.28f * AudioCounter++ / 48000))); //sinf is really slow
		//*dst ++ = (int16_t)(-10000 + (AudioCounter+=500) % 20000);
	}


	/* There seems to be no advantage in deferring tud_audio_write to tud_audio_tx_done_pre_load_cb
	 * as in the 4 mic example.
	 * Also, a delay in this callback has the same effect of a delay in the main loop.
	 */
	tud_audio_write((uint8_t *)mic_buf, (uint16_t) (2 * 48000 /1000));

#endif

#if 0
	if (spk_data_size)
	{
		if (current_resolution == 16)
		{
			int16_t *src = (int16_t*)spk_buf;
			int16_t *limit = (int16_t*)spk_buf + spk_data_size / 2;
			int16_t *dst = (int16_t*)mic_buf;
			while (src < limit)
			{
				// Combine two channels into one
				int32_t left = *src++;
				int32_t right = *src++;
				*dst++ = (int16_t) ((left >> 1) + (right >> 1));
			}
			tud_audio_write((uint8_t *)mic_buf, (uint16_t) (spk_data_size / 2));
			spk_data_size = 0;
		}
		else if (current_resolution == 24)
		{
			int32_t *src = spk_buf;
			int32_t *limit = spk_buf + spk_data_size / 4;
			int32_t *dst = mic_buf;
			while (src < limit)
			{
				// Combine two channels into one
				int32_t left = *src++;
				int32_t right = *src++;
				*dst++ = (int32_t) ((uint32_t) ((left >> 1) + (right >> 1)) & 0xffffff00ul);
			}
			tud_audio_write((uint8_t *)mic_buf, (uint16_t) (spk_data_size / 2));
			spk_data_size = 0;
		}
	}

#endif

	return true;
}


//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	(void) itf;
	(void) rts;

	// TODO set some indicator
	if ( dtr )
	{
		// Terminal connected
	}else
	{
		// Terminal disconnected
	}
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
	(void) itf;
}

void cdc_task(void)
{
#if 0
	// connected() check for DTR bit
	// Most but not all terminal client set this when making connection
	if ( tud_cdc_connected() )
	{
		//TODO: terminal shows message only after a key is pressed. Why?
		if (SendGreetings && tud_cdc_write_available())
		{
			tud_cdc_write("\n\rHello!\n\r", 8);
			tud_cdc_write_flush();
			SendGreetings = 0;
		}
		// connected and there are data available
		if ( tud_cdc_available() )
		{
			// read data
			char buf[64];
			uint32_t count = tud_cdc_read(buf, sizeof(buf));
			(void) count;

			// Echo back
			// Note: Skip echo by commenting out write() and write_flush()
			// for throughput test e.g
			//    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
			tud_cdc_write(buf, count);
			tud_cdc_write_flush();
		}
	}
#endif
}


//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void audio_task(void)
{
	// #define LOOPBACK_EXAMPLE
#ifdef LOOPBACK_EXAMPLE

	// When new data arrived, copy data from speaker buffer, to microphone buffer
	// and send it over
	// Only support speaker & headphone both have the same resolution
	// If one is 16bit another is 24bit be care of LOUD noise !

#if 0
	//generate nasty noise
	mic_buf[0] = 20000;
	mic_buf[1] = 20000;
	mic_buf[2] = 20000;
	mic_buf[3] = 20000;
	tud_audio_write((uint8_t *)mic_buf, (uint16_t) (48));
#endif


	if (spk_data_size)
	{
		if (current_resolution == 16)
		{
			int16_t *src = (int16_t*)spk_buf;
			int16_t *limit = (int16_t*)spk_buf + spk_data_size / 2;
			int16_t *dst = (int16_t*)mic_buf;
			while (src < limit)
			{
				// Combine two channels into one
				int32_t left = *src++;
				int32_t right = *src++;
				*dst++ = (int16_t) ((left >> 1) + (right >> 1));
			}
			tud_audio_write((uint8_t *)mic_buf, (uint16_t) (spk_data_size / 2));
			spk_data_size = 0;
		}
		else if (current_resolution == 24)
		{
			int32_t *src = spk_buf;
			int32_t *limit = spk_buf + spk_data_size / 4;
			int32_t *dst = mic_buf;
			while (src < limit)
			{
				// Combine two channels into one
				int32_t left = *src++;
				int32_t right = *src++;
				*dst++ = (int32_t) ((uint32_t) ((left >> 1) + (right >> 1)) & 0xffffff00ul);
			}
			tud_audio_write((uint8_t *)mic_buf, (uint16_t) (spk_data_size / 2));
			spk_data_size = 0;
		}
	}
#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM13)
	{
		tud_task();
		//		audio_task(); //not needed anymore
		MainLoopCounter++;  //used with debugger to check frequency of main loop
		//        cdc_task(); //not needed anymore
		//	led_blinking_task();
	}

	if (htim->Instance == TIM7)
	{
#ifdef USE_KEYER
		DoKeyer();
#endif
#ifdef USE_SCAMP
		TXScamp();
#endif
	}
}


int32_t adc_ReadInternalTemp(void)

{

	//https://github.com/sweesineng/STM32_ADC_MultiCh_SingleConv_Polling/blob/main/Core/Src/main.c
	int32_t Temp;
	uint32_t adcTempVal;
	uint32_t adcVRefVal;
	uint32_t VRefMilliVoltsValue;
	//used for MPU temperature reading
	HAL_OK != HAL_ADC_Start(&hadc3);
	HAL_OK != HAL_ADC_PollForConversion(&hadc3, 100);
	adcTempVal = HAL_ADC_GetValue(&hadc3);

	HAL_OK != HAL_ADC_PollForConversion(&hadc3, 100);
		   adcVRefVal = HAL_ADC_GetValue(&hadc3);

	HAL_ADC_Stop(&hadc3);
	//used for MPU temperature reading.
	//Start ADC and let it sample while doing other things. First reading after power up will be wrong.
	HAL_OK != HAL_ADC_Start(&hadc3);


	// Calculating VRef voltage
	VRefMilliVoltsValue = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adcVRefVal, ADC_RESOLUTION_16B);
	Temp = __HAL_ADC_CALC_TEMPERATURE(VRefMilliVoltsValue, adcTempVal, ADC_RESOLUTION_16B);

	return Temp; // 55-59 °C at startup, 80°C after warming up

}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	//We use our clock init function and we prevent the use of the STM32Cube tool generated one. Not possible to overclock with it.
#ifdef EXECUTE_CUBE_CLOCK_INIT
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */
#endif

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();


	// Don't forget to set a different SamplingRate in main.c
	SystemClock_Config_For_OC();

	/* Configure the peripherals common clocks */
	//PeriphCommonClock_Config();
	//	 SystemClock_Config();
	HAL_Delay(20);  //needed for USB setup. USB sometimes (and almost always on an Android phone) does not initialize
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_DAC1_Init();
	MX_LPTIM2_Init();
	MX_TIM6_Init();
	MX_USART3_UART_Init();
	MX_TIM4_Init();
	MX_TIM7_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_TIM13_Init();
	MX_ADC3_Init();
	/* USER CODE BEGIN 2 */
	/* Enable D-Cache---------------------------------------------------------*/
#ifdef USE_DCACHE
	SCB_EnableDCache();
#endif
	TU_ASSERT(tusb_init());
	// Initialise again so we can change divider with a #define in main.h
	MX_TIM6_Init_Custom_Rate();


	/* Run the ADC calibration in single-ended mode */
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
	{
		// Calibration Error
		Error_Handler();
	}

	if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
	{
		// Calibration Error
		Error_Handler();
	}

	if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
	{
		// Calibration Error
		Error_Handler();
	}

	HAL_Delay(1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	RXVolume= 0.1;
	//	LED_GREEN_ON;
	//	LED_GREEN_OFF;

	// Set now the default values for some variables
	SetFstep(2);
	cwpitch = CWPITCH;
	os_time = 0;
	meanavg = 0.f;
	Qfactor = 0.987f;         // Q factor for the CW peak filter
	Muted   = false;
	AMindex  = LSBindex = 1;
	USBindex = CWindex  = 1;
	bw[AM]   = bw[LSB]  = Wide;
	bw[USB]  = bw[CW]   = Wide;
	agc[AM]  = agc[LSB] = Slow;
	agc[USB] = Slow;
	agc[CW]  = Fast;
	AGC_decay[Fast] = 0.9995f;
	AGC_decay[Slow] = 0.99995f;
	Hangcount[Fast] = 2;
	Hangcount[Slow] = 30;
	AgcThreshold    = 1.92e-4f;
	pk = 0.02f;

	//	DualADCGainCorrection = 2048.f;  //2048 nominal value for ADC with no error

	HAdc1 = &hadc1;
#ifdef FAKE_SQUARE_RF_SIGNAL

	uint16_t k;

	// ARMRadio for M7 ADC 60 M: Generate a fake RF carrier at 3750.000 / 16 = 234.375 KHz
	// ARMRadio for M7 ADC 150 M: Generate a fake RF carrier at 9375.000 / 16 = 585.9375 KHz
	// ARMRadio for M7 ADC 160 M: Generate a fake RF carrier at 10000.000 / 16 = 625 KHz
	// ARMRadio for M7 ADC 120 M : Generate a fake RF carrier at 7500.000 / 16 = 468.750 KHz
	// ARMRadio for M4: Generate a fake RF carrier at 1.785714 / 16 = 111.607 KHz

	for (k=0; k< BSIZE; k++)
	{
		if (k % 16 > 7) //(k % 16 > 7)
			TestSignalData[k] = 2048 + 100;
		else
			TestSignalData[k] = 2048 - 100;
	}

#endif


#ifdef FAKE_SINE_RF_SIGNAL

	uint16_t k;

	// ARMRadio for M7 ADC 60 M: Generate a fake RF carrier at 3750.000 / 16 = 234.375 KHz
	// ARMRadio for M7 ADC 150 M: Generate a fake RF carrier at 9375.000 / 16 = 585.9375 KHz
	// ARMRadio for M7 ADC 160 M: Generate a fake RF carrier at 10000.000 / 16 = 625 KHz
	// ARMRadio for M7 ADC 120 M : Generate a fake RF carrier at 7500.000 / 16 = 468.750 KHz
	// ARMRadio for M4: Generate a fake RF carrier at 1.785714 / 16 = 111.607 KHz

	for (k=0; k< BSIZE; k++)
	{
		TestSignalData[k] = 2048 + 100.0 * sin(PI * 256.f * (float)k / (float)BSIZE);
	}
#endif

#ifdef FAKE_NO_RF_SIGNAL
	uint16_t k;
	// ARMRadio for M7 ADC 60 M: Generate a fake RF carrier at 3750.000 / 16 = 234.375 KHz
	// ARMRadio for M7 ADC 150 M: Generate a fake RF carrier at 9375.000 / 16 = 585.9375 KHz
	// ARMRadio for M7 ADC 160 M: Generate a fake RF carrier at 10000.000 / 16 = 625 KHz
	// ARMRadio for M7 ADC 120 M : Generate a fake RF carrier at 7500.000 / 16 = 468.750 KHz
	// ARMRadio for M4: Generate a fake RF carrier at 1.785714 / 16 = 111.607 KHz

	for (k=0; k< BSIZE; k++)
	{
		TestSignalData[k] = 2048;
	}
#endif

#ifdef CLK_600M_CPU_150M_ADC
	SamplingRate = ((150000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_500M_CPU_120M_ADC
	SamplingRate = ((120000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_500M_CPU_128M_ADC
	SamplingRate = ((128000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_480M_CPU_120M_ADC
	SamplingRate = ((120000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_600M_CPU_60M_ADC
	SamplingRate = ((60000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_600M_CPU_160M_ADC
	SamplingRate = ((160000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_600M_CPU_160M_ADC_XTAL25
	SamplingRate = ((160000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_620M_CPU_160M_ADC_XTAL25
	SamplingRate = ((160000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_640M_CPU_160M_ADC_XTAL25
	SamplingRate = ((160000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_600M_CPU_150M_ADC_XTAL25
	SamplingRate = ((150000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_600M_CPU_120M_ADC_XTAL25
	SamplingRate = ((120000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_600M_CPU_128M_ADC_XTAL25
	SamplingRate = ((128000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
#ifdef CLK_600M_CPU_96M_ADC_XTAL25
	SamplingRate = ((96000000) / 4) * 2 / 8.f;//ADC Clock /async div * 2 ADC channels /8 cycles for 12 bit ADC
#endif
	CarrierEnable(0);
	TXSwitch(0);
	SetFOut(7000000);

	CWThreshold = 0.01;

	// SamplingRate = SamplingRate * 4000000.f / 3999300.f; // Correct Xtal error

	//SamplingRate = SamplingRate - 180; // Correct Xtal error
	//SamplingRate = SamplingRate * (7300000.f / (7300000.f + (150.f / 2.0f))); // Correct Xtal error. One half of the tuning error since sampling is twice
	SamplingRate += SamplingRate * XTAL_F_ERROR / 2.0;
	AudioRate = SamplingRate / 4 /16.f / 4.f; //First decimation was 16, now is 64
	SDR_compute_IIR_parms();  // compute the IIR parms for the CW peak filter

	// init the decimating FIR control blocks
	arc = arm_fir_decimate_init_f32(&SfirR, NUMFIRCOEFS, 4, FIRcoefs, FIRstate1R, BSIZE*4);
	while(arc != ARM_MATH_SUCCESS)
	{};   // spin loop if error
	arc = arm_fir_decimate_init_f32(&SfirI, NUMFIRCOEFS, 4, FIRcoefs, FIRstate1I, BSIZE*4);
	while(arc != ARM_MATH_SUCCESS)
	{};   // spin loop if error

	Load_Presets();
	Tune_Preset(1);      // Set the initial tuning to Preset 1


#ifdef KEYER
	keyerState = IDLE;
	keyerControl = IAMBICB;      // Or 0 for IAMBICA
	keyer_speed = 15;
	loadWPM(keyer_speed);        // Fix speed at 15 WPM
	keyer_mode = 1; //->  iambic
	keyer_swap = 0; //->  DI/DAH
	txdelay = 10;
#endif //KEYER

	TxPowerOut = MID_POWER_OUT;

	DisplayStatus();    // Display status, it would not be shown until a user input was given
	if (HAL_ADCEx_MultiModeStart_DMA(&hadc1,
			(uint32_t *)aADCDualConvertedValues,
			BSIZE   //Source code says transfer size is in bytes, but it is in number of transfers
			//We transfer BSIZE * 4 bytes, it equals 2 * BSIZE samples (1024) because we have full and half interrupts
	) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

	///////////////
	if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIM_Base_Start(&htim6);
	//	HAL_TIM_Base_Start(&htim7);
	HAL_TIM_Base_Start(&htim2);


	//TIM6->CNT = 0;
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)AudioOut, BSIZE * 2, DAC_ALIGN_12B_R);

	//Test for TX
	HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE0);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0); // TX gate bias


#ifdef  WSPR_BEACON_MODE
	WSPRBeaconState = NO_FIX;
#endif

	// Funziona? serve?		SET_BIT(hadc1.Instance->CFGR, ADC_CFGR_AWD1EN);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

#ifdef WSPR_BEACON_MODE
	//Pressing the encoder knob during startup enters DCF77 sync and WSPR beacon mode
	if (ENC_BUTTON)
	{
		SetMode((Mode)CW);
		LOfreq = DCF77_FREQ;
		WSPRBeaconMode = 1;
	}
#endif

#if 1
	if (HAL_TIM_Base_Start_IT(&htim13) != HAL_OK)
	{
		Error_Handler();
	}
#endif

	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */


		/* ADC conversion buffer complete variable is updated into ADC conversions*/
		/* complete callback.*/

		UserInput();
#ifdef WSPR_BEACON_MODE
		DCF77StatusDisplay();
#endif
		//	HAL_Delay(100);
		HAL_Delay(50);

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLL2PCLK, RCC_MCODIV_1);
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_LPTIM2;
	PeriphClkInitStruct.PLL2.PLL2M = 2;
	PeriphClkInitStruct.PLL2.PLL2N = 12;
	PeriphClkInitStruct.PLL2.PLL2P = 3;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
	PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
	PeriphClkInitStruct.PLL3.PLL3M = 2;
	PeriphClkInitStruct.PLL3.PLL3N = 12;
	PeriphClkInitStruct.PLL3.PLL3P = 2;
	PeriphClkInitStruct.PLL3.PLL3Q = 8;
	PeriphClkInitStruct.PLL3.PLL3R = 4;
	PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
	PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
	PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
	PeriphClkInitStruct.Lptim2ClockSelection = RCC_LPTIM2CLKSOURCE_PLL2;
	PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = {0};
	ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_DUALMODE_INTERL;
	multimode.DualModeData = ADC_DUALMODEDATAFORMAT_32_10_BITS;
	multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_2CYCLES;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analog WatchDog 1
	 */
	AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
	AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
	AnalogWDGConfig.Channel = ADC_CHANNEL_5;
	AnalogWDGConfig.ITMode = ENABLE;
	AnalogWDGConfig.HighThreshold = 4094;
	AnalogWDGConfig.LowThreshold = 1;
	if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.ContinuousConvMode = ENABLE;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
	hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	hadc2.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analog WatchDog 1
	 */
	AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
	AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
	AnalogWDGConfig.Channel = ADC_CHANNEL_5;
	AnalogWDGConfig.ITMode = ENABLE;
	AnalogWDGConfig.HighThreshold = 4094;
	AnalogWDGConfig.LowThreshold = 1;
	if (HAL_ADC_AnalogWDGConfig(&hadc2, &AnalogWDGConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void)
{

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC3_Init 1 */

	hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64; //TODO: the MX code generator does not generate this line for ADC3?
	/* USER CODE END ADC3_Init 1 */

	/** Common config
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.Resolution = ADC_RESOLUTION_16B;
	hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc3.Init.LowPowerAutoWait = DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.NbrOfConversion = 2;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
	hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	hadc3.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc3) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void)
{

	/* USER CODE BEGIN DAC1_Init 0 */

	/* USER CODE END DAC1_Init 0 */

	DAC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN DAC1_Init 1 */

	/* USER CODE END DAC1_Init 1 */

	/** DAC Initialization
	 */
	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK)
	{
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	/** DAC channel OUT2 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DAC1_Init 2 */

	/* USER CODE END DAC1_Init 2 */

}

/**
 * @brief LPTIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPTIM2_Init(void)
{

	/* USER CODE BEGIN LPTIM2_Init 0 */

	/* USER CODE END LPTIM2_Init 0 */

	/* USER CODE BEGIN LPTIM2_Init 1 */

	/* USER CODE END LPTIM2_Init 1 */
	hlptim2.Instance = LPTIM2;
	hlptim2.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
	hlptim2.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
	hlptim2.Init.UltraLowPowerClock.Polarity = LPTIM_CLOCKPOLARITY_RISING;
	hlptim2.Init.UltraLowPowerClock.SampleTime = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;
	hlptim2.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
	hlptim2.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
	hlptim2.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
	hlptim2.Init.CounterSource = LPTIM_COUNTERSOURCE_EXTERNAL;
	hlptim2.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
	hlptim2.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
	if (HAL_LPTIM_Init(&hlptim2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN LPTIM2_Init 2 */

	/* USER CODE END LPTIM2_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */
	//Init.Period should give 10 KHz
	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 10000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 15000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 8;
	sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 8;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 0;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 8191;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void)
{

	/* USER CODE BEGIN TIM7_Init 0 */

	/* USER CODE END TIM7_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM7_Init 1 */

	/* USER CODE END TIM7_Init 1 */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 0;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 8192;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM7_Init 2 */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 0;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 30000; //SCAMP is called at Fclock / 2 / 30000 = 10 KHz
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE END TIM7_Init 2 */

}

/**
 * @brief TIM13 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM13_Init(void)
{

	/* USER CODE BEGIN TIM13_Init 0 */

	/* USER CODE END TIM13_Init 0 */

	/* USER CODE BEGIN TIM13_Init 1 */

	/* USER CODE END TIM13_Init 1 */
	htim13.Instance = TIM13;
	htim13.Init.Prescaler = 250;
	htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim13.Init.Period = 500;
	htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM13_Init 2 */

	/* USER CODE END TIM13_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void)
{

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LedYellow_Pin|LedGreen_Pin|LedRed_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, RXTX_Pin|TX_ENA_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : IN_SW01_Pin */
	GPIO_InitStruct.Pin = IN_SW01_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(IN_SW01_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SwInt1_Pin */
	GPIO_InitStruct.Pin = SwInt1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SwInt1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : KEYER_DASH_Pin KEYER_DOT_Pin */
	GPIO_InitStruct.Pin = KEYER_DASH_Pin|KEYER_DOT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LedYellow_Pin LedGreen_Pin LedRed_Pin */
	GPIO_InitStruct.Pin = LedYellow_Pin|LedGreen_Pin|LedRed_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : RXTX_Pin TX_ENA_Pin */
	GPIO_InitStruct.Pin = RXTX_Pin|TX_ENA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : ENC_BUTTON_Pin */
	GPIO_InitStruct.Pin = ENC_BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(ENC_BUTTON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/**
 * @brief  Conversion complete callback in non blocking mode
 * @param  AdcHandle : ADC handle

 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
	/* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes */
#ifdef USE_DCACHE
	SCB_InvalidateDCache_by_Addr((uint32_t *) &aADCDualConvertedValues[BSIZE/2], 2*BSIZE);
#endif
	ADC_Stream0_Handler(1);
	/* Set variable to report DMA transfer status to main program */
	ubADCDualConversionComplete = SET;
}

/**
 * @brief  Conversion DMA half-transfer callback in non blocking mode
 * @param  hadc: ADC handle
 * */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	/* Invalidate Data Cache to get the updated content of the SRAM on the first half of the ADC converted data buffer: 32 bytes */
#ifdef USE_DCACHE
	SCB_InvalidateDCache_by_Addr((uint32_t *) &aADCDualConvertedValues[0], 2*BSIZE);
#endif
	ADC_Stream0_Handler(0);
	/* Reset variable to report DMA transfer status to main program */
	ubADCDualConversionComplete = RESET;
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	ValidAudioHalf = &AudioOut[BSIZE];
	//	LED_RED_ON;
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	ValidAudioHalf = &AudioOut[0];
	//	LED_RED_OFF;
}

void SystemClock_Config_For_OC(void)

{
	/* MCO2 is not correctly initialized by CubeMX if LPTIM or another peripheral is not enabled with the
	 * clock from the same PLL as MCO2.
	 */


	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
	/** Macro to configure the PLL clock source
	 */
	__HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
	/** Initializes the CPU, AHB and APB busses clocks
	 */





	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 480;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;

#ifdef USE_EXTERNAL_OSCILLATOR
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS; // External clock on pin 29 CN 11 (PF0/PH0)
#else
	RCC_OscInitStruct.HSEState = RCC_HSE_ON; //Xtal oscillator
#endif
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 2;
#ifdef CLK_600M_CPU_150M_ADC
	RCC_OscInitStruct.PLL.PLLN = 300;
#endif
#ifdef CLK_500M_CPU_120M_ADC
	//RCC_OscInitStruct.PLL.PLLN = 250;
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 400;
	XTalFreq = 25000000;
#endif
#ifdef CLK_500M_CPU_128M_ADC
	RCC_OscInitStruct.PLL.PLLN = 250;
#endif
#ifdef CLK_480M_CPU_120M_ADC
	RCC_OscInitStruct.PLL.PLLN = 240;
#endif
#ifdef CLK_600M_CPU_60M_ADC
	RCC_OscInitStruct.PLL.PLLN = 300;
#endif
#ifdef CLK_600M_CPU_160M_ADC
	RCC_OscInitStruct.PLL.PLLN = 300;
#endif
#ifdef CLK_600M_CPU_160M_ADC_XTAL25
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 480;
	XTalFreq = 25000000;
#endif
#ifdef CLK_620M_CPU_160M_ADC_XTAL25
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 496;
	XTalFreq = 25000000;
#endif
#ifdef CLK_640M_CPU_160M_ADC_XTAL25
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 512;
	XTalFreq = 25000000;
#endif
#ifdef CLK_600M_CPU_150M_ADC_XTAL25
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 480;
	XTalFreq = 25000000;
#endif
#ifdef CLK_600M_CPU_120M_ADC_XTAL25
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 480;
	XTalFreq = 25000000;
#endif
#ifdef CLK_600M_CPU_128M_ADC_XTAL25
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 480;
	XTalFreq = 25000000;
#endif

#ifdef CLK_600M_CPU_96M_ADC_XTAL25
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 480;
	XTalFreq = 25000000;
#endif

	XTalFreq += XTalFreq * XTAL_F_ERROR;

	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) //was FLASH_LATENCY_4
	{
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_LPTIM2
			|RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
	PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;


	PeriphClkInitStruct.PLL2.PLL2M = 4;
	PeriphClkInitStruct.PLL2.PLL2N = 240;
	PeriphClkInitStruct.PLL2.PLL2P = 16;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL3.PLL3P = 2;
	PeriphClkInitStruct.PLL3.PLL3Q = 8;
	PeriphClkInitStruct.PLL3.PLL3R = 4;
	PeriphClkInitStruct.PLL3.PLL3M = 4;
#ifdef CLK_600M_CPU_150M_ADC
	PeriphClkInitStruct.PLL3.PLL3N = 300;
#endif
#ifdef CLK_500M_CPU_120M_ADC

	PeriphClkInitStruct.PLL2.PLL2M = 4;
	PeriphClkInitStruct.PLL2.PLL2N = 38;
	PeriphClkInitStruct.PLL2.PLL2P = 24;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL3.PLL3M = 5;
	PeriphClkInitStruct.PLL3.PLL3N = 120;
	PeriphClkInitStruct.PLL3.PLL3R = 5;

#endif
#ifdef CLK_500M_CPU_128M_ADC
	PeriphClkInitStruct.PLL3.PLL3N = 256;
#endif
#ifdef CLK_480M_CPU_120M_ADC
	PeriphClkInitStruct.PLL3.PLL3N = 120;
#endif
#ifdef CLK_600M_CPU_60M_ADC
	PeriphClkInitStruct.PLL3.PLL3N = 120;
#endif
#ifdef CLK_600M_CPU_160M_ADC
	PeriphClkInitStruct.PLL3.PLL3N = 320;
#endif
#ifdef CLK_600M_CPU_160M_ADC_XTAL25

	PeriphClkInitStruct.PLL2.PLL2M = 4;
	PeriphClkInitStruct.PLL2.PLL2N = 38;
	PeriphClkInitStruct.PLL2.PLL2P = 24;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL3.PLL3M = 5;
	PeriphClkInitStruct.PLL3.PLL3N = 160;
	PeriphClkInitStruct.PLL3.PLL3R = 5;
#endif
#ifdef CLK_620M_CPU_160M_ADC_XTAL25

	PeriphClkInitStruct.PLL2.PLL2M = 4;
	PeriphClkInitStruct.PLL2.PLL2N = 38;
	PeriphClkInitStruct.PLL2.PLL2P = 24;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL3.PLL3M = 5;
	PeriphClkInitStruct.PLL3.PLL3N = 160;
	PeriphClkInitStruct.PLL3.PLL3R = 5;
#endif
#ifdef CLK_640M_CPU_160M_ADC_XTAL25

	PeriphClkInitStruct.PLL2.PLL2M = 4;
	PeriphClkInitStruct.PLL2.PLL2N = 38;
	PeriphClkInitStruct.PLL2.PLL2P = 24;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL3.PLL3M = 5;
	PeriphClkInitStruct.PLL3.PLL3N = 160;
	PeriphClkInitStruct.PLL3.PLL3R = 5;
#endif
#ifdef CLK_600M_CPU_150M_ADC_XTAL25

	PeriphClkInitStruct.PLL2.PLL2M = 4;
	PeriphClkInitStruct.PLL2.PLL2N = 38;
	PeriphClkInitStruct.PLL2.PLL2P = 24;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL3.PLL3M = 5;
	PeriphClkInitStruct.PLL3.PLL3N = 150;
	PeriphClkInitStruct.PLL3.PLL3R = 5;
#endif
#ifdef CLK_600M_CPU_120M_ADC_XTAL25

	PeriphClkInitStruct.PLL2.PLL2M = 4;
	PeriphClkInitStruct.PLL2.PLL2N = 38;
	PeriphClkInitStruct.PLL2.PLL2P = 24;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL3.PLL3M = 5;
	PeriphClkInitStruct.PLL3.PLL3N = 120;
	PeriphClkInitStruct.PLL3.PLL3R = 5;
#endif
#ifdef CLK_600M_CPU_128M_ADC_XTAL25

	PeriphClkInitStruct.PLL2.PLL2M = 4;
	PeriphClkInitStruct.PLL2.PLL2N = 38;
	PeriphClkInitStruct.PLL2.PLL2P = 24;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL3.PLL3M = 5;
	PeriphClkInitStruct.PLL3.PLL3N = 128;
	PeriphClkInitStruct.PLL3.PLL3R = 5;
#endif
#ifdef CLK_600M_CPU_96M_ADC_XTAL25

	PeriphClkInitStruct.PLL2.PLL2M = 4;
	PeriphClkInitStruct.PLL2.PLL2N = 38;
	PeriphClkInitStruct.PLL2.PLL2P = 24;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL3.PLL3M = 5;
	PeriphClkInitStruct.PLL3.PLL3N = 96;
	PeriphClkInitStruct.PLL3.PLL3R = 5;
#endif
	PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
	PeriphClkInitStruct.PLL2.PLL2FRACN = 0;

	PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_1;
	PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
	PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
	PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;

	PeriphClkInitStruct.Lptim2ClockSelection = RCC_LPTIM2CLKSOURCE_PLL2;
	PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Enable USB Voltage detector TODO: is it needed?
	 */
	HAL_PWREx_EnableUSBVoltageDetector();
	HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLL2PCLK, RCC_MCODIV_1);

	//	  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_HSE, RCC_MCODIV_3);
	//	  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_10);
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
	OVFDetected = OVF_TIMEOUT;
	/* Reset register IER */
	__HAL_ADC_DISABLE_IT(&hadc1, (ADC_IT_AWD1));
	__HAL_ADC_DISABLE_IT(&hadc2, (ADC_IT_AWD1));
}


void PrintUI(uint8_t* UartTXString)
{
#ifdef UART_UI
	HAL_UART_Transmit(&huart3, (uint8_t *) UartTXString, strlen(UartTXString), 100);
#endif
#ifdef USB_UI
	//	CDC_Transmit_FS(UartTXString, strlen((char *)UartTXString));

	tud_cdc_write(UartTXString, strlen((char *)UartTXString));
	tud_cdc_write_flush();
	HAL_Delay(1);
#endif
}

#ifdef CW_DECODER
void DisplayCW(void)
{
	static uint8_t PosColumn;
	static uint8_t PosRow = 13;
	if ((uint8_t)DecodedCWChar == 0)
		return;
	NCharReceived++;

	if (PosColumn++ >= 40)
	{
		PosColumn = 1;
		PosRow++;
		if (PosRow >= 13 + 4)
		{
			PosRow = 13;
		}
		//Clear line
		sprintf((char*)UartTXString, "\e[%d;%dH                                                         ", PosRow, PosColumn);
		PrintUI(UartTXString);
	}

	sprintf((char*)UartTXString, "\e[%d;%dH%c", PosRow, PosColumn, DecodedCWChar);
	PrintUI(UartTXString);

	DecodedCWChar = 0;
}
#endif

#ifdef USE_SCAMP
void TXSCAMPString(void)
{
	uint8_t s[100] = "SSS SSS SSS DE I4NZX";
	//uint8_t s[100] = "S";
	PrepareBits(s, & TXMessage);
}
#endif

void UserInput(void)
{
	volatile HAL_StatusTypeDef result;

	if (WSPRBeaconState == SEND_WSPR)
	{
		HAL_ADCEx_MultiModeStop_DMA(&hadc1);

		//		HAL_Delay(100);
		SendWSPR(); //endless loop, only way to exit is by CW keying.
		if (HAL_ADCEx_MultiModeStart_DMA(&hadc1,
				(uint32_t *)aADCDualConvertedValues,
				BSIZE   //Source code says transfer size is in bytes, but it is in number of transfers
				//We transfer BSIZE * 4 bytes, it equals 2 * BSIZE samples (1024) because we have full and half interrupts
		) != HAL_OK)
		{
			/* Start Error */
			Error_Handler();
		}
		TXSwitch(0);
		CarrierEnable(0);
		WSPRBeaconState = NO_FIX;
	}
#ifdef UART_UI
	__HAL_UART_SEND_REQ (&huart3, UART_RXDATA_FLUSH_REQUEST);
	__HAL_UART_CLEAR_OREFLAG (&huart3);
	result = (HAL_UART_Receive_IT(&huart3, (uint8_t *) UartRXString, 1 )) ;
#endif
	if (USBRXLength)
	{
		result = HAL_OK;
		USBRXLength = 0;
	}
	else
	{
		result = HAL_ERROR;
	}
	if ( tud_cdc_connected() )
	{
		// connected and there are data available
		if ( tud_cdc_available() )
		{
			// read data
			char buf[64];
			uint32_t count = tud_cdc_read(UartRXString, sizeof(UartRXString));
			result = HAL_OK;
		}
	}
	else
		result = HAL_ERROR;


	if (result == HAL_OK)
	{
		UartRXDataReady = RESET;
		switch (UartRXString[0])
		{
		case 43: //+
			RXVolume += 0.1;
			if (RXVolume > 1.0)
				RXVolume = 1.0;
			break;
		case 45: //-
			RXVolume -= 0.1;
			if (RXVolume < 0)
				RXVolume = 0;
			break;
			//Keyboard and encoder F step is 1/2 because most QSO are at freq. multiple of 500 Hz
		case 49: //1
			FminusClicked(1); break; //change to 2 for step equal to selection
		case 50: //2
			FplusClicked(1); break;  //change to 2 for step equal to selection
		case 51: //3
			SetFstep(5);  break;
		case 52: //4
			SetFstep(4);  break;
		case 53: //5
			SetFstep(3);  break;
		case 54: //6
			SetFstep(2);  break;
		case 55: //7
			SetFstep(1); break;
		case 56: //8
			SetFstep(0); break;
		case 57: //9
			SetFstep(9); break;
		case 66: //B
			DisableDisplay = 1; break;
		case 67: //C
			SendCWMessage(0); break;
		case 68: //D
			SendCWMessage(1); break;
		case 74: //J
			TxPowerOut = LOW_POWER_OUT;
			break;
		case 75: //K
			TxPowerOut = MID_POWER_OUT;
			break;
		case 76: //L
			TxPowerOut = MAX_POWER_OUT;
			break;
		case 87: //W
			if (ShowWF)
				ShowWF=0;
			else
				ShowWF=1;
			break;
		case 89: //Y
			SetWSPRPLLCoeff((double)LOfreq, FracDivCoeff, FracPWMCoeff);
			TransmittingWSPR = 1;
			SendWSPR();
			break;
		case 90: //Z
			uwTick = SystemSeconds = SystemMinutes = 0;
			break;
		case 108: //l
			SetMode((Mode)LSB); break;
		case 117: //u
			SetMode((Mode)USB); break;
		case 97: //a
			SetMode((Mode)AM); break;
		case 98: //b
			DisableDisplay = 0; break;
		case 99: //c
			SetMode((Mode)CW); break;
		case 102: //f
			SetAGC((Agctype)Fast);  break;
		case 103: //g
			keyer_speed -= 1;
			if (keyer_speed < 3)
				keyer_speed = 3;
			loadWPM(keyer_speed);
			break;
		case 104: //h
			keyer_speed += 1;
			if (keyer_speed > 50)
				keyer_speed = 50;
			loadWPM(keyer_speed);
			break;
		case 110: //n
			SetBW((Bwidth)Narrow);  break;
		case 114: //r
			TXSwitch(0);
			CarrierEnable(0);
			break;
		case 115: //s
			SetAGC((Agctype)Slow);  break;
		case 116: //t
			TXSwitch(1);
			CarrierEnable(1);
			break;
		case 118: //v
			//Test for PLL
			SetFracPLL(20);  break;
		case 119: //w
			SetBW((Bwidth)Wide);  break;
#ifdef USE_SCAMP
		case 122: //z
			TXSwitch(1);  //TX will be switched off at end of message
			CarrierEnable(1);
			TXSCAMPString();  break;
#endif





		}

		DisplayStatus();
	}

	int16_t DiffEncVal;


	EncVal = TIM4->CNT;
	DiffEncVal = (int32_t) (EncVal - LastEncVal);
	if (DiffEncVal < 0)
	{
		FplusClicked(-DiffEncVal); // One encoder click is two counts
		DisplayStatus();
		LastEncVal = EncVal;
	}
	if (DiffEncVal > 0)
	{
		FminusClicked(DiffEncVal); // One encoder click is two counts
		DisplayStatus();
		LastEncVal = EncVal;
	}

	//TODO: Refactor this code. Move constants to main.h and choose better variable's names.
	// Bandscope floor and range should be changed from UI

	/* Console escape codes:
	 *
	 *
	 *  \e[   equals ESC
	 * ESC[{line};{column}H   moves cursor to line #, column #
	 *
	 * ESC[38;5;{ID}m 	Set foreground color.
	 * ESC[48;5;{ID}m 	Set background color.
	 *
	 * Where {ID} should be replaced with the color index from 0 to 255 of the following color table:
	 * The table starts with the original 16 colors (0-15).
	 * The proceeding 216 colors (16-231) or formed by a 3bpc RGB value offset by 16, packed into a single value.
	 * The final 24 colors (232-255) are grayscale starting from a shade slighly lighter than black,
	 * ranging up to shade slightly darker than white.
	 *
	 * Only Putty seems to know about 256-color mode, minicom doesn't.
	 * Android terminal emulators doesn't even move cursor, only exception is DroidTerm.
	 *
	 */

	if (!DisableDisplay)
	{
		int32_t Temp = adc_ReadInternalTemp();
		SValue = 4 + 10 / 3.01 * log10(PeakAudioValue * 2000.0);
#ifdef WSPR_BEACON_MODE
		sprintf((char*)UartTXString, "\e[1;1HS %-4.1f     T %d:%2d:%2d  \r", SValue, DCF77Hour, (int)SystemMinutes, (int)SystemSeconds);
#else
		sprintf((char*)UartTXString, "\e[1;1HS %-4.1f TEMP %d      \r", SValue, Temp);
#endif
		PrintUI(UartTXString);

#ifdef CW_DECODER
		sprintf((char*)UartTXString, "\e[7;1HS %-4.1f, %-4.1f, %-4.1f, %d      ", CWLevel*100, SignalAverage*100, (CWLevel - BaseNoiseLevel)*100, CurrentAverageDah);
		PrintUI(UartTXString);
		DisplayCW();
#endif


#ifdef TEST_WF
#ifdef COLOR_BANDSCOPE
		//palette is 16 + 36R + 6G + B
		//we want black -> B -> BG -> G -> GR -> R -> RB
		static const uint8_t WFColorLookup[31] =
		{
				16,
				17, 18, 19, 20, 21,
				27, 33, 39, 45, 51,
				50, 49, 48, 47, 46,
				82, 118, 154, 190, 226,
				220, 214, 208, 202, 196,
				197, 198, 199, 200, 201
		};
#endif
#ifdef ASCII_BANDSCOPE

		static const uint8_t WFColorLookup[10] =
		{
				' ','.',':','-','=','+','*','#','%','@'
		};
#endif
		int i, j;
		uint8_t BucketColor;
		float StrongestSignal, BigBucketValue;
		uint8_t WFString[20];

		if (ShowWF) {

			sprintf((char*)UartTXString, "\e[11;1H");

			for (i = 256; i >= 0; i -= 8)
			{
				StrongestSignal = 0;
				for (j = 0; j < 8; j++)
				{
					if (StrongestSignal < WFBuffer[i + j])
						StrongestSignal = WFBuffer[i + j];
				}
#ifdef COLOR_BANDSCOPE
				BigBucketValue = 50 * log(StrongestSignal + 1.01);
				if (BigBucketValue >30)
					BigBucketValue =30;

				BucketColor = WFColorLookup[(uint8_t)BigBucketValue];
				sprintf((char*)WFString, "\e[48;5;%dm ", BucketColor);
#endif
#ifdef ASCII_BANDSCOPE
				BigBucketValue = 18 * log(StrongestSignal + 1.01);
				if (BigBucketValue >9)
					BigBucketValue =9;
				BucketColor = WFColorLookup[(uint8_t)BigBucketValue];
				sprintf((char*)WFString, "%c", BucketColor);
#endif
				strcat(UartTXString, (int8_t *)WFString);
			}
			for (i=FFTLEN-1; i>(FFTLEN-256); i -= 8)
			{
				StrongestSignal = 0;
				for (j = 0; j < 8; j++)
				{
					if (StrongestSignal < WFBuffer[i - j])
						StrongestSignal = WFBuffer[i - j];
				}
				BigBucketValue = 100 * log(StrongestSignal + 1);
				if (BigBucketValue >30)
					BigBucketValue =30;
#ifdef COLOR_BANDSCOPE
				BucketColor = WFColorLookup[(uint8_t)BigBucketValue];
				sprintf((char*)WFString, "\e[48;5;%dm ", BucketColor);
#endif
#ifdef ASCII_BANDSCOPE
				BigBucketValue = 18 * log(StrongestSignal + 1.01);
				if (BigBucketValue >9)
					BigBucketValue =9;
				BucketColor = WFColorLookup[(uint8_t)BigBucketValue];
				sprintf((char*)WFString, "%c", BucketColor);
#endif
				strcat(UartTXString, (int8_t *)WFString);
			}

			sprintf((char*)WFString, "\e[48;5;16m"); // set black background
			strcat(UartTXString, (int8_t *)WFString);
			PrintUI(UartTXString);
		}
#endif

		if (OVFDetected)
		{
			sprintf((char*)UartTXString, "\e[4;1HOVF\r");
			OVFDetected--;
			/* Clear ADC analog watchdog flag */
			__HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_AWD1);
			__HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_AWD1);
			if (!OVFDetected)
			{
				/* Reset register IER */
				__HAL_ADC_ENABLE_IT(&hadc1, (ADC_IT_AWD1));
				__HAL_ADC_ENABLE_IT(&hadc2, (ADC_IT_AWD1));
			}
		} else
		{
			sprintf((char*)UartTXString, "\e[4;1H   \r");
		}
		PrintUI(UartTXString);
	}

	/*
	static uint8_t n;
	sprintf((char*)UartTXString, "\e[8;1H\e[38;5;91m%d123456789012345678901234567890\e[T\r", n++);
#ifdef UART_UI
	HAL_UART_Transmit(&huart3, (uint8_t *) UartTXString, strlen(UartTXString), 100);
#endif
#ifdef USB_UI
	CDC_Transmit_FS(UartTXString, strlen(UartTXString));

#endif
	 */
}


void DisplayStatus(void)
{
	static char StringMode[8];
	static char StringWidth[8];
	static char StringAGC[8];
	static char StringStep[8];
	static char StringTxPower[8];

	if (!DisableDisplay)
	{
		switch(Fstep)
		{
		case 1:			strcpy(StringStep,"   1 "); break;
		case 10: 		strcpy(StringStep,"  10 "); break;
		case 100: 		strcpy(StringStep," 100 "); break;
		case 1000: 		strcpy(StringStep,"   1K"); break;
		case 9000: 		strcpy(StringStep,"   9K"); break;
		case 10000:		strcpy(StringStep,"  10K"); break;
		case 100000: 	strcpy(StringStep," 100K"); break;
		}

		switch(CurrentMode)
		{
		case LSB: strcpy(StringMode,"LSB"); break;
		case USB: strcpy(StringMode,"USB"); break;
		case AM: strcpy(StringMode,"AM"); break;
		case CW: strcpy(StringMode,"CW"); break;
		}
		switch (CurrentAGC)
		{
		case Fast: strcpy(StringAGC,"Fast"); break;
		case Slow: strcpy(StringAGC,"Slow"); break;
		}
		switch (CurrentBW)
		{
		case Narrow: strcpy(StringWidth,"Narrow"); break;
		case Wide: strcpy(StringWidth,"Wide"); break;
		}
		switch (TxPowerOut)
		{
		case LOW_POWER_OUT: strcpy(StringTxPower,"Low"); break;
		case MID_POWER_OUT: strcpy(StringTxPower,"Mid"); break;
		case MAX_POWER_OUT: strcpy(StringTxPower,"Max"); break;
		}
		//		sprintf((char *)UartTXString, "\e[3;1HFreq %5.3f  Step %s\e[5;1HMode %s BW %s AGGprovaprova %s ERR %d WPM %d PWR %s Volume %1.1f   \r", LOfreq/1000.f, StringStep, StringMode, StringWidth, StringAGC, TXFreqError, keyer_speed, StringTxPower, RXVolume);
		//sprintf((char *)UartTXString, "\e[6;1H123456789012345678901234567890123456789012345678901234567890\r");
		sprintf((char *)UartTXString, "\e[3;1HFreq %5.3f  Step %s\e[5;1HMode %s BW %s    \r", LOfreq/1000.f, StringStep, StringMode, StringWidth);
		//TODO: TinyUSB seems to have a 64 byte limit in USB out buffer. For now we split the string.
		//		Best solution would be to have an auto split write function

		PrintUI(UartTXString);
		sprintf((char *)UartTXString, "\e[6;1HAGC %s ERR %d WPM %d PWR %s Volume %1.1f   \r", StringAGC, TXFreqError, keyer_speed, StringTxPower, RXVolume);
		PrintUI(UartTXString);
	}
}


/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
void MX_TIM6_Init_Custom_Rate(void)
{

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 0;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;


#ifdef CLK_600M_CPU_150M_ADC
	htim6.Init.Period = 8191;
#endif
#ifdef CLK_500M_CPU_120M_ADC
	htim6.Init.Period = 8532; //was 8541;
#endif
#ifdef CLK_500M_CPU_128M_ADC
	htim6.Init.Period = 7999;
#endif
#ifdef CLK_480M_CPU_120M_ADC
	htim6.Init.Period = 8191;
#endif
#ifdef CLK_600M_CPU_60M_ADC
	htim6.Init.Period = 20479;
#endif
#ifdef CLK_600M_CPU_160M_ADC
	htim6.Init.Period = 7679;
#endif
#ifdef CLK_600M_CPU_160M_ADC_XTAL25
	htim6.Init.Period = 7679; //was 7679
#endif
#ifdef CLK_620M_CPU_160M_ADC_XTAL25
	htim6.Init.Period = 7935;
#endif
#ifdef CLK_640M_CPU_160M_ADC_XTAL25
	htim6.Init.Period = 8191;
#endif
#ifdef CLK_600M_CPU_150M_ADC_XTAL25
	htim6.Init.Period = 8191; //was 8191
#endif
#ifdef CLK_600M_CPU_120M_ADC_XTAL25
	htim6.Init.Period = 10239; //was 8191
#endif
#ifdef CLK_600M_CPU_128M_ADC_XTAL25
	htim6.Init.Period = 9599; //
#endif
#ifdef CLK_600M_CPU_96M_ADC_XTAL25
	htim6.Init.Period = 12800; //was 8191
#endif
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}



/**
 * @brief  Rx Transfer completed callback
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report end of DMA Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{

	/* Turn LED2 on: Transfer in reception process is correct */
	LED_RED_OFF;
}

void SetFOut(uint32_t FHz)
{
	volatile uint32_t DivN2;


	/* FOut = FXtal * M / N / P / MCODIV
FXtal = 8 MHz

Local oscillator is 10.7 - 40.7 MHz

MCODIV = 1
M = 4
P = 24
MCODIV = 1
	 */
	DivN2 = (((uint64_t)FHz * 24 * 4 * 0x2000) / (uint64_t)25000000) >> 13;
	//	FracN2 = (((uint64_t)FHz * 24 * 4* 0x2000) / (uint64_t)8000000) & 0x1FFF;

	__HAL_RCC_PLL2_DISABLE();
	__HAL_RCC_PLL2_CONFIG(4, DivN2, 24, 2, 2);
	__HAL_RCC_PLL2_ENABLE();
}


void SetFracPLL(uint32_t Coeff)
{
	volatile uint32_t i;
	__HAL_RCC_PLL2FRACN_DISABLE();
	for (i=0; i< 50; i++)
	{
		i++;
		i--;
	}
	//	i = READ_REG(RCC->PLLCFGR);
	/*	__DMB(); //Memory barrier protection does not solve the delay issue
	__ISB();
	__DSB();
	 */
	__HAL_RCC_PLL2FRACN_CONFIG(Coeff); // 0-8191, can be issued at any time  TODO: It seems necessary to have a delay between disable and set new value
	__HAL_RCC_PLL2FRACN_ENABLE();
}


/*
 * WSPR
 * 40 m band 7040.0 - 7040.2 kHz
 * 4 symbols spaced 1.4648 Hz
 * for 7040.1 PLL coeffs are N 450, M 17, P 94, FracDiv 48
 */


void SetWSPRPLLCoeff(double TXFreq, uint16_t *FracDivCoeff, uint16_t *FracPWMCoeff)
{

	volatile double TF, OutFHigherStep, OutF, MinDiff = 999999999;
	uint32_t m, n, p, od;
	volatile uint32_t fm, fn, fp, fod, FracDiv, i;
	LastTXFreq = (float)TXFreq;
#define TEST_COEFF 1
	for (i = 0; i < 4; i++) {
		TF = TXFreq + i * 1.4648f * TEST_COEFF; // WSPR shift
		MinDiff = 999999999;
		od = 1;
		//looking for coefficients just below the desired frequency. This is because the fractional divider generates lower frequencies with 0, and higher with 8191
		for (m = 2; m <= 25; m++) //was 64
		{
			for (n = 2; n <= 512; n++) //was 1
			{
				for (p = 2; p <= 128; p += 2) {
					OutF = XTalFreq * n / m / p / od;
					if (((TF - OutF) < MinDiff) && ((TF - OutF) > 0)
							&& ((XTalFreq * n / m) > 150000000.0)
							&& ((XTalFreq * n / m) < 960000000.0)) {
						MinDiff = abs(OutF - TF);

						fp = p;
						fn = n;
						fm = m;
						fod = od;
					}
				}
			}
		}
		if (fn < 511) {
			OutF = XTalFreq * fn / fm / fp / fod;
			OutFHigherStep = XTalFreq * (fn + 1) / fm / fp / fod;
			FracDiv = (uint32_t) ((TF - OutF) / (OutFHigherStep - OutF)
					* 8192 * 8); //FracDiv PWM has 8 levels
		} else {
			FracDiv = 8191 * 8;
		}

		FracDivPWM = LowestWSPRToneFracDivPWM = FracDiv & 0x07;
		FracDiv >>= 0x03;
		FracDivCoeff[i] = FracDiv;
		FracPWMCoeff[i] = FracDivPWM;
	}
	__HAL_RCC_PLL2_DISABLE();
	__HAL_RCC_PLL2_CONFIG(fm, fn, fp, 2, 1); //These parameters should stay the same for the 4 WSPR tones
	__HAL_RCC_PLL2_ENABLE();
}


void SetTXPLL(float TF)
{
#ifdef SCAMP_FSK
	uint32_t fm, fn, fp, fd;
	float fdCalc;

	fm = 14;
	fn = 447;  // Can be used only from abt 7002 to 7017, for now
	fp = 114;
	fdCalc = 8192.f * (TF * (float)fm * (float)fp / XTalFreq -(float)fn);
	fd = roundf(fdCalc);

	__HAL_RCC_PLL2_DISABLE();
	__HAL_RCC_PLL2_CONFIG(fm, fn, fp, 2, 1);
	__HAL_RCC_PLL2_ENABLE();
	SetFracPLL(fd);
	SpaceFracDiv = fd; //will be used for FSK
	MarkFracDiv = fd - 53; //see xls file for calculating this magic number
#else

	volatile float OutFHigherStep, OutF, MinDiff = 999999999;
	uint32_t m, n, p, od;
	volatile uint32_t fm, fn, fp, fod, FracDiv;


	MinDiff = 999999999;
	od = 1;
	//looking for coefficients just below the desired frequency. This is because the fractional divider generates lower frequencies with 0, and higher with 8191
	for (m = 2; m <= 25; m++) //was 64
	{
		for (n = 2; n <= 512; n++) //was 1
		{
			for (p = 2; p <= 128; p+=2)
			{
				OutF = XTalFreq * n / m / p / od;
				if (((TF - OutF) < MinDiff) && ((TF - OutF) > 0) && ((XTalFreq * n / m)> 150000000.0) && ((XTalFreq * n / m)< 960000000.0))
				{
					MinDiff = abs(OutF - TF);

					fp = p;
					fn = n;
					fm = m;
					fod = od;
				}
			}
		}
	}
	if (fn < 511)
	{
		OutF = XTalFreq * fn / fm / fp / fod;
		OutFHigherStep = XTalFreq * (fn + 1) / fm / fp / fod;
		FracDiv = (uint32_t) ((TF - OutF) / (OutFHigherStep - OutF)  * 8192);
	}
	else
	{
		FracDiv = 8191;
	}

	TXFreqError = MinDiff;
	__HAL_RCC_PLL2_DISABLE();
	__HAL_RCC_PLL2_CONFIG(fm, fn, fp, 2, 1);
	__HAL_RCC_PLL2_ENABLE();

	SetFracPLL(FracDiv);
#endif

}


void TXSwitch(uint8_t Status)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (Status)
	{
		TransmissionEnabled = 1;
		//TODO: TXFreq should be calculated in a low priority task every time F is changed, during RX. In this way TX would start immediately and without
		// audio noise caused by RX starving
		if (LastTXFreq != LOfreq)
		{
			SetTXPLL(LOfreq);
			LastTXFreq = LOfreq;
		}

		/*Configure GPIO pin : PC9 */
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		RELAY_TX_ON;
		LED_YELLOW_ON;
	}
	else
	{
		/*Configure GPIO pin : PC9 */
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode =  GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		RELAY_TX_OFF;
		LED_YELLOW_OFF;
		TransmissionEnabled = 0;

	}
}


void CarrierEnable(uint8_t Status)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (Status)
	{
		//TODO: Ramping
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
		//approx bias vs. VRMS 50 Ohm out vs power
		//4095 17.1  5.8
		//2048 13.1  3.4
		//1024 7.5	 1.1
		// 256 3.8   0.3
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, TxPowerOut); // TX gate bias
		/*Configure GPIO pin : PC9 */
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		TXCarrierEnabled = 1;
		LED_GREEN_ON;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0); // TX gate bias. TODO: Need ramping
		TXCarrierEnabled = 0;
		/*Configure GPIO pin : PC9 */
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode =  GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		LED_GREEN_OFF;
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	while(1)
	{
		if((os_time % 50) == 0)  // blink fast the two leds in case of errors
			LED_switch();
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
