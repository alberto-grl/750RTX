
#include "globals.h"

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

// TODO: refactor. Tested only with TX_DELAY and SEMI_QSK defined.

//#ifdef KEYER
// Iambic Morse Code Keyer Sketch, Contribution by Uli, DL2DBG. Copyright (c) 2009 Steven T. Elliott Source: http://openqrp.org/?p=343,  Trimmed by Bill Bishop - wrb[at]wrbishop.com.  This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version. This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details: Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.

/*
int keyer_speed = 15;
static unsigned long ditTime;                    // No. nseconds per dit
static uint8_t keyerControl;
static uint8_t keyerState;
static uint8_t keyer_mode = 1; //->  era 2:SINGLE
static uint8_t keyer_swap = 0; //->  DI/DAH

static uint32_t ktimer;
static int Key_state;
int debounce;
static uint32_t semi_qsk_timeout;
static uint8_t tx = 0;
static uint8_t txdelay = 0;

 */

void update_PaddleLatch() // Latch dit and/or dah press, called by keyer routine
{
	if(KEYER_DASH) {
		keyerControl |= keyer_swap ? DAH_L : DIT_L;
	}
	if(KEYER_DOT) {
		keyerControl |= keyer_swap ? DIT_L : DAH_L;
	}
}

void loadWPM (int wpm) // Calculate new time constants based on wpm value
{

	ditTime = (1200ULL)/wpm;   //ditTime = 1200/wpm;

}
//#endif //KEYER



void switch_rxtx(uint8_t tx_enable){

#ifdef TX_DELAY
#ifdef SEMI_QSK
	if(!(semi_qsk_timeout))
#endif
		if((txdelay) && (tx_enable) && (!(tx))){  // key-up TX relay in advance before actual transmission
			TXSwitch(1);

			// TODO mettere ritardo   delay(F_MCU / 16000000 * txdelay);
			HAL_Delay(txdelay);
		}
#endif //TX_DELAY
	tx = tx_enable;
	if(tx_enable){  // tx

	//Save AGC status
		Saved_hangcnt = hangcnt;
		Saved_pk = pk;

#ifdef SEMI_QSK
		semi_qsk_timeout = 0;
#endif
		if (tx_enable)
			CarrierEnable(1);
	}
	else
	{  // rx
		if(!(semi_qsk_timeout)){

			CarrierEnable(0);
#ifdef SEMI_QSK
#ifdef KEYER
			semi_qsk_timeout = HAL_GetTick() + ditTime * 8;
#else
			semi_qsk_timeout = HAL_GetTick() + 8 * 8;  // no keyer? assume dit-time of 20 WPM
#endif //KEYER
#endif //SEMI_QSK

		} else {
			// restore AGC setting

			hangcnt = Saved_hangcnt;
			pk = Saved_pk;
#ifdef SEMI_QSK
			semi_qsk_timeout = 0;
#endif
		}
	}

	if(tx_enable){ // tx
		TXSwitch(1);
	} else {  // rx
#ifdef KEY_CLICK
		if(OCR1BL != 0) {
			for(uint16_t i = 0; i != 31; i++) {   // ramp down of amplitude: soft falling edge to prevent key clicks
				OCR1BL = lut[pgm_read_byte_near(ramp[i])];
				delayMicroseconds(60);
			}
		}
#endif //KEY_CLICK

#ifdef SEMI_QSK
		if((!semi_qsk_timeout) )   // enable RX when no longer in semi-qsk phase; so RX and NTX/PTX outputs are switching only when in RX mode
#endif //SEMI_QSK
		{
			TXSwitch(0);
			semi_qsk_timeout = 0;
		}
	}
}





void DoKeyer(void)
{

#ifdef SEMI_QSK
	if((semi_qsk_timeout) && (HAL_GetTick() > semi_qsk_timeout)){
		TXSwitch(0);
		semi_qsk_timeout = 0;
	}  // delayed QSK RX
#endif

	if(keyer_mode != SINGLE){  // check DIT/DAH keys for CW

		switch(keyerState){ // Basic Iambic Keyer, keyerControl contains processing flags and keyer mode bits, Supports Iambic A and B, State machine based, uses calls to millis() for timing.
		case IDLE: // Wait for direct or latched paddle press
			if((KEYER_DASH) ||
					(KEYER_DOT) ||
					(keyerControl & 0x03))
			{
#ifdef CW_MESSAGE
				cw_msg_event = 0;  // clear cw message event
#endif //CW_MESSAGE
				update_PaddleLatch();
				keyerState = CHK_DIT;
			}
			break;
		case CHK_DIT: // See if the dit paddle was pressed
			if(keyerControl & DIT_L) {
				keyerControl |= DIT_PROC;
				ktimer = ditTime;
				keyerState = KEYED_PREP;
			} else {
				keyerState = CHK_DAH;
			}
			break;
		case CHK_DAH: // See if dah paddle was pressed
			if(keyerControl & DAH_L) {
				ktimer = ditTime*3;
				keyerState = KEYED_PREP;
			} else {
				keyerState = IDLE;
			}
			break;
		case KEYED_PREP: // Assert key down, start timing, state shared for dit or dah
			Key_state = HIGH;
			switch_rxtx(Key_state);
			ktimer += HAL_GetTick();                 // set ktimer to interval end time
			keyerControl &= ~(DIT_L + DAH_L);   // clear both paddle latch bits
			keyerState = KEYED;                 // next state
			break;
		case KEYED: // Wait for timer to expire
			if(HAL_GetTick() > ktimer) {            // are we at end of key down ?
				Key_state = LOW;
				switch_rxtx(Key_state);
				ktimer = HAL_GetTick() + ditTime;    // inter-element time
				keyerState = INTER_ELEMENT;     // next state
			} else if(keyerControl & IAMBICB) {
				update_PaddleLatch();           // early paddle latch in Iambic B mode
			}
			break;
		case INTER_ELEMENT:
			// Insert time between dits/dahs
			update_PaddleLatch();               // latch paddle state
			if(HAL_GetTick() > ktimer) {            // are we at end of inter-space ?
				if(keyerControl & DIT_PROC) {             // was it a dit or dah ?
					keyerControl &= ~(DIT_L + DIT_PROC);   // clear two bits
					keyerState = CHK_DAH;                  // dit done, check for dah
				} else {
					keyerControl &= ~(DAH_L);              // clear dah latch
					keyerState = IDLE;                     // go idle
				}
			}
			break;
		}
	}
}
//#pragma GCC pop_options
