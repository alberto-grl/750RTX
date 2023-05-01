
#include "globals.h"

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

// TODO: refactor. Tested only with TX_DELAY and SEMI_QSK defined.

//#ifdef KEYER
// Iambic Morse Code Keyer Sketch, Contribution by Uli, DL2DBG. Copyright (c) 2009 Steven T. Elliott Source: http://openqrp.org/?p=343,  Trimmed by Bill Bishop - wrb[at]wrbishop.com.  This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version. This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details: Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
//  QCX-SSB.ino - https://github.com/threeme3/QCX-SSB
//
//  Copyright 2019, 2020, 2021   Guido PE1NNZ <pe1nnz@amsat.org>
//
// Adapted by Alberto I4NZX
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
/*
 *
int keyer_speed = 15;
static unsigned long ditTime;                    // No. mseconds per dit
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

	ditTime = (1200ULL)/wpm;   //ditTime = 1200/wpm; time in msec

}
//#endif //KEYER



void switch_rxtx(uint8_t tx_enable){

#ifdef TX_DELAY
#ifdef SEMI_QSK
	if(!(semi_qsk_timeout))
#endif
		if((txdelay) && (tx_enable) && (!(tx))){  // key-up TX relay in advance before actual transmission
			//Save AGC status
					Saved_pk = pk;

			TXSwitch(1);

			// TODO mettere ritardo   delay(F_MCU / 16000000 * txdelay);
			HAL_Delay(txdelay);
		}
#endif //TX_DELAY
	tx = tx_enable;
	if(tx_enable){  // tx


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
			// restore AGC setting

						pk = Saved_pk;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
#ifdef USE_KEYER
	DoKeyer();
#endif
#ifdef USE_SCAMP
	TXScamp();
#endif

}



void DoKeyer(void)
{

#ifdef SEMI_QSK
	if((semi_qsk_timeout) && (HAL_GetTick() > (semi_qsk_timeout - 100)))
	{  // RX buffers need to be cleared before restoring AGC level
		TXSwitch(0);

	}  // delayed QSK RX
	if((semi_qsk_timeout) && (HAL_GetTick() > semi_qsk_timeout)){
			TXSwitch(0);
			semi_qsk_timeout = 0;
						pk = Saved_pk;
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

//Begin CWMessage

const char m2c[] = "~ ETIANMSURWDKGOHVF*L*PJBXCYZQ**54S3***2**+***J16=/***H*7*G*8*90************?_****\"**.****@***'**-********;!*)*****,****:****";


uint8_t delayWithKeySense(uint32_t ms){
  uint32_t event = HAL_GetTick() + ms;
  while(HAL_GetTick() < event){
    if(KEYER_DASH || KEYER_DOT){
      return 1;  // stop when button/key pressed
    }
  }
  return 0;
}

char cw_msg[6][48] = { "CQ CQ CQ CQ DE I4NZX I4NZX I4NZX K", "CQ SOTA DE I4NZX/P I4NZX/P K", "GE TKS 5NN 5NN NAME IS GUIDO GUIDO HW?", "FB RPTR TX 5W 5W ANT INV V 73 CUAGN", "73 TU E E", "PE1NNN" };


uint8_t cw_msg_interval = 5; // number of seconds CW message is repeated
uint32_t cw_msg_event = 0;
uint8_t cw_msg_id = 0; // selected message

int cw_tx_char(char ch){    // Transmit message in CW
  char sym;
  for(uint8_t j = 0; (sym = (m2c[j])); j++){  // lookup msg[i] in m2c, skip if not found
    if(sym == ch){  // found -> transmit CW character j
      uint8_t k = 0x80; for(; !(j & k); k >>= 1); k >>= 1; // shift start of cw code to MSB
      if(k == 0) delayWithKeySense(ditTime * 4); // space -> add word space (was 4)
      else {
        for(; k; k >>= 1){ // send dit/dah one by one, until everythng is sent
          switch_rxtx(1);  // key-on  tx
          if(delayWithKeySense(ditTime * ((j & k) ? 3 : 1))){ switch_rxtx(0); return 1; } // symbol: dah or dih length
          switch_rxtx(0);  // key-off tx
          if(delayWithKeySense(ditTime)) return 1;   // add symbol space
        }
        if(delayWithKeySense(ditTime * 2)) return 1; // add letter space (was 2)
      }
      break; // next character
    }
  }
  return 0;
}

int cw_tx(char* msg){
  for(uint8_t i = 0; msg[i]; i++){  // loop over message
    if(cw_tx_char(msg[i])) return 1;
  }
  return 0;
}


void SendCWMessage(uint8_t MessageNo)
{
	cw_tx(cw_msg[MessageNo]);
}
//#pragma GCC pop_options
