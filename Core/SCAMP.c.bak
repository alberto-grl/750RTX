/*
 * SCAMP.c
 *
 *  Created on: 16 feb 2022
 *      Author: albytest
 */

#include "stdio.h"
#include <SCAMP.h>
#include "main.h"
#include "Globals.h"

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

#ifdef USE_SCAMP
const uint8_t ecc_6bit_codesymbols[60] = {'\0',   '\b',   '\r',    ' ',    '!',   0x22,   0x27,    '(',
		')',    '*',    '+',    ',',    '-',    '.',    '/',    '0',
		'1',    '2',    '3',    '4',    '5',    '6',    '7',    '8',
		'9',    ':',    ';',    '=',    '?',    '@',    'A',    'B',
		'C',    'D',    'E',    'F',    'G',    'H',    'I',    'J',
		'K',    'L',    'M',    'N',    'O',    'P',    'Q',    'R',
		'S',    'T',    'U',    'V',    'W',    'X',    'Y',    'Z',
		0x5C,   '^',    '`',    '~' };
/* Last 4 symbols can be interpreted as diacritical marks, 0x5C is diaeresis/umlaut */
/* 0x27 can be interpreted as acute diacritical mark */


const uint16_t golay_matrix[12] =
{
		0b110111000101,
		0b101110001011,
		0b011100010111,
		0b111000101101,
		0b110001011011,
		0b100010110111,
		0b000101101111,
		0b001011011101,
		0b010110111001,
		0b101101110001,
		0b011011100011,
		0b111111111110
};


uint32_t eccfr_add_reversal_bits(uint32_t codeword)
{
	uint32_t outword = 0;
	uint8_t i;

	for (i=0;i<6;i++)
	{
		if (i>0)
			outword = (outword << 5);
		codeword = (codeword << 4);
		uint8_t temp = (codeword >> 24) & 0x0F;
		outword |= (temp | (((temp & 0x08) ^ 0x08) << 1));
	}
	return outword;
}


void eccfr_code_word_put_mem_buf(uint16_t code, void *st)
{
	eccfr_code_word_put_mem_buf_struct *s = (eccfr_code_word_put_mem_buf_struct *) st;
	if (s->cur_word < s->max_words)
		s->code_word_array[s->cur_word++] = code;
}


/* convert character to 6-bit code if it exists */
/* should probably replace this with an inverse look up table later */
uint8_t eccfr_find_code_in_table(uint8_t c)
{
	uint8_t i;
	if ((c >= 'a') && (c <= 'z')) c -= 32;
	if (c =='\n') c = '\r';
	if (c == 127) c = '\b';
	for (i=1;i<(sizeof(ecc_6bit_codesymbols)/sizeof(uint8_t));i++)
		if (ecc_6bit_codesymbols[i] == c) return i;
	return 0xFF;
}

/* encode 8 bit bytes to 12 bit code words.
   code words that correspond to a 6-bit symbols are encoded as 6 bits.
   otherwise they are encoded as an 8-bit binary raw data word.
   If a byte that can be encoded as a 6 bit symbol precedes one that can
   not be encoded as a 6 bit symbol, and there is an extra symbol slot
   in the current word, fill it with a zero. */
void eccfr_bytes_to_code_words(uint8_t *bytes, uint8_t num_bytes, eccfr_code_word_put ecwp, void *st)
{
	uint8_t cur_byte = 0;
	uint16_t last_code_word = 0;
	uint16_t code_word;
	while (cur_byte < num_bytes)
	{
		uint8_t b = bytes[cur_byte++];
		uint8_t code1 = eccfr_find_code_in_table(b);
		if (code1 == 0xFF)
		{
			code_word = ((uint16_t)(0xF00)) | b;
		} else
		{
			code_word = (uint16_t)code1;
			if (cur_byte < num_bytes)
			{
				b = bytes[cur_byte];
				uint8_t code2 = eccfr_find_code_in_table(b);
				if (code2 != 0xFF)
				{
					code_word |= (((uint16_t)code2) << 6);
					cur_byte++;
				}
			}
			if (code_word == last_code_word)
				ecwp(0, st);
		}
		ecwp(code_word, st);
		last_code_word = code_word;
	}
}

/* encode 8 bit bytes as 12 bit code words, always encoding as 8-bit raw */
void eccfr_raw_bytes_to_code_words(uint8_t *bytes, uint8_t num_bytes, eccfr_code_word_put ecwp, void *st)
{
	uint8_t cur_byte = 0;
	while (cur_byte < num_bytes)
	{
		uint8_t b = bytes[cur_byte++];
		ecwp( ((uint16_t)(0xF00)) | b, st );
	}
}

uint16_t golay_mult(uint16_t wd_enc)
{
	uint16_t enc = 0;
	uint8_t i;
	for (i=12;i>0;)
	{
		i--;
		if (wd_enc & 1) enc ^= golay_matrix[i];
		wd_enc >>= 1;
	}
	return enc;
}



uint32_t golay_encode(uint16_t wd_enc)
{
	uint16_t enc = golay_mult(wd_enc);
	return (((uint32_t)enc) << 12) | wd_enc;
}



void PrepareBits(uint8_t *s, OutData_t *TXMessage)
{

	uint16_t codes[255];

	uint8_t l, i;
	eccfr_code_word_put_mem_buf_struct ecwpmbs;
	l = strlen((char*) s);

	ecwpmbs.code_word_array = codes;
	ecwpmbs.cur_word = 0;
	ecwpmbs.max_words = sizeof(codes) / sizeof(uint16_t);
	eccfr_bytes_to_code_words((uint8_t*) s, l, eccfr_code_word_put_mem_buf, &ecwpmbs);
	//input string is converted to 12 bits tokens, each contains two chars

	for (i = 0; i < ecwpmbs.cur_word; i++)
	{
		TXMessage->OutCodes[i + 2] = golay_encode(ecwpmbs.code_word_array[i]); //leave space for preamble and sync word
		TXMessage->OutCodes[i + 2] = eccfr_add_reversal_bits(TXMessage->OutCodes[i + 2]);
	}
	TXMessage->OutCodes[0] = 0x3FFFFFC3;  //preamble
	TXMessage->OutCodes[1] = 0x3ED19D1E;  //sync

	TXMessage->OutLength = ecwpmbs.cur_word + 2;
}

uint8_t GetTXBit(OutData_t* TXMessage, uint32_t n)
{
	uint8_t i;
	uint32_t Frame;
	Frame = TXMessage->OutCodes[n / 30];

	if (Frame & (1 << (29 -(n % 30))))
		return 1;
	else
		return 0;

}


void TXScamp(void)
{
	/*
	 * Called every 100 usec by TIM7
	 *
	 */
	static uint32_t i, j, k, TXBitN;
	static uint32_t TXSamplesLeft;
	static float TXLevel, LastTXLevel;
	uint8_t TXBit;

	volatile static long testcount0, testcount1;


	if (TXMessage.OutLength == 0)
	{
		return;
	}


	if (TXSamplesLeft == 0)
	{
		TXBit = GetTXBit(& TXMessage, TXBitN);
		if (TXBit == 1)
		{
			TXLevel = 1.0;
		}
		if (TXBit == 0)
		{
			TXLevel = 0.0;
		}
		TXSamplesLeft = N_SAMPLES_PER_BIT;


		if (TXBitN < TXMessage.OutLength * 30) //N. bits in SCAMP frame
		{
			TXBitN++;
			TXSamplesLeft = N_SAMPLES_PER_BIT;
		}
		else
		{
			TXMessage.OutLength = 0; //TX finished
			TXBitN = 0;
			TXSamplesLeft = N_SAMPLES_PER_BIT;
			TXLevel = 0.0;
			CarrierEnable(0);
			TXSwitch(0);
		}
	}
#endif

#ifdef SCAMP_OOK
	if (TXLevel > 0.5)
	{
		CarrierEnable(1);
		testcount1++;
	}
	else
	{
		CarrierEnable(0);
		testcount0++;
	}
#endif
#ifdef SCAMP_FSK
	if (TXLevel > 0.5)
	{
		if (LastTXLevel < 0.5)
			SetFracPLL(SpaceFracDiv);
		testcount1++;
	}
	else
	{
		if (LastTXLevel > 0.5)
		   SetFracPLL(MarkFracDiv);
		testcount0++;
	}
	LastTXLevel = TXLevel;

	if(TXSamplesLeft > 0)
		TXSamplesLeft--;

}
#endif
//#pragma GCC pop_options
