/*
 * SCAMP.h
 *
 *  Created on: 16 feb 2022
 *      Author: albytest
 */

#ifndef INC_SCAMP_H_
#define INC_SCAMP_H_

#include <stdint.h>

//Bit period in usec / callback period in usec
#define N_SAMPLES_PER_BIT (40000 / 100)


/* in memory buffer encoder / decoder callbacks */
typedef struct _eccfr_code_word_put_mem_buf_struct
{
  uint16_t *code_word_array;
  uint8_t cur_word;
  uint8_t max_words;
} eccfr_code_word_put_mem_buf_struct;


typedef void (*eccfr_code_word_put)(uint16_t, void *);

void eccfr_raw_bytes_to_code_words(uint8_t *, uint8_t, eccfr_code_word_put, void *);

#endif /* INC_SCAMP_H_ */
