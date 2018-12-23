
#ifndef _FIFO_H_
#define _FIFO_H_

#include "stm32f4xx.h" 

#define true 1
#define false 0

typedef struct _fifo {
	uint8_t* buf;
	uint16_t length;
	uint16_t head;
	uint16_t tail;
} fifo_t;


uint8_t fifo_read_ch(fifo_t* fifo, uint8_t* ch);
uint8_t fifo_write_ch(fifo_t* fifo, uint8_t ch);
uint16_t fifo_free(fifo_t* fifo);
uint16_t fifo_used(fifo_t* fifo);
void fifo_init(fifo_t* fifo, uint8_t* buf, uint16_t length);

#endif  /*_FIFO_H_*/



