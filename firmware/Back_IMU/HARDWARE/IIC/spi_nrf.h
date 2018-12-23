#ifndef __SPI_H
#define __SPI_H
#include "include.h"

#define SPI_CE_H()   GPIO_SetBits(GPIOB, GPIO_Pin_10) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOB, GPIO_Pin_10)

#define SPI_CSN_H()  GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define SPI_CSN_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_12)

void Spi1_Init(void);
u8 Spi_RW(u8 dat);
		 
#endif


