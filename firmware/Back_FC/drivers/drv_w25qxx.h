#ifndef __DRIVER_AT45DB_H
#define __DRIVER_AT45DB_H
#include "stm32f4xx.h"

#ifdef  USE_SPI1
#define SPI1_CLOCK                RCC_APB2Periph_SPI1
#define SPI1_AF                   GPIO_AF_SPI1
#define SPI1_GPIO_CLOCK           RCC_AHB1Periph_GPIOA
#define SPI1_SCK_PORT             GPIOA
#define SPI1_MISO_PORT            GPIOA
#define SPI1_MOSI_PORT            GPIOA
#define SPI1_SCK_PIN              GPIO_Pin_5
#define SPI1_MISO_PIN             GPIO_Pin_6
#define SPI1_MOSI_PIN             GPIO_Pin_7
#define SPI1_SCK_SOURCE           GPIO_PinSource5
#define SPI1_MISO_SOURCE          GPIO_PinSource6
#define SPI1_MOSI_SOURCE          GPIO_PinSource7
#define SPI1_DMA_RX_ST               DMA1_Stream3
#define SPI1_DMA_RX_CHANNEL       DMA_Channel_0
#define SPI1_DMA_RX_FLAGS         (DMA_IT_TEIF3 | DMA_IT_DMEIF3 | DMA_IT_FEIF3 | DMA_IT_TCIF3 | DMA_IT_HTIF3)
#define SPI1_DMA_RX_IRQ           DMA1_Stream3_IRQn
#define SPI1_DMA_RX_HANDLER       DMA1_Stream3_IRQHandler
#define SPI1_DMA_TX_ST               DMA1_Stream4
#define SPI1_DMA_TX_CHANNEL       DMA_Channel_0
#define SPI1_DMA_TX_FLAGS         (DMA_IT_TEIF4 | DMA_IT_DMEIF4 | DMA_IT_FEIF4 | DMA_IT_TCIF4 | DMA_IT_HTIF4)
#endif

/* Jedec Flash Information structure */
typedef struct
{
    uint8_t initialized;
    uint16_t sector_size;
    uint16_t sector_count;
    uint32_t capacity;
} flash_info_t;


void Flash_Init(void);
void Flash_SectorErase(uint32_t address,uint8_t state);
void Flash_PageRead(uint32_t address,uint8_t* buffer,  uint32_t lenght);
void Flash_PageWrite(uint32_t address,uint8_t* buffer,  uint32_t lenght);
void Flash_SectorsRead(uint32_t address,uint8_t *buffer,uint16_t count);
void Flash_SectorsWrite(uint32_t address,uint8_t *buffer,uint16_t count);
flash_info_t *Flash_GetInfo(void);

#endif
