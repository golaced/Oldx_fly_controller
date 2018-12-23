#ifndef __SPI_H
#define __SPI_H
#include "include.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//SPI 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

				    
// SPI总线速度设置 
#define SPI_SPEED_2   		0
#define SPI_SPEED_4   		1
#define SPI_SPEED_8   		2
#define SPI_SPEED_16  		3
#define SPI_SPEED_32 		4
#define SPI_SPEED_64 		5
#define SPI_SPEED_128 		6
#define SPI_SPEED_256 		7
						  	    													  
void SPI2_Init(void);			 //初始化SPI1口
u8 Spi_RW(u8 TxData);//SPI1总线读写一个字节

#define MPU9250 0
#define CS_ACCGRO 0 
#define NRF2401 1
#define MS5611  2
#define CS_FLASH 3
void SPI_CS(u8 sel,u8 set);
void SPI_Receive(uint8_t *pData, uint16_t Size);
#define SPI_CE_H()   GPIO_SetBits(GPIOC, GPIO_Pin_2) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOC, GPIO_Pin_2)

#define SPI_CSN_H()  SPI_CS(NRF2401,1)//GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define SPI_CSN_L()  SPI_CS(NRF2401,0)//GPIO_ResetBits(GPIOB, GPIO_Pin_12) 


#endif

