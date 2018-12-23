#ifndef _I2C_SOFT_H
#define	_I2C_SOFT_H

#include "stm32f4xx.h"
#include "time.h"

#define SCL_IMU1_H         ANO_GPIO_I2C_IMU1->BSRRL = I2C_IMU1_Pin_SCL
#define SCL_IMU1_L         ANO_GPIO_I2C_IMU1->BSRRH = I2C_IMU1_Pin_SCL
#define SDA_IMU1_H         ANO_GPIO_I2C_IMU1->BSRRL = I2C_IMU1_Pin_SDA
#define SDA_IMU1_L         ANO_GPIO_I2C_IMU1->BSRRH = I2C_IMU1_Pin_SDA
#define SCL_IMU1_read      ANO_GPIO_I2C_IMU1->IDR  & I2C_IMU1_Pin_SCL
#define SDA_IMU1_read      ANO_GPIO_I2C_IMU1->IDR  & I2C_IMU1_Pin_SDA

/***************I2C GPIO定义******************/
#define ANO_GPIO_I2C_IMU1	GPIOA
#define I2C_IMU1_Pin_SCL		GPIO_Pin_6
#define I2C_IMU1_Pin_SDA		GPIO_Pin_7
#define ANO_RCC_I2C_IMU1		RCC_AHB1Periph_GPIOA
/*********************************************/

#define SCL_IMU1_H4         ANO_GPIO_I2C_IMU14->BSRRL = I2C_IMU1_Pin_SCL4
#define SCL_IMU1_L4         ANO_GPIO_I2C_IMU14->BSRRH = I2C_IMU1_Pin_SCL4
#define SDA_IMU1_H4         ANO_GPIO_I2C_IMU14->BSRRL = I2C_IMU1_Pin_SDA4
#define SDA_IMU1_L4         ANO_GPIO_I2C_IMU14->BSRRH = I2C_IMU1_Pin_SDA4
#define SCL_IMU1_read4      ANO_GPIO_I2C_IMU14->IDR  & I2C_IMU1_Pin_SCL4
#define SDA_IMU1_read4      ANO_GPIO_I2C_IMU14->IDR  & I2C_IMU1_Pin_SDA4

/***************I2C GPIO定义******************/
#define ANO_GPIO_I2C_IMU14	GPIOB
#define I2C_IMU1_Pin_SCL4		GPIO_Pin_8
#define I2C_IMU1_Pin_SDA4		GPIO_Pin_9
#define ANO_RCC_I2C_IMU14		RCC_AHB1Periph_GPIOB
/*********************************************/
extern volatile u8 I2C_IMU1_FastMode;

void I2c_IMU1_Soft_Init(void);
void I2c_IMU1_Soft_SendByte(u8 SendByte);
u8 I2c_IMU1_Soft_ReadByte(u8);

u8 IIC_IMU1_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_IMU1_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
u8 IIC_IMU1_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_IMU1_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

#endif
