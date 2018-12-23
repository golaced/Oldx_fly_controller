#ifndef _I2C_SOFT_H
#define	_I2C_SOFT_H

#include "stm32f4xx.h"
#include "time.h"

#define SCL_H         ANO_GPIO_I2C->BSRRL = I2C_Pin_SCL
#define SCL_L         ANO_GPIO_I2C->BSRRH = I2C_Pin_SCL
#define SDA_H         ANO_GPIO_I2C->BSRRL = I2C_Pin_SDA
#define SDA_L         ANO_GPIO_I2C->BSRRH = I2C_Pin_SDA
#define SCL_read      ANO_GPIO_I2C->IDR  & I2C_Pin_SCL
#define SDA_read      ANO_GPIO_I2C->IDR  & I2C_Pin_SDA

#define SCL_H_F         ANO_GPIO_I2C->BSRRL = I2C_Pin_SCL_F
#define SCL_L_F         ANO_GPIO_I2C->BSRRH = I2C_Pin_SCL_F
#define SDA_H_F         ANO_GPIO_I2C->BSRRL = I2C_Pin_SDA_F
#define SDA_L_F        ANO_GPIO_I2C->BSRRH = I2C_Pin_SDA_F
#define SCL_read_F      ANO_GPIO_I2C->IDR  & I2C_Pin_SCL_F
#define SDA_read_F      ANO_GPIO_I2C->IDR  & I2C_Pin_SDA_F
/***************I2C GPIO定义******************/
#define ANO_GPIO_I2C	GPIOC

#define I2C_Pin_SCL_F		GPIO_Pin_5
#define I2C_Pin_SDA_F		GPIO_Pin_4

#define I2C_Pin_SCL		GPIO_Pin_4
#define I2C_Pin_SDA		GPIO_Pin_5

#define ANO_RCC_I2C		RCC_AHB1Periph_GPIOC
/*********************************************/
extern volatile u8 I2C_FastMode;

void I2c_Soft_Init(void);
void I2c_Soft_SendByte(u8 SendByte);
u8 I2c_Soft_ReadByte(u8);

//int I2c_Soft_Single_Write(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
//int I2c_Soft_Single_Read(u8 SlaveAddress,u8 REG_Address);
//int I2c_Soft_Mult_Read(u8 SlaveAddress,u8 REG_Address,u8 * ptChar,u8 size);

u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

#endif
