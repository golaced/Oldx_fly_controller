#ifndef __IOI2C_H
#define __IOI2C_H
#include "stm32f4xx.h"
#include "include.h"

#if NEW_IMU
#define SDA_IMU1_IN()  {GPIOB->MODER&=~(3<<(6*2));GPIOB->MODER|=0<<6*2;}	//PB9输入模式
#define SDA_IMU1_OUT() {GPIOB->MODER&=~(3<<(6*2));GPIOB->MODER|=1<<6*2;} //PB9输出模式


//IO操作函数	 
#define IIC_IMU1_SCL    PBout(7) //SCL
#define IIC_IMU1_SDA    PBout(6) //SDA	 
#define READ_IMU1_SDA   PBin(6)  //输入SDA 


#define SDA_IMU1_IN4()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9输入模式
#define SDA_IMU1_OUT4() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式


//IO操作函数	 
#define IIC_IMU1_SCL4    PBout(8) //SCL
#define IIC_IMU1_SDA4    PBout(9) //SDA	 
#define READ_IMU1_SDA4  PBin(9)  //输入SDA 

#else
#define SDA_IMU1_IN()  {GPIOA->MODER&=~(3<<(7*2));GPIOA->MODER|=0<<7*2;}	//PB9输入模式
#define SDA_IMU1_OUT() {GPIOA->MODER&=~(3<<(7*2));GPIOA->MODER|=1<<7*2;} //PB9输出模式


//IO操作函数	 
#define IIC_IMU1_SCL    PAout(6) //SCL
#define IIC_IMU1_SDA    PAout(7) //SDA	 
#define READ_IMU1_SDA   PAin(7)  //输入SDA 
#endif
//IIC所有操作函数
void IIC_IMU1_Init(void);                //初始化IIC的IO口				 
void IIC_IMU1_Start(void);				//发送IIC开始信号
void IIC_IMU1_Stop(void);	  			//发送IIC停止信号
void IIC_IMU1_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_IMU1_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_IMU1_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_IMU1_Ack(void);					//IIC发送ACK信号
void IIC_IMU1_NAck(void);				//IIC不发送ACK信号

void IIC_IMU1_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_IMU1_Read_One_Byte(u8 daddr,u8 addr);	 
unsigned char I2C_IMU1_Readkey(unsigned char I2C_Addr);

unsigned char I2C_IMU1_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IIC_IMU1writeByte(unsigned char dev, unsigned char reg, unsigned char data);
u8 IIC_IMU1writeBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IIC_IMU1writeBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IIC_IMU1writeBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IIC_IMU1readBytes(u8 dev, u8 reg, u8 length, u8 *data);
extern u8 IMU1_Fast;
#endif

//------------------End of File----------------------------
