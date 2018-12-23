#include "spi.h"
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


//以下是SPI模块的初始化代码，配置成主机模式 						  
//SPI口初始化
//这里针是对SPI1的初始化
void SPI1_Init(void)
{	 
	u16 tempreg=0;
	RCC->AHB1ENR|=1<<0;    	//使能PORTA时钟	   
	RCC->APB2ENR|=1<<12;   	//SPI1时钟使能 
	GPIO_Set(GPIOB,7<<3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);	//PB3~5复用功能输出	
  	GPIO_AF_Set(GPIOB,3,5);	//PB3,AF5
 	GPIO_AF_Set(GPIOB,4,5);	//PB4,AF5
 	GPIO_AF_Set(GPIOB,5,5);	//PB5,AF5 

	//这里只针对SPI口初始化
	RCC->APB2RSTR|=1<<12;	//复位SPI1
	RCC->APB2RSTR&=~(1<<12);//停止复位SPI1
	tempreg|=0<<10;			//全双工模式	
	tempreg|=1<<9;			//软件nss管理
	tempreg|=1<<8;			 
	tempreg|=1<<2;			//SPI主机  
	tempreg|=0<<11;			//8位数据格式	
	tempreg|=1<<1;			//空闲模式下SCK为1 CPOL=1 
	tempreg|=1<<0;			//数据采样从第2个时间边沿开始,CPHA=1  
 	//对SPI1属于APB2的外设.时钟频率最大为84Mhz频率.
	tempreg|=7<<3;			//Fsck=Fpclk1/256
	tempreg|=0<<7;			//MSB First  
	tempreg|=1<<6;			//SPI启动 
	SPI1->CR1=tempreg; 		//设置CR1  
	SPI1->I2SCFGR&=~(1<<11);//选择SPI模式
	SPI1_ReadWriteByte(0xff);//启动传输		 
}   
//SPI1速度设置函数
//SpeedSet:0~7
//SPI速度=fAPB2/2^(SpeedSet+1)
//fAPB2时钟一般为84Mhz
void SPI1_SetSpeed(u8 SpeedSet)
{
	SpeedSet&=0X07;			//限制范围
	SPI1->CR1&=0XFFC7; 
	SPI1->CR1|=SpeedSet<<3;	//设置SPI1速度  
	SPI1->CR1|=1<<6; 		//SPI设备使能	  
} 
//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{		 			 
	while((SPI1->SR&1<<1)==0);		//等待发送区空 
	SPI1->DR=TxData;	 	  		//发送一个byte  
	while((SPI1->SR&1<<0)==0);		//等待接收完一个byte  
 	return SPI1->DR;          		//返回收到的数据				    
}








