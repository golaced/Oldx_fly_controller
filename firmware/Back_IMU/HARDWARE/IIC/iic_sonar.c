#include "myiic_sonar.h"
#include "delay.h"
#include "time.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//IIC 驱动函数	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/6/10 
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

//初始化IIC
void IIC_Init_Sonar(void)
{					     
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOB时钟

  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化
	SCL_s=1;
	SDA_s=1;
}


#define uchar unsigned char
#define uint unsigned int
 
u16 sum;    
 
/***************************************************************************
*******************************************************************************/
 u8 IICwriteBytes_S(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start_S();
	IIC_Send_Byte_S(dev);	   //发送写命令
	IIC_Wait_Ack_S();
	IIC_Send_Byte_S(reg);   //发送地址
    IIC_Wait_Ack_S();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte_S(data[count]); 
		IIC_Wait_Ack_S(); 
	 }
	IIC_Stop_S();//产生一个停止条件

    return 1; //status == 0;
}
 
void Single_WriteI2C(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{       
   IICwriteBytes_S(SlaveAddress, REG_Address, 1, &REG_data);
}          
//**************************************
//从I2C设备读取一个字节数据
//**************************************
u8 Single_ReadI2C(u8 SlaveAddress,u8 REG_Address)
{
    u8 REG_data=0;
	unsigned char res=0;
	
	IIC_Start_S();	
	IIC_Send_Byte_S(SlaveAddress);	   //发送写命令
	res++;
	IIC_Wait_Ack_S();
	IIC_Send_Byte_S(REG_Address); res++;  //发送地址
	IIC_Wait_Ack_S();	  
	//IIC_Stop();//产生一个停止条件	
	IIC_Start_S();
	IIC_Send_Byte_S(SlaveAddress+1); res++;          //进入接收模式			   
	IIC_Wait_Ack_S();
	res=IIC_Read_Byte_S(0);	   
    IIC_Stop_S();//产生一个停止条件

	return res;
}
 
u32 Read_KS10X(u8 SlaveAddress)
{    
//       delay_ms(80);
         sum=Single_ReadI2C(SlaveAddress,0x02);             //高8位
         sum<<=8;
         sum+=Single_ReadI2C(SlaveAddress,0x03);                //低8位
         return sum;
}
 
void Change_Addr(u8 OldAdddr,u8 NewAddr)
{
//       delay_ms(500);
         Single_WriteI2C(OldAdddr,0x02,0x9a);             //默认原地址是0x00;
         delay_ms(1);
         Single_WriteI2C(OldAdddr,0x02,0x92);
         delay_ms(1);
         Single_WriteI2C(OldAdddr,0x02,0x9e);
         delay_ms(1);
         Single_WriteI2C(OldAdddr,0x02,NewAddr);
         delay_ms(100);
 
//           Single_WriteI2C(NewAddr,0x02,0xb0);
//           delay_ms(80);
}
 
float Read_KS10X_Data(u8 SlaveAddress)
{
        float sumx;
        float sum_x;
        Single_WriteI2C(SlaveAddress,0x02,0xb4);
        Delay_ms(80);
         
        sumx=Read_KS10X(SlaveAddress);          //读出第一个超声波的数据
       
        sum_x=(float)sumx/10;
//      printf(\"%lf\r\n\",sum_x);
 
        return sum_x;
}
 
void KS10X_Change_Addr_Init(u8 OldAddr,u8 NewAddr)  //此函数用来实现选择超声波的地址
{
        Change_Addr(OldAddr,NewAddr);
        delay_ms(80);
}
 
 

 
//****************************************
// 定义KS10X内部地址
//****************************************
 
#define SlaveAddress1   0xE8    //IIC写入时的地址字节数据，+1为读取
#define SlaveAddress2   0xd0
#define SlaveAddress3   0xd2
#define SlaveAddress4   0xd4
#define SlaveAddress5   0xd6
#define SlaveAddress6   0xd8
#define SlaveAddress7   0xda
#define SlaveAddress8   0xdc
#define SlaveAddress9   0xde
#define SlaveAddress10  0xe0
#define SlaveAddress11  0xe2
#define SlaveAddress12  0xe4
#define SlaveAddress13  0xe6
#define SlaveAddress14  0xea
#define SlaveAddress15  0xec
#define SlaveAddress16  0xee
#define SlaveAddress17  0xf8
#define SlaveAddress18  0xfa
#define SlaveAddress19  0xfc
#define SlaveAddress20  0xfe
 
void IIC_Start_S(void)
{
	SDA_OUT_s();     //sda线输出
	SDA_s=1;	  	  
	SCL_s=1;
	Delay_us(4);
 	SDA_s=0;//START:when CLK is high,DATA change form high to low 
	Delay_us(4);
	SCL_s=0;//钳住I2C总线，准备发送或接收数据 
}

/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/	  
void IIC_Stop_S(void)
{
	SDA_OUT_s();//sda线输出
	SCL_s=0;
	SDA_s=0;//STOP:when CLK is high DATA change form low to high
 	Delay_us(4);
	SCL_s=1; 
	SDA_s=1;//发送I2C总线结束信号
	Delay_us(4);							   	
}

/**************************实现函数********************************************
*函数原型:		u8 IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
u8 IIC_Wait_Ack_S(void)
{
	u8 ucErrTime=0;
	SDA_IN_s();      //SDA设置为输入  
	SDA_s=1;Delay_us(1);	   
	SCL_s=1;Delay_us(1);	 
	while(READ_SDA_s)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop_S();
			return 1;
		}
	  Delay_us(1);
	}
	SCL_s=0;//时钟输出0 	   
	return 0;  
} 

/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
*******************************************************************************/
void IIC_Ack_S(void)
{
	SCL_s=0;
	SDA_OUT_s();
	SDA_s=0;
	Delay_us(2);
	SCL_s=1;
	Delay_us(2);
	SCL_s=0;
}
	
/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
*******************************************************************************/	    
void IIC_NAck_S(void)
{
	SCL_s=0;
	SDA_OUT_s();
	SDA_s=1;
	Delay_us(2);
	SCL_s=1;
	Delay_us(2);
	SCL_s=0;
}					 				     

/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(u8 txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/		  
void IIC_Send_Byte_S(u8 txd)
{                        
    u8 t;   
	SDA_OUT_s(); 	    
    SCL_s=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        SDA_s=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_us(2);   
		SCL_s=1;
		Delay_us(2); 
		SCL_s=0;	
		Delay_us(2);
    }	 
} 	 
   
/**************************实现函数********************************************
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
u8 IIC_Read_Byte_S(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN_s();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        SCL_s=0; 
        Delay_us(2);
		SCL_s=1;
        receive<<=1;
        if(READ_SDA_s)receive++;   
		Delay_us(2); 
    }					 
    if (ack)
        IIC_Ack_S(); //发送ACK 
    else
        IIC_NAck_S();//发送nACK  
    return receive;
}
 
//int main()
//{
//      Stm32_Clock_Init(9);
//      delay_init(72);
//      uart_init(72,9600);
//      I2C2_Init();
// 
////    KS10X_Change_Addr_Init(SlaveAddress3,SlaveAddress2);  
//      //在换了KS10X后才执行，且执行一次  ,执行完一次后就消去
//      //这样这个模块将有固定的地址
// 
//      while(1)
//      {                  
//             sum_2=Read_KS10X_Data(SlaveAddress2);
//             sum_1=Read_KS10X_Data(SlaveAddress1);
//              
//             printf(\"%lf\r\n\",sum_1);
//             printf(\"%lf\r\n\r\n\",sum_2);
//      }
//}

































