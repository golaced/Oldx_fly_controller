#include "include.h"
#include "i2c_soft.h"

volatile u8 I2C_FastMode;

void I2c_Soft_delay()
{ 
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	
	if(!I2C_FastMode)
	{
		u8 i = 15;
		while(i--);
	}
}

void I2c_Soft_Init()
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  RCC_AHB1PeriphClockCmd(ANO_RCC_I2C , ENABLE );
	#if USE_MINI_FC_FLOW_BOARD
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_F | I2C_Pin_SDA_F;
	#else
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL | I2C_Pin_SDA;
	#endif
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(ANO_GPIO_I2C, &GPIO_InitStructure);		
}

int I2c_Soft_Start()
{ 
	#if USE_MINI_FC_FLOW_BOARD
	SDA_H_F;
	SCL_H_F;
	I2c_Soft_delay();
	if(!SDA_read_F)return 0;	//SDA线为低电平则总线忙,退出
	SDA_L_F;
	I2c_Soft_delay();
	if(SDA_read_F) return 0;	//SDA线为高电平则总线出错,退出
	SDA_L_F;
	I2c_Soft_delay();
	return 1;	
	#else
	SDA_H;
	SCL_H;
	I2c_Soft_delay();
	if(!SDA_read)return 0;	//SDA线为低电平则总线忙,退出
	SDA_L;
	I2c_Soft_delay();
	if(SDA_read) return 0;	//SDA线为高电平则总线出错,退出
	SDA_L;
	I2c_Soft_delay();
	return 1;	
  #endif
}

void I2c_Soft_Stop()
{
	#if USE_MINI_FC_FLOW_BOARD
	SCL_L_F;
	I2c_Soft_delay();
	SDA_L_F;
	I2c_Soft_delay();
	SCL_H_F;
	I2c_Soft_delay();
	SDA_H_F;
	I2c_Soft_delay();
	#else
	SCL_L;
	I2c_Soft_delay();
	SDA_L;
	I2c_Soft_delay();
	SCL_H;
	I2c_Soft_delay();
	SDA_H;
	I2c_Soft_delay();
	#endif
}

void I2c_Soft_Ask()
{
	#if USE_MINI_FC_FLOW_BOARD
	SCL_L_F;
	I2c_Soft_delay();
	SDA_L_F;
	I2c_Soft_delay();
	SCL_H_F;
	I2c_Soft_delay();
	SCL_L_F;
	I2c_Soft_delay();
	#else
	SCL_L;
	I2c_Soft_delay();
	SDA_L;
	I2c_Soft_delay();
	SCL_H;
	I2c_Soft_delay();
	SCL_L;
	I2c_Soft_delay();
	#endif
}

void I2c_Soft_NoAsk()
{
	#if USE_MINI_FC_FLOW_BOARD
	SCL_L_F;
	I2c_Soft_delay();
	SDA_H_F;
	I2c_Soft_delay();
	SCL_H_F;
	I2c_Soft_delay();
	SCL_L_F;
	I2c_Soft_delay();
	#else
	SCL_L;
	I2c_Soft_delay();
	SDA_H;
	I2c_Soft_delay();
	SCL_H;
	I2c_Soft_delay();
	SCL_L;
	I2c_Soft_delay();
	#endif
}

int I2c_Soft_WaitAsk(void) 	 //返回为:=1无ASK,=0有ASK
{
#if USE_MINI_FC_FLOW_BOARD	
	u8 ErrTime = 0;
	SCL_L_F;
	I2c_Soft_delay();
	SDA_H_F;			
	I2c_Soft_delay();
	SCL_H_F;
	I2c_Soft_delay();
	while(SDA_read_F)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop();
			return 1;
		}
	}
	SCL_L_F;
	I2c_Soft_delay();
	return 0;
#else	
  u8 ErrTime = 0;
	SCL_L;
	I2c_Soft_delay();
	SDA_H;			
	I2c_Soft_delay();
	SCL_H;
	I2c_Soft_delay();
	while(SDA_read)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop();
			return 1;
		}
	}
	SCL_L;
	I2c_Soft_delay();
	return 0;
	#endif
}

void I2c_Soft_SendByte(u8 SendByte) //数据从高位到低位//
{
#if USE_MINI_FC_FLOW_BOARD	
    u8 i=8;
    while(i--)
    {
        SCL_L_F;
        I2c_Soft_delay();
      if(SendByte&0x80)
        SDA_H_F;  
      else 
        SDA_L_F;   
        SendByte<<=1;
        I2c_Soft_delay();
				SCL_H_F;
				I2c_Soft_delay();
    }
    SCL_L_F;
#else	
    u8 i=8;
    while(i--)
    {
        SCL_L;
        I2c_Soft_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2c_Soft_delay();
				SCL_H;
				I2c_Soft_delay();
    }
    SCL_L;
#endif		
}  

//读1个字节，ack=1时，发送ACK，ack=0，发送NACK
u8 I2c_Soft_ReadByte(u8 ask)  //数据从高位到低位//
{ 
#if USE_MINI_FC_FLOW_BOARD		
	  u8 i=8;
    u8 ReceiveByte=0;

    SDA_H_F;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_F;
      I2c_Soft_delay();
			SCL_H_F;
      I2c_Soft_delay();	
      if(SDA_read_F)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_F;

	if (ask)
		I2c_Soft_Ask();
	else
		I2c_Soft_NoAsk();  
    return ReceiveByte;
#else	
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2c_Soft_delay();
			SCL_H;
      I2c_Soft_delay();	
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;

	if (ask)
		I2c_Soft_Ask();
	else
		I2c_Soft_NoAsk();  
    return ReceiveByte;
#endif	
} 


// IIC写一个字节数据
u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress<<1);   
	if(I2c_Soft_WaitAsk())
	{
		I2c_Soft_Stop();
		return 1;
	}
	I2c_Soft_SendByte(REG_Address);       
	I2c_Soft_WaitAsk();	
	I2c_Soft_SendByte(REG_data);
	I2c_Soft_WaitAsk();   
	I2c_Soft_Stop(); 
	return 0;
}

// IIC读1字节数据
u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data)
{      		
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress<<1); 
	if(I2c_Soft_WaitAsk())
	{
		I2c_Soft_Stop();
		return 1;
	}
	I2c_Soft_SendByte(REG_Address);     
	I2c_Soft_WaitAsk();
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress<<1 | 0x01);
	I2c_Soft_WaitAsk();
	*REG_data= I2c_Soft_ReadByte(0);
	I2c_Soft_Stop();
	return 0;
}	

// IIC写n字节数据
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress<<1); 
	if(I2c_Soft_WaitAsk())
	{
		I2c_Soft_Stop();
		return 1;
	}
	I2c_Soft_SendByte(REG_Address); 
	I2c_Soft_WaitAsk();
	while(len--) 
	{
		I2c_Soft_SendByte(*buf++); 
		I2c_Soft_WaitAsk();
	}
	I2c_Soft_Stop();
	return 0;
}

// IIC读n字节数据
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress<<1); 
	if(I2c_Soft_WaitAsk())
	{
		I2c_Soft_Stop();
		return 1;
	}
	I2c_Soft_SendByte(REG_Address); 
	I2c_Soft_WaitAsk();
	
	I2c_Soft_Start();
	I2c_Soft_SendByte(SlaveAddress<<1 | 0x01); 
	I2c_Soft_WaitAsk();
	while(len) 
	{
		if(len == 1)
		{
			*buf = I2c_Soft_ReadByte(0);
		}
		else
		{
			*buf = I2c_Soft_ReadByte(1);
		}
		buf++;
		len--;
	}
	I2c_Soft_Stop();
	return 0;
}


