#include "iic2.h"



 void Delay_us_IMU1(u8 num)
{ 
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	
	if(!IMU1_Fast)
	{
		u8 i = 15;
		while(i--);
	}
} 
/**************************实现函数********************************************
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/
void IIC_IMU1_Init(void)
{			
 GPIO_InitTypeDef  GPIO_InitStructure;
  #if USE_VER_5
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	IIC_IMU1_SCL4=1;
	IIC_IMU1_SDA4=1;
	#else
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOB时钟

  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	IIC_IMU1_SCL=1;
	IIC_IMU1_SDA=1;
	#endif
}

/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号
*******************************************************************************/
void IIC_IMU1_Start(void)
{
	#if USE_VER_5
		SDA_IMU1_OUT4();     //sda线输出
	IIC_IMU1_SDA4=1;	  	  
	IIC_IMU1_SCL4=1;
	Delay_us_IMU1(4);
 	IIC_IMU1_SDA4=0;//START:when CLK is high,DATA change form high to low 
	Delay_us_IMU1(4);
	IIC_IMU1_SCL4=0;//钳住I2C总线，准备发送或接收数据 
	#else
	SDA_IMU1_OUT();     //sda线输出
	IIC_IMU1_SDA=1;	  	  
	IIC_IMU1_SCL=1;
	Delay_us_IMU1(4);
 	IIC_IMU1_SDA=0;//START:when CLK is high,DATA change form high to low 
	Delay_us_IMU1(4);
	IIC_IMU1_SCL=0;//钳住I2C总线，准备发送或接收数据 
	#endif
}

/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/	  
void IIC_IMU1_Stop(void)
{
		#if USE_VER_5
	SDA_IMU1_OUT4();//sda线输出
	IIC_IMU1_SCL4=0;
	IIC_IMU1_SDA4=0;//STOP:when CLK is high DATA change form low to high
 	Delay_us_IMU1(4);
	IIC_IMU1_SCL4=1; 
	IIC_IMU1_SDA4=1;//发送I2C总线结束信号
	Delay_us_IMU1(4);				
	#else
	SDA_IMU1_OUT();//sda线输出
	IIC_IMU1_SCL=0;
	IIC_IMU1_SDA=0;//STOP:when CLK is high DATA change form low to high
 	Delay_us_IMU1(4);
	IIC_IMU1_SCL=1; 
	IIC_IMU1_SDA=1;//发送I2C总线结束信号
	Delay_us_IMU1(4);				
#endif	
}

/**************************实现函数********************************************
*函数原型:		u8 IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
u8 IIC_IMU1_Wait_Ack(void)
{
	u8 ucErrTime=0;
		#if USE_VER_5
	SDA_IMU1_IN4();      //SDA设置为输入  
	IIC_IMU1_SDA4=1;Delay_us_IMU1(1);	   
	IIC_IMU1_SCL4=1;Delay_us_IMU1(1);	 
	while(READ_IMU1_SDA4)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_IMU1_Stop();
			return 1;
		}
	  Delay_us_IMU1(1);
	}
	IIC_IMU1_SCL4=0;//时钟输出0 	
	#else
	SDA_IMU1_IN();      //SDA设置为输入  
	IIC_IMU1_SDA=1;Delay_us_IMU1(1);	   
	IIC_IMU1_SCL=1;Delay_us_IMU1(1);	 
	while(READ_IMU1_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_IMU1_Stop();
			return 1;
		}
	  Delay_us_IMU1(1);
	}
	IIC_IMU1_SCL=0;//时钟输出0 	
	#endif	
	return 0;  
} 

/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
*******************************************************************************/
void IIC_IMU1_Ack(void)
{
		#if USE_VER_5
	IIC_IMU1_SCL4=0;
	SDA_IMU1_OUT4();
	IIC_IMU1_SDA4=0;
	Delay_us_IMU1(2);
	IIC_IMU1_SCL4=1;
	Delay_us_IMU1(2);
	IIC_IMU1_SCL4=0;
	#else
	IIC_IMU1_SCL=0;
	SDA_IMU1_OUT();
	IIC_IMU1_SDA=0;
	Delay_us_IMU1(2);
	IIC_IMU1_SCL=1;
	Delay_us_IMU1(2);
	IIC_IMU1_SCL=0;
		#endif
}
	
/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
*******************************************************************************/	    
void IIC_IMU1_NAck(void)
{
		#if USE_VER_5
	IIC_IMU1_SCL4=0;
	SDA_IMU1_OUT4();
	IIC_IMU1_SDA4=1;
	Delay_us_IMU1(2);
	IIC_IMU1_SCL4=1;
	Delay_us_IMU1(2);
	IIC_IMU1_SCL4=0;
	#else
	IIC_IMU1_SCL=0;
	SDA_IMU1_OUT();
	IIC_IMU1_SDA=1;
	Delay_us_IMU1(2);
	IIC_IMU1_SCL=1;
	Delay_us_IMU1(2);
	IIC_IMU1_SCL=0;
		#endif
}					 				     

/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(u8 txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/		  
void IIC_IMU1_Send_Byte(u8 txd)
{                        
    u8 t;  
	#if USE_VER_5
	SDA_IMU1_OUT4(); 	    
    IIC_IMU1_SCL4=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_IMU1_SDA4=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_us_IMU1(2);   
		IIC_IMU1_SCL4=1;
		Delay_us_IMU1(2); 
		IIC_IMU1_SCL4=0;	
		Delay_us_IMU1(2);
    }	 
	#else	
	SDA_IMU1_OUT(); 	    
    IIC_IMU1_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_IMU1_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_us_IMU1(2);   
		IIC_IMU1_SCL=1;
		Delay_us_IMU1(2); 
		IIC_IMU1_SCL=0;	
		Delay_us_IMU1(2);
    }	 
			#endif
} 	 
   
/**************************实现函数********************************************
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
u8 IIC_IMU1_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
		#if USE_VER_5
		SDA_IMU1_IN4();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_IMU1_SCL4=0; 
        Delay_us_IMU1(2);
		IIC_IMU1_SCL4=1;
        receive<<=1;
        if(READ_IMU1_SDA4)receive++;   
		Delay_us_IMU1(2); 
    }					 
    if (ack)
        IIC_IMU1_Ack(); //发送ACK 
    else
        IIC_IMU1_NAck();//发送nACK  
	#else
	SDA_IMU1_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_IMU1_SCL=0; 
        Delay_us_IMU1(2);
		IIC_IMU1_SCL=1;
        receive<<=1;
        if(READ_IMU1_SDA)receive++;   
		Delay_us_IMU1(2); 
    }					 
    if (ack)
        IIC_IMU1_Ack(); //发送ACK 
    else
        IIC_IMU1_NAck();//发送nACK  
		#endif
    return receive;
}

/**************************实现函数********************************************
*函数原型:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	I2C_Addr  目标设备地址
		addr	   寄存器地址
返回   读出来的值
*******************************************************************************/ 
unsigned char I2C_IMU1_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_IMU1_Start();	
	IIC_IMU1_Send_Byte(I2C_Addr);	   //发送写命令
	res++;
	IIC_IMU1_Wait_Ack();
	IIC_IMU1_Send_Byte(addr); res++;  //发送地址
	IIC_IMU1_Wait_Ack();	  
	//IIC_Stop();//产生一个停止条件	
	IIC_IMU1_Start();
	IIC_IMU1_Send_Byte(I2C_Addr+1); res++;          //进入接收模式			   
	IIC_IMU1_Wait_Ack();
	res=IIC_IMU1_Read_Byte(0);	   
    IIC_IMU1_Stop();//产生一个停止条件

	return res;
}


/**************************实现函数********************************************
*函数原型:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要读的字节数
		*data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/ 
u8 IIC_IMU1readBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_IMU1_Start();
	IIC_IMU1_Send_Byte(dev);	   //发送写命令
	IIC_IMU1_Wait_Ack();
	IIC_IMU1_Send_Byte(reg);   //发送地址
    IIC_IMU1_Wait_Ack();	  
	IIC_IMU1_Start();
	IIC_IMU1_Send_Byte(dev+1);  //进入接收模式	
	IIC_IMU1_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_IMU1_Read_Byte(1);  //带ACK的读数据
		 	else  data[count]=IIC_IMU1_Read_Byte(0);	 //最后一个字节NACK
	}
    IIC_IMU1_Stop();//产生一个停止条件
    return count;
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要写的字节数
		*data  将要写的数据的首地址
返回   返回是否成功
*******************************************************************************/ 
u8 IIC_IMU1writeBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_IMU1_Start();
	IIC_IMU1_Send_Byte(dev);	   //发送写命令
	IIC_IMU1_Wait_Ack();
	IIC_IMU1_Send_Byte(reg);   //发送地址
    IIC_IMU1_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_IMU1_Send_Byte(data[count]); 
		IIC_IMU1_Wait_Ack(); 
	 }
	IIC_IMU1_Stop();//产生一个停止条件

    return 1; //status == 0;
}

/**************************实现函数********************************************
*函数原型:		u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	dev  目标设备地址
		reg	   寄存器地址
		*data  读出的数据将要存放的地址
返回   1
*******************************************************************************/ 
u8 IIC_IMU1readByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_IMU1_ReadOneByte(dev, reg);
    return 1;
}

/**************************实现函数********************************************
*函数原型:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*功　　能:	    写入指定设备 指定寄存器一个字节
输入	dev  目标设备地址
		reg	   寄存器地址
		data  将要写入的字节
返回   1
*******************************************************************************/ 
unsigned char IIC_IMU1writeByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IIC_IMU1writeBytes(dev, reg, 1, &data);
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitStart  目标字节的起始位
		length   位长度
		data    存放改变目标字节位的值
返回   成功 为1 
 		失败为0
*******************************************************************************/ 
u8 IIC_IMU1writeBitsm(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IIC_IMU1readByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IIC_IMU1writeByte(dev, reg, b);
    } else {
        return 0;
    }
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitNum  要修改目标字节的bitNum位
		data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1 
 		失败为0
*******************************************************************************/ 
u8 IIC_IMU1writeBitm(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IIC_IMU1readByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IIC_IMU1writeByte(dev, reg, b);
}

//------------------End of File----------------------------
