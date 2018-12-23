
#include "Soft_I2C_PX4.h"
#include "delay.h"


/**************************I2C GPIO配置*********************************/
void Soft_I2C_Init_PX4(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	GPIO_InitStructure.GPIO_Pin   = I2C_Pin_SCL_PX4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(I2C_GPIO_PX4_S, &GPIO_InitStructure);		

	GPIO_InitStructure.GPIO_Pin   = I2C_Pin_SDA_PX4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(I2C_GPIO_PX4_D, &GPIO_InitStructure);		
}
u8 iic_px4_delay=1;
/***************************I2C 延时函数*******************************/
static void I2C_delay_PX4(void)
{

	
	
	delay_us(iic_px4_delay);
//	volatile int i = 10;
//	while(i)
//		i--;	
	
}

 uint8_t I2C_Start_PX4(void)
{
	SDA_H_PX4;
	SCL_H_PX4;
	I2C_delay_PX4();
	//SDA线为低电平则总线忙,退出
	if(!SDA_Read_PX4) return 0;
	SDA_L_PX4;
	I2C_delay_PX4();
	//SDA线为高电平则总线出错,退出
	if(SDA_Read_PX4) return 0;
	SDA_L_PX4;
	I2C_delay_PX4();
	return 1;	
}

 void I2C_Stop_PX4(void)
{
	SCL_L_PX4;
	I2C_delay_PX4();
	SDA_L_PX4;
	I2C_delay_PX4();
	SCL_H_PX4;
	I2C_delay_PX4();
	SDA_H_PX4;
	I2C_delay_PX4();
}

 void I2C_Ack_PX4(void)
{	
	SCL_L_PX4;
	I2C_delay_PX4();
	SDA_L_PX4;
	I2C_delay_PX4();
	SCL_H_PX4;
	I2C_delay_PX4();
	SCL_L_PX4;
	I2C_delay_PX4();
}   

 void I2C_NoAck_PX4(void)
{	
	SCL_L_PX4;
	I2C_delay_PX4();
	SDA_H_PX4;
	I2C_delay_PX4();
	SCL_H_PX4;
	I2C_delay_PX4();
	SCL_L_PX4;
	I2C_delay_PX4();
} 

/************************Wait I2C 应答********************************/
//返回为:=1有ACK	=0无ACK
 uint8_t I2C_WaitAck_PX4(void)
{
	SCL_L_PX4;
	I2C_delay_PX4();
	SDA_H_PX4;			
	I2C_delay_PX4();
	SCL_H_PX4;
	I2C_delay_PX4();
	if(SDA_Read_PX4)
	{
		SCL_L_PX4;
		I2C_delay_PX4();
		return 0;
	}
	SCL_L_PX4;
	I2C_delay_PX4();
	return 1;
}

//数据从高位到低位
 void I2C_SendByte_PX4(uint8_t byte) 
{
    u8 i=8;
    while(i--)
    {
        SCL_L_PX4;
        I2C_delay_PX4();
		if(byte&0x80)
			SDA_H_PX4;  
		else 
			SDA_L_PX4;   
        byte <<= 1;
        I2C_delay_PX4();
		SCL_H_PX4;
		I2C_delay_PX4();
    }
    SCL_L_PX4;
}  

//数据从高位到低位
 uint8_t I2C_ReceiveByte_PX4(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H_PX4;
    while (i--)
	{
        byte <<= 1;
        SCL_L_PX4;
        I2C_delay_PX4();
        SCL_H_PX4;
        I2C_delay_PX4();
        if (SDA_Read_PX4) 
		{
            byte |= 0x01;
        }
    }
    SCL_L_PX4;
    return byte;
}


/*******************************************************************************
* 函数名  : I2C_Single_Write
* 描述    : 向I2C设备写入一字节数据
* 输入    : 设备地址	寄存器地址		待写入的字节
* 输出    : 无
* 返回    : 0 or 1
****************************************************************************** */
uint8_t I2C_Single_Write_PX4(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t data)
{
    if(!I2C_Start_PX4()) return 0;
	//发送设备地址+写信号 
    I2C_SendByte_PX4(SlaveAddress << 1 | I2C_Direction_Transmitter);   
    if(!I2C_WaitAck_PX4()){
		I2C_Stop_PX4(); 
		return 0;
	}  
    I2C_SendByte_PX4(REG_Address );   
    I2C_WaitAck_PX4();	
    I2C_SendByte_PX4(data);
    I2C_WaitAck_PX4();   
    I2C_Stop_PX4(); 
    return 1;
}


/*******************************************************************************
* 函数名  : I2C_Single_Read
* 描述    : 从I2C设读取一字节数据
* 输入    : 设备地址	寄存器地址
* 输出    : 无
* 返回    : 0 or 数据
****************************************************************************** */
uint8_t I2C_Single_Read_PX4(uint8_t SlaveAddress, uint8_t REG_Address)
{
	uint8_t data;
    if (!I2C_Start_PX4()) return 0;
	//发送设备地址+写信号
    I2C_SendByte_PX4(SlaveAddress << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck_PX4()) {
        I2C_Stop_PX4();
        return 0;
    }
    I2C_SendByte_PX4(REG_Address);
    I2C_WaitAck_PX4();
    I2C_Start_PX4();
    I2C_SendByte_PX4(SlaveAddress << 1 | I2C_Direction_Receiver);
    I2C_WaitAck_PX4();
	data = I2C_ReceiveByte_PX4();
    I2C_NoAck_PX4();
	I2C_Stop_PX4();
	return data;
}

/*******************************************************************************
* 函数名  : I2C_WriteBuffer
* 描述    : 向I2C设备写入多字节数据
* 输入    : 设备地址	寄存器地址		数据长度		待写入数据
* 输出    : 无
* 返回    : 0 or 1
****************************************************************************** */
uint8_t I2C_WriteBuffer_PX4(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    int i;
    if (!I2C_Start_PX4())
        return 0;
    I2C_SendByte_PX4(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck_PX4()) {
        I2C_Stop_PX4();
        return 0;
    }
    I2C_SendByte_PX4(reg);
    I2C_WaitAck_PX4();
    for (i = 0; i < len; i++) {
        I2C_SendByte_PX4(data[i]);
        if (!I2C_WaitAck_PX4()) {
            I2C_Stop_PX4();
            return 0;
        }
    }
    I2C_Stop_PX4();
    return 1;
}

/*******************************************************************************
* 函数名  : I2C_ReadBuffer
* 描述    : 从I2C设读出多字节数据
* 输入    : 设备地址	寄存器地址		待读数据长度	待读数据地址	
* 输出    : 读出的数据
* 返回    : 0 or 1
****************************************************************************** */
uint8_t I2C_ReadBuffer_PX4(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2C_Start_PX4())
        return 0;
    I2C_SendByte_PX4(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck_PX4()) {
        I2C_Stop_PX4();
        return 0;
    }
    I2C_SendByte_PX4(reg);
    I2C_WaitAck_PX4();
    I2C_Start_PX4();
    I2C_SendByte_PX4(addr << 1 | I2C_Direction_Receiver);
    I2C_WaitAck_PX4();
    while (len) {
        *buf = I2C_ReceiveByte_PX4();
        if (len == 1)
            I2C_NoAck_PX4();
        else
            I2C_Ack_PX4();
        buf++;
        len--;
    }
    I2C_Stop_PX4();
    return 1;
}
//----------------------------------------------------------------------------

/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号
*******************************************************************************/
void IIC_Start_PX4(void)
{
	SDA_OUT_PX4();     //sda线输出
	IIC_SDA_PX4=1;	  	  
	IIC_SCL_PX4=1;
	delay_us(4);
 	IIC_SDA_PX4=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_PX4=0;//钳住I2C总线，准备发送或接收数据 
}

/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/	  
void IIC_Stop_PX4(void)
{
	SDA_OUT_PX4();//sda线输出
	IIC_SCL_PX4=0;
	IIC_SDA_PX4=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL_PX4=1; 
	IIC_SDA_PX4=1;//发送I2C总线结束信号
	delay_us(4);		
}

/**************************实现函数********************************************
*函数原型:		u8 IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
u8 IIC_Wait_Ack_PX4(void)
{
	u8 ucErrTime=0;
	SDA_IN_PX4();      //SDA设置为输入  
	IIC_SDA_PX4=1;delay_us(1);	   
	IIC_SCL_PX4=1;delay_us(1);	 
	while(READ_SDA_PX4)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop_PX4();
			return 1;
		}
	  delay_us(1);
	}
	IIC_SCL_PX4=0;//时钟输出0 	   
	return 0;  	
} 

/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
*******************************************************************************/
void IIC_Ack_PX4(void)
{
	IIC_SCL_PX4=0;
	SDA_OUT_PX4();
	IIC_SDA_PX4=0;
	delay_us(2);
	IIC_SCL_PX4=1;
	delay_us(2);
	IIC_SCL_PX4=0;
	
}
	
/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
*******************************************************************************/	    
void IIC_NAck_PX4(void)
{
	IIC_SCL_PX4=0;
	SDA_OUT_PX4();
	IIC_SDA_PX4=1;
	delay_us(2);
	IIC_SCL_PX4=1;
	delay_us(2);
	IIC_SCL_PX4=0;
}					 				     

/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(u8 txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/		  
void IIC_Send_Byte_PX4(u8 txd)
{ 
    u8 t;   
	SDA_OUT_PX4(); 	    
    IIC_SCL_PX4=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA_PX4=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   
		IIC_SCL_PX4=1;
		delay_us(2); 
		IIC_SCL_PX4=0;	
		delay_us(2);
    }		
} 	 
   
/**************************实现函数********************************************
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
u8 IIC_Read_Byte_PX4(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN_PX4();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL_PX4=0; 
        delay_us(2);
		IIC_SCL_PX4=1;
        receive<<=1;
        if(READ_SDA_PX4)receive++;   
		delay_us(2); 
    }					 
    if (ack)
        IIC_Ack_PX4(); //发送ACK 
    else
        IIC_NAck_PX4();//发送nACK  
    return receive;		
}

/**************************实现函数********************************************
*函数原型:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	I2C_Addr  目标设备地址
		addr	   寄存器地址
返回   读出来的值
*******************************************************************************/ 
unsigned char I2C_ReadOneByte_PX4(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start_PX4();	
	IIC_Send_Byte_PX4(I2C_Addr);	   //发送写命令
	res++;
	IIC_Wait_Ack_PX4();
	IIC_Send_Byte_PX4(addr); res++;  //发送地址
	IIC_Wait_Ack_PX4();	  
	//IIC_Stop();//产生一个停止条件	
	IIC_Start_PX4();
	IIC_Send_Byte_PX4(I2C_Addr+1); res++;          //进入接收模式			   
	IIC_Wait_Ack_PX4();
	res=IIC_Read_Byte_PX4(0);	   
    IIC_Stop_PX4();//产生一个停止条件

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
u8 IICreadBytes_PX4(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_Start_PX4();
	IIC_Send_Byte_PX4(dev);	   //发送写命令
	IIC_Wait_Ack_PX4();
	IIC_Send_Byte_PX4(reg);   //发送地址
    IIC_Wait_Ack_PX4();	  
	IIC_Start_PX4();
	IIC_Send_Byte_PX4(dev+1);  //进入接收模式	
	IIC_Wait_Ack_PX4();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte_PX4(1);  //带ACK的读数据
		 	else  data[count]=IIC_Read_Byte_PX4(0);	 //最后一个字节NACK
	}
    IIC_Stop_PX4();//产生一个停止条件
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
u8 IICwriteBytes_PX4(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start_PX4();
	IIC_Send_Byte_PX4(dev);	   //发送写命令
	IIC_Wait_Ack_PX4();
	IIC_Send_Byte_PX4(reg);   //发送地址
    IIC_Wait_Ack_PX4();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte_PX4(data[count]); 
		IIC_Wait_Ack_PX4(); 
	 }
	IIC_Stop_PX4();//产生一个停止条件

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
u8 IICreadByte_PX4(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte_PX4(dev, reg);
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
unsigned char IICwriteByte_PX4(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes_PX4(dev, reg, 1, &data);
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
u8 IICwriteBitsm_PX4(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte_PX4(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte_PX4(dev, reg, b);
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
u8 IICwriteBitm_PX4(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte_PX4(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte_PX4(dev, reg, b);
}

//------------------End of File----------------------------

