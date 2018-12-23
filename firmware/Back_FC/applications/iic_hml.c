
#include "iic_hml.h"

  void I2c_delay()
{ 
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	
	if(1)
	{
		u8 i = 15;
		while(i--);
	}
}
/**************************实现函数********************************************
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/
void IIC_Init(void)
{			
 GPIO_InitTypeDef  GPIO_InitStructure;
  #if USE_ZIN_BMP
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	IIC_SCL_T=1;
	IIC_SDA_T=1;
	#else
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOB时钟

  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
	
	#if USE_MINI_FC_FLOW_BOARD
	IIC_SCL_F=1;
	IIC_SDA_F=1;
	#else
	IIC_SCL=1;
	IIC_SDA=1;
	#endif
	#endif
}

/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号
*******************************************************************************/
void IIC_Start(void)
{
  #if USE_ZIN_BMP
	SDA_OUT_T();     //sda线输出
	IIC_SDA_T=1;	  	  
	IIC_SCL_T=1;
	I2c_delay();
 	IIC_SDA_T=0;//START:when CLK is high,DATA change form high to low 
	I2c_delay();
	IIC_SCL_T=0;//钳住I2C总线，准备发送或接收数据 
#else	
#if USE_MINI_FC_FLOW_BOARD
	SDA_OUT_F();     //sda线输出
	IIC_SDA_F=1;	  	  
	IIC_SCL_F=1;
	I2c_delay();
 	IIC_SDA_F=0;//START:when CLK is high,DATA change form high to low 
	I2c_delay();
	IIC_SCL_F=0;//钳住I2C总线，准备发送或接收数据 
#else	
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	I2c_delay();
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	I2c_delay();
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
#endif
#endif	
}

/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/	  
void IIC_Stop(void)
{
  #if USE_ZIN_BMP
	SDA_OUT_T();//sda线输出
	IIC_SCL_T=0;
	IIC_SDA_T=0;//STOP:when CLK is high DATA change form low to high
 	I2c_delay();
	IIC_SCL_T=1; 
	IIC_SDA_T=1;//发送I2C总线结束信号
	I2c_delay();	
#else	
#if USE_MINI_FC_FLOW_BOARD
	SDA_OUT_F();//sda线输出
	IIC_SCL_F=0;
	IIC_SDA_F=0;//STOP:when CLK is high DATA change form low to high
 	I2c_delay();
	IIC_SCL_F=1; 
	IIC_SDA_F=1;//发送I2C总线结束信号
	I2c_delay();		
#else	
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	I2c_delay();
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	I2c_delay();		
#endif	
#endif	
}

/**************************实现函数********************************************
*函数原型:		u8 IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
  #if USE_ZIN_BMP
u8 ucErrTime=0;
	SDA_IN_T();      //SDA设置为输入  
	IIC_SDA_T=1;Delay_us(1);	   
	IIC_SCL_T=1;Delay_us(1);	 
	while(READ_SDA_T)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
	  Delay_us(1);
	}
	IIC_SCL_T=0;//时钟输出0 	   
	return 0;  

#else	
#if USE_MINI_FC_FLOW_BOARD
	u8 ucErrTime=0;
	SDA_IN_F();      //SDA设置为输入  
	IIC_SDA_F=1;Delay_us(1);	   
	IIC_SCL_F=1;Delay_us(1);	 
	while(READ_SDA_F)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
	  Delay_us(1);
	}
	IIC_SCL_F=0;//时钟输出0 	   
	return 0;  
#else	
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;Delay_us(1);	   
	IIC_SCL=1;Delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
	  Delay_us(1);
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
#endif	
#endif
} 

/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
*******************************************************************************/
void IIC_Ack(void)
{
  #if USE_ZIN_BMP
	IIC_SCL_T=0;
	SDA_OUT_T();
	IIC_SDA_T=0;
	Delay_us(2);
	IIC_SCL_T=1;
	Delay_us(2);
	IIC_SCL_T=0;
#else	
#if USE_MINI_FC_FLOW_BOARD
	IIC_SCL_F=0;
	SDA_OUT_F();
	IIC_SDA_F=0;
	Delay_us(2);
	IIC_SCL_F=1;
	Delay_us(2);
	IIC_SCL_F=0;
#else	
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	Delay_us(2);
	IIC_SCL=1;
	Delay_us(2);
	IIC_SCL=0;
#endif	
#endif	
}
	
/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
*******************************************************************************/	    
void IIC_NAck(void)
{
 #if USE_ZIN_BMP	
		IIC_SCL_T=0;
	SDA_OUT_T();
	IIC_SDA_T=1;
	Delay_us(2);
	IIC_SCL_T=1;
	Delay_us(2);
	IIC_SCL_T=0;
	#else
#if USE_MINI_FC_FLOW_BOARD
	IIC_SCL_F=0;
	SDA_OUT_F();
	IIC_SDA_F=1;
	Delay_us(2);
	IIC_SCL_F=1;
	Delay_us(2);
	IIC_SCL_F=0;
#else	
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	Delay_us(2);
	IIC_SCL=1;
	Delay_us(2);
	IIC_SCL=0;
#endif	
	#endif
}					 				     

/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(u8 txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{ 
 #if USE_ZIN_BMP	
    u8 t;   
	SDA_OUT_T(); 	    
    IIC_SCL_T=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA_T=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_us(2);   
		IIC_SCL_T=1;
		Delay_us(2); 
		IIC_SCL_T=0;	
		Delay_us(2);
    }	
#else	
#if USE_MINI_FC_FLOW_BOARD
    u8 t;   
	SDA_OUT_F(); 	    
    IIC_SCL_F=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA_F=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_us(2);   
		IIC_SCL_F=1;
		Delay_us(2); 
		IIC_SCL_F=0;	
		Delay_us(2);
    }	
#else	
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_us(2);   
		IIC_SCL=1;
		Delay_us(2); 
		IIC_SCL=0;	
		Delay_us(2);
    }	
#endif	
#endif		
} 	 
   
/**************************实现函数********************************************
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
 #if USE_ZIN_BMP	
	unsigned char i,receive=0;
	SDA_IN_T();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL_T=0; 
        Delay_us(2);
		IIC_SCL_T=1;
        receive<<=1;
        if(READ_SDA_T)receive++;   
		Delay_us(2); 
    }					 
    if (ack)
        IIC_Ack(); //发送ACK 
    else
        IIC_NAck();//发送nACK  
    return receive;
#else	
#if USE_MINI_FC_FLOW_BOARD
	unsigned char i,receive=0;
	SDA_IN_F();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL_F=0; 
        Delay_us(2);
		IIC_SCL_F=1;
        receive<<=1;
        if(READ_SDA_F)receive++;   
		Delay_us(2); 
    }					 
    if (ack)
        IIC_Ack(); //发送ACK 
    else
        IIC_NAck();//发送nACK  
    return receive;
#else	
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        Delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		Delay_us(2); 
    }					 
    if (ack)
        IIC_Ack(); //发送ACK 
    else
        IIC_NAck();//发送nACK  
    return receive;
#endif	
#endif		
}

/**************************实现函数********************************************
*函数原型:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	I2C_Addr  目标设备地址
		addr	   寄存器地址
返回   读出来的值
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start();	
	IIC_Send_Byte(I2C_Addr);	   //发送写命令
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //发送地址
	IIC_Wait_Ack();	  
	//IIC_Stop();//产生一个停止条件	
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //进入接收模式			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
    IIC_Stop();//产生一个停止条件

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
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev);	   //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //发送地址
    IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev+1);  //进入接收模式	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //带ACK的读数据
		 	else  data[count]=IIC_Read_Byte(0);	 //最后一个字节NACK
	}
    IIC_Stop();//产生一个停止条件
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
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //发送地址
    IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();//产生一个停止条件

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
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
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
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
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
u8 IICwriteBitsm(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
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
u8 IICwriteBitm(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}


#define ZIN_35_ADDRESS 0xB0 //模块地址 最低位读写标志位 0：写  1：读

//请根据以下命令操作模块 ADDR（写）+ CMD命令
#define MODULE_REST 0x20 	//无特殊要求 可以不用
#define MODULE_GET_OFFSET 0x21  //陀螺仪校准命令
#define READ_FLASH 0x22  //使模块内部读取 校准数据

#define EKF_FILTER_ON 0x24
#define EKF_FILTER_OFF 0x25

//请根据以下命令读取模块数据  ADDR（写）+REG+ADDR（读 = 写+1） + ......数据串，支持连续地址上的数据连续读取
#define MPU_ACC_READ 0x30
#define MPU_GYRO_READ 0x31
#define ANGLE_READ 0x32
#define HEIGHT_READ 0X33 //高度数据寄存器地址
#define RATE_READ 0X34 //
float rate,height;
struct _st_angle
{
	float roll;
	float pitch;
	float yaw;
}angle;



/******************************************************************************
 * 函数名称: I2c_Start
 * 函数功能: I2c  起始信号
 * 入口参数: 无
 ******************************************************************************/
static uint8_t I2c_Start(void)
{
    SDA_H;
    SCL_H;
	I2c_delay();
    if (!SDA_read)
        return 0;
    SDA_L;
    I2c_delay();
    if (SDA_read)
        return 0;
    SCL_L;
    I2c_delay();
    return 1;
}

/******************************************************************************
 * 函数名称: I2c_Stop
 * 函数功能: I2c  停止信号
 * 入口参数: 无
 ******************************************************************************/
static void I2c_Stop(void)
{
    SCL_L;
    I2c_delay();
    SDA_L;
	I2c_delay();
    I2c_delay();
    SCL_H;
	I2c_delay();
    SDA_H;
    I2c_delay();
}

/******************************************************************************
 * 函数名称: I2c_Ack
 * 函数功能: I2c  产生应答信号
 * 入口参数: 无
 ******************************************************************************/
static void I2c_Ack(void)
{
    SCL_L;
    I2c_delay();
    SDA_L;
    I2c_delay();
    SCL_H;
	I2c_delay();
	I2c_delay();
	I2c_delay();
    I2c_delay();
    SCL_L;
    I2c_delay();
}

/******************************************************************************
 * 函数名称: I2c_NoAck
 * 函数功能: I2c  产生NAck
 * 入口参数: 无
 ******************************************************************************/
static void I2c_NoAck(void)
{
    SCL_L;
    I2c_delay();
    SDA_H;
    I2c_delay();
    SCL_H;
	I2c_delay();
	I2c_delay();
	I2c_delay();
    I2c_delay();
    SCL_L;
    I2c_delay();
}

/*******************************************************************************
 *函数名称:	I2c_WaitAck
 *函数功能:	等待应答信号到来
 *返回值：   1，接收应答失败
 *           0，接收应答成功
 *******************************************************************************/
static uint8_t I2c_WaitAck(void)
{
    SCL_L;
    I2c_delay();
    SDA_H;
    I2c_delay();
    SCL_H;
	I2c_delay();
	I2c_delay();
    I2c_delay();
	
    if (SDA_read) {
        SCL_L;
        return 0;
    }
    SCL_L;
    return 1;
}

/******************************************************************************
 * 函数名称: I2c_SendByte
 * 函数功能: I2c  发送一个字节数据
 * 入口参数: byte  发送的数据
 ******************************************************************************/
static void I2c_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--) {
        SCL_L;
        I2c_delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2c_delay();
        SCL_H;
		I2c_delay();
		I2c_delay();
		I2c_delay();
    }
    SCL_L;
}

/******************************************************************************
 * 函数名称: I2c_ReadByte
 * 函数功能: I2c  读取一个字节数据
 * 入口参数: 无
 * 返回值	 读取的数据
 ******************************************************************************/
static uint8_t I2c_ReadByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_L;
        I2c_delay();
		I2c_delay();
        SCL_H;
		I2c_delay();
        I2c_delay();
		I2c_delay();
        if (SDA_read) {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}

/******************************************************************************
 * 函数名称: i2cWriteBuffer
 * 函数功能: I2c       向设备的某一个地址写入固定长度的数据
 * 入口参数: addr,     设备地址
 *           reg，     寄存器地址
 *			 len，     数据长度
 *			 *data	   数据指针
 * 返回值	 1
 ******************************************************************************/
uint8_t i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
    int i;
    if (!I2c_Start())
        return 0;
    I2c_SendByte(addr);
    if (!I2c_WaitAck()) {
        I2c_Stop();
        return 0;
    }
    I2c_SendByte(reg);
    I2c_WaitAck();
    for (i = 0; i < len; i++) {
        I2c_SendByte(data[i]);
        if (!I2c_WaitAck()) {
            I2c_Stop();
            return 0;
        }
    }
    I2c_Stop();
    return 1;
}
/////////////////////////////////////////////////////////////////////////////////
int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
	if(i2cWriteBuffer(addr,reg,len,data))
	{
		return 1;
	}
	else
	{
		return 0;
	}
	//return 0;
}

/******************************************************************************
 * 函数名称: i2cread
 * 函数功能: I2c  向设备的某一个地址读取固定长度的数据
 * 入口参数: addr,   设备地址
 *           reg，   寄存器地址首地址
 *			 len，   数据长度
 *			 *buf	 数据指针
 * 返回值	 成功 返回 1
 *           错误 返回 0
 ******************************************************************************/


/*****************************************************************************
 *函数名称:	i2cWrite
 *函数功能:	写入指定设备 指定寄存器一个字节
 *入口参数： addr 目标设备地址
 *		     reg   寄存器地址
 *		     data 读出的数据将要存放的地址
 *******************************************************************************/
uint8_t i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (!I2c_Start())
        return 0;
    I2c_SendByte(addr);
    if (!I2c_WaitAck()) {
        I2c_Stop();
        return 0;
    }
    I2c_SendByte(reg);
    I2c_WaitAck();
    I2c_SendByte(data);
    I2c_WaitAck();
    I2c_Stop();
    return 1;
}


uint8_t i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2c_Start())
        return 0;
    I2c_SendByte(addr);
    if (!I2c_WaitAck()) {
        I2c_Stop();
        return 0;
    }
    I2c_SendByte(reg);
    I2c_WaitAck();
	I2c_Stop();
    I2c_Start();
    I2c_SendByte(addr+1);
    if (!I2c_WaitAck()) {
        I2c_Stop();
        return 0;
    }
    while (len) {
        *buf = I2c_ReadByte();
        if (len == 1)
            I2c_NoAck();
        else
            I2c_Ack();
        buf++;
        len--;
    }
    I2c_Stop();
    return 1;
}


int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	if(i2cRead(addr,reg,len,buf))
	{
		return 1;
	}
	else
	{
		return 0;
	}
	//return 0;
}

  float ByteToFloat( u8* byteArry)
{
  return *((float*)byteArry);
}
float z_speed123;
u8 buf_test[4];
void read_zin(void){
	  //i2cRead(ZIN_35_ADDRESS,MPU_ACC_READ,12,(uint8_t *)&mpu.accX);    //得出MPU的原始值（经过滤波处理的值），每两个字节为一个数据，共6个short int型数据，
		
		//IICreadBytes(ZIN_35_ADDRESS,ANGLE_READ,12,(uint8_t *)&angle);       //得出角度值，每四个字节为一个数据，共3个float型数据
		IICreadBytes(ZIN_35_ADDRESS,RATE_READ,4,(uint8_t *)&z_speed123);         //得出垂直方向上的速度，四个字节为一个float数据
   // z_speed123=ByteToFloat(buf_test);
		//IICreadBytes(ZIN_35_ADDRESS,HEIGHT_READ,4,(uint8_t *)&height);     //得出高度值，四个字节为一个float数据
}
	//------------------End of File----------------------------
