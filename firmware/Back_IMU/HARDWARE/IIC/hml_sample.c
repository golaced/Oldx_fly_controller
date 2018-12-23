

#include "include.h"
#include "iic.h"
#include "hml_sample.h"
#include "parameter.h"
#include "my_math.h"
#include "iic_soft.h"
#include "filter.h"
#include "hml5833l.h"

ak8975_t ak8975 = { {0,0,0},{-1,-1,-1},{1,0.8538,0.9389},{0,0,0} };

#define SOFT_I2C1_GPIO              GPIOB
#define SOFT_I2C1_PIN_SCL           GPIO_Pin_4
#define SOFT_I2C1_PIN_SDA           GPIO_Pin_3
#define SOFT_I2C1_DELAY             0

#define SOFT_I2C2_GPIO              GPIOA
#define SOFT_I2C2_PIN_SCL           GPIO_Pin_7
#define SOFT_I2C2_PIN_SDA           GPIO_Pin_7
#define SOFT_I2C2_DELAY             0

/**********************************************************************************************************
*? ? ?: Soft_I2c_Delay
*????: ??IIC????
*?    ?: ???
*? ? ?: ?
**********************************************************************************************************/
void Soft_I2c_Delay(uint8_t deviceNum)
{
    uint8_t i;
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();

    if(deviceNum == 1)
    {
        i = SOFT_I2C1_DELAY;    
    }  
    else if(deviceNum == 2)
    {
        i = SOFT_I2C2_DELAY;
    }  	

    while(i--);
}

/**********************************************************************************************************
*? ? ?: Soft_I2c_SCL_H
*????: SCL????
*?    ?: ???
*? ? ?: ?
**********************************************************************************************************/
void Soft_I2c_SCL_H(uint8_t deviceNum)
{
    if(deviceNum == 1)
    {
        SOFT_I2C1_GPIO->BSRRL = SOFT_I2C1_PIN_SCL;       
    }  
    else if(deviceNum == 2)
    {
        SOFT_I2C2_GPIO->BSRRL = SOFT_I2C2_PIN_SCL;
    }  
}

/**********************************************************************************************************
*? ? ?: Soft_I2c_SCL_L
*????: SCL????
*?    ?: ???
*? ? ?: ?
**********************************************************************************************************/
void Soft_I2c_SCL_L(uint8_t deviceNum)
{
    if(deviceNum == 1)
    {
        SOFT_I2C1_GPIO->BSRRH = SOFT_I2C1_PIN_SCL;       
    }  
    else if(deviceNum == 2)
    {
        SOFT_I2C2_GPIO->BSRRH = SOFT_I2C2_PIN_SCL;
    } 
}

/**********************************************************************************************************
*? ? ?: Soft_I2c_SDA_L
*????: SDA????
*?    ?: ???
*? ? ?: ?
**********************************************************************************************************/
void Soft_I2c_SDA_H(uint8_t deviceNum)
{
    if(deviceNum == 1)
    {
        SOFT_I2C1_GPIO->BSRRL = SOFT_I2C1_PIN_SDA;       
    }  
    else if(deviceNum == 2)
    {
        SOFT_I2C2_GPIO->BSRRL = SOFT_I2C2_PIN_SDA;
    }  
}

/**********************************************************************************************************
*? ? ?: Soft_I2c_SDA_L
*????: SDA????
*?    ?: ???
*? ? ?: ?
**********************************************************************************************************/
void Soft_I2c_SDA_L(uint8_t deviceNum)
{
    if(deviceNum == 1)
    {
        SOFT_I2C1_GPIO->BSRRH = SOFT_I2C1_PIN_SDA;       
    }  
    else if(deviceNum == 2)
    {
        SOFT_I2C2_GPIO->BSRRH = SOFT_I2C2_PIN_SDA;
    } 
}

/**********************************************************************************************************
*? ? ?: Soft_I2c_SCL_Read
*????: ??SCL????:0?1
*?    ?: ???
*? ? ?: ?
**********************************************************************************************************/
uint8_t Soft_I2c_SCL_Read(uint8_t deviceNum)
{
    if(deviceNum == 1)
    {
        if(!(SOFT_I2C1_GPIO->IDR & SOFT_I2C1_PIN_SCL))
            return 0;
        else
            return 1;
    }  
    else if(deviceNum == 2)
    {
        if(!(SOFT_I2C2_GPIO->IDR & SOFT_I2C2_PIN_SCL))
            return 0;
        else
            return 1;
    } 
    else
    {
        return 0;
    }
}

/**********************************************************************************************************
*? ? ?: Soft_I2c_SDA_Read
*????: ??SDA????:0?1
*?    ?: ???
*? ? ?: ?
**********************************************************************************************************/
uint8_t Soft_I2c_SDA_Read(uint8_t deviceNum)
{
    if(deviceNum == 1)
    {
        if(!(SOFT_I2C1_GPIO->IDR & SOFT_I2C1_PIN_SDA))
            return 0;
        else
            return 1;
    }  
    else if(deviceNum == 2)
    {
        if(!(SOFT_I2C2_GPIO->IDR & SOFT_I2C2_PIN_SDA))
            return 0;
        else
            return 1;
    } 
    else
    {
        return 0;
    }
}

/**********************************************************************************************************
*? ? ?: Soft_I2c_Start
*????: ??I2C??????
*?    ?: ???
*? ? ?: ?
**********************************************************************************************************/
bool Soft_I2c_Start(uint8_t deviceNum)
{
	Soft_I2c_SDA_H(deviceNum);
	Soft_I2c_SCL_H(deviceNum);
	Soft_I2c_Delay(deviceNum);
	if(!Soft_I2c_SDA_Read(deviceNum))
        return 0;
	Soft_I2c_SDA_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	if(Soft_I2c_SDA_Read(deviceNum)) 
        return 0;	
	Soft_I2c_SDA_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	return 1;	
}

/**********************************************************************************************************
*? ? ?: Soft_I2c_Stop
*????: ??I2C??????
*?    ?: ???
*? ? ?: ?
**********************************************************************************************************/
void Soft_I2c_Stop(uint8_t deviceNum)
{
	Soft_I2c_SCL_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SDA_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SCL_H(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SDA_H(deviceNum);
	Soft_I2c_Delay(deviceNum);
} 

/**********************************************************************************************************
*? ? ?: Soft_I2c_Ack
*????: ??I2C????
*?    ?: ???
*? ? ?: ?
**********************************************************************************************************/
void Soft_I2c_Ack(uint8_t deviceNum)
{	
	Soft_I2c_SCL_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SDA_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SCL_H(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SCL_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
}   

/**********************************************************************************************************
*? ? ?: Soft_I2c_NoAck
*????: ??I2C?????
*?    ?: ???
*? ? ?: ?
**********************************************************************************************************/
void Soft_I2c_NoAck(uint8_t deviceNum)
{	
	Soft_I2c_SCL_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SDA_H(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SCL_H(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SCL_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
} 

/**********************************************************************************************************
*? ? ?: Soft_I2c_WaitAck
*????: ????????
*?    ?: ???
*? ? ?: ????棗1?????
**********************************************************************************************************/
bool Soft_I2c_WaitAck(uint8_t deviceNum) 	
{
	Soft_I2c_SCL_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SDA_H(deviceNum);			
	Soft_I2c_Delay(deviceNum);
	Soft_I2c_SCL_H(deviceNum);
	Soft_I2c_Delay(deviceNum);
	if(Soft_I2c_SDA_Read(deviceNum))
	{
        Soft_I2c_SCL_L(deviceNum);
        Soft_I2c_Delay(deviceNum);
        return 0;
	}
	Soft_I2c_SCL_L(deviceNum);
	Soft_I2c_Delay(deviceNum);
	return 1;
}

/**********************************************************************************************************
*? ? ?: Soft_I2c_SendByte
*????: ??????
*?    ?: ???
*? ? ?: ?
**********************************************************************************************************/
void Soft_I2c_SendByte(uint8_t deviceNum, u8 SendByte) 
{
    u8 i=8;
    while(i--)
    {
        Soft_I2c_SCL_L(deviceNum);
        Soft_I2c_Delay(deviceNum);
        if(SendByte&0x80)
            Soft_I2c_SDA_H(deviceNum);  
        else 
            Soft_I2c_SDA_L(deviceNum);   
        SendByte<<=1;
        Soft_I2c_Delay(deviceNum);
        Soft_I2c_SCL_H(deviceNum);
        Soft_I2c_Delay(deviceNum);
    }
    Soft_I2c_SCL_L(deviceNum);
}  

/**********************************************************************************************************
*? ? ?: Soft_I2c_ReadByt
*????: ??????
*?    ?: ???
*? ? ?: ??????
**********************************************************************************************************/
uint8_t Soft_I2c_ReadByte(uint8_t deviceNum) 
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    Soft_I2c_SDA_H(deviceNum);				
    while(i--)
    {
        ReceiveByte<<=1;      
        Soft_I2c_SCL_L(deviceNum);
        Soft_I2c_Delay(deviceNum);
        Soft_I2c_SCL_H(deviceNum);
        Soft_I2c_Delay(deviceNum);	
        if(Soft_I2c_SDA_Read(deviceNum))
        {
            ReceiveByte|=0x01;
        }
    }
    Soft_I2c_SCL_L(deviceNum);
    return ReceiveByte;
} 

/**********************************************************************************************************
*? ? ?: Soft_I2c_Single_Write
*????: ???????
*?    ?: ??? ???? ????? ????
*? ? ?: ????
**********************************************************************************************************/
bool Soft_I2c_Single_Write(uint8_t deviceNum, u8 SlaveAddress,u8 REG_Address,u8 REG_data)		
{
  	if(!Soft_I2c_Start(deviceNum))
        return false;
    Soft_I2c_SendByte(deviceNum, SlaveAddress);  
    if(!Soft_I2c_WaitAck(deviceNum))
    {
        Soft_I2c_Stop(deviceNum); 
        return false;
    }
    Soft_I2c_SendByte(deviceNum, REG_Address);        
    Soft_I2c_WaitAck(deviceNum);	
    Soft_I2c_SendByte(deviceNum, REG_data);
    Soft_I2c_WaitAck(deviceNum);   
    Soft_I2c_Stop(deviceNum); 
    return true;
}

/**********************************************************************************************************
*? ? ?: Soft_I2c_Single_Read
*????: ???????
*?    ?: ??? ???? ?????
*? ? ?: ????
**********************************************************************************************************/
uint8_t Soft_I2C_Single_Read(uint8_t deviceNum, u8 SlaveAddress,u8 REG_Address)
{   
    uint8_t REG_data;     	
    if(!Soft_I2c_Start(deviceNum))
        return false;
    Soft_I2c_SendByte(deviceNum, SlaveAddress);  
    if(!Soft_I2c_WaitAck(deviceNum))
    {
        Soft_I2c_Stop(deviceNum);
        return false;
    }
    Soft_I2c_SendByte(deviceNum, (u8)REG_Address); 
    Soft_I2c_WaitAck(deviceNum);
    Soft_I2c_Start(deviceNum);
    Soft_I2c_SendByte(deviceNum, SlaveAddress+1);
    Soft_I2c_WaitAck(deviceNum);

	REG_data = Soft_I2c_ReadByte(deviceNum);
    Soft_I2c_NoAck(deviceNum);
    Soft_I2c_Stop(deviceNum);
    return REG_data;
}	

/**********************************************************************************************************
*? ? ?: Soft_I2C_Multi_Read
*????: ?????????
*?    ?: ??? ???? ????? ??????? ????
*? ? ?: ????
**********************************************************************************************************/
bool Soft_I2C_Multi_Read(uint8_t deviceNum, u8 SlaveAddress,u8 REG_Address,u8 * ptChar,u8 size)
{
    uint8_t i;
    
    if(size < 1)
        return false;
    if(!Soft_I2c_Start(deviceNum))
		return false;
    Soft_I2c_SendByte(deviceNum, SlaveAddress);
    if(!Soft_I2c_WaitAck(deviceNum))
    {
        Soft_I2c_Stop(deviceNum);
        return false;
    }
    Soft_I2c_SendByte(deviceNum, REG_Address);    
    Soft_I2c_WaitAck(deviceNum);
    
    Soft_I2c_Start(deviceNum);
    Soft_I2c_SendByte(deviceNum, SlaveAddress+1);
    Soft_I2c_WaitAck(deviceNum);
    
    for(i=1;i<size; i++)
    {
        *ptChar++ = Soft_I2c_ReadByte(deviceNum);
        Soft_I2c_Ack(deviceNum);
    }
    *ptChar++ = Soft_I2c_ReadByte(deviceNum);
    Soft_I2c_NoAck(deviceNum);
    Soft_I2c_Stop(deviceNum);
    return true;    
}	

/**************************实现函数********************************************
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/
void Soft_I2c_Open(uint8_t deviceNum)
{
    GPIO_InitTypeDef  GPIO_InitStructure; 
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    
    if(deviceNum == 1)
    {
        GPIO_InitStructure.GPIO_Pin =  SOFT_I2C1_PIN_SCL | SOFT_I2C1_PIN_SDA;
        GPIO_Init(SOFT_I2C1_GPIO, &GPIO_InitStructure);	
    }  
    else if(deviceNum == 2)
    {
        GPIO_InitStructure.GPIO_Pin =  SOFT_I2C2_PIN_SCL | SOFT_I2C2_PIN_SDA;
        GPIO_Init(SOFT_I2C2_GPIO, &GPIO_InitStructure);	
    }      
}



#define QMC5883L_Addr   0x1A     
#define QMC5883L_HX_L   0x00
#define QMC5883L_HX_H   0x01
#define QMC5883L_HY_L   0x02
#define QMC5883L_HY_H   0x03
#define QMC5883L_HZ_L   0x04 
#define QMC5883L_HZ_H   0x05
#define QMC5883L_CTR1   0x09
#define QMC5883L_SRPERIOD 0x0B

#define QMC_2G ((float)0.00008333f)  // 12000 LSB/G
#define QMC_8G ((float)0.00033333f)  // 3000  LSB/G

#define QMC5883_MAG_TO_GAUSS QMC_2G
typedef struct{
	float x;
	float y;
	float z;
}Vector3f_t;
typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}Vector3i_t;
Vector3i_t magRaw;

static void QMC5883_WriteReg(u8 REG_Address,u8 REG_data)
{
    Soft_I2c_Single_Write(1, QMC5883L_Addr, REG_Address, REG_data);
}

static uint8_t QMC5883_ReadReg(u8 REG_Address)
{
    return Soft_I2C_Single_Read(1, QMC5883L_Addr, REG_Address);
}

/**********************************************************************************************************
*? ? ?: QMC5883_Detect
*????: ??QMC5883????
*?    ?: ?
*? ? ?: ????
**********************************************************************************************************/
bool QMC5883_Detect(void)
{
   QMC5883_WriteReg(QMC5883L_SRPERIOD,0x01);
	 QMC5883_WriteReg(QMC5883L_CTR1,0x0D);  //2Guass 200Hz	
	 delay_ms(50);
	
	if(QMC5883_ReadReg(QMC5883L_CTR1) == 0x0D)
		module.outer_hml=1;
}

/**********************************************************************************************************
*? ? ?: QMC5883_Init
*????: QMC5883????????
*?    ?: ?
*? ? ?: ?
**********************************************************************************************************/
void QMC5883_Init(void)
{
	QMC5883_WriteReg(QMC5883L_SRPERIOD, 0x01);
	delay_ms(5);  
	QMC5883_WriteReg(QMC5883L_CTR1, 0x1d);//0x0D);  //2Guass 200Hz	
}

/**********************************************************************************************************
*? ? ?: QMC5883_Update
*????: QMC5883????
*?    ?: ?
*? ? ?: ?
**********************************************************************************************************/
int mag_outer[3];
void QMC5883_Update(void)
{
	uint8_t buffer[6];

	buffer[1] = QMC5883_ReadReg(QMC5883L_HX_L);	
	buffer[0] = QMC5883_ReadReg(QMC5883L_HX_H);
	magRaw.x = (int16_t)buffer[0] << 8 | buffer[1];
	
	buffer[3] = QMC5883_ReadReg(QMC5883L_HY_L);
	buffer[2] = QMC5883_ReadReg(QMC5883L_HY_H);
	magRaw.y = (int16_t)buffer[2] << 8 | buffer[3];

	buffer[5] = QMC5883_ReadReg(QMC5883L_HZ_L);	
	buffer[4] = QMC5883_ReadReg(QMC5883L_HZ_H); 
	magRaw.z = (int16_t)buffer[4] << 8 | buffer[5];
	mag_outer[0]=magRaw.x/2;
	mag_outer[1]=magRaw.y/2;
	mag_outer[2]=magRaw.z/2;
}

/**********************************************************************************************************
*? ? ?: QMC5883_Read
*????: ?????????,????????
*?    ?: ??????
*? ? ?: ?
**********************************************************************************************************/
void QMC5883_Read(Vector3f_t* mag)
{
	mag->x = magRaw.x * QMC5883_MAG_TO_GAUSS;
	mag->y = magRaw.y * QMC5883_MAG_TO_GAUSS;
	mag->z = magRaw.z * QMC5883_MAG_TO_GAUSS;
}


void Outer_Compass_init(void)
{
   Soft_I2c_Open(1);
	 QMC5883_Detect();
   QMC5883_Init();
}