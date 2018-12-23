

#include "stm32f4xx.h"

#define SCL_H_PX4         I2C_GPIO_PX4_S->BSRRL = I2C_Pin_SCL_PX4
#define SCL_L_PX4         I2C_GPIO_PX4_S->BSRRH = I2C_Pin_SCL_PX4
#define SDA_H_PX4         I2C_GPIO_PX4_D->BSRRL = I2C_Pin_SDA_PX4
#define SDA_L_PX4         I2C_GPIO_PX4_D->BSRRH = I2C_Pin_SDA_PX4
#define SCL_Read_PX4      I2C_GPIO_PX4_S->IDR  & I2C_Pin_SCL_PX4
#define SDA_Read_PX4      I2C_GPIO_PX4_D->IDR  & I2C_Pin_SDA_PX4

#define I2C_GPIO_PX4_S	GPIOC
#define I2C_Pin_SCL_PX4		GPIO_Pin_12
#define I2C_GPIO_PX4_D	GPIOD
#define I2C_Pin_SDA_PX4		GPIO_Pin_2


//#define I2C_GPIO_PX4_S	GPIOD
//#define I2C_Pin_SCL_PX4		GPIO_Pin_2
//#define I2C_GPIO_PX4_D	GPIOC
//#define I2C_Pin_SDA_PX4		GPIO_Pin_12


#define PIN_SCL_PX4 12
#define PIN_SDA_PX4 2
//IO操作函数	 
#define IIC_SCL_PX4    PCout(PIN_SCL_PX4) //SCL
#define IIC_SDA_PX4    PDout(PIN_SDA_PX4) //SDA	 
#define READ_SDA_PX4   PDin(PIN_SDA_PX4)  //输入SDA 

#define SDA_IN_PX4()  {I2C_GPIO_PX4_D->MODER&=~(3<<(PIN_SDA_PX4*2));I2C_GPIO_PX4_D->MODER|=0<<PIN_SDA_PX4*2;}	//PB9输入模式
#define SDA_OUT_PX4() {I2C_GPIO_PX4_D->MODER&=~(3<<(PIN_SDA_PX4*2));I2C_GPIO_PX4_D->MODER|=1<<PIN_SDA_PX4*2;} //PB9输出模式

//IIC所有操作函数
void IIC_Init_PX4(void);                //初始化IIC的IO口				 
void IIC_Start_PX4(void);				//发送IIC开始信号
void IIC_Stop_PX4(void);	  			//发送IIC停止信号
void IIC_Send_Byt_PX4e(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte_PX4(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack_PX4(void); 				//IIC等待ACK信号
void IIC_Ack_PX4(void);					//IIC发送ACK信号
void IIC_NAck_PX4(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte_PX4(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte_PX4(u8 daddr,u8 addr);	 
unsigned char I2C_Readkey_PX4(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte_PX4(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte_PX4(unsigned char dev, unsigned char reg, unsigned char data);
u8 IICwriteBytes_PX4(u8 dev, u8 reg, u8 length, u8* data);
u8 IICwriteBits_PX4(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IICwriteBit_PX4(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICreadBytes_PX4(u8 dev, u8 reg, u8 length, u8 *data);

//------------------End of File----------------------------


void Soft_I2C_Init_PX4(void);
uint8_t I2C_Single_Write_PX4(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t data);
uint8_t I2C_Single_Read_PX4(uint8_t SlaveAddress, uint8_t REG_Address);
uint8_t I2C_WriteBuffer_PX4(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
uint8_t I2C_ReadBuffer_PX4(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

uint8_t I2C_Start_PX4(void);
void I2C_Stop_PX4(void);
void I2C_Ack_PX4(void);
void I2C_NoAck_PX4(void);
uint8_t I2C_WaitAck_PX4(void);
void I2C_SendByte_PX4(uint8_t byte) ;
uint8_t I2C_ReceiveByte_PX4(void);


#define PX4FLOW_ADDR  0x42
//??????,???2??????????????,????????????,???????????  ??????

//??????
//???????iic??
#define FRAME_COUNT_SUM     0x00                    //uint16_t counts created I2C frames [frames]
//X????????????*10
#define PIXEL_FLOW_X_SUM    0x02                    //int16_t latest x flow measurement in pixels*10 [pixels]
//Y????????????*10
#define PIXEL_FLOW_Y_SUM    0x04                    //int16_t latest y flow measurement in pixels*10 [pixels]
//X???
#define FLOW_COMP_M_X       0x06                    //int16_t x velocity*1000 [meters/sec]
//Y???
#define FLOW_COMP_M_Y       0x08                    //int16_t y velocity*1000 [meters/sec]
//??????
#define QUAL                0x0a                    //int16_t Optical flow quality / confidence [0: bad, 255: maximum quality]
//X??????
#define GYRO_X_RATE         0x0c                    //int16_t latest gyro x rate [rad/sec]
//Y??????
#define GYRO_Y_RATE         0x0e                    //int16_t latest gyro y rate [rad/sec]
//Z??????
#define GYRO_Z_RATE         0x10                    //int16_t latest gyro z rate [rad/sec]
//???????
#define GYRO_RANGE          0x12                    //uint8_t gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec]
//???????????????????
#define SONAR_TIMESTAMP1     0x13                    //uint8_t time since last sonar update [milliseconds] 
//????  ??:????,  ??:????
#define GROUND_DISTANCE1    0x14                    //int16_t Ground distance in meters*1000 [meters]. Positive value: distance known. 
                                                            //Negative value: Unknown distance
//????????
//???????????????
#define FRAME_COUNT_SINCE_LAST_READOUT  0x16        //uint16_t number of flow measurements since last I2C readout [frames]
//?????iic??? X?????????
#define PIXEL_FLOW_X_INTEGRAL           0x18        //int16_t  accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
//?????iic??? Y?????????
#define PIXEL_FLOW_Y_INTEGRAL           0x1a        //int16_t  accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
//?????iic???,X???????
#define GYRO_X_RATE_INTEGRAL            0x1c        //int16_t  accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000]  
//?????iic???,Y???????
#define GYRO_Y_RATE_INTEGRAL            0x1e        //int16_t  accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] 
//?????iic???,Z???????
#define GYRO_Z_RATE_INTEGRAL            0x20        //int16_t  accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000]
//???????iic???????
#define INTEGRATION_TIMESPAN            0x22        //uint32_t accumulation timespan in microseconds since last I2C readout [microseconds]
//???????????????????
#define SONAR_TIMESTAMP2                 0x26        //uint32_t time since last sonar update [microseconds]
//????
#define GROUND_DISTANCE2                0x2a        //int16_t  Ground distance in meters*1000 [meters*1000]
//?????
#define GYRO_TEMPERATURE                0x2c        //int16_t  Temperature * 100 in centi-degrees Celsius [degcelsius*100] 
//????????
#define QUALITY                         0x2e        //uint8_t averaged quality of accumulated flow values [0:bad quality;255: max quality]


//?????????????
u8 flow_read_data(u8 addr,u8 reg,u8 len,u8 *buf);
//?8??????
uint8_t     readu8_date(u8 addr,u8 reg);
//?16??????
uint16_t    readu16_date(u8 addr,u8 reg);
//?16??????
int16_t     reads16_date(u8 addr,u8 reg);
//?32??????
uint32_t    readu32_date(u8 addr,u8 reg);
