#ifndef _SPI_H_
#define _SPI_H_
#include "stm32f4xx.h"
#include "spi.h"

/* MPU9250          ------------------------------------------- */
#define SELF_TEST_X_GYRO		0X00
#define SELF_TEST_Y_GYRO		0X01
#define SELF_TEST_Z_GYRO		0X02

#define SELF_TEST_X_ACCEL		0X0D
#define SELF_TEST_Y_ACCEL		0X0E
#define SELF_TEST_Z_ACCEL		0X0F

#define XG_OFFSET_H					0X13
#define XG_OFFSET_L					0X14
#define YG_OFFSET_H					0X15
#define YG_OFFSET_L					0X16
#define ZG_OFFSET_H					0X17
#define ZG_OFFSET_L					0X18

#define SMPLRT_DIV					0X19 //====陀螺仪采样率 典型值为0X07  1000/(1+7)=125HZ
#define CONFIG							0X1A //====低通滤波器  典型值0x06 5hz
#define GYRO_CONFIG					0X1B //====陀螺仪测量范围 0X18 正负2000度
#define ACCEL_CONFIG				0X1C //====加速度计测量范围 0X18 正负16g
#define ACCEL_CONFIG_2				0X1D //====加速度计低通滤波器 0x06 5hz

#define LP_ACCEL_ODR				0X1E
#define WOM_THR							0X1F
#define FIFO_EN							0X23

#define I2C_MST_CTRL        0x24
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV0_DO         0x63 //output reg

#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define	INT_PIN_CFG	        0x37
#define INT_ENABLE  				0x38

#define ACCEL_XOUT_H				0X3B  //====加速度计输出数据
#define ACCEL_XOUT_L				0X3C
#define ACCEL_YOUT_H				0X3D
#define ACCEL_YOUT_L				0X3E
#define ACCEL_ZOUT_H				0X3F
#define ACCEL_ZOUT_L				0X40

#define TEMP_OUT_H					0X41  //====温度计输出数据
#define TEMP_OUT_L					0X42

#define GYRO_XOUT_H					0X43  //====陀螺仪输出数据
#define GYRO_XOUT_L					0X44
#define GYRO_YOUT_H					0X45
#define GYRO_YOUT_L					0X46
#define GYRO_ZOUT_H					0X47
#define GYRO_ZOUT_L					0X48

#define EXT_SENS_DATA_00    0x49  //MPU9250 IIC外挂器件读取返回寄存器00
#define EXT_SENS_DATA_01    0x4a  //MPU9250 IIC外挂器件读取返回寄存器01
#define EXT_SENS_DATA_02    0x4b  //MPU9250 IIC外挂器件读取返回寄存器02
#define EXT_SENS_DATA_03    0x4c  //MPU9250 IIC外挂器件读取返回寄存器03

#define I2C_MST_DELAY_CTRL  0x67
#define PWR_MGMT_1					0X6B //====电源管理1 典型值为0x00
#define PWR_MGMT_2					0X6C //====电源管理2 典型值为0X00

#define WHO_AM_I						0X75 //====器件ID MPU9250默认ID为0X71
#define USER_CTRL						0X6A //====用户配置 当为0X10时使用SPI模式

#define I2C_SLVx_EN         0x80
#define I2C_SLV4_DONE       0x40
#define I2C_SLV4_NACK       0x10

/* AK8963 Reg In MPU9250 -------------------------------------- */

#define AK8963_ADDR             		0x0c
#define AK8963_DeviceID             0x48

/* Read-only Reg */
#define AK8963_WIA                  0x00
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
#define AK8963_ST2                  0x09
/* Write/Read Reg */
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F
/* Read-only Reg ( ROM ) */
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12
/* Status */
#define AK8963_STATUS_DRDY          0x01
#define AK8963_STATUS_DOR           0x02
#define AK8963_STATUS_HOFL          0x08

#define MPU9250_CS PBout(12)			//====MPU9250的片选位
#define MPU9250_CS1 PCout(4)			//====MPU9250的片选位

/* UNION Data 	   --------------------------------*/
typedef union
{
    int16_t value;
    uint8_t bytes[2];
} int16andUint8_t;

typedef union
{
    int32_t value;
    uint8_t bytes[4];
} int32andUint8_t;

typedef union
{
    uint16_t value;
    uint8_t bytes[2];
} uint16andUint8_t;

extern int16andUint8_t rawAccel[3];
extern int16andUint8_t rawGyro[3];
extern int16andUint8_t rawMag[3];
extern int16andUint8_t rawMPU6050Temperature;
/* DEFINE Data 	  --------------------------------*/
#define XORIENT   0
#define YORIENT    1
#define ZORIENT    2

#define ROLL     0
#define PITCH    1
#define YAW      2
uint8_t Mpu9250_Init(void);
uint8_t SPI1_ReadWriteByte(uint8_t TxData);
uint8_t MPU9250_Write_Reg(uint8_t reg,uint8_t value);
uint8_t MPU9250_Read_Reg(uint8_t reg);
void MPU92_Mag_WriteReg( uint8_t writeAddr, uint8_t writeData );
uint8_t MPU92_Mag_ReadReg( uint8_t readAddr );
void MPU9250_ReadValue(void);
void orientIMU(void);
#endif
