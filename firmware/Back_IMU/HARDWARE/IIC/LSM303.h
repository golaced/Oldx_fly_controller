#ifndef __LSM303_H
#define __LSM303_H


#include "parameter.h"
#include "stm32f4xx.h"
#include  <math.h>

#define LSM303DLHC_FS_1_3_GA 0x20
#define LSM303DLHC_M_SENSITIVITY_XY_1_3Ga 1100
#define LSM303DLHC_M_SENSITIVITY_Z_1_3Ga 980
#define LSM303DLHC_FS_1_9_GA 0x40
#define LSM303DLHC_M_SENSITIVITY_XY_1_9Ga 855
#define LSM303DLHC_M_SENSITIVITY_Z_1_9Ga 760
#define LSM303DLHC_FS_2_5_GA 0x60
#define LSM303DLHC_M_SENSITIVITY_XY_2_5Ga 670
#define LSM303DLHC_M_SENSITIVITY_Z_2_5Ga 600
#define LSM303DLHC_FS_4_0_GA 0x80
#define LSM303DLHC_M_SENSITIVITY_XY_4Ga 450
#define LSM303DLHC_M_SENSITIVITY_Z_4Ga 400
#define LSM303DLHC_FS_4_7_GA 0xa0
#define LSM303DLHC_M_SENSITIVITY_XY_4_7Ga 400
#define LSM303DLHC_M_SENSITIVITY_Z_4_7Ga 355
#define LSM303DLHC_FS_5_6_GA 0xc0
#define LSM303DLHC_M_SENSITIVITY_XY_5_6Ga 330
#define LSM303DLHC_M_SENSITIVITY_Z_5_6Ga 295
#define LSM303DLHC_FS_8_1_GA 0xe0
#define LSM303DLHC_M_SENSITIVITY_XY_8_1Ga 230
#define LSM303DLHC_M_SENSITIVITY_Z_8_1Ga 205


//磁场内部寄存器***********************************
/* Magnetometer registers */
#define LSM303DLHC_CRA_REG_M		0x00	/* Configuration register A */
#define LSM303DLHC_CRB_REG_M		0x01	/* Configuration register B */
#define LSM303DLHC_MR_REG_M			0x02	/* Mode register */
/* resume state index */
#define RES_CRA_REG_M		0	/* Configuration register A */
#define RES_CRB_REG_M		1	/* Configuration register B */
#define RES_MR_REG_M		2	/* Mode register */
/* Output register start address*/
#define OUT_X_M			    0x03
/* Magnetic Sensor Operation Mode */
#define NORMAL_MODE     	0x00
#define POS_BIAS         	0x01
#define NEG_BIAS         	0x02
#define CC_MODE          	0x00
#define SC_MODE			    0x01
#define SLEEP_MODE		    0x03


#define LSM303DLHC_OUT_X_H_M 0x03
#define LSM303DLHC_OUT_X_L_M 0x04
#define LSM303DLHC_OUT_Z_H_M 0x05
#define LSM303DLHC_OUT_Z_L_M 0x06
#define LSM303DLHC_OUT_Y_H_M 0x07
#define LSM303DLHC_OUT_Y_L_M 0x08


//加速度内部寄存器***********************************
#define AXISDATA_REG	  	    0x28
#define WHOAMI_LSM303DLH_ACC	0x32	/*	Expctd content for WAI	*/
/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x0F	/*	WhoAmI register		*/
#define LSM303DLHC_CTRL_REG1_A		0x20	/*				*/
#define CTRL_REG2		0x21	/*				*/
#define CTRL_REG3		0x22	/*				*/
#define LSM303DLHC_CTRL_REG4_A		0x23	/*				*/
#define	CTRL_REG5		0x24	/*				*/

#define	INT_CFG1		0x30	/*	interrupt 1 config	*/
#define	INT_SRC1		0x31	/*	interrupt 1 source	*/
#define	INT_THS1		0x32	/*	interrupt 1 threshold	*/
#define	INT_DUR1		0x33	/*	interrupt 1 duration	*/

#define	INT_CFG2		0x34	/*	interrupt 2 config	*/
#define	INT_SRC2		0x35	/*	interrupt 2 source	*/
#define	INT_THS2		0x36	/*	interrupt 2 threshold	*/
#define	INT_DUR2		0x37	/*	interrupt 2 duration	*/

#define LSM303DLHC_FULLSCALE_2G            ((uint8_t)0x00)  /*!< ? g */
#define LSM303DLHC_FULLSCALE_4G            ((uint8_t)0x10)  /*!< ? g */
#define LSM303DLHC_FULLSCALE_8G            ((uint8_t)0x20)  /*!< ? g */
#define LSM303DLHC_FULLSCALE_16G           ((uint8_t)0x30)  /*!< ?6 g */


#define LSM_Acc_Sensitivity_2g     (float)     1            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     2           /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     4           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     12         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */

#define LSM303DLHC_OUT_X_L_A 0x28



#define D_SA0_HIGH_ADDRESS              0x1d // D with SA0 high
#define D_SA0_LOW_ADDRESS               0x1e // D with SA0 low or non-D magnetometer
#define NON_D_MAG_ADDRESS               0x1e // D with SA0 low or non-D magnetometer
#define NON_D_ACC_SA0_LOW_ADDRESS       0x18 // non-D accelerometer with SA0 low
#define NON_D_ACC_SA0_HIGH_ADDRESS      0x19 // non-D accelerometer with SA0 high

#define	MAG_I2C_ADDRESS    0x3C	
#define	ACC_I2C_ADDRESS    0x1D//0x32 >> 1 



#define LSM303A_I2C_ADDR				((u8)0x32)
#define LSM303M_I2C_ADDR				((u8)0x3C)

#define LSM303A_OUT_X_L					((u8)0x28)
#define LSM303A_OUT_X_H					((u8)0x29)
#define LSM303A_OUT_Y_L					((u8)0x2A)
#define LSM303A_OUT_Y_H					((u8)0x2B)
#define LSM303A_OUT_Z_L					((u8)0x2C)
#define LSM303A_OUT_Z_H					((u8)0x2D)
#define LSM303A_CTRL_REG1				((u8)0x20)
#define LSM303A_CTRL_REG2				((u8)0x21)
#define LSM303A_CTRL_REG3				((u8)0x22)
#define LSM303A_CTRL_REG4				((u8)0x23)
#define LSM303A_CTRL_REG5				((u8)0x24)
typedef struct LSM303DLHCMag_Init
{
	int Temperature_Sensor;
	int MagOutput_DataRate;
	int MagFull_Scale;
	int Working_Mode;
}LSM303DLHCMag_InitTypeDef;

typedef struct LSM303DLHCAcc_Init
{
	int Power_Mode;
	int AccOutput_DataRate;
	int Axes_Enable;
	int AccFull_Scale;
	int BlockData_Update;
	int Endianness;
	int High_Resolution;
}LSM303DLHCAcc_InitTypeDef;

typedef struct LSM303DLHCFilter_Init
{
	int HighPassFilter_Mode_Selection;
	int HighPassFilter_AOI1;
	int HighPassFilter_AOI2;
}LSM303DLHCFilter_InitTypeDef;


uint16_t LSM303DLHC_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer);
uint16_t LSM303DLHC_Read(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead);
void LSM303DLHC_AccInit(LSM303DLHCAcc_InitTypeDef *LSM303DLHCAcc_InitStruct);
void LSM303DLHC_MagInit(LSM303DLHCMag_InitTypeDef *LSM303DLHC_InitStruct);
void Demo_CompassReadMag(float* pfData);
void Demo_CompassReadAcc(float* pfData);
void LSD_Sample(void);


typedef struct 
{
	char Acc_CALIBRATE;
	char Gyro_CALIBRATE;
  xyz_s16_t Acc_I16;
	xyz_s16_t Gyro_I16;

	xyz_f_t Acc;
	xyz_f_t Gyro;

//	XYZ_STRUCT Acc_deg;
	xyz_f_t Gyro_deg;
	
	xyz_f_t Acc_Offset;
	xyz_f_t Gyro_Offset;
	xyz_f_t Gyro_Auto_Offset;
	float Temprea_Offset;
	float Gyro_Temprea_Adjust;
	float ACC_Temprea_Adjust;

	s16 Tempreature;
	float TEM_LPF;
	float Ftempreature;
	
	xyz_s16_t Mag_Adc;			//采样值
	xyz_f_t Mag_Offset;		//偏移值
	xyz_f_t 	Mag_Gain;			//比例缩放	
  xyz_f_t 	Mag_Val;			//纠正后的值
	u8 Mag_CALIBRATED;
	float yaw;
}LSM303_STRUCT;

extern LSM303_STRUCT lsm303;
#endif
