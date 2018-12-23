#include "LSM303.h"
#include "lsm1.h"
#include "iic_imu1.h"
#include "iic_soft.h"
#include "iic.h"
#include  <math.h>  

uint8_t LSM303DLHC_AccReadID(void)
{  
  uint8_t ctrl = 0x00;
  
  ctrl = I2C_ReadOneByte(ACC_I2C_ADDRESS,LSM303DLHC_WHO_AM_I_ADDR);
//  /* Low level init */
//  COMPASSACCELERO_IO_Init();
//  
//  /* Read value at Who am I register address */
//  ctrl = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_WHO_AM_I_ADDR);
  
  return ctrl;
}


void LSM303DLHC_MagInit(LSM303DLHCMag_InitTypeDef *LSM303DLHC_InitStruct)
{  

	unsigned char cra_regm = 0x00, crb_regm = 0x00, mr_regm = 0x00;

	LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;
	LSM303DLHC_InitStructure.Temperature_Sensor = 0;	//LSM303DLHC_TEMPSENSOR_DISABLE;  //???0x00
	LSM303DLHC_InitStructure.MagOutput_DataRate = 0x14; //LSM303DLHC_ODR_30_HZ ;     //???0x14
	LSM303DLHC_InitStructure.MagFull_Scale = 0xe0; //LSM303DLHC_FS_8_1_GA;    //???0xE0
	LSM303DLHC_InitStructure.Working_Mode = 0; //LSM303DLHC_CONTINUOS_CONVERSION;    //???0x00

	if(LSM303DLHC_InitStruct)
	{
		cra_regm |= (uint8_t) (LSM303DLHC_InitStruct->Temperature_Sensor | LSM303DLHC_InitStruct->MagOutput_DataRate);
		crb_regm |= (uint8_t) (LSM303DLHC_InitStruct->MagFull_Scale);
		mr_regm |= (uint8_t) (LSM303DLHC_InitStruct->Working_Mode);  
	}
	else
	{
		cra_regm |= (uint8_t) (LSM303DLHC_InitStructure.Temperature_Sensor | LSM303DLHC_InitStructure.MagOutput_DataRate);
		crb_regm |= (uint8_t) (LSM303DLHC_InitStructure.MagFull_Scale);
		mr_regm |= (uint8_t) (LSM303DLHC_InitStructure.Working_Mode);
	}
//	USART_printf("init_Mag-->cra_regm:%x ,crb_regm:%x ,mr_regm:%x\n\r",cra_regm,crb_regm,mr_regm);
	IICwriteByte(MAG_I2C_ADDRESS, LSM303DLHC_CRA_REG_M, cra_regm);
	IICwriteByte(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, crb_regm);
	IICwriteByte(MAG_I2C_ADDRESS, LSM303DLHC_MR_REG_M, mr_regm);
}

void LSM303A_Init( void )
{
    
  IICwriteByte(LSM303A_I2C_ADDR,LSM303A_CTRL_REG1, 0x37);
  IICwriteByte(LSM303A_I2C_ADDR,LSM303A_CTRL_REG4, 0x00);
}
void LSM303DLHC_AccInit(LSM303DLHCAcc_InitTypeDef *LSM303DLHCAcc_InitStruct)
{
	unsigned char ctrl1 = 0x00, ctrl4 = 0x00;
	LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
	
	LSM303DLHCAcc_InitStructure.Power_Mode = 0; //LSM303DLHC_NORMAL_MODE; //???0x00
	LSM303DLHCAcc_InitStructure.AccOutput_DataRate = 0x40; //LSM303DLHC_ODR_50_HZ; //???0x40
	LSM303DLHCAcc_InitStructure.Axes_Enable = 0x07; //LSM303DLHC_AXES_ENABLE;  //???0x07
	LSM303DLHCAcc_InitStructure.AccFull_Scale = 0; //LSM303DLHC_FULLSCALE_2G;  //???0x00
	LSM303DLHCAcc_InitStructure.BlockData_Update = 0; //LSM303DLHC_BlockUpdate_Continous; //???0x00
	LSM303DLHCAcc_InitStructure.Endianness = 0; //LSM303DLHC_BLE_LSB;  //???0x00
	LSM303DLHCAcc_InitStructure.High_Resolution = 0x08; //LSM303DLHC_HR_ENABLE;    //???0x08

	if(LSM303DLHCAcc_InitStruct)
	{
		ctrl1 |= (uint8_t) (LSM303DLHCAcc_InitStruct->Power_Mode | LSM303DLHCAcc_InitStruct->AccOutput_DataRate \
							| LSM303DLHCAcc_InitStruct->Axes_Enable);
		ctrl4 |= (uint8_t) (LSM303DLHCAcc_InitStruct->BlockData_Update | LSM303DLHCAcc_InitStruct->Endianness | \
							LSM303DLHCAcc_InitStruct->AccFull_Scale|LSM303DLHCAcc_InitStruct->High_Resolution);
	}
	else
	{
		ctrl1 |= (uint8_t) (LSM303DLHCAcc_InitStructure.Power_Mode | LSM303DLHCAcc_InitStructure.AccOutput_DataRate \
							| LSM303DLHCAcc_InitStructure.Axes_Enable);
		ctrl4 |= (uint8_t) (LSM303DLHCAcc_InitStructure.BlockData_Update | LSM303DLHCAcc_InitStructure.Endianness | \
							LSM303DLHCAcc_InitStructure.AccFull_Scale | LSM303DLHCAcc_InitStructure.High_Resolution);
	}
//	USART_printf("init_Acc-->ctrl1:%x ,ctrl4:%x\n\r",ctrl1,ctrl4);
	IICwriteByte(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A, ctrl1);
	IICwriteByte(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrl4);
	//LSM303A_Init();
	LSM303DLHC_AccReadID();
}
//void LSM303DLHC_AccFilterConfig
//LSM303DLHCFilter_InitTypeDef LSM303DLHCFilter_InitStructure;
//LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection = 0x80; //LSM303DLHC_HPM_NORMAL_MODE; //???0x80
//LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = 0x10; //LSM303DLHC_HPFCF_16;     //???0x10
//LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = 0; //LSM303DLHC_HPF_AOI1_DISABLE;     //???0x00
//LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = 0; //LSM303DLHC_HPF_AOI2_DISABLE;     //???0x00
//LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);

void Demo_CompassReadMag(float* pfData)
{
	static unsigned char buffer[6] = {0};
	unsigned char CTRLB = 0;
	uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
	uint8_t i =0;
	IICreadBytes(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, 1, &CTRLB);
	IICreadBytes(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, 1, buffer);
	IICreadBytes(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, 1,buffer+1);
	IICreadBytes(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, 1,buffer+2);
	IICreadBytes(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, 1,buffer+3);
	IICreadBytes(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, 1,buffer+4);
	IICreadBytes(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, 1,buffer+5);
	switch(CTRLB & 0xE0)
	{
		case LSM303DLHC_FS_1_3_GA:
			Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga; //??1100
			Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;   //??980
			break;
		case LSM303DLHC_FS_1_9_GA:
			Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;//??855
			Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga; //??760
			break;
		case LSM303DLHC_FS_2_5_GA:
			Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga; //??670
			Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga; //??600
			break;
		case LSM303DLHC_FS_4_0_GA:
			Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga; //??450
			Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga; //??400
			break;
		case LSM303DLHC_FS_4_7_GA:
			Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga; //??400
			Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;//??355
			break;
		case LSM303DLHC_FS_5_6_GA:
			Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;//??330
			Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;//??295
			break;
		case LSM303DLHC_FS_8_1_GA:
			Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;//??230
			Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;//??205
			break;
	}
	for(i=0; i<2; i++)
	{
		pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])*1000)/Magn_Sensitivity_XY;
	}
	pfData[2]=(float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/Magn_Sensitivity_Z;
}

void Demo_CompassReadAcc(float* pfData)
{
	int16_t pnRawData[3] = {0};
	unsigned char ctrlx[2] = {0};
	unsigned char buffer[6] = {0}, cDivider=0;
	uint8_t i = 0;
	float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
	//IICreadBytes(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, 2,ctrlx);
	//USART_printf("ctrlx[0]:%d ctrlx[1]:%d\n\r",ctrlx[0],ctrlx[1]);
	//IICreadBytes(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, 6,buffer);
	IICreadBytes(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A,2, ctrlx);
	IICreadBytes(ACC_I2C_ADDRESS, LSM303A_OUT_X_H, 1, buffer);
	IICreadBytes(ACC_I2C_ADDRESS, LSM303A_OUT_X_L, 1,buffer+1);
	IICreadBytes(ACC_I2C_ADDRESS, LSM303A_OUT_Y_H, 1,buffer+2);
	IICreadBytes(ACC_I2C_ADDRESS, LSM303A_OUT_Y_L, 1,buffer+3);
	IICreadBytes(ACC_I2C_ADDRESS, LSM303A_OUT_Z_H, 1,buffer+4);
	IICreadBytes(ACC_I2C_ADDRESS, LSM303A_OUT_Z_L, 1,buffer+5);
	if(ctrlx[1]&0x40)
		cDivider=64;//??
	else
		cDivider=16; //??
	if(!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) 
	{
		for(i=0; i<3; i++)
		{
			pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i])/cDivider;
		}
	}	
	else 
	{
		for(i=0; i<3; i++)
		pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])/cDivider;
	}
	IICreadBytes(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A,2, ctrlx);
	if(ctrlx[1]&0x40)
	{
		LSM_Acc_Sensitivity = 0.25;
	}
	else
	{
		switch(ctrlx[0] & 0x30)
		{
			case LSM303DLHC_FULLSCALE_2G:
				LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g; //1.0f
				break;
			case LSM303DLHC_FULLSCALE_4G:
				LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g; //0.5f
			case LSM303DLHC_FULLSCALE_8G:
				LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g; //0.25f
				break;
			case LSM303DLHC_FULLSCALE_16G:
				LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g; //0.0834f
				break;
		}
	}
	for(i=0; i<3; i++)
	{
		pfData[i]=(float)pnRawData[i]/LSM_Acc_Sensitivity;
	}
}

//********读取加速度数据************************************
void LAM303A_Raed(float *Data)
{
  static unsigned char buffer[6]={0}, ctrlx[2],i; 
	unsigned char cDivider;
  signed short  int pnRawData[3];
	float LSM_Acc_Sensitivity=LSM_Acc_Sensitivity_2g;
	int16_t cen;
	IICreadBytes(LSM303A_I2C_ADDR,LSM303A_OUT_X_L|0x80,6,buffer);
	IICreadBytes(LSM303A_I2C_ADDR,LSM303A_CTRL_REG4|0x80,2,ctrlx);
   
  if(ctrlx[1]&0x40)
    cDivider=64;
  else
    cDivider=16;

  /* check in the control register4 the data alignment*/
  if(!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) /* Little Endian Mode or FIFO mode */
  {
    for(i=0; i<3; i++)
    {
			cen=(int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i];
      pnRawData[i]=(int16_t)(cen)/cDivider;
     
    }
  }
  else /* Big Endian Mode */
  {
    for(i=0; i<3; i++)
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])/cDivider;
  }
  /* Read the register content */
  IICreadBytes(LSM303A_I2C_ADDR,LSM303A_CTRL_REG4,2,ctrlx);


  if(ctrlx[1]&0x40)
  {
    /* FIFO mode */
    LSM_Acc_Sensitivity = 0.25;
  }
  else
  {
    /* normal mode */
    /* switch the sensitivity value set in the CRTL4*/
    switch(ctrlx[0] & 0x30)
    {
    case LSM303DLHC_FULLSCALE_2G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
      break;
    case LSM303DLHC_FULLSCALE_4G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
      break;
    case LSM303DLHC_FULLSCALE_8G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
      break;
    case LSM303DLHC_FULLSCALE_16G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
      break;
    }
  }

  /* Obtain the mg value for the three axis */
  for(i=0; i<3; i++)
  {
    Data[i]=(float)pnRawData[i]*LSM_Acc_Sensitivity;
  }
}

/********************************************/
static float Data_conversion(float *AccBuffer,float *MagBuffer)
{
  unsigned char i;
	float HeadingValue = 0.0f;
	float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f;
  float fTiltedX,fTiltedY = 0.0f;
	float fcosf=0;
	      for(i=0;i<3;i++)
        AccBuffer[i] /= 100.0f;
      
      fNormAcc = sqrt((AccBuffer[0]*AccBuffer[0])+(AccBuffer[1]*AccBuffer[1])+(AccBuffer[2]*AccBuffer[2]));
      
      fSinRoll = AccBuffer[1]/fNormAcc;
      fCosRoll = sqrt(1.0-(fSinRoll * fSinRoll));
      fSinPitch = AccBuffer[0]/fNormAcc;
      fCosPitch = sqrt(1.0-(fSinPitch * fSinPitch));

      
      fTiltedX = MagBuffer[0]*fCosPitch + MagBuffer[2]*fSinPitch;
      fTiltedY = MagBuffer[0]*fSinRoll*fSinPitch + MagBuffer[1]*fCosRoll - MagBuffer[2]*fSinRoll*fCosPitch;
			
      fcosf=fTiltedX /sqrt(fTiltedX*fTiltedX+fTiltedY*fTiltedY);
	
			if(fTiltedY>0)
			  HeadingValue = (float)(acos(fcosf)*180/PI);
			else
				HeadingValue =360-(float)(acos(fcosf)*180/PI);
			
      //HeadingValue = (float) ((atan2f((float)fTiltedY,(float)fTiltedX))*180)/PI;
      HeadingValue+=11;//地磁的北极和地球的北极相差11度左右。
			if(HeadingValue>360)
				HeadingValue=HeadingValue-360;
   
	    return HeadingValue ;
	
	
}

LSM303_STRUCT lsm303;
void LSD_Sample(void)
{
float acc[3],hml[3];
	
  Demo_CompassReadMag(hml);
	Demo_CompassReadAcc(acc);
	lsm303.Acc.x=acc[0];
	lsm303.Acc.y=acc[1];
	lsm303.Acc.z=acc[2];
	lsm303.Mag_Adc.x=hml[0];
	lsm303.Mag_Adc.y=hml[1];
	lsm303.Mag_Adc.z=hml[2];
	
	lsm303.yaw=Data_conversion(acc,hml);

}