#include "LIS3MDL.h"
#include "iic_soft.h"
#include "flash_w25.h"
#include "iic.h"
#include "iic_imu1.h"
#include "iic2.h"
#include "hml5833l.h"
#include "hml_sample.h"
#include "filter.h"
#include  <math.h> 
#include "cycle_cal_oldx.h"
#define FILTER_NUM 			10					//滑动平均滤波数值个数
u8 IMU1_Fast=0;
LIS3MDL_S lis3mdl;
Cal_Cycle_OLDX acc_lsq;
#define LIS3MDL_SA1_HIGH_ADDRESS  0x1E
#define LIS3MDL_SA1_LOW_ADDRESS   0x1C
#define LIS3MDL_ADDRESS1  (LIS3MDL_SA1_HIGH_ADDRESS << 1)
#define LIS3MDL_ADDRESS2  (LIS3MDL_SA1_LOW_ADDRESS << 1)
#define TEST_REG_ERROR -1

#define LIS3MDL_WHO_ID  0x3D

#define LIS3MDL_IIC_ID LIS3MDL_ADDRESS1

#define DS33_WHO_AM_I_ID     0x69  
#define DS33_SA0_HIGH_ADDRESS 0x6b
#define DS33_SA0_LOW_ADDRESS  0x6a
// Reads the 3 mag channels and stores them in vector m
#define DS33_ADDRESS1  (DS33_SA0_HIGH_ADDRESS << 1)
#define DS33_ADDRESS2  (DS33_SA0_LOW_ADDRESS << 1)
#define DS33_IIC_ID DS33_ADDRESS1


#define LPSSA0_HIGH_ADDRESS 0x5d<<	1
#define LPSSA0_LOW_ADDRESS  0x5c<<	1
#define LPS_IIC_ID LPSSA0_HIGH_ADDRESS
float baroAlt,baro_set,for_fly_ready;
u8 Lis3mdl_SPI_RD(u8 add,u8 sel)
{
 u8  tmp;
	SPI_CS(sel,0);
	SPI1_ReadWriteByte(add|0x80);
	tmp=SPI1_ReadWriteByte(0xff);  
	SPI_CS(sel,1);
 return tmp;
}
#define ST_SENSORS_SPI_READ			0x80
//写程序：
void Lis3mdl_SPI_WR(u8 add,u8 wrdata,u8 sel)
{
  SPI_CS(sel,0);
	if(sel!=CS_LIS)
	//SPI1_ReadWriteByte(add&0x7F );	
	SPI1_ReadWriteByte(add);	
	else
  SPI1_ReadWriteByte(add);
  SPI1_ReadWriteByte(wrdata);
  SPI_CS(sel,1);
}

void SPI_BufferRead(u8*buf, u8 add, u8 len,u8 sel)
{
	u8 i=0;
 SPI_CS(sel,0);
	if(sel!=CS_LIS)
	SPI1_ReadWriteByte(add|ST_SENSORS_SPI_READ);	
	else
	SPI1_ReadWriteByte(add|0xC0);//连续读增加地址
	for(i=0;i<len;i++)
	{
	if(sel!=CS_LIS)
	 *buf++ = SPI1_ReadWriteByte(0xff); 
	else
	 *buf++ = SPI1_ReadWriteByte(0xff); 
	} 
 SPI_CS(sel,1);
}



/* Write Data.
*/
void LSM6DS33_IO_Write(uint8_t* pBuffer,
                                               uint8_t  DeviceAddr,
                                               uint8_t  RegisterAddr,
                                               uint16_t NumByteToWrite
                                              )
{ uint16_t temp;

  SPI_CS(CS_DS33,1);

  //Send Device Address.
  SPI1_ReadWriteByte(RegisterAddr);

  for(temp = 0; temp < NumByteToWrite; temp ++)
  {
    SPI1_ReadWriteByte( *(pBuffer + temp));
  }

  SPI_CS(CS_DS33,0);


}

/* Read Data.
*/
void LSM6DS33_IO_Read(uint8_t* pBuffer,
                                              uint8_t  DeviceAddr,
                                              uint8_t  RegisterAddr,
                                              uint16_t NumByteToRead
                                             )
{ uint16_t temp;

  SPI_CS(CS_DS33,1);

  //Send Device Address.
  SPI1_ReadWriteByte((RegisterAddr | 0x80));

  for(temp = 0; temp < NumByteToRead; temp ++)
  {
    *(pBuffer + temp) = SPI1_ReadWriteByte( 0x00);
  }

  SPI_CS(CS_DS33,0);

}

uint8_t id[3] ;
void LIS3MDL_enableDefault(void)
{
 //------------init hml
   // 0x70 = 0b01110000
    // OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
	  #if USE_VER_3
	  Lis3mdl_SPI_WR(CTRL_REG1, 0x74,CS_LIS);
	  Lis3mdl_SPI_WR(CTRL_REG2, 0x60,CS_LIS);
	  Lis3mdl_SPI_WR(CTRL_REG3, 0x00,CS_LIS);
	  Lis3mdl_SPI_WR(CTRL_REG4, 0x0C,CS_LIS);
	  #else
    IIC_IMU1writeByte(LIS3MDL_IIC_ID,CTRL_REG1, 0x74);//20hz
 
    // 0x00 = 0b00000000
    // FS = 00 (+/- 4 gauss full scale)
    IIC_IMU1writeByte(LIS3MDL_IIC_ID,CTRL_REG2, 0x60);

    // 0x00 = 0b00000000
    // MD = 00 (continuous-conversion mode)
    IIC_IMU1writeByte(LIS3MDL_IIC_ID,CTRL_REG3, 0x00);

    // 0x0C = 0b00001100
    // OMZ = 11 (ultra-high-performance mode for Z)
    IIC_IMU1writeByte(LIS3MDL_IIC_ID,CTRL_REG4, 0x0C);
		#endif
    #if USE_VER_3
	  SPI_CS(CS_LIS,0);
	  SPI1_ReadWriteByte(0x80|0x0f);
	  u8 l_u8_ID= SPI1_ReadWriteByte(0xFF);
	  id[0] = l_u8_ID;
//		if(LIS3MDL_IIC_ID==l_u8_ID)
//			 module.hml =1; 
//		else
//			 module.hml= 0; 
			 SPI_CS(CS_LIS,1);
	  #else
    id[0] = I2C_IMU1_ReadOneByte(LIS3MDL_IIC_ID,WHO_AM_I);
		#endif
//---------------init acc & gro		
		#if USE_VER_3
		//Lis3mdl_SPI_WR(0x21,0x04,CS_DS33);
		//Delay_ms(10);
	  Lis3mdl_SPI_WR(CTRL1_XL, 0x4f,CS_DS33);
	  Lis3mdl_SPI_WR(CTRL2_G, 0x4c,CS_DS33);
	  Lis3mdl_SPI_WR(CTRL3_C, 0x04,CS_DS33);
		
		Delay_ms(10);
		LSM6_readGyro(0);
		#else
		// 0x80 = 0b10000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
    IIC_IMU1writeByte(DS33_IIC_ID,CTRL1_XL, 0x4f);//50hz 8g  0x88);

    // Gyro

    // 0x80 = 0b010000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 dps)
    IIC_IMU1writeByte(DS33_IIC_ID,CTRL2_G, 0x4c);//2000 50hz

    // Common

    // 0x04 = 0b00000100
    // IF_INC = 1 (automatically increment register address)
    IIC_IMU1writeByte(DS33_IIC_ID,CTRL3_C, 0x04);
		#endif
		#if USE_VER_3
	  SPI_CS(CS_DS33,0);
	  SPI1_ReadWriteByte(0x80|0x0f);
	  u8 l_u8_ID1= SPI1_ReadWriteByte(0xFF);
	  id[1] = l_u8_ID1;
//		if(DS33_WHO_AM_I_ID==l_u8_ID1)
//			 module.acc=module.gyro =1; 
//		else
//			 module.acc=module.gyro= 0; 
			 SPI_CS(CS_DS33,1);
	  #else
    id[1] = I2C_IMU1_ReadOneByte(DS33_IIC_ID,WHO_AM_I);
		#endif
		
//------------------------init bmp
		// 0xB0 = 0b10110000
    // PD = 1 (active mode);  ODR = 011 (12.5 Hz pressure & temperature output data rate)
		#if !USE_VER_3
		IIC_IMU1writeByte(LPS_IIC_ID,CTRL_REG1, 0xB4);//0xB4);
		id[2] = I2C_IMU1_ReadOneByte(LPS_IIC_ID,WHO_AM_I);
		#endif
}


void LIS3MDL_read(u8 fast)
{ u8 buffer[6];

  IMU1_Fast=fast;
	#if USE_VER_3
	SPI_BufferRead(buffer, OUT_X_L, 6, CS_LIS);

   lis3mdl.Mag_Adc.x = (buffer[1] << 8) | buffer[0];
   lis3mdl.Mag_Adc.y = (buffer[3] << 8) | buffer[2];
   lis3mdl.Mag_Adc.z = (buffer[5] << 8) | buffer[4];
  //*temperature = (buffer[7] << 8) | buffer[6];
	#else
  IIC_IMU1readBytes(LIS3MDL_IIC_ID, OUT_X_H, 1,buffer);
	IIC_IMU1readBytes(LIS3MDL_IIC_ID, OUT_X_L, 1,buffer+1);
	IIC_IMU1readBytes(LIS3MDL_IIC_ID, OUT_Y_H, 1,buffer+2);
	IIC_IMU1readBytes(LIS3MDL_IIC_ID, OUT_Y_L, 1,buffer+3);
	IIC_IMU1readBytes(LIS3MDL_IIC_ID, OUT_Z_H, 1,buffer+4);
	IIC_IMU1readBytes(LIS3MDL_IIC_ID, OUT_Z_L, 1,buffer+5);
	
  // combine high and low bytes
	#if USE_VER_5
	lis3mdl.Mag_Adc.x = -(int16_t)(buffer[0] << 8 | buffer[1]);
  lis3mdl.Mag_Adc.y = -(int16_t)(buffer[2] << 8 | buffer[3]);
  lis3mdl.Mag_Adc.z = (int16_t)(buffer[4] << 8 | buffer[5]);
	#else
  lis3mdl.Mag_Adc.x = (int16_t)(buffer[0] << 8 | buffer[1]);
  lis3mdl.Mag_Adc.y = (int16_t)(buffer[2] << 8 | buffer[3]);
  lis3mdl.Mag_Adc.z = (int16_t)(buffer[4] << 8 | buffer[5]);
	#endif
	#endif
	
//	#if USE_VER_4
//	lis3mdl.Mag_Adc.x /=10.;
//	lis3mdl.Mag_Adc.y /=10.;
//	lis3mdl.Mag_Adc.z /=10.;
//	#endif
}



// Reads the 3 accelerometer channels and stores them in vector a
void LSM6_readAcc(u8 fast)
{u8 buffer[6];
	IMU1_Fast=fast;
	#if USE_VER_3
//   uint8_t tempReg[2] = {0,0};
//    LSM6DS33_IO_Read(&tempReg[0], LSM6DS33_XG_MEMS_ADDRESS, LSM6DS33_XG_OUT_X_L_XL, 2);
//    lis3mdl.Acc_I16.x = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);
//    LSM6DS33_IO_Read(&tempReg[0], LSM6DS33_XG_MEMS_ADDRESS, LSM6DS33_XG_OUT_Y_L_XL, 2); 
//    lis3mdl.Acc_I16.y = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);
//    LSM6DS33_IO_Read(&tempReg[0], LSM6DS33_XG_MEMS_ADDRESS, LSM6DS33_XG_OUT_Z_L_XL, 2); 
//    lis3mdl.Acc_I16.z = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);
	SPI_BufferRead(buffer, OUTX_L_XL, 6, CS_DS33);

  lis3mdl.Acc_I16.x = (buffer[1] << 8) | buffer[0];
	lis3mdl.Acc_I16.y = (buffer[3] << 8) | buffer[2];
	lis3mdl.Acc_I16.z = (buffer[5] << 8) | buffer[4];
  //*temperature = (buffer[7] << 8) | buffer[6];
	#else
  IIC_IMU1readBytes(DS33_IIC_ID, OUTX_H_XL, 1, buffer);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTX_L_XL, 1,buffer+1);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTY_H_XL, 1,buffer+2);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTY_L_XL, 1,buffer+3);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTZ_H_XL, 1,buffer+4);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTZ_L_XL, 1,buffer+5);
  // combine high and low bytes
	#if USE_VER_5
	lis3mdl.Acc_I16.x = -(int16_t)(buffer[0] << 8 | buffer[1]);
  lis3mdl.Acc_I16.y = -(int16_t)(buffer[2] << 8 | buffer[3]);
  lis3mdl.Acc_I16.z = (int16_t)(buffer[4] << 8 | buffer[5]);
	#else
  lis3mdl.Acc_I16.x = (int16_t)(buffer[0] << 8 | buffer[1]);
  lis3mdl.Acc_I16.y = (int16_t)(buffer[2] << 8 | buffer[3]);
  lis3mdl.Acc_I16.z = (int16_t)(buffer[4] << 8 | buffer[5]);
	#endif
	#endif
}

// Reads the 3 gyro channels and stores them in vector g
void LSM6_readGyro(u8 fast)
{u8 buffer[6];
	IMU1_Fast=fast;
	#if USE_VER_3
	SPI_BufferRead(buffer, OUTX_L_G, 6, CS_DS33);

	lis3mdl.Gyro_I16.x = (buffer[1] << 8) | buffer[0];
	lis3mdl.Gyro_I16.y = (buffer[3] << 8) | buffer[2];
	lis3mdl.Gyro_I16.z = (buffer[5] << 8) | buffer[4];
  //*temperature = (buffer[7] << 8) | buffer[6];
	#else
  IIC_IMU1readBytes(DS33_IIC_ID, OUTX_H_G, 1,buffer);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTX_L_G, 1,buffer+1);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTY_H_G, 1,buffer+2);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTY_L_G, 1,buffer+3);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTZ_H_G, 1,buffer+4);
	IIC_IMU1readBytes(DS33_IIC_ID, OUTZ_L_G, 1,buffer+5);
  // combine high and low bytes
	#if USE_VER_5
	lis3mdl.Gyro_I16.x = -(int16_t)(buffer[0] << 8 | buffer[1]);
  lis3mdl.Gyro_I16.y = -(int16_t)(buffer[2] << 8 | buffer[3]);
  lis3mdl.Gyro_I16.z = (int16_t)(buffer[4] << 8 | buffer[5]);
	#else
  lis3mdl.Gyro_I16.x = (int16_t)(buffer[0] << 8 | buffer[1]);
  lis3mdl.Gyro_I16.y = (int16_t)(buffer[2] << 8 | buffer[3]);
  lis3mdl.Gyro_I16.z = (int16_t)(buffer[4] << 8 | buffer[5]);
	#endif
	#endif
}


// reads pressure and returns raw 24-bit sensor output
int32_t LPS_readPressureRaw(void)
{
  u8 pxl ;
  u8 pl ;
  u8 ph ;
  IIC_IMU1readBytes(LPS_IIC_ID, PRESS_OUT_XL, 1,&pxl);
	IIC_IMU1readBytes(LPS_IIC_ID, PRESS_OUT_L, 1,&pl);
	IIC_IMU1readBytes(LPS_IIC_ID, PRESS_OUT_H, 1,&ph);
  // combine bytes
  return (int32_t)(int8_t)ph << 16 | (uint16_t)pl << 8 | pxl;
}

// reads temperature and returns raw 16-bit sensor output
int16_t LPS_readTemperatureRaw(void)
{
  u8 tl ;
  u8 th ;
  IIC_IMU1readBytes(LPS_IIC_ID, TEMP_OUT_L_BMP, 1,&tl);
	IIC_IMU1readBytes(LPS_IIC_ID, TEMP_OUT_H_BMP, 1,&th);
  // combine bytes
  return (int16_t)(th << 8 | tl);
}
// reads temperature in degrees C
float LPS_readTemperatureC(void)
{
  return 42.5 + (float)LPS_readTemperatureRaw() / 480;
}

// reads temperature in degrees F
float LPS_readTemperatureF(void)
{
  return 108.5 + (float)LPS_readTemperatureRaw() / 480 * 1.8;
}


// converts pressure in mbar to altitude in meters, using 1976 US
// Standard Atmosphere model (note that this formula only applies to a
// height of 11 km, or about 36000 ft)
//  If altimeter setting (QNH, barometric pressure adjusted to sea
//  level) is given, this function returns an indicated altitude
//  compensated for actual regional pressure; otherwise, it returns
//  the pressure altitude above the standard pressure level of 1013.25
//  mbar or 29.9213 inHg
float LPS_pressureToAltitudeMeters(float pressure_mbar, float altimeter_setting_mbar)
{
  return (1 - pow(pressure_mbar / altimeter_setting_mbar, 0.190263)) * 44330.8;
}

// converts pressure in inHg to altitude in feet; see notes above
float LPS_pressureToAltitudeFeet(float pressure_inHg, float altimeter_setting_inHg)
{
  return (1 - pow(pressure_inHg / altimeter_setting_inHg, 0.190263)) * 145442;
}



// Reads the 3 gyro channels and stores them in vector g
void LP_readbmp(u8 fast)
{ static float pre_off;
	float pre_temp;
  static u8 init;
	static u8 cnt;
	IMU1_Fast=fast;
	if(cnt++>20){cnt=0;
	lis3mdl.Tem_bmp=LPS_readTemperatureC();}
	pre_temp=(float)LPS_readPressureRaw()/4096;
	if(!init&&pre_temp!=0)
	{pre_off=pre_temp;init=1;}
	else{
	lis3mdl.Pressure=pre_temp;
	lis3mdl.Alt=LPS_pressureToAltitudeMeters(pre_temp,pre_off);
	}
}
#define CALIBRATING_MAG_CYCLES              1000  //校准时间持续20s
int MAX_HML;
void LIS_CalOffset_Mag(void)
{ static xyz_f_t	Mag_Reg;
	static xyz_f_t	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 }, MagSum;
	static xyz_f_t	MagMAXo = { -100 , -100 , -100 }, MagMINo = { 100 , 100 , 100 }, MagSumo;
	static uint16_t cnt_m=0;
	static u8 hml_cal_temp=0;
  static u8 state_cal_hml;
	static u8 init;
	static u16 cnt1;
	if(lis3mdl.Mag_CALIBRATED)
	{	
		if(!init){init=1;
		#if USE_CYCLE_HML_CAL	
				cycle_init_oldx(&hml_lsq)	;
			if(module.outer_hml)
				cycle_init_oldx(&hml_lsq1)	;
		#endif
		MagMAX.x=MagMAX.y=MagMAX.z=-100;MagMIN.x=MagMIN.y=MagMIN.z=100;	
		MagMAXo.x=MagMAXo.y=MagMAXo.z=-100;MagMINo.x=MagMINo.y=MagMINo.z=100;				
		}
		
		float norm=sqrt(lis3mdl.Gyro_deg.x*lis3mdl.Gyro_deg.x+lis3mdl.Gyro_deg.y*lis3mdl.Gyro_deg.y+lis3mdl.Gyro_deg.z*lis3mdl.Gyro_deg.z);		
		if(norm<11)
			cnt1++;
		if(cnt1>888){cnt1=0;
			lis3mdl.Mag_CALIBRATED=0;}
		#if USE_VER_4
		MAX_HML=2500;	
		#else
		MAX_HML=1500;
		#endif	
		if(ABS(lis3mdl.Mag_Adc.x)<MAX_HML&&ABS(lis3mdl.Mag_Adc.y)<MAX_HML&&ABS(lis3mdl.Mag_Adc.z)<MAX_HML&&norm>11)
		{ cnt1=0;
			#if USE_CYCLE_HML_CAL
			if(hml_lsq.size<350&&(fabs(Mag_Reg.x-lis3mdl.Mag_Adc.x)>25||fabs(Mag_Reg.y-lis3mdl.Mag_Adc.y)>25||fabs(Mag_Reg.z-lis3mdl.Mag_Adc.z)>25)){
				cycle_data_add_oldx(&hml_lsq, (float)lis3mdl.Mag_Adc.x/1000.,(float)lis3mdl.Mag_Adc.y/1000.,(float)lis3mdl.Mag_Adc.z/1000.);
			if(module.outer_hml)
				cycle_data_add_oldx(&hml_lsq1, (float)mag_outer[0]/1000.,(float)mag_outer[1]/1000.,(float)mag_outer[2]/1000.);
		  }
			#endif
			MagMAX.x = MAX(lis3mdl.Mag_Adc.x, MagMAX.x);
			MagMAX.y = MAX(lis3mdl.Mag_Adc.y, MagMAX.y);
			MagMAX.z = MAX(lis3mdl.Mag_Adc.z, MagMAX.z);
			
			MagMIN.x = MIN(lis3mdl.Mag_Adc.x, MagMIN.x);
			MagMIN.y = MIN(lis3mdl.Mag_Adc.y, MagMIN.y);
			MagMIN.z = MIN(lis3mdl.Mag_Adc.z, MagMIN.z);		
			
			MagMAXo.x = MAX(mag_outer[0], MagMAXo.x);
			MagMAXo.y = MAX(mag_outer[1], MagMAXo.y);
			MagMAXo.z = MAX(mag_outer[2], MagMAXo.z);
			
			MagMINo.x = MIN(mag_outer[0], MagMINo.x);
			MagMINo.y = MIN(mag_outer[1], MagMINo.y);
			MagMINo.z = MIN(mag_outer[2], MagMINo.z);	
			
			if(cnt_m >= CALIBRATING_MAG_CYCLES*3)
			{ float sphere_x,sphere_y,sphere_z,sphere_r;
				init=0;
				#if USE_CYCLE_HML_CAL
				cycle_cal_oldx(&hml_lsq, 855,0.001,  &sphere_x, &sphere_y, &sphere_z, &sphere_r);
				if(fabs(sphere_r)>0){
				lis3mdl.Mag_Offset_c.x=(hml_lsq.Off[0]*1000);
				lis3mdl.Mag_Offset_c.y=(hml_lsq.Off[1]*1000);
				lis3mdl.Mag_Offset_c.z=(hml_lsq.Off[2]*1000);
				lis3mdl.Mag_Gain_c.x =  (hml_lsq.Gain[0]);
				lis3mdl.Mag_Gain_c.y =  (hml_lsq.Gain[1]);
				lis3mdl.Mag_Gain_c.z =  (hml_lsq.Gain[2]);	
				}
				cycle_cal_oldx(&hml_lsq1, 855,0.001,  &sphere_x, &sphere_y, &sphere_z, &sphere_r);
				if(fabs(sphere_r)>0){
				lis3mdl.Mag_Offset_co.x=(hml_lsq1.Off[0]*1000);
				lis3mdl.Mag_Offset_co.y=(hml_lsq1.Off[1]*1000);
				lis3mdl.Mag_Offset_co.z=(hml_lsq1.Off[2]*1000);
				lis3mdl.Mag_Gain_co.x =  (hml_lsq1.Gain[0]);
				lis3mdl.Mag_Gain_co.y =  (hml_lsq1.Gain[1]);
				lis3mdl.Mag_Gain_co.z =  (hml_lsq1.Gain[2]);	
				}
				#endif
				lis3mdl.Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
				lis3mdl.Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
				lis3mdl.Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);
	      lis3mdl.Mag_Offseto.x = (int16_t)((MagMAXo.x + MagMINo.x) * 0.5f);
				lis3mdl.Mag_Offseto.y = (int16_t)((MagMAXo.y + MagMINo.y) * 0.5f);
				lis3mdl.Mag_Offseto.z = (int16_t)((MagMAXo.z + MagMINo.z) * 0.5f);
				MagSum.x = MagMAX.x - MagMIN.x;
				MagSum.y = MagMAX.y - MagMIN.y;
				MagSum.z = MagMAX.z - MagMIN.z;
				MagSumo.x = MagMAXo.x - MagMINo.x;
				MagSumo.y = MagMAXo.y - MagMINo.y;
				MagSumo.z = MagMAXo.z - MagMINo.z;
				float temp_max=MagSum.x ;
				if( MagSum.y>temp_max)
					temp_max=MagSum.y;
			  if( MagSum.z>temp_max)
					temp_max=MagSum.z;
				
				lis3mdl.Mag_Gain.x =  temp_max/MagSum.x ;
				lis3mdl.Mag_Gain.y =  temp_max/MagSum.y ;
				lis3mdl.Mag_Gain.z =  temp_max/MagSum.z ;
				temp_max=MagSumo.x ;
				if( MagSumo.y>temp_max)
					temp_max=MagSumo.y;
			  if( MagSumo.z>temp_max)
					temp_max=MagSumo.z;
				
				lis3mdl.Mag_Gaino.x =  temp_max/MagSumo.x ;
				lis3mdl.Mag_Gaino.y =  temp_max/MagSumo.y ;
				lis3mdl.Mag_Gaino.z =  temp_max/MagSumo.z ;
				#if USE_CYCLE_HML_CAL
				if(lis3mdl.Mag_Gain_c.x>0&&lis3mdl.Mag_Gain_c.y>0&&lis3mdl.Mag_Gain_c.z>0
					&&lis3mdl.Mag_Gain_c.x<1.5&&lis3mdl.Mag_Gain_c.y<1.5&&lis3mdl.Mag_Gain_c.z<1.5&&
					fabs(lis3mdl.Mag_Offset_c.x)<1500&&fabs(lis3mdl.Mag_Offset_c.y)<1500&&fabs(lis3mdl.Mag_Offset_c.z)<1500){
				lis3mdl.Mag_Gain.x =  lis3mdl.Mag_Gain_c.x;
				lis3mdl.Mag_Gain.y =  lis3mdl.Mag_Gain_c.y;
				lis3mdl.Mag_Gain.z =  lis3mdl.Mag_Gain_c.z;
				lis3mdl.Mag_Offset.x =  lis3mdl.Mag_Offset_c.x;
				lis3mdl.Mag_Offset.y =  lis3mdl.Mag_Offset_c.y;
				lis3mdl.Mag_Offset.z =  lis3mdl.Mag_Offset_c.z;
					}
				if(lis3mdl.Mag_Gain_co.x>0&&lis3mdl.Mag_Gain_co.y>0&&lis3mdl.Mag_Gain_co.z>0
					&&lis3mdl.Mag_Gain_co.x<1.5&&lis3mdl.Mag_Gain_co.y<1.5&&lis3mdl.Mag_Gain_co.z<1.5&&
					fabs(lis3mdl.Mag_Offset_co.x)<1500&&fabs(lis3mdl.Mag_Offset_co.y)<1500&&fabs(lis3mdl.Mag_Offset_co.z)<1500){
				lis3mdl.Mag_Gaino.x =  lis3mdl.Mag_Gain_co.x;
				lis3mdl.Mag_Gaino.y =  lis3mdl.Mag_Gain_co.y;
				lis3mdl.Mag_Gaino.z =  lis3mdl.Mag_Gain_co.z;
				lis3mdl.Mag_Offseto.x =  lis3mdl.Mag_Offset_co.x;
				lis3mdl.Mag_Offseto.y =  lis3mdl.Mag_Offset_co.y;
				lis3mdl.Mag_Offseto.z =  lis3mdl.Mag_Offset_co.z;
				}
				#endif
			  WRITE_PARM();
				init_ahrs=0;
				cnt_m = 0;
				lis3mdl.Mag_CALIBRATED = 0;
			}
			Mag_Reg.x=lis3mdl.Mag_Adc.x;
			Mag_Reg.y=lis3mdl.Mag_Adc.y;
			Mag_Reg.z=lis3mdl.Mag_Adc.z;
			
		}

		cnt_m++;
		
	}
	else
	{

	}
}
#define OFFSET_AV_NUM_ACC 18
#define OFFSET_AV_NUM 50
s32 sum_temp[7]= {0,0,0,0,0,0,0};
float sum_temp_att[2]={0};
s32 sum_temp_3d[7]= {0,0,0,0,0,0,0};
u16 acc_sum_cnt = 0,acc_sum_cnt_3d=0,acc_smple_cnt_3d=0,gyro_sum_cnt = 0;
void LIS_Data_Offset(void)
{static u8 acc_cal_temp=0,gro_cal_temp=0;
static u8 state_cal_acc,state_cal_gro;
static u8 init;
	
	if(lis3mdl.Acc_CALIBRATE == 1)
	{
    acc_sum_cnt++;
		sum_temp[A_X] += lis3mdl.Acc_I16.x;
		sum_temp[A_Y] += lis3mdl.Acc_I16.y;
		sum_temp[A_Z] += lis3mdl.Acc_I16.z - 65536/16;   // +-8G
		sum_temp[TEM] += lis3mdl.Tempreature;

    if( acc_sum_cnt >= OFFSET_AV_NUM )
		{
			lis3mdl.Acc_Offset.x = sum_temp[A_X]/OFFSET_AV_NUM;
			lis3mdl.Acc_Offset.y = sum_temp[A_Y]/OFFSET_AV_NUM;
			lis3mdl.Acc_Offset.z = sum_temp[A_Z]/OFFSET_AV_NUM;
			lis3mdl.Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
			acc_sum_cnt =0;
			lis3mdl.Acc_CALIBRATE = 0;
			WRITE_PARM();
			sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;
		}	
	}

// 3d cal
		static xyz_f_t ACC_Reg;
		static u8 acc_3d_step_reg,acc_3d_step_imu;
		float sphere_x,sphere_y,sphere_z,sphere_r;
	
	  if(acc_3d_step==6)
		  acc_3d_step_imu=6;
    else if(acc_3d_step_imu!=6)
			acc_3d_step_imu=acc_3d_step;
		switch(acc_3d_step_imu)
			{ 
			case 0:
				acc_smple_cnt_3d=acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;
				cycle_init_oldx(&acc_lsq);
			break;
			default:
			if(acc_lsq.size<360&&(fabs(ACC_Reg.x-lis3mdl.Acc_I16.x)>0||fabs(ACC_Reg.y-lis3mdl.Acc_I16.y)>0||fabs(ACC_Reg.z-lis3mdl.Acc_I16.z)>0))
			{
			if(acc_3d_step_imu>acc_3d_step_reg)
			acc_smple_cnt_3d=acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;		
			acc_sum_cnt_3d++;
      sum_temp_3d[A_X] += lis3mdl.Acc_I16.x;
      sum_temp_3d[A_Y] += lis3mdl.Acc_I16.y;
      sum_temp_3d[A_Z] += lis3mdl.Acc_I16.z;   
				if(acc_sum_cnt_3d>OFFSET_AV_NUM_ACC){
					if(acc_smple_cnt_3d<12){
					acc_smple_cnt_3d++;	
					xyz_f_t data;	
					data.x = sum_temp_3d[A_X]/OFFSET_AV_NUM_ACC;
					data.y = sum_temp_3d[A_Y]/OFFSET_AV_NUM_ACC;
					data.z = sum_temp_3d[A_Z]/OFFSET_AV_NUM_ACC;	
					acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;	
					cycle_data_add_oldx(&acc_lsq, (float)data.x/1000.,(float)data.y/1000.,(float)data.z/1000.);}
					else if(acc_3d_step_imu==6){
					acc_3d_step_imu=0;	
					cycle_cal_oldx(&acc_lsq, 666,0.001,  &sphere_x, &sphere_y, &sphere_z, &sphere_r);	
					lis3mdl.Off_3d.x=(acc_lsq.Off[0]*1000);
					lis3mdl.Off_3d.y=(acc_lsq.Off[1]*1000);
					lis3mdl.Off_3d.z=(acc_lsq.Off[2]*1000);
					lis3mdl.Gain_3d.x =  (acc_lsq.Gain[0]);
					lis3mdl.Gain_3d.y =  (acc_lsq.Gain[1]);
					lis3mdl.Gain_3d.z =  (acc_lsq.Gain[2]);	
          WRITE_PARM();					
          init_hml_norm=1;
					if((fabs(lis3mdl.Off_3d.x)<400&&fabs(lis3mdl.Off_3d.y)<400&&fabs(lis3mdl.Off_3d.z)<400)||lis3mdl.Cali_3d==0)
					module.acc=2;
					else
					module.acc=1;

					if(fabs(lis3mdl.Gyro_Offset.x)<200&&fabs(lis3mdl.Gyro_Offset.y)<200&&fabs(lis3mdl.Gyro_Offset.z)<200)
					module.gyro=2;
					else
					module.gyro=1;	
					}		 		
				} 
					acc_3d_step_reg=acc_3d_step_imu;	
			}
			break;
		}
		ACC_Reg.x=lis3mdl.Acc_I16.x;
	  ACC_Reg.y=lis3mdl.Acc_I16.y;
		ACC_Reg.z=lis3mdl.Acc_I16.z;

//
	if(lis3mdl.Gyro_CALIBRATE)
	{
		gyro_sum_cnt++;
		sum_temp[G_X] += lis3mdl.Gyro_I16.x;
		sum_temp[G_Y] += lis3mdl.Gyro_I16.y;
		sum_temp[G_Z] += lis3mdl.Gyro_I16.z;
		sum_temp[TEM] += lis3mdl.Tempreature;

    if( gyro_sum_cnt >= OFFSET_AV_NUM )
		{
			lis3mdl.Gyro_Offset.x = (float)sum_temp[G_X]/OFFSET_AV_NUM;
			lis3mdl.Gyro_Offset.y = (float)sum_temp[G_Y]/OFFSET_AV_NUM;
			lis3mdl.Gyro_Offset.z = (float)sum_temp[G_Z]/OFFSET_AV_NUM;
			lis3mdl.Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
			gyro_sum_cnt =0;
			if(lis3mdl.Gyro_CALIBRATE == 1)
					WRITE_PARM();
			lis3mdl.Gyro_CALIBRATE = 0;
			sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
		}
	}
}


static void Transform(float itx,float ity,float itz,float *it_x,float *it_y,float *it_z)
{
	*it_x = itx;
	*it_y = ity;
	*it_z = itz;

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
			
      HeadingValue = (float) ((atan2f((float)fTiltedY,(float)fTiltedX))*180)/PI;
      HeadingValue+=11;//地磁的北极和地球的北极相差11度左右。
			if(HeadingValue>360)
				HeadingValue=HeadingValue-360;
   
	    return HeadingValue ;
}

//#define IIR_ACC_ODER_20HZ_4
#define IIR_ACC_ODER_20HZ_10
//#define IIR_ACC_ODER_50HZ_10
#if defined(IIR_ACC_ODER_20HZ_4)
#define  IIR_ORDER_ACC     4      //使用IIR滤波器的阶数
static double b_IIR[IIR_ORDER_ACC+1] ={ 0.004824  ,  0.0193  ,  0.02894  ,  0.01929 ,   0.0048};  //系数b
static double a_IIR[IIR_ORDER_ACC+1] ={ 1.0000   ,-2.3695   , 2.3139  , -1.0546  ,  0.1873};//系数a
#elif defined(IIR_ACC_ODER_20HZ_10)
#define  IIR_ORDER_ACC     10      //使用IIR滤波器的阶数
static double b_IIR[IIR_ORDER_ACC+1] ={ 0.0000016835  ,  0.000016835  ,  0.0000757611  ,  0.0002020 ,   0.00035355, 0.00042426, 0.0003535520 ,0.0002020,0.00007576,0.000016835,0.0000016835};  //系数b
static double a_IIR[IIR_ORDER_ACC+1] ={ 1.0000   ,-5.987589   , 16.67219 , -28.25878  ,  32.15975648, -25.601749, 14.405687, -5.6470743,1.473727,-0.230919345,0.016479};//系数a
#elif defined(IIR_ACC_ODER_50HZ_10)
#define  IIR_ORDER_ACC     10      //使用IIR滤波器的阶数
static double b_IIR[IIR_ORDER_ACC+1] ={
0.002896  , 
0.028964  ,  
0.130344  , 
0.347573 ,   
0.608253,
	0.7299,
0.608253,
0.34757,
0.13034,
0.02896,
0.00289644};  //系数b
static double a_IIR[IIR_ORDER_ACC+1] ={ 
1.0000   ,
-0.00000000000000070392394887475548 ,   
1.3403832676990335             ,    
-0.00000000000000071634263963324618  ,
0.54535390952381768              ,     
-0.0000000000000002072992030116616 ,   
0.077041166101195616           ,     
-0.00000000000000001851233148950247   ,
0.0031654815483433932      ,
-0.00000000000000000036075315368053437,	
0.000016778797726675533              
};//系数a

#endif
static double InPut_IIR[3][IIR_ORDER_ACC+1] = {0};
static double OutPut_IIR[3][IIR_ORDER_ACC+1] = {0};

#define IIR_ORDER_GRO 10
static double b_IIR_gro[IIR_ORDER_ACC+1] ={ 0.0000016835  ,  0.000016835  ,  0.0000757611  ,  0.0002020 ,   0.00035355, 0.00042426, 0.0003535520 ,0.0002020,0.00007576,0.000016835,0.0000016835};  //系数b
static double a_IIR_gro[IIR_ORDER_ACC+1] ={ 1.0000   ,-5.987589   , 16.67219 , -28.25878  ,  32.15975648, -25.601749, 14.405687, -5.6470743,1.473727,-0.230919345,0.016479};//系数a
static double InPut_IIR_gro[3][IIR_ORDER_GRO+1] = {0};
static double OutPut_IIR_gro[3][IIR_ORDER_GRO+1] = {0};

#define IIR_ORDER_MAG 10
static double b_IIR_mag[IIR_ORDER_MAG+1] ={
0.002896  , 
0.028964  ,  
0.130344  , 
0.347573 ,   
0.608253,
	0.7299,
0.608253,
0.34757,
0.13034,
0.02896,
0.00289644};  //系数b
static double a_IIR_mag[IIR_ORDER_MAG+1] ={ 
1.0000   ,
-0.00000000000000070392394887475548 ,   
1.3403832676990335             ,    
-0.00000000000000071634263963324618  ,
0.54535390952381768              ,     
-0.0000000000000002072992030116616 ,   
0.077041166101195616           ,     
-0.00000000000000001851233148950247   ,
0.0031654815483433932      ,
-0.00000000000000000036075315368053437,	
0.000016778797726675533              
};
static double InPut_IIR_mag[3][IIR_ORDER_MAG+1] = {0};
static double OutPut_IIR_mag[3][IIR_ORDER_MAG+1] = {0};

static float lis3mdl_tmp[7],mpu_fil_tmp[7];
static s16 FILT_BUF[7][(FILTER_NUM + 1)];
static uint8_t filter_cnt = 0,filter_cnt_old = 0;
utilFilter_t acc_tempFilter;
void LIS_Data_Prepare(float T)
{	static u8 init;
	u8 i;
	s32 FILT_TMP[7] = {0,0,0,0,0,0,0};
  float Gyro_tmp[3];
  LIS_Data_Offset();
	LIS_CalOffset_Mag();
	Gyro_tmp[0] = lis3mdl.Gyro_I16.x ;//
  Gyro_tmp[1] = lis3mdl.Gyro_I16.y ;//
	Gyro_tmp[2] = lis3mdl.Gyro_I16.z ;//
  if(!init)
	{init=1;
	utilFilterInit(&acc_tempFilter, 0.005, 5.0, 20);
	}

	lis3mdl.TEM_LPF += 2 *3.14f *T *(lis3mdl.Tempreature - lis3mdl.TEM_LPF);
	lis3mdl.Ftempreature = lis3mdl.TEM_LPF/340.0f + 36.5f;

//======================================================================
	if( ++filter_cnt > FILTER_NUM )	
	{
		filter_cnt = 0;
		filter_cnt_old = 1;
	}
	else
	{
		filter_cnt_old = (filter_cnt == FILTER_NUM)? 0 : (filter_cnt + 1);
	}
//10 170 4056
	/* 得出校准后的数据 */	
	if(lis3mdl.Cali_3d){
	lis3mdl_tmp[A_X] = (lis3mdl.Acc_I16.x - lis3mdl.Off_3d.x);//*lis3mdl.Gain_3d.x;// - lis3mdl.Acc_Offset.x*en_off_3d_off;
	lis3mdl_tmp[A_Y] = (lis3mdl.Acc_I16.y - lis3mdl.Off_3d.y);//*lis3mdl.Gain_3d.y;// - lis3mdl.Acc_Offset.y*en_off_3d_off;
	lis3mdl_tmp[A_Z] = (lis3mdl.Acc_I16.z - lis3mdl.Off_3d.z);//*lis3mdl.Gain_3d.z;// - lis3mdl.Acc_Offset.z*en_off_3d_off;
	}
   else{			
  lis3mdl_tmp[A_X] = (lis3mdl.Acc_I16.x - lis3mdl.Acc_Offset.x);
	lis3mdl_tmp[A_Y] = (lis3mdl.Acc_I16.y - lis3mdl.Acc_Offset.y);
	lis3mdl_tmp[A_Z] = (lis3mdl.Acc_I16.z - lis3mdl.Acc_Offset.z);
	 }
	lis3mdl_tmp[G_X] = (Gyro_tmp[0] - lis3mdl.Gyro_Offset.x );
	lis3mdl_tmp[G_Y] = (Gyro_tmp[1] - lis3mdl.Gyro_Offset.y );
	lis3mdl_tmp[G_Z] = (Gyro_tmp[2] - lis3mdl.Gyro_Offset.z );


	/* 更新滤波滑动窗口数组 */
	FILT_BUF[A_X][filter_cnt] = lis3mdl_tmp[A_X];
	FILT_BUF[A_Y][filter_cnt] = lis3mdl_tmp[A_Y];
	FILT_BUF[A_Z][filter_cnt] = lis3mdl_tmp[A_Z];
	FILT_BUF[G_X][filter_cnt] = lis3mdl_tmp[G_X]; 
	FILT_BUF[G_Y][filter_cnt] = lis3mdl_tmp[G_Y];
	FILT_BUF[G_Z][filter_cnt] = lis3mdl_tmp[G_Z];

	for(i=0;i<FILTER_NUM;i++)
	{
		FILT_TMP[A_X] += FILT_BUF[A_X][i];
		FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
		FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
		FILT_TMP[G_X] += FILT_BUF[G_X][i];
		FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
		FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
	}

mpu_fil_tmp[A_X] = (float)( FILT_TMP[A_X] )/(float)FILTER_NUM;
mpu_fil_tmp[A_Y] = (float)( FILT_TMP[A_Y] )/(float)FILTER_NUM;
mpu_fil_tmp[A_Z] = (float)( FILT_TMP[A_Z] )/(float)FILTER_NUM;
mpu_fil_tmp[G_X] = (float)( FILT_TMP[G_X] )/(float)FILTER_NUM;
mpu_fil_tmp[G_Y] = (float)( FILT_TMP[G_Y] )/(float)FILTER_NUM;
mpu_fil_tmp[G_Z] = (float)( FILT_TMP[G_Z] )/(float)FILTER_NUM;

	
float AccBuffer[3],MagBuffer[3];
	AccBuffer[0]=lis3mdl.Acc_I16.x;
	AccBuffer[1]=lis3mdl.Acc_I16.y;
	AccBuffer[2]=lis3mdl.Acc_I16.z;
	MagBuffer[0]=lis3mdl.Mag_Val.x;
	MagBuffer[1]=lis3mdl.Mag_Val.y;
	MagBuffer[2]=lis3mdl.Mag_Val.z;
	
	
	
	/*坐标转换*/
	Transform(mpu_fil_tmp[A_X],mpu_fil_tmp[A_Y],mpu_fil_tmp[A_Z],&lis3mdl.Acc.x,&lis3mdl.Acc.y,&lis3mdl.Acc.z);
	Transform(mpu_fil_tmp[G_X],mpu_fil_tmp[G_Y],mpu_fil_tmp[G_Z],&lis3mdl.Gyro.x,&lis3mdl.Gyro.y,&lis3mdl.Gyro.z);
#define EN_LF_GRO 1
#if EN_LF_GRO
	lis3mdl.Gyro_deg.x = IIR_I_Filter((lis3mdl.Gyro.x *TO_ANGLE) , InPut_IIR_gro[0], OutPut_IIR_gro[0], b_IIR_gro, IIR_ORDER_MAG+1, a_IIR_gro, IIR_ORDER_GRO+1);
	lis3mdl.Gyro_deg.y = IIR_I_Filter((lis3mdl.Gyro.y *TO_ANGLE) , InPut_IIR_gro[1], OutPut_IIR_gro[1], b_IIR_gro, IIR_ORDER_MAG+1, a_IIR_gro, IIR_ORDER_GRO+1);
	lis3mdl.Gyro_deg.z = IIR_I_Filter((lis3mdl.Gyro.z *TO_ANGLE) , InPut_IIR_gro[2], OutPut_IIR_gro[2], b_IIR_gro, IIR_ORDER_MAG+1, a_IIR_gro, IIR_ORDER_GRO+1);
#else	
	lis3mdl.Gyro_deg.x = lis3mdl.Gyro.x *TO_ANGLE;
	lis3mdl.Gyro_deg.y = lis3mdl.Gyro.y *TO_ANGLE;
	lis3mdl.Gyro_deg.z = lis3mdl.Gyro.z *TO_ANGLE;
#endif
#define EN_LF_ACC 1
#if EN_LF_ACC
	lis3mdl.Acc_t.x = IIR_I_Filter(lis3mdl.Acc.y, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER_ACC+1, a_IIR, IIR_ORDER_ACC+1);
	lis3mdl.Acc_t.y = IIR_I_Filter(-lis3mdl.Acc.x, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER_ACC+1, a_IIR, IIR_ORDER_ACC+1);
	lis3mdl.Acc_t.z = IIR_I_Filter(lis3mdl.Acc.z, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER_ACC+1, a_IIR, IIR_ORDER_ACC+1);

#else
  lis3mdl.Acc_t.x=lis3mdl.Acc.y;
	lis3mdl.Acc_t.y=-lis3mdl.Acc.x;
  lis3mdl.Acc_t.z=lis3mdl.Acc.z;
#endif		

	lis3mdl.Gyro_t.x=lis3mdl.Gyro.y;
	lis3mdl.Gyro_t.y=-lis3mdl.Gyro.x;
  lis3mdl.Gyro_t.z=lis3mdl.Gyro.z;
	
	lis3mdl.Gyro_deg_t.x = lis3mdl.Gyro_t.x *TO_ANGLE;
	lis3mdl.Gyro_deg_t.y = lis3mdl.Gyro_t.y *TO_ANGLE;
	lis3mdl.Gyro_deg_t.z = lis3mdl.Gyro_t.z *TO_ANGLE;

#define EN_LF_HML 0
#if EN_LF_HML
	lis3mdl.Mag_Val.x = IIR_I_Filter((lis3mdl.Mag_Adc.x - lis3mdl.Mag_Offset.x) , InPut_IIR_mag[0], OutPut_IIR_mag[0], b_IIR_mag, IIR_ORDER_MAG+1, a_IIR_mag, IIR_ORDER_MAG+1);
	lis3mdl.Mag_Val.y = IIR_I_Filter((lis3mdl.Mag_Adc.y - lis3mdl.Mag_Offset.y) , InPut_IIR_mag[1], OutPut_IIR_mag[1], b_IIR_mag, IIR_ORDER_MAG+1, a_IIR_mag, IIR_ORDER_MAG+1);
	lis3mdl.Mag_Val.z = IIR_I_Filter((lis3mdl.Mag_Adc.z - lis3mdl.Mag_Offset.z) , InPut_IIR_mag[2], OutPut_IIR_mag[2], b_IIR_mag, IIR_ORDER_MAG+1, a_IIR_mag, IIR_ORDER_MAG+1);
	#else
  lis3mdl.Mag_Val.x = firstOrderFilter((lis3mdl.Mag_Adc.x - lis3mdl.Mag_Offset.x),&firstOrderFilters[HML_LOWPASS_X],T);//*lis3mdl.Mag_Gain.x ;
	lis3mdl.Mag_Val.y = firstOrderFilter((lis3mdl.Mag_Adc.y - lis3mdl.Mag_Offset.y),&firstOrderFilters[HML_LOWPASS_Y],T);//*lis3mdl.Mag_Gain.y ;
	lis3mdl.Mag_Val.z = firstOrderFilter((lis3mdl.Mag_Adc.z - lis3mdl.Mag_Offset.z),&firstOrderFilters[HML_LOWPASS_Z],T);//*lis3mdl.Mag_Gain.z ;
#endif
	
	lis3mdl.Mag_Val_t.x=lis3mdl.Mag_Val.y;
	lis3mdl.Mag_Val_t.y=-lis3mdl.Mag_Val.x;
	lis3mdl.Mag_Val_t.z=lis3mdl.Mag_Val.z;
	
	lis3mdl.Mag_Valo.x = firstOrderFilter((mag_outer[0] - lis3mdl.Mag_Offseto.x),&firstOrderFilters[HML_LOWPASS_Xo],T)*lis3mdl.Mag_Gaino.x ;
	lis3mdl.Mag_Valo.y = firstOrderFilter((mag_outer[1] - lis3mdl.Mag_Offseto.y),&firstOrderFilters[HML_LOWPASS_Yo],T)*lis3mdl.Mag_Gaino.y ;
	lis3mdl.Mag_Valo.z = firstOrderFilter((mag_outer[2] - lis3mdl.Mag_Offseto.z),&firstOrderFilters[HML_LOWPASS_Zo],T)*lis3mdl.Mag_Gaino.z ;

	lis3mdl.Mag_Val_to.x=-lis3mdl.Mag_Valo.y;
	lis3mdl.Mag_Val_to.y=-lis3mdl.Mag_Valo.x;
	lis3mdl.Mag_Val_to.z=-lis3mdl.Mag_Valo.z;
	if(module.outer_hml)
	{
		lis3mdl.Mag_Val_t.x=lis3mdl.Mag_Val_to.x;
		lis3mdl.Mag_Val_t.y=lis3mdl.Mag_Val_to.y;
		lis3mdl.Mag_Val_t.z=lis3mdl.Mag_Val_to.z;
	}
	static u8 cnt;
	if(cnt++>125){cnt=0;
	lis3mdl.hmlOneMAG = sqrt(lis3mdl.Mag_Val_t.x*(lis3mdl.Mag_Val_t.x) + lis3mdl.Mag_Val_t.y*(lis3mdl.Mag_Val_t.y) + lis3mdl.Mag_Val_t.z*lis3mdl.Mag_Val_t.z)*1.1;
		if(lis3mdl.hmlOneMAG>440)
			module.hml=2;
		else 
			module.hml=1;
	}
		static float yaw_reg;
		if(ABS(To_180_degrees(Yaw-yaw_reg)/T-lis3mdl.Gyro_deg_t.z)>120)
    module.hml=0;
		yaw_reg=Yaw;
	
	
	lis3mdl.yaw=Data_conversion(AccBuffer,MagBuffer);
	static u8 state[3];
	switch (state[0]){
		case 0:
		if(imu_fushion.Acc_CALIBRATE)
	{lis3mdl.Acc_CALIBRATE=mpu6050.Acc_CALIBRATE=1;
	 state[0]=1;}
	  break;
		case 1:
			if(lis3mdl.Acc_CALIBRATE==0&&mpu6050.Acc_CALIBRATE==0)
	    { state[0]=0; imu_fushion.Acc_CALIBRATE=0;}
			break;
  }
	switch (state[1]){
		case 0:
		if(imu_fushion.Gyro_CALIBRATE)
	{lis3mdl.Gyro_CALIBRATE=mpu6050.Gyro_CALIBRATE=1;
	 state[1]=1;}
	  break;
		case 1:
			if(lis3mdl.Gyro_CALIBRATE==0&&mpu6050.Gyro_CALIBRATE==0)
	    { state[1]=0; imu_fushion.Gyro_CALIBRATE=0;}
			break;
  }
	switch (state[2]){
		case 0:
		if(imu_fushion.Mag_CALIBRATED)
	{lis3mdl.Mag_CALIBRATED=ak8975.Mag_CALIBRATED=1;
	 state[2]=1;}
	  break;
		case 1:
			if(lis3mdl.Mag_CALIBRATED==0&&ak8975.Mag_CALIBRATED==0)
	    { state[2]=0; imu_fushion.Mag_CALIBRATED=0;}
			break;
  }
}
