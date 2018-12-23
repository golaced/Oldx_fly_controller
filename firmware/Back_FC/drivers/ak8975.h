#ifndef _AK8975_H_
#define	_AK8975_H_

#include "stm32f4xx.h"
#include "stdbool.h"
#include "include.h"

#define HMC58X3_ADDR 0x3C // 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

#define HMC58X3_R_YM (7)  //!< Register address for YM.
#define HMC58X3_R_YL (8)  //!< Register address for YL.
#define HMC58X3_R_ZM (5)  //!< Register address for ZM.
#define HMC58X3_R_ZL (6)  //!< Register address for ZL.

#define HMC58X3_R_STATUS 9
#define HMC58X3_R_IDA 10
#define HMC58X3_R_IDB 11
#define HMC58X3_R_IDC 12

#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06
extern unsigned char HMC5883_calib;
extern int16_t  HMC5883_maxx,HMC5883_maxy,HMC5883_maxz,
		 HMC5883_minx,HMC5883_miny,HMC5883_minz;

void HMC5883L_SetUp(void);	//初始化
void HMC58X3_getID(char id[3]);	//读芯片ID
void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z); //读ADC
void HMC58X3_mgetValues(float *arry); //IMU 专用的读取磁力计值
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z);
void HMC5883L_Start_Calib(void);
void HMC5883L_Save_Calib(void);
void HMC_FIX_TEST(void);
void HMC_FIX(u8 state);
extern void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z) ;



#define CALIBRATING_MAG_CYCLES              20  //校准时间持续20s

typedef struct 
{ 
	xyz_s16_t Mag_Adc;			//采样值
	xyz_f_t Mag_Offset;		//偏移值
	xyz_f_t 	Mag_Gain;			//比例缩放	
  xyz_f_t 	Mag_Val;			//纠正后的值
	u8 Mag_CALIBRATED,Mag_CALIBRATE;
}ak8975_t;

extern ak8975_t ak8975,ak8975_fc;

void ANO_AK8975_CalOffset_Mag(void);
void ANO_AK8975_Read(void);


extern u8 Mag_CALIBRATED,Mag_CALIBRATED_R;
extern u8 ak8975_ok,HMC5883_calib;

#endif

