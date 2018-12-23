

#include "hml_sample.h"
#include "parameter.h"
#include "my_math.h"
#include "include.h"
#include "iic_soft.h"
#include "filter.h"
#include "hml5833l.h"

ak8975_t ak8975 = { {0,0,0},{-1,-1,-1},{1,0.8538,0.9389},{0,0,0} };

bool ANO_AK8975_Run(void)
{
	return IIC_Write_1Byte(AK8975_ADDRESS,AK8975_CNTL,0x01);	
}

xyz_f_t XYZ_STRUCT_COPY(float x,float y, float z)
{
	xyz_f_t m ;
	m.x = x;
	m.y = y;
	m.z = z;
	return m;
}
#define  IIR_ORDER     4      //使用IIR滤波器的阶数
static double b_IIR_hml[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
static double a_IIR_hml[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
static double InPut_IIR_hml[3][IIR_ORDER+1] = {0};
static double OutPut_IIR_hml[3][IIR_ORDER+1] = {0};
u8 ak8975_ok;
void ANO_AK8975_Read_Mag_Data(void)
{
	int16_t mag_temp[3];
//u8 ak8975_buffer[6]; //接收数据缓存
	
	I2C_FastMode = 0;
	/*
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HXL,&ak8975_buffer[0]); 
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HXH,&ak8975_buffer[1]);
	mag_temp[1] = -((((int16_t)ak8975_buffer[1]) << 8) | ak8975_buffer[0]) ;  //磁力计X轴

	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HYL,&ak8975_buffer[2]);
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HYH,&ak8975_buffer[3]);
	mag_temp[0] = -((((int16_t)ak8975_buffer[3]) << 8) | ak8975_buffer[2]) ;  //磁力计Y轴

	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HZL,&ak8975_buffer[4]);
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HZH,&ak8975_buffer[5]);
	mag_temp[2] =  ((((int16_t)ak8975_buffer[5]) << 8) | ak8975_buffer[4]) ;  //磁力计Z轴	
	
	ak8975.Mag_Adc.x = mag_temp[0];
	ak8975.Mag_Adc.y = mag_temp[1];
	ak8975.Mag_Adc.z = mag_temp[2];
	*/
	 HMC58X3_getRaw(&mag_temp[0], &mag_temp[1],&mag_temp[2]);
	//ak8975.Mag_Adc.x = mag_temp[0];
	//ak8975.Mag_Adc.y = mag_temp[1];
	//ak8975.Mag_Adc.z = mag_temp[2];
	ak8975.Mag_Adc.x= IIR_I_Filter(mag_temp[0], InPut_IIR_hml[0], OutPut_IIR_hml[0], b_IIR_hml, IIR_ORDER+1, a_IIR_hml, IIR_ORDER+1);
  ak8975.Mag_Adc.y= IIR_I_Filter(mag_temp[1], InPut_IIR_hml[1], OutPut_IIR_hml[1], b_IIR_hml, IIR_ORDER+1, a_IIR_hml, IIR_ORDER+1);
	ak8975.Mag_Adc.z= IIR_I_Filter(mag_temp[2], InPut_IIR_hml[2], OutPut_IIR_hml[2], b_IIR_hml, IIR_ORDER+1, a_IIR_hml, IIR_ORDER+1);
	ak8975.Mag_Val.x = (ak8975.Mag_Adc.x - ak8975.Mag_Offset.x) ;
	ak8975.Mag_Val.y = (ak8975.Mag_Adc.y - ak8975.Mag_Offset.y)*ak8975.Mag_Gain.y ;
	ak8975.Mag_Val.z = (ak8975.Mag_Adc.z - ak8975.Mag_Offset.z)*ak8975.Mag_Gain.z ;
	//磁力计中点矫正	
	ANO_AK8975_CalOffset_Mag();
	
	//AK8975采样触发
//	ANO_AK8975_Run();
}

xyz_f_t ANO_AK8975_Get_Mag(void)
{
	return ak8975.Mag_Val;
}





u8 Mag_CALIBRATED = 0,Mag_CALIBRATED_R=0;;
//磁力计中点矫正

void ANO_AK8975_CalOffset_Mag(void)
{
	static xyz_f_t	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 }, MagSum;
	static uint16_t cnt_m=0;
	static u8 hml_cal_temp=0;
static u8 state_cal_hml;
	switch(state_cal_hml)
	{
		case 0:if(Mag_CALIBRATED_R!=hml_cal_temp)
		{				Mag_CALIBRATED=1;state_cal_hml=1;}break;
		case 1:if(Mag_CALIBRATED==0)
		{			hml_cal_temp=Mag_CALIBRATED_R;state_cal_hml=0;}break;
	}
	
	if(Mag_CALIBRATED)
	{	
		#if USE_CYCLE_HML_CAL
		if(ABS(ak8975.Mag_Adc.x)<500&&ABS(ak8975.Mag_Adc.y)<500&&ABS(ak8975.Mag_Adc.z)<500)
		    HMC_CAL_HML();
		
		#else
		if(ABS(ak8975.Mag_Adc.x)<500&&ABS(ak8975.Mag_Adc.y)<500&&ABS(ak8975.Mag_Adc.z)<500)
		{
			MagMAX.x = MAX(ak8975.Mag_Adc.x, MagMAX.x);
			MagMAX.y = MAX(ak8975.Mag_Adc.y, MagMAX.y);
			MagMAX.z = MAX(ak8975.Mag_Adc.z, MagMAX.z);
			
			MagMIN.x = MIN(ak8975.Mag_Adc.x, MagMIN.x);
			MagMIN.y = MIN(ak8975.Mag_Adc.y, MagMIN.y);
			MagMIN.z = MIN(ak8975.Mag_Adc.z, MagMIN.z);		
			
			if(cnt_m == CALIBRATING_MAG_CYCLES)
			{
				ak8975.Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
				ak8975.Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
				ak8975.Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);
	
				MagSum.x = MagMAX.x - MagMIN.x;
				MagSum.y = MagMAX.y - MagMIN.y;
				MagSum.z = MagMAX.z - MagMIN.z;
				
				ak8975.Mag_Gain.y = MagSum.x / MagSum.y;
				ak8975.Mag_Gain.z = MagSum.x / MagSum.z;
				
			WRITE_PARM();//Param_SaveMagOffset(&ak8975.Mag_Offset);//param_Save();//保存数据
				cnt_m = 0;
				Mag_CALIBRATED = 0;
			}
		}
		#endif
		cnt_m++;
		
	}
	else
	{

	}
}

void ANO_AK8975_Read(void)
{
		//读取磁力计
		ANO_AK8975_Read_Mag_Data();
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

