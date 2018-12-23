#include "ano_of.h"
#include "include.h"
ANOF ano_flow;
uint8_t		OF_QUA,OF_LIGHT;
int8_t		OF_DX,OF_DY;
int16_t		OF_DX2,OF_DY2,OF_DX2FIX,OF_DY2FIX;
uint16_t	OF_ALT,OF_ALT2;
int16_t		OF_GYR_X,OF_GYR_Y,OF_GYR_Z;
int16_t		OF_GYR_X2,OF_GYR_Y2,OF_GYR_Z2;
int16_t		OF_ACC_X,OF_ACC_Y,OF_ACC_Z;
int16_t		OF_ACC_X2,OF_ACC_Y2,OF_ACC_Z2;
float		OF_ATT_ROL,OF_ATT_PIT,OF_ATT_YAW;
float		OF_ATT_S1,OF_ATT_S2,OF_ATT_S3,OF_ATT_S4;
#if USE_ANO_FLOW
void AnoOF_DataAnl(uint8_t *data_buf,uint8_t num);
	
void AnoOF_GetOneByte(uint8_t data)
{
	static uint8_t _datatemp[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		_datatemp[0]=data;
	}
	else if(state==1&&data==0xAA)
	{
		state=2;
		_datatemp[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		_datatemp[2]=data;
	}
	else if(state==3&&data<45)
	{
		state = 4;
		_datatemp[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		_datatemp[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		_datatemp[4+_data_cnt]=data;
		AnoOF_DataAnl(_datatemp,_data_cnt+5);
	}
	else
		state = 0;
}

void AnoOF_DataAnl(uint8_t *data_buf,uint8_t num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		
	
	if(*(data_buf+2)==0X51)//光流信息
	{
		if(*(data_buf+4)==0)//原始光流信息
		{
			OF_QUA 		= *(data_buf+5);
			OF_DX  		= *(data_buf+6);
			OF_DY  		= *(data_buf+7);
			OF_LIGHT  	= *(data_buf+8);
		}
		else if(*(data_buf+4)==1)//融合后光流信息
		{
			OF_QUA 		= *(data_buf+5);
			OF_DX2		= (int16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
			OF_DY2		= (int16_t)(*(data_buf+8)<<8)|*(data_buf+9) ;
			OF_DX2FIX	= (int16_t)(*(data_buf+10)<<8)|*(data_buf+11) ;
			OF_DY2FIX	= (int16_t)(*(data_buf+12)<<8)|*(data_buf+13) ;
			OF_LIGHT  	= *(data_buf+14);
			
			switch(FLOW_SET_ANGLE){
				case 0:
			ano_flow.spdy=(float)OF_DX2FIX/100.;
			ano_flow.spdx=-(float)OF_DY2FIX/100.;
				break;
				case 4:
			ano_flow.spdx=(float)OF_DX2FIX/100.;
			ano_flow.spdy=(float)OF_DY2FIX/100.;	
				
				break;
			}
			ano_flow.qual=OF_QUA;
		}
	}
	if(*(data_buf+2)==0X52)//高度信息
	{
		if(*(data_buf+4)==0)//原始高度信息
		{
			OF_ALT = (uint16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
			ano_flow.o_h=(float)OF_ALT/100.;
		}
		else if(*(data_buf+4)==1)//融合后高度信息
		{
			OF_ALT2 = (uint16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
			ano_flow.h=(float)OF_ALT2/100.;
		}
	}
	if(*(data_buf+2)==0X53)//惯性数据
	{
		if(*(data_buf+4)==0)//原始数据
		{
			OF_GYR_X = (int16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
			OF_GYR_Y = (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) ;
			OF_GYR_Z = (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) ;
			OF_ACC_X = (int16_t)(*(data_buf+11)<<8)|*(data_buf+12) ;
			OF_ACC_Y = (int16_t)(*(data_buf+13)<<8)|*(data_buf+14) ;
			OF_ACC_Z = (int16_t)(*(data_buf+15)<<8)|*(data_buf+16) ;
		}
		else if(*(data_buf+4)==1)//滤波后数据
		{
			OF_GYR_X2 = (int16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
			OF_GYR_Y2 = (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) ;
			OF_GYR_Z2 = (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) ;
			OF_ACC_X2 = (int16_t)(*(data_buf+11)<<8)|*(data_buf+12) ;
			OF_ACC_Y2 = (int16_t)(*(data_buf+13)<<8)|*(data_buf+14) ;
			OF_ACC_Z2 = (int16_t)(*(data_buf+15)<<8)|*(data_buf+16) ;
		}
	}
	if(*(data_buf+2)==0X54)//姿态信息
	{
		if(*(data_buf+4)==0)//欧拉角格式
		{
			OF_ATT_ROL = ((int16_t)(*(data_buf+5)<<8)|*(data_buf+6)) * 0.01 ;
			OF_ATT_PIT = ((int16_t)(*(data_buf+7)<<8)|*(data_buf+8)) * 0.01 ;
			OF_ATT_YAW = ((int16_t)(*(data_buf+9)<<8)|*(data_buf+10)) * 0.01 ;
		}
		else if(*(data_buf+4)==1)//四元数格式
		{
			OF_ATT_S1 = ((int16_t)(*(data_buf+5)<<8)|*(data_buf+6)) * 0.0001 ;
			OF_ATT_S2 = ((int16_t)(*(data_buf+7)<<8)|*(data_buf+8)) * 0.0001 ;
			OF_ATT_S3 = ((int16_t)(*(data_buf+9)<<8)|*(data_buf+10)) * 0.0001 ;
			OF_ATT_S4 = ((int16_t)(*(data_buf+11)<<8)|*(data_buf+12)) * 0.0001 ;
		}
	}
}

#endif