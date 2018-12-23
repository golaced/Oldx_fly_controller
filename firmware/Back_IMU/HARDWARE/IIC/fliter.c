#include "include.h"
#include "filter.h"
#include "my_math.h"

void anotc_filter_1(float base_hz,float gain_hz,float dT,float in,_filter_1_st *f1)
{
	LPF_1_(gain_hz,dT,(in - f1->out),f1->a); //低通后的变化量

	f1->b = my_pow(in - f1->out);

	f1->e_nr = LIMIT(safe_div(my_pow(f1->a),((f1->b) + my_pow(f1->a)),0),0,1); //变化量的有效率
	
	LPF_1_(base_hz *f1->e_nr,dT,in,f1->out); //低通跟踪
}

float my_deathzoom1(float x,float ref,float zoom)//my_deadzone
{
	float t;
	if(x>ref)
	{
		t = x - zoom;
		if(t<ref)
		{
			t = ref;
		}
	}
	else
	{
		t = x + zoom;
		if(t>ref)
		{
			t = ref;
		}
	}
  return (t);
}

 void Moving_Average1(float moavarray[],u16 len ,u16 *fil_cnt,float in,float *out)
{
	u16 width_num;
	float last;

	width_num = len ;
	
	if( ++*fil_cnt >= width_num )	
	{
		*fil_cnt = 0; //now
	}
	
	last = moavarray[ *fil_cnt ];
	
	moavarray[ *fil_cnt ] = in;
	
	*out += ( in - ( last  ) )/(float)( width_num ) ;
	*out += 0.00001f *(in - *out);  //次要修正
	
}


 void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out)
{
	u16 width_num;
	
	width_num = len ;
	
	if( ++fil_cnt[0] > width_num )	
	{
		fil_cnt[0] = 0; //now
		fil_cnt[1] = 1; //old
	}
	else
	{
		fil_cnt[1] = (fil_cnt[0] == width_num)? 0 : (fil_cnt[0] + 1);
	}
	
	moavarray[ fil_cnt[0] ] = in;
	*out += ( in - ( moavarray[ fil_cnt[1] ]  ) )/(float)( width_num ) ;
	
}



#define MED_WIDTH_NUM 20
#define MED_FIL_ITEM  35

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
float med_filter_out[MED_FIL_ITEM];

u8 med_fil_cnt[MED_FIL_ITEM];
// 1  2  3                                9
float Moving_Median(u8 item,u8 width_num,float in)
{
	u8 i,j;
	float t;
	float tmp[MED_WIDTH_NUM];
	if(width_num==0)
		return in;
	if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
	{
		return 0;
	}
	else
	{
		if( ++med_fil_cnt[item] >= width_num )	
		{
			med_fil_cnt[item] = 0;
		}
		
		med_filter_tmp[item][ med_fil_cnt[item] ] = in;
		
		for(i=0;i<width_num;i++)
		{
			tmp[i] = med_filter_tmp[item][i];
		}
		
		for(i=0;i<width_num-1;i++)
		{
			for(j=0;j<(width_num-1-i);j++)
			{
				if(tmp[j] > tmp[j+1])
				{
					t = tmp[j];
					tmp[j] = tmp[j+1];
					tmp[j+1] = t;
				}
			}
		}

		
		return ( tmp[(u16)width_num/2] );
	}
}


/*====================================================================================================*/
/*====================================================================================================*
** 函数名称: IIR_I_Filter
** 功能描述: IIR直接I型滤波器
** 输    入: InData 为当前数据
**           *x     储存未滤波的数据
**           *y     储存滤波后的数据
**           *b     储存系数b
**           *a     储存系数a
**           nb     数组*b的长度
**           na     数组*a的长度
**           LpfFactor
** 输    出: OutData         
** 说    明: 无
** 函数原型: y(n) = b0*x(n) + b1*x(n-1) + b2*x
n-2) -
                    a1*y(n-1) - a2*y(n-2)
**====================================================================================================*/
/*====================================================================================================*/
double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na)
{
  double z1,z2;
  short i;
  double OutData;
  
  for(i=nb-1; i>0; i--)
  {
    x[i]=x[i-1];
  }
  
  x[0] = InData;
  
  for(z1=0,i=0; i<nb; i++)
  {
    z1 += x[i]*b[i];
  }
  
  for(i=na-1; i>0; i--)
  {
    y[i]=y[i-1];
  }
  
  for(z2=0,i=1; i<na; i++)
  {
    z2 += y[i]*a[i];
  }
  
  y[0] = z1 - z2; 
  OutData = y[0];
    
  return OutData;
}


/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
fp32 LPF_1st(fp32 oldData, fp32 newData, fp32 lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}


void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out) //小范围内正确。
{
	static s8 pn;
	static float h_tmp_x,h_tmp_y;
	
	h_tmp_x = my_sqrt(my_pow(ref->z) + my_pow(ref->y));
	h_tmp_y = my_sqrt(my_pow(ref->z) + my_pow(ref->x));
	
	pn = ref->z < 0? -1 : 1;
	
	  out->x = ( h_tmp_x *in->x - pn *ref->x *in->z ) ;
		out->y = ( pn *h_tmp_y *in->y - ref->y *in->z ) ;
	
// 	 out->x = h_tmp_x *in->x - ref->x *in->z;
// 	 out->y = ref->z *in->y - ref->y *in->z;
	
	out->z = ref->x *in->x + ref->y *in->y + ref->z *in->z ;
}


///////////////////////////////////////////////////////////////////////////////

// TAU = Filter Time Constant
// T   = Filter Sample Time

// A   = 2 * TAU / T

// Low Pass:
// GX1 = 1 / (1 + A)
// GX2 = 1 / (1 + A)
// GX3 = (1 - A) / (1 + A)

// High Pass:
// GX1 =  A / (1 + A)
// GX2 = -A / (1 + A)
// GX3 = (1 - A) / (1 + A)

///////////////////////////////////////larger tau, smoother filter

#define ACC_HIGHPASS_TAU         4//0.005f;
#define ACC_LOWPASS_TAU         0.1f
#define ACC_LOWPASS_SAMPLE_TIME 0.01f
#define ACC_LOWPASS_A          (2.0f * ACC_LOWPASS_TAU / ACC_LOWPASS_SAMPLE_TIME )
#define ACC_LOWPASS_GX1       (1.0f / (1.0f + ACC_LOWPASS_A))
#define ACC_LOWPASS_GX2       (1.0f / (1.0f + ACC_LOWPASS_A))
#define ACC_LOWPASS_GX3       ((1.0f - ACC_LOWPASS_A) / (1.0f + ACC_LOWPASS_A))
//---------------------------------------------------------------------------------
#define ACC_UKF_LOWPASS_TAU        0.05f
#define ACC_UKF_LOWPASS_SAMPLE_TIME 0.01f
#define ACC_UKF_LOWPASS_A          (2.0f * ACC_UKF_LOWPASS_TAU / ACC_UKF_LOWPASS_SAMPLE_TIME )
#define ACC_UKF_LOWPASS_GX1        (1.0f / (1.0f + ACC_UKF_LOWPASS_A))
#define ACC_UKF_LOWPASS_GX2        (1.0f / (1.0f + ACC_UKF_LOWPASS_A))
#define ACC_UKF_LOWPASS_GX3       ((1.0f - ACC_UKF_LOWPASS_A) / (1.0f + ACC_UKF_LOWPASS_A))
//---------------------------------------------------------------------------------
#if FLOW_USE_OPENMV
#define FLOW_LOWPASS_TAU        0.01//0.025f
#else
#define FLOW_LOWPASS_TAU        0.0051//0.025f
#endif
#define FLOW_LOWPASS_SAMPLE_TIME 0.01f
#define FLOW_LOWPASS_A          (2.0f * FLOW_LOWPASS_TAU / FLOW_LOWPASS_SAMPLE_TIME )
#define FLOW_LOWPASS_GX1        (1.0f / (1.0f + FLOW_LOWPASS_A))
#define FLOW_LOWPASS_GX2        (1.0f / (1.0f + FLOW_LOWPASS_A))
#define FLOW_LOWPASS_GX3       ((1.0f - FLOW_LOWPASS_A) / (1.0f + FLOW_LOWPASS_A))
//---------------------------------------------------------------------------------
#define HML_LOWPASS_TAU        0.025f
#define HML_LOWPASS_SAMPLE_TIME 0.02f
#define	HML_LOWPASS_A           2.0f * 0.05 * 500.0f
#define	HML_LOWPASS_GX1        (1.0f / (1.0f + HML_LOWPASS_A))
#define	HML_LOWPASS_GX2        (1.0f / (1.0f + HML_LOWPASS_A))
#define	HML_LOWPASS_GX3        ((1.0f - HML_LOWPASS_A) / (1.0f + HML_LOWPASS_A))

firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

void initFirstOrderFilter(float T)
{ 
//  ACC_LOWPASS_SAMPLE_TIME=T;
//	ACC_LOWPASS_A       =    (2.0f * ACC_LOWPASS_TAU / ACC_LOWPASS_SAMPLE_TIME );
//	ACC_LOWPASS_GX1    =     (1.0f / (1.0f + ACC_LOWPASS_A));
//	ACC_LOWPASS_GX2    =     (1.0f / (1.0f + ACC_LOWPASS_A));
//	ACC_LOWPASS_GX3     =    ((1.0f - ACC_LOWPASS_A) / (1.0f + ACC_LOWPASS_A));
////	ACC_LOWPASS_A       =    (2.0f * ACC_HIGHPASS_TAU / ACC_LOWPASS_SAMPLE_TIME);
////  ACC_LOWPASS_GX1     =    ( ACC_LOWPASS_A / (1.0f + ACC_LOWPASS_A));
////  ACC_LOWPASS_GX2     =   (-ACC_LOWPASS_A / (1.0f + ACC_LOWPASS_A));
////  ACC_LOWPASS_GX3     =    ((1.0f - ACC_LOWPASS_A) / (1.0f + ACC_LOWPASS_A));


  firstOrderFilters[ACC_LOWPASS_X].gx1 = ACC_LOWPASS_GX1;
	firstOrderFilters[ACC_LOWPASS_X].gx2 = ACC_LOWPASS_GX2;
	firstOrderFilters[ACC_LOWPASS_X].gx3 = ACC_LOWPASS_GX3;
	firstOrderFilters[ACC_LOWPASS_Y].gx1 = ACC_LOWPASS_GX1;
	firstOrderFilters[ACC_LOWPASS_Y].gx2 = ACC_LOWPASS_GX2;
	firstOrderFilters[ACC_LOWPASS_Y].gx3 = ACC_LOWPASS_GX3;
	firstOrderFilters[ACC_LOWPASS_Z].gx1 = ACC_LOWPASS_GX1;
	firstOrderFilters[ACC_LOWPASS_Z].gx2 = ACC_LOWPASS_GX2;
	firstOrderFilters[ACC_LOWPASS_Z].gx3 = ACC_LOWPASS_GX3;
	
//  ACC_UKF_LOWPASS_SAMPLE_TIME=T;
//	ACC_UKF_LOWPASS_A       =    (2.0f * ACC_UKF_LOWPASS_TAU / ACC_UKF_LOWPASS_SAMPLE_TIME );
//	ACC_UKF_LOWPASS_GX1    =     (1.0f / (1.0f + ACC_UKF_LOWPASS_A));
//	ACC_UKF_LOWPASS_GX2    =     (1.0f / (1.0f + ACC_UKF_LOWPASS_A));
//	ACC_UKF_LOWPASS_GX3     =    ((1.0f - ACC_UKF_LOWPASS_A) / (1.0f + ACC_UKF_LOWPASS_A));
  firstOrderFilters[FLOW_LOWPASS_X].gx1 = FLOW_LOWPASS_GX1;
	firstOrderFilters[FLOW_LOWPASS_X].gx2 = FLOW_LOWPASS_GX2;
	firstOrderFilters[FLOW_LOWPASS_X].gx3 = FLOW_LOWPASS_GX3;
	firstOrderFilters[FLOW_LOWPASS_Y].gx1 = FLOW_LOWPASS_GX1;
	firstOrderFilters[FLOW_LOWPASS_Y].gx2 = FLOW_LOWPASS_GX2;
	firstOrderFilters[FLOW_LOWPASS_Y].gx3 = FLOW_LOWPASS_GX3;

	

  firstOrderFilters[ACC_UKF_LOWPASS_X].gx1 = ACC_UKF_LOWPASS_GX1;
	firstOrderFilters[ACC_UKF_LOWPASS_X].gx2 = ACC_UKF_LOWPASS_GX2;
	firstOrderFilters[ACC_UKF_LOWPASS_X].gx3 = ACC_UKF_LOWPASS_GX3;
	firstOrderFilters[ACC_UKF_LOWPASS_Y].gx1 = ACC_UKF_LOWPASS_GX1;
	firstOrderFilters[ACC_UKF_LOWPASS_Y].gx2 = ACC_UKF_LOWPASS_GX2;
	firstOrderFilters[ACC_UKF_LOWPASS_Y].gx3 = ACC_UKF_LOWPASS_GX3;
	firstOrderFilters[ACC_UKF_LOWPASS_Z].gx1 = ACC_UKF_LOWPASS_GX1;
	firstOrderFilters[ACC_UKF_LOWPASS_Z].gx2 = ACC_UKF_LOWPASS_GX2;
	firstOrderFilters[ACC_UKF_LOWPASS_Z].gx3 = ACC_UKF_LOWPASS_GX3;
//	HML_LOWPASS_A       =     2.0f * 0.05 * 500.0f;//(2.0f * HML_LOWPASS_TAU / HML_LOWPASS_SAMPLE_TIME );
//	HML_LOWPASS_GX1    =     (1.0f / (1.0f + HML_LOWPASS_A));
//	HML_LOWPASS_GX2    =     (1.0f / (1.0f + HML_LOWPASS_A));
//	HML_LOWPASS_GX3     =    ((1.0f - HML_LOWPASS_A) / (1.0f + HML_LOWPASS_A));
  firstOrderFilters[HML_LOWPASS_X].gx1 = HML_LOWPASS_GX1;
	firstOrderFilters[HML_LOWPASS_X].gx2 = HML_LOWPASS_GX2;
	firstOrderFilters[HML_LOWPASS_X].gx3 = HML_LOWPASS_GX3;
	firstOrderFilters[HML_LOWPASS_Y].gx1 = HML_LOWPASS_GX1;
	firstOrderFilters[HML_LOWPASS_Y].gx2 = HML_LOWPASS_GX2;
	firstOrderFilters[HML_LOWPASS_Y].gx3 = HML_LOWPASS_GX3;
	firstOrderFilters[HML_LOWPASS_Z].gx1 = HML_LOWPASS_GX1;
	firstOrderFilters[HML_LOWPASS_Z].gx2 = HML_LOWPASS_GX2;
	firstOrderFilters[HML_LOWPASS_Z].gx3 = HML_LOWPASS_GX3;
	
	firstOrderFilters[HML_LOWPASS_Xo].gx1 = HML_LOWPASS_GX1;
	firstOrderFilters[HML_LOWPASS_Xo].gx2 = HML_LOWPASS_GX2;
	firstOrderFilters[HML_LOWPASS_Xo].gx3 = HML_LOWPASS_GX3;
	firstOrderFilters[HML_LOWPASS_Yo].gx1 = HML_LOWPASS_GX1;
	firstOrderFilters[HML_LOWPASS_Yo].gx2 = HML_LOWPASS_GX2;
	firstOrderFilters[HML_LOWPASS_Yo].gx3 = HML_LOWPASS_GX3;
	firstOrderFilters[HML_LOWPASS_Zo].gx1 = HML_LOWPASS_GX1;
	firstOrderFilters[HML_LOWPASS_Zo].gx2 = HML_LOWPASS_GX2;
	firstOrderFilters[HML_LOWPASS_Zo].gx3 = HML_LOWPASS_GX3;
}


float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters,float T)
{   static u8 init; 
    float output;
    if(!init){init=1;initFirstOrderFilter(T);}
		//initFirstOrderFilter(T);
    output = filterParameters->gx1 * input +
             filterParameters->gx2 * filterParameters->previousInput -
             filterParameters->gx3 * filterParameters->previousOutput;

    filterParameters->previousInput  = input;
		if(isnan(output))
		output=	filterParameters->previousOutput;
		
    filterParameters->previousOutput = output;

    return output;
}


void utilFilterReset(utilFilter_t *f, float setpoint) {
    f->z1 = setpoint;
}

void utilFilterReset3(utilFilter_t *f, float setpoint) {
    utilFilterReset(&f[0], setpoint);
    utilFilterReset(&f[1], setpoint);
    utilFilterReset(&f[2], setpoint);
}

// larger tau, smoother filter
void utilFilterInit(utilFilter_t *f, float dt, float tau, float setpoint) {
    f->tc = dt / tau;
    utilFilterReset(f, setpoint);
}

void utilFilterInit3(utilFilter_t *f, float dt, float tau, float setpoint) {
    utilFilterInit(&f[0], dt, tau, setpoint);
    utilFilterInit(&f[1], dt, tau, setpoint);
    utilFilterInit(&f[2], dt, tau, setpoint);
}

float utilFilter(utilFilter_t *f, float signal) {
    register float z1;

    z1 = f->z1 + (signal - f->z1) * f->tc;

    f->z1 = z1;

    return z1;
}






//-----Butterworth变量-----//
Butter_Parameter Butter_80HZ_Parameter_Acce={
  //200hz---80hz
1,     1.14298050254,   0.4128015980962,
0.638945525159,    1.277891050318,    0.638945525159
};

Butter_Parameter Butter_60HZ_Parameter_Acce={
  //200hz---60hz
1,   0.3695273773512,   0.1958157126558,
0.3913357725018,   0.7826715450035,   0.3913357725018
};

Butter_Parameter Butter_50HZ_Parameter_Acce={
  //200hz---50hz
1,-1.300707181133e-16,   0.1715728752538,
0.2065720838261,   0.4131441676523,   0.2065720838261,
};


Butter_Parameter Butter_30HZ_Parameter_Acce={
  //200hz---30hz
1,  -0.7477891782585,    0.272214937925,
0.1311064399166,   0.2622128798333,   0.1311064399166
};
Butter_Parameter Butter_20HZ_Parameter_Acce={
  //200hz---20hz
  1,    -1.14298050254,   0.4128015980962,
  0.06745527388907,   0.1349105477781,  0.06745527388907
};
Butter_Parameter Butter_15HZ_Parameter_Acce={
  //200hz---15hz
  1,   -1.348967745253,   0.5139818942197,
  0.04125353724172,  0.08250707448344,  0.04125353724172
};

Butter_Parameter Butter_10HZ_Parameter_Acce={
  //200hz---10hz
  1,   -1.561018075801,   0.6413515380576,
  0.02008336556421,  0.04016673112842,  0.02008336556421};
Butter_Parameter Butter_5HZ_Parameter_Acce={
  //200hz---5hz
  1,   -1.778631777825,   0.8008026466657,
  0.005542717210281,  0.01108543442056, 0.005542717210281
};

Butter_Parameter Butter_2HZ_Parameter_Acce={
  //200hz---2hz
  1,   -1.911197067426,   0.9149758348014,
  0.0009446918438402,  0.00188938368768,0.0009446918438402
};


Butter_BufferData Butter_Buffer[3],Butter_Buffer_gps[3];
Butter_BufferData Butter_Buffer_Feedback[3];
float Butter_parameter[2][3]={
  //200hz---21hz
        0.07319880848434,   0.1463976169687,  0.07319880848434,
        1,   -1.102497726129,   0.3952929600662
};

/*************************************************
函数名:	float LPButter_Acce5Hz(float curr_input)
说明:	加速度计低通滤波器
入口:	float curr_input	当前输入加速度计
出口:	无
备注:	2阶Butterworth低通滤波器
		Cutoff Fre. 5Hz
*************************************************/


float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
        /* 加速度计Butterworth滤波 */
	/* 获取最新x(n) */
        static int LPB_Cnt=0;
        Buffer->Input_Butter[2]=curr_input;
        if(LPB_Cnt>=100)
        {
	/* Butterworth滤波 */
        Buffer->Output_Butter[2]=
         Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
	+Parameter->b[2] * Buffer->Input_Butter[0]
        -Parameter->a[1] * Buffer->Output_Butter[1]
        -Parameter->a[2] * Buffer->Output_Butter[0];
        }
        else
        {
          Buffer->Output_Butter[2]=Buffer->Input_Butter[2];
          LPB_Cnt++;
        }
	/* x(n) 序列保存 */
        Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
        Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) 序列保存 */
        Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
        Buffer->Output_Butter[1]=Buffer->Output_Butter[2];


        return Buffer->Output_Butter[2];
}
