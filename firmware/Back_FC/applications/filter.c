#include "include.h"
#include "filter.h"
#include "mymath.h"

#define MED_WIDTH_NUM 11
#define MED_FIL_ITEM  5


void DigitalLPF(float in, float* out, float cutoff_freq, float dt) {
	  float input_reg=in;
    if (cutoff_freq <= 0.0f || dt <= 0.0f) {
        *out = input_reg;
    }
    float rc = 1.0f/(2*3.1415926*cutoff_freq);
    float alpha = LIMIT(dt/(dt+rc), 0.0f, 1.0f);
    *out += (input_reg - *out) * alpha;
}

float DigitalLPF_NEW(float in, float out, float cutoff_freq, float dt) {
	  float input_reg=in;
	  float out_reg=out;
    if (cutoff_freq <= 0.0f || dt <= 0.0f) {
        return input_reg;
    }
    float rc = 1.0f/(2*3.1415926*cutoff_freq);
    float alpha = LIMIT(dt/(dt+rc), 0.0f, 1.0f);
    out_reg += (input_reg - out) * alpha;
		return out_reg;	
}


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
	else if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
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


void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out) //小范围内正确。
{
	static s8 pn;
	static float h_tmp_x,h_tmp_y;
	
	h_tmp_x = my_sqrt(my_pow(ref->z) + my_pow(ref->y));
	h_tmp_y = my_sqrt(my_pow(ref->z) + my_pow(ref->x));
	
	pn = ref->z < 0? -1 : 1;
	
	  out->x = ( h_tmp_x *in->x - pn *ref->x *in->z ) ;
		out->y = ( pn *h_tmp_y *in->y - ref->y *in->z ) ;
	
	out->z = ref->x *in->x + ref->y *in->y + ref->z *in->z ;
}

void rc_first_filter(float in,float *out,float fc,float dt)
{ //fc 截止频率  Hz
  float temp=*out;
  *out=temp+(1/(1+1/(2*3.14*dt*fc)))*(in-temp);
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

///////////////////////////////////////

float ACC_HIGHPASS_TAU        = 4.0;
float ACC_LOWPASS_TAU        = 0.05;
float ACC_LOWPASS_SAMPLE_TIME =0.02f;
float ACC_LOWPASS_A        ;
float ACC_LOWPASS_GX1      ;
float ACC_LOWPASS_GX2      ;
float ACC_LOWPASS_GX3      ;

float BARO_LOWPASS_TAU        = 0.05f;
float BARO_LOWPASS_SAMPLE_TIME =0.02f;
float BARO_LOWPASS_A        ;
float BARO_LOWPASS_GX1      ;
float BARO_LOWPASS_GX2      ;
float BARO_LOWPASS_GX3      ;

float FLOW_LOWPASS_TAU        = 0.005f;
float FLOW_LOWPASS_SAMPLE_TIME =0.02f;
float FLOW_LOWPASS_A        ;
float FLOW_LOWPASS_GX1      ;
float FLOW_LOWPASS_GX2      ;
float FLOW_LOWPASS_GX3      ;
firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

void initFirstOrderFilter(float T)
{ 
	ACC_LOWPASS_SAMPLE_TIME= T;
	ACC_LOWPASS_A       =    (2.0f * ACC_LOWPASS_TAU / ACC_LOWPASS_SAMPLE_TIME );
	ACC_LOWPASS_GX1    =     (1.0f / (1.0f + ACC_LOWPASS_A));
	ACC_LOWPASS_GX2    =     ACC_LOWPASS_GX1;
	ACC_LOWPASS_GX3     =    ((1.0f - ACC_LOWPASS_A) / (1.0f + ACC_LOWPASS_A));
	
//	ACC_LOWPASS_A       =    (2.0f * ACC_HIGHPASS_TAU / ACC_LOWPASS_SAMPLE_TIME);
//  ACC_LOWPASS_GX1     =    ( ACC_LOWPASS_A / (1.0f + ACC_LOWPASS_A));
//  ACC_LOWPASS_GX2     =   (-ACC_LOWPASS_A / (1.0f + ACC_LOWPASS_A));
//  ACC_LOWPASS_GX3     =    ((1.0f - ACC_LOWPASS_A) / (1.0f + ACC_LOWPASS_A));
  firstOrderFilters[ACC_LOWPASS_X].gx1 = ACC_LOWPASS_GX1;
	firstOrderFilters[ACC_LOWPASS_X].gx2 = ACC_LOWPASS_GX2;
	firstOrderFilters[ACC_LOWPASS_X].gx3 = ACC_LOWPASS_GX3;
	firstOrderFilters[ACC_LOWPASS_Y].gx1 = ACC_LOWPASS_GX1;
	firstOrderFilters[ACC_LOWPASS_Y].gx2 = ACC_LOWPASS_GX2;
	firstOrderFilters[ACC_LOWPASS_Y].gx3 = ACC_LOWPASS_GX3;
	firstOrderFilters[ACC_LOWPASS_Z].gx1 = ACC_LOWPASS_GX1;
	firstOrderFilters[ACC_LOWPASS_Z].gx2 = ACC_LOWPASS_GX2;
	firstOrderFilters[ACC_LOWPASS_Z].gx3 = ACC_LOWPASS_GX3;
	
	BARO_LOWPASS_SAMPLE_TIME= T;
	BARO_LOWPASS_A       =    (2.0f * BARO_LOWPASS_TAU / BARO_LOWPASS_SAMPLE_TIME );
	BARO_LOWPASS_GX1    =     (1.0f / (1.0f + BARO_LOWPASS_A));
	BARO_LOWPASS_GX2    =     BARO_LOWPASS_GX1;
	BARO_LOWPASS_GX3     =    ((1.0f - BARO_LOWPASS_A) / (1.0f + BARO_LOWPASS_A));
	firstOrderFilters[BARO_LOWPASS].gx1 = BARO_LOWPASS_GX1;
	firstOrderFilters[BARO_LOWPASS].gx2 = BARO_LOWPASS_GX2;
	firstOrderFilters[BARO_LOWPASS].gx3 = BARO_LOWPASS_GX3;
	
	FLOW_LOWPASS_SAMPLE_TIME= T;
	FLOW_LOWPASS_A       =    (2.0f * FLOW_LOWPASS_TAU / FLOW_LOWPASS_SAMPLE_TIME );
	FLOW_LOWPASS_GX1    =     (1.0f / (1.0f + FLOW_LOWPASS_A));
	FLOW_LOWPASS_GX2    =     FLOW_LOWPASS_GX1;
	FLOW_LOWPASS_GX3     =    ((1.0f - FLOW_LOWPASS_A) / (1.0f + FLOW_LOWPASS_A));
	firstOrderFilters[FLOW_LOWPASS_X].gx1 = FLOW_LOWPASS_GX1;
	firstOrderFilters[FLOW_LOWPASS_X].gx2 = FLOW_LOWPASS_GX2;
	firstOrderFilters[FLOW_LOWPASS_X].gx3 = FLOW_LOWPASS_GX3;
	firstOrderFilters[FLOW_LOWPASS_Y].gx1 = FLOW_LOWPASS_GX1;
	firstOrderFilters[FLOW_LOWPASS_Y].gx2 = FLOW_LOWPASS_GX2;
	firstOrderFilters[FLOW_LOWPASS_Y].gx3 = FLOW_LOWPASS_GX3;
}


float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters,float T)
{   static u8 init; 
    float output;
    if(!init){init=1;initFirstOrderFilter(T);}
		initFirstOrderFilter(T);
    output = filterParameters->gx1 * input +
             filterParameters->gx2 * filterParameters->previousInput -
             filterParameters->gx3 * filterParameters->previousOutput;

    filterParameters->previousInput  = input;
    filterParameters->previousOutput = output;

    return output;
}






//-----Butterworth±äÁ¿-----//
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
º¯ÊýÃû:	float LPButter_Acce5Hz(float curr_input)
ËµÃ÷:	¼ÓËÙ¶È¼ÆµÍÍ¨ÂË²¨Æ÷
Èë¿Ú:	float curr_input	µ±Ç°ÊäÈë¼ÓËÙ¶È¼Æ
³ö¿Ú:	ÎÞ
±¸×¢:	2½×ButterworthµÍÍ¨ÂË²¨Æ÷
		Cutoff Fre. 5Hz
*************************************************/


float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
        /* ¼ÓËÙ¶È¼ÆButterworthÂË²¨ */
	/* »ñÈ¡×îÐÂx(n) */
        static int LPB_Cnt=0;
        Buffer->Input_Butter[2]=curr_input;
        if(LPB_Cnt>=100)
        {
	/* ButterworthÂË²¨ */
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
	/* x(n) ÐòÁÐ±£´æ */
        Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
        Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) ÐòÁÐ±£´æ */
        Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
        Buffer->Output_Butter[1]=Buffer->Output_Butter[2];


        return Buffer->Output_Butter[2];
}
