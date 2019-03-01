#ifndef __FILTER_H
#define __FILTER_H

#include "parameter.h"

#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))

#define LPF_1(A,B,C,D) LPF_1_((A),(B),(C),*(D));

typedef struct
{
	float a;
	float b;
	float e_nr;
	float out;
} _filter_1_st;

extern void DigitalLPF(float in, float* out, float cutoff_freq, float dt);
extern float DigitalLPF_NEW(float in, float out, float cutoff_freq, float dt);
void Moving_Average1(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out);
void Moving_Average(float moavarray[],u16 len ,u16 *fil_cnt,float in,float *out);
float Moving_Median(u8 item,u8 width_num,float in);
#define _xyz_f_t xyz_f_t
void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out);
double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
void rc_first_filter(float in,float *out,float fc,float dt);




#define NUMBER_OF_FIRST_ORDER_FILTERS 20
#define ACC_LOWPASS_X 0
#define ACC_LOWPASS_Y 1
#define ACC_LOWPASS_Z 2
#define BARO_LOWPASS   10
#define FLOW_LOWPASS_X   11
#define FLOW_LOWPASS_Y   12
typedef struct firstOrderFilterData {
  float   gx1;
  float   gx2;
  float   gx3;
  float   previousInput;
  float   previousOutput;
} firstOrderFilterData_t;

extern firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

void initFirstOrderFilter(float T);
float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters,float T);


typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
 const float a[3];
 const float b[3];
}Butter_Parameter;
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);
//-----Butterworth±‰¡ø-----//
extern Butter_Parameter Butter_80HZ_Parameter_Acce;
extern Butter_Parameter Butter_60HZ_Parameter_Acce;
extern Butter_Parameter Butter_50HZ_Parameter_Acce;
extern Butter_Parameter Butter_30HZ_Parameter_Acce;
extern Butter_Parameter Butter_20HZ_Parameter_Acce;
extern Butter_Parameter Butter_15HZ_Parameter_Acce;
extern Butter_Parameter Butter_10HZ_Parameter_Acce;
extern Butter_Parameter Butter_5HZ_Parameter_Acce;
extern Butter_Parameter Butter_2HZ_Parameter_Acce;

extern Butter_BufferData Butter_Buffer[3],Butter_Buffer_gps[3];
extern Butter_BufferData Butter_Buffer_Feedback[3];
#endif
