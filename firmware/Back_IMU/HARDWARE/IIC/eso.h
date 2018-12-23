#ifndef _ESO_H_
#define	_ESO_H_

#include "stm32f4xx.h"

typedef struct
{
  float beta0,beta1,beta2,beta3;
	float disturb,disturb_u;
	float alfa1,alfa2,alfa0,tao,KP,KD,KI,e;
	float z[3];
	float h,integer;
	u8 init,level;
	u8 n,out_mode,use_td;
  
	float v1,v2,h0,r0,b0,h1,r1,c,u;
}ESO;

extern ESO eso_att_outter[4],eso_att_inner[4];
extern ESO eso_att_outter_c[4],eso_att_inner_c[4];
float fst(float x1,float x2,float w,float h);
float fal(float e,float alfa,float delta);
float fst2(float x1,float x2,float w, float h);
float sign(float x);

extern float ESO_3N(ESO *eso_in,float v,float y,float u,float T,float MAX);
extern float ESO_2N(ESO *eso_in,float v,float y,float u,float T,float MAX);

extern float ATT_CONTRL_OUTER_ESO_3(ESO *eso_in,float v,float y,float u,float T,float MAX);
extern float ATT_CONTRL_INNER_ESO_3(ESO *eso_in,float v,float y,float u,float T,float MAX);
extern float HIGH_CONTROL_SPD_ESO(ESO *eso_in,float v,float y,float u,float T,float MAX);
#endif

