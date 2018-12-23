#ifndef _NEURON_PID_H_
#define	_NEURON_PID_H_

#include "stm32f4xx.h"

typedef struct
{
	float K,K1,Q,P,b0,c,V,V1,L,Kmax,Kmin;
	float w_k1[3],w_k[3],w[3],beta[3],alfa;
  float x[3];
	float error,error1,error2;
	float u,u1;
	float integer;
	u8 init;
	

}NEURON_PID_S;

extern NEURON_PID_S neuron_pid_outter[4],neuron_pid_inner[4];

extern void NEURON_PID(NEURON_PID_S *neuron_pid,float ero,float rin, float MAX,float T);
extern void NEURON_PID_LQ(NEURON_PID_S *neuron_pid,float ero,float rin ,float MAX,float T);
extern float AWDF_P(float in1,float in2);
extern float AWDF_R(float in1,float in2);
#endif

