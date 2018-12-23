#include "ukf_baro.h"
#include "ukf_oldx_baro.h"
#include "my_math.h"
#include "filter.h"
float q_baro_ukf=0.01;
float r_baro_ukf=0.6;///1;
double P_baro[9]={
	     1,0,0,
       0,1,0,
       0,0,1}; 
//double X_ukf_baro[6];
int acc_flag_baro=-1;
float flt_ukf_baro[2]={0.5,0.5};
void ukf_baro_task(float baro,float accz,float T)
{
static u8 init;
if(!init)
{
init=1;
ukf_oldx_baro_initialize();
}	
double A_baro[9]=
		{  1,0,0,
			 T,1,0,
			 T*T/2,T,1};
double H_baro[9]={
			 1,0,0,
       0,0,0,
       0,0,1};
double Q_baro[9]={
			 q_baro_ukf,0,0,
       0,q_baro_ukf,0,
       0,0,q_baro_ukf};
double R_baro[9]={
			 r_baro_ukf,0,0,
       0,r_baro_ukf,0,
       0,0,r_baro_ukf};	 
			 
double Z_baro[3]={baro,0,accz*acc_flag_baro};
			 
ukf_oldx_baro(A_baro, X_ukf_baro, P_baro, H_baro, Z_baro, Q_baro, R_baro);		 
X_ukf_baro[3]=Moving_Median(21,3,X_ukf_baro[0]);
X_ukf_baro[4]=Moving_Median(22,3,X_ukf_baro[1]);
}