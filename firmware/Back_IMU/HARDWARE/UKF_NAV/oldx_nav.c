#include "ukf_task.h"
#include "ukf_oldx.h"
#include "ukf_oldx_baro.h"
#include "ukf_nav.h"
#include "my_math.h"
#include "usart_fc.h"
float q_flow=0.01;
float r_flow=1;//0.6;///1;
 double P_flow[36]={
	     1,0,0,0,0,0,
       0,1,0,0,0,0,
       0,0,1,0,0,0,
       0,0,0,1,0,0,
       0,0,0,0,1,0,
       0,0,0,0,0,1};
 double X_ukf[6]={0,0,0,0,0,0};
float flow_gain=1;//2;
int acc_flag_flow[2]={1,1};
float X_ukf_Pos[2];


float q_baro_ukf=0.01;
float r_baro_ukf=0.6;///1;
 double P_baro[9]={
	     1,0,0,
       0,1,0,
       0,0,1}; 
 double X_ukf_baro[6]={0,0,0,0,0,0};
int acc_flag_baro=-1;
float flt_ukf_baro[2]={0.5,0.5};

const char flag_tst[2]={1,1};
double P_all[81]={
	     1,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,
       0,0,1,0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,
       0,0,0,0,1,0,0,0,0,
       0,0,0,0,0,1,0,0,0,
			 0,0,0,0,0,0,1,0,0,
       0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,0,0,1};
 double X_ukf_all[9]={0,0,0,0,0,0};
void oldx_nav(
float posx,float posy,float posz,
float spdx,float spdy,float spdz,	
float accx,float accy,float accz,
float T,char flag[2],u8 state)
{
static u8 init;

const double A_all[81]=
			 {1,          0,    0,    0,      0,    0,    0,        0,     0, 
				0.5,        1,    0,    0,      0,    0,    0,        0,     0, 
				0.5*0.5/2, 0.5,   1,    0,      0,    0,    0,        0,     0, 
				0,          0,    0,    1,      0,    0,    0,        0,     0, 
				0,          0,    0,    0.5,    1,    0,    0,        0,     0, 
				0,          0,    0,    0.5*0.5/2,0.5,1,    0,        0,     0, 
				0,          0,    0,     0,     0,    0,    1,        0,     0,
				0,          0,    0,     0,     0,    0,    0.5,      1,     0,
				0,          0,    0,     0,     0,    0,    0.5*0.5/2,0.5,    1 
				 };
const double H_all[81]={
			 0,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,
       0,0,1,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,
       0,0,0,0,1,0,0,0,0,
       0,0,0,0,0,1,0,0,0,
			 0,0,0,0,0,0,1,0,0,
       0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1};
const double Q_all[81]={
			 0.01,0,0,0,0,0,0,0,0,
       0,0.01,0,0,0,0,0,0,0,
       0,0,0.01,0,0,0,0,0,0,
       0,0,0,0.01,0,0,0,0,0,
       0,0,0,0,0.01,0,0,0,0,
       0,0,0,0,0,0.01,0,0,0,
			 0,0,0,0,0,0,0.01,0,0,
       0,0,0,0,0,0,0,0.01,0,
       0,0,0,0,0,0,0,0,0.01};
const double R_all[81]={
			 1,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,
       0,0,1,0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,
       0,0,0,0,1,0,0,0,0,
       0,0,0,0,0,1,0,0,0,
			 0,0,0,0,0,0,1,0,0,
       0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,0,0,1};

			 
 double Z_all[9]={0,spdx*flow_gain,accx*acc_flag_flow[0],0,spdy*flow_gain,-accy*acc_flag_flow[1],
 posz,0,accz*acc_flag_baro};	 




if(!init){init=1;
 ukf_nav_initialize();
}	
//if(flag_tst[0])
//;//ukf_oldx(A_flow, X_ukf, P_flow, H_flow, Z_flow, Q_flow, R_flow);	
//if(flag_tst[1])
//ukf_oldx_baro(A_baro, X_ukf_baro, P_baro, H_baro, Z_baro, Q_baro, R_baro);	
ukf_nav( A_all, X_ukf_all, P_all, H_all, Z_all, Q_all, R_all);

X_ukf_Pos[0]=-X_ukf[0];//X
X_ukf_Pos[1]=-X_ukf[3];//Y

}