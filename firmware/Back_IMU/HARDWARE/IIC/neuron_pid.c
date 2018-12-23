#include "include.h"
#include "neuron_pid.h"

NEURON_PID_S neuron_pid_outter[4],neuron_pid_inner[4];

void NEURON_PID(NEURON_PID_S *neuron_pid,float ero,float rin ,float MAX,float T)
{ u8 i;float w[3];
  float temp,temp1;
	if(!neuron_pid->init){neuron_pid->init=1;
		neuron_pid->K=neuron_pid->K1=0.8;
		neuron_pid->alfa=neuron_pid->K1/50;
		neuron_pid->beta[0]=0.6;//p
		neuron_pid->beta[1]=0.6;//i
		neuron_pid->beta[2]=0.6;//d
		neuron_pid->w_k1[0]=0.001;
		neuron_pid->w_k1[1]=0.001;
		neuron_pid->w_k1[2]=0.001;
	}
	neuron_pid->beta[0]=neuron_pid->beta[1]=neuron_pid->beta[2]=SPID.YI*0.001;
	neuron_pid->error=ero; 
	
	neuron_pid->x[0]=neuron_pid->error;
	neuron_pid->x[1]=neuron_pid->error-neuron_pid->error1;
	neuron_pid->x[2]=neuron_pid->error-2*neuron_pid->error1+neuron_pid->error2;
	
	//neuron_pid->K=LIMIT(neuron_pid->K1+neuron_pid->alfa*pow(ero,3),0,5);//pow(LIMIT(rin,0.0001,999),2) 
	
	for(i=0;i<3;i++)
  neuron_pid->w_k[i]=neuron_pid->w_k1[i]+neuron_pid->beta[i]*neuron_pid->error*neuron_pid->u1*(2*neuron_pid->error-neuron_pid->error1);
  
 
	
	temp= fabs(neuron_pid->w_k[0])+ fabs(neuron_pid->w_k[1])+ fabs(neuron_pid->w_k[2]);
	if(temp==0)temp=0.000001;
	for(i=0;i<3;i++)
	w[i]=neuron_pid->w_k[i]/temp;
	for(i=0;i<3;i++)
	temp1+=neuron_pid->x[i]*fabs(w[i]);
	
	//neuron_pid->u=LIMIT(neuron_pid->u1+neuron_pid->K*(temp1) ,-MAX,MAX);
	neuron_pid->u=LIMIT(0+neuron_pid->K*(temp1) ,-MAX,MAX);
//save
	neuron_pid->error2=neuron_pid->error1; 
	neuron_pid->error1=neuron_pid->error; 
	
	neuron_pid->u1=neuron_pid->u; 
	neuron_pid->K1=neuron_pid->K;
	
	
	for(i=0;i<3;i++) 
	neuron_pid->w_k1[i]=neuron_pid->w_k[i];
}




float sign_1(float x)
{
  if(x>0)
      return(1);
  if(x<0)
      return(-1);
}


void NEURON_PID_LQ(NEURON_PID_S *neuron_pid,float ero,float rin ,float MAX,float T)
{ u8 i;float w[3];
  float temp,temp1,temp_2;
	static float intger;
	if(!neuron_pid->init){neuron_pid->init=1;
		neuron_pid->K=neuron_pid->K1=0.313;
		neuron_pid->alfa=neuron_pid->K1/50;
		neuron_pid->P=2;
		neuron_pid->Q=0.5;
		neuron_pid->b0=0.5;
		neuron_pid->V1=0.01;
		neuron_pid->L=neuron_pid->c=0.05;
		neuron_pid->Kmax=1.0;
		neuron_pid->Kmin=0.01;
		neuron_pid->beta[0]=15.1;//i
		neuron_pid->beta[1]=50.02;//p
		neuron_pid->beta[2]=20.02;//d
		neuron_pid->w_k1[0]=0.3;
		neuron_pid->w_k1[1]=0.3;
		neuron_pid->w_k1[2]=0.3;
	}
//	neuron_pid->beta[0]=neuron_pid->beta[1]=neuron_pid->beta[2]=SPID.YI*0.001;
	neuron_pid->error=ero; 
	
	neuron_pid->x[0]=neuron_pid->error;//i
	neuron_pid->x[1]=neuron_pid->error-neuron_pid->error1;//p
	neuron_pid->x[2]=neuron_pid->error-2*neuron_pid->error1+neuron_pid->error2;
	//------------------------------------------adpate K------------------------------------
	//neuron_pid->K=LIMIT(neuron_pid->K1+neuron_pid->alfa*pow(ero,3),0,2);//pow(LIMIT(rin,0.0001,999),2) 
//	if(sign_1(neuron_pid->error)==sign_1(neuron_pid->error1)&&(neuron_pid->K<neuron_pid->Kmax))
//	neuron_pid->K=neuron_pid->K1+neuron_pid->c*neuron_pid->K1/neuron_pid->V1;
//	else if(neuron_pid->K>neuron_pid->Kmin)
//	neuron_pid->K=0.75*neuron_pid->K1;	
//	
//	if(neuron_pid->K>neuron_pid->Kmin&&neuron_pid->K<neuron_pid->Kmax)
//	neuron_pid->V=neuron_pid->V1+neuron_pid->L*sign_1(fabs(neuron_pid->x[1])-neuron_pid->V1*fabs(neuron_pid->x[2]));
	
	//------------------------------------------adapte w------------------------------------
	for(i=0;i<3;i++)
	temp_2+=neuron_pid->w[i]*neuron_pid->x[i];
	
	//neuron_pid->w_k[0]=(neuron_pid->w_k1[0]+neuron_pid->beta[0]*neuron_pid->error*neuron_pid->u1*(2*neuron_pid->error-neuron_pid->error1));
	//if(fabs(neuron_pid->w_k[0]<10)neuron_pid->w_k[0]=25;
	for(i=0;i<3;i++)
  neuron_pid->w_k[i]=neuron_pid->w_k1[i]+
	neuron_pid->beta[i]*1*(neuron_pid->P*neuron_pid->b0*neuron_pid->error*neuron_pid->x[i]
	-neuron_pid->Q*1*temp_2*neuron_pid->x[i]);
  
	//let it to 1
	temp= fabs(neuron_pid->w_k[0])+fabs (neuron_pid->w_k[1])+fabs (neuron_pid->w_k[2]);
	if(temp==0)temp=0.000001;
	for(i=0;i<3;i++)
	neuron_pid->w[i]=neuron_pid->w_k[i]/temp;
	
//out put
  neuron_pid->integer+=ctrl_2.PID[PIDROLL].ki/4  *neuron_pid->error*T;
	Thr_Weight=0;
	/* 角度误差积分限幅 */
	neuron_pid->integer = LIMIT( neuron_pid->integer, -Thr_Weight *CTRL_2_INT_LIMIT,Thr_Weight *CTRL_2_INT_LIMIT );
  //neuron_pid->integer +=neuron_pid->K*neuron_pid->x[0]*fabs(neuron_pid->w[0]);
	//neuron_pid->integer =LIMIT(neuron_pid->integer, -Thr_Weight *CTRL_2_INT_LIMIT,Thr_Weight *CTRL_2_INT_LIMIT);
	//temp1=neuron_pid->K*neuron_pid->x[1]*fabs(neuron_pid->w[1]) //p
	//			+neuron_pid->integer 											//i
	//			+neuron_pid->K*neuron_pid->x[2]*fabs(neuron_pid->w[2]);//d

	for(i=0;i<3;i++)
	//temp1+=neuron_pid->x[i]*(w[i]);
	temp1+=neuron_pid->x[i]*fabs(neuron_pid->w[i]);	
	
	//neuron_pid->u=LIMIT(neuron_pid->u1+neuron_pid->K*(temp1) ,-MAX,MAX);
	neuron_pid->u=LIMIT(0+neuron_pid->K*(temp1)+neuron_pid->integer ,-MAX,MAX);
//save
	neuron_pid->error2=neuron_pid->error1; 
	neuron_pid->error1=neuron_pid->error; 
	
	neuron_pid->u1=neuron_pid->u; 
	neuron_pid->K1=neuron_pid->K;
	neuron_pid->V1=neuron_pid->V;
	
	for(i=0;i<3;i++) 
	neuron_pid->w_k1[i]=neuron_pid->w_k[i];
}





float AWDF_P(float in1,float in2)
{
float w[2];
float out;
u8 i,j;
float temp;
static float Variance[2];
static float Rr[2],R_1r[2];
static float Rr_bmp[2],R_1r_bmp[2];
Rr[0]=0.75*R_1r[0]+0.25*pow(in1,2);
R_1r[0]=Rr[0];

Rr[1]=0.75*R_1r[1]+0.25*in1*in2;
R_1r[1]=Rr[1];

Rr_bmp[0]=0.75*R_1r_bmp[0]+0.25*pow(in2,2);
R_1r_bmp[0]=Rr_bmp[0];

Rr_bmp[1]=0.75*R_1r_bmp[1]+0.25*in1*in2;
R_1r_bmp[1]=Rr_bmp[1];

Variance[0]=fabs(Rr[0]-Rr[1]);
Variance[1]=fabs(Rr_bmp[0]-Rr_bmp[1]);

w[0]=1/(Variance[0]*(1/Variance[0]+1/Variance[1]));
w[1]=1/(Variance[1]*(1/Variance[0]+1/Variance[1]));
temp=w[0]+w[1];
return out= w[1]/temp;//*fifo[0][4]+ w[2]/temp*fifo[1][4];
}

float AWDF_R(float in1,float in2)
{
float w[2];
float out;
u8 i,j;
float temp;
static float Variance[2];
static float Rr[2],R_1r[2];
static float Rr_bmp[2],R_1r_bmp[2];


Rr[0]=0.75*R_1r[0]+0.25*pow(in1,2);
R_1r[0]=Rr[0];

Rr[1]=0.75*R_1r[1]+0.25*in1*in2;
R_1r[1]=Rr[1];

Rr_bmp[0]=0.75*R_1r_bmp[0]+0.25*pow(in2,2);
R_1r_bmp[0]=Rr_bmp[0];

Rr_bmp[1]=0.75*R_1r_bmp[1]+0.25*in1*in2;
R_1r_bmp[1]=Rr_bmp[1];

Variance[0]=fabs(Rr[0]-Rr[1]);
Variance[1]=fabs(Rr_bmp[0]-Rr_bmp[1]);

w[0]=1/(Variance[0]*(1/Variance[0]+1/Variance[1]));
w[1]=1/(Variance[1]*(1/Variance[0]+1/Variance[1]));
temp=w[0]+w[1];
return out= w[1]/temp;//*fifo[0][4]+ w[2]/temp*fifo[1][4];
}


