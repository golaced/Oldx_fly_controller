#include "eso.h"
#define LIMIT_ESO( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

ESO eso_att_outter_c[4],eso_att_inner_c[4];
ESO eso_pos_spd[3];
ESO eso_pos[3];

static float my_deathzoom1(float x,float zoom)
{
	float t;
	if(x>0)
	{
		t = x - zoom;
		if(t<0)
		{
			t = 0;
		}
	}
	else
	{
		t = x + zoom;
		if(t>0)
		{
			t = 0;
		}
	}
  return (t);
}

static float my_sqrt(float number)
{
	long i;
	float x, y;
	const float f = 1.5F;
	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );

	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) );
	y = y * ( f - ( x * y * y ) );
	return number * y;
} 

static float sign(float x)
{
  if(x>0)
      return(1);
  if(x<0)
      return(-1);
}

static float fst2(float x1,float x2,float w, float h)
{
	float td_y=0;
	float a0=0,a1,a2;
	float a=0;
	float fhan=0;
	float d=0;
	float d0=0;//dead
	float sy,sa;
	
	d=w*h*h;
	a0=h*x2;
	td_y=x1+a0;
	a1=my_sqrt(d*(d+8*fabs(td_y)));
	a2=a0+sign(td_y)*(a1-d)/2;
	sy=(sign(td_y+d)-sign(td_y-d))/2;
	a=(a0+td_y-a2)*sy+a2;
	sa=(sign(a+d)-sign(a-d))/2;
	fhan=-w*(a/d-sign(a))*sa-w*sign(a);
	return(fhan);
}

static float fal(float e,float alfa,float delta)
{
	float y=0.0;
	if(fabs(e)>delta) y=pow(fabs(e),alfa)*sign(e);
	else			  y=e/(pow(delta,1.0-alfa)+0.000001);
	return(y);	
}


static void ESO_Safe(ESO *eso_in,u16 thr)
{ static float Thr_tmp;
	
	Thr_tmp += 10 *3.14f *eso_in->h0 *(thr/400.0f - Thr_tmp); //????
	eso_in->Thr_Weight = LIMIT_ESO(Thr_tmp*1.75,0,1);    							//??????????????
	
	if( thr < 100 )
	{
		eso_in->Thr_Low = 1;
	}
	else
	{
		eso_in->Thr_Low = 0;
	}
}

void OLDX_SMOOTH_IN_ESO(ESO *eso_in,float in)
{
eso_in->v1+=eso_in->h0*eso_in->v2;                        //td_x1=v1;
eso_in->v2+=eso_in->h0*fst2(eso_in->v1-in,eso_in->v2,eso_in->r0,eso_in->h0);           //td_x2=v2;
}

//------------------------------ESO--------------------------------
float ESO_2N(ESO *eso_in,float v,float y,float u,float T,float MAX,float ero_px4)             // v是控制系统的输入，y是控制系统的输出，反馈给ESO，u是ADRC的输出控制量
{
	float e=0,fe,fe1;
	eso_in->h=eso_in->h0;
//********  eso  *************
if(eso_in->Thr_Low||eso_in->auto_b0==0)
	eso_in->b01=eso_in->b0;
	e=eso_in->e=my_deathzoom1(eso_in->z[0]-y,eso_in->eso_dead);
	fe=fal(e,0.5,eso_in->h0);
	fe1=fal(e,0.25,eso_in->h0);
	eso_in->z[0]+=eso_in->h0*(eso_in->z[1]-eso_in->beta0*e+eso_in->b01*eso_in->Thr_Weight *u);
	eso_in->z[1]+=-eso_in->h0*eso_in->beta1*e;
	eso_in->z[1]=LIMIT_ESO(eso_in->z[1],-MAX*eso_in->Thr_Weight*(eso_in->b01+1),MAX*eso_in->Thr_Weight*(eso_in->b01+1));
	if(eso_in->n==0)
		eso_in->n=1;
	return eso_in->disturb=LIMIT_ESO(eso_in->z[1]/eso_in->n,-MAX,MAX);
}

float ESO_3N(ESO *eso_in,float v,float y,float u,float T,float MAX)             // v是控制系统的输入，y是控制系统的输出，反馈给ESO，u是ADRC的输出控制量
{
	float e=0,fe,fe1;
	eso_in->h=eso_in->h0;
//********  eso  *************
	e=eso_in->e=my_deathzoom1(eso_in->z[0]-y,eso_in->eso_dead);
	fe=fal(e,0.5,eso_in->h0);
	fe1=fal(e,0.25,eso_in->h0);
	eso_in->z[0]+=eso_in->h*(eso_in->z[1]-eso_in->beta0*e);
	eso_in->z[1]+=eso_in->h*(eso_in->z[2]-eso_in->beta1*fe+eso_in->b01*eso_in->Thr_Weight *u);
	eso_in->z[2]+=-eso_in->h*eso_in->beta2*fe1;
		if(eso_in->n==0)
		eso_in->n=1;
	return eso_in->disturb=LIMIT_ESO(eso_in->z[2]/eso_in->n,-MAX,MAX);
}  

float flt_eso=0.5;
float ESO_CONTROL(ESO *eso_in,float v,float y,float u,float T,float MAX,float ero_px4)
{static float e0,e1,e2;
if(eso_in->not_use_px4)
{
e1=my_deathzoom1(v-eso_in->z[0],eso_in->eso_dead);
e2=eso_in->v2-eso_in->z[1];	
}	
else 
{
e1=my_deathzoom1(ero_px4,eso_in->eso_dead);
e2=eso_in->z[1];
}
if(eso_in->err_limit>0)	
eso_in->u=eso_in->KP*LIMIT_ESO(e1,-eso_in->err_limit,eso_in->err_limit);
else
eso_in->u=eso_in->KP*e1;	
	if(eso_in->b01!=0){
  switch(eso_in->level){
		case 1:eso_in->disturb_u=eso_in->z[1]/eso_in->b01;break; 
		case 2:eso_in->disturb_u=eso_in->z[2]/eso_in->b01;break;
	}
	if(fabs(e1)>eso_in->eso_dead)
  eso_in->disturb_u_reg=eso_in->disturb_u;
	eso_in->u-=eso_in->Thr_Weight *eso_in->disturb_u_reg;	
	}
return  eso_in->u=LIMIT_ESO(eso_in->u+0*eso_in->integer,-MAX,MAX);
}


//姿态内环-<------------------------------<-----------
float OLDX_ATT_CONTRL_INNER_ESO(ESO *eso_in,float v,float y,float u,float T,float MAX,float kp_in,u16 thr)
{ if(!eso_in->init)
	{	
  eso_in->init=1;
	eso_in->level=1;//系统阶次	
 //-------------跟踪器
	eso_in->use_td=0;		
	eso_in->r0=4000;//跟踪速度
  eso_in->h0=T;//滤波因子
 //-------------观测器
	eso_in->beta0=1/(eso_in->h0);
	eso_in->beta1=1/(30*pow(eso_in->h0,2));
	eso_in->beta2=1000;	
	eso_in->auto_b0=0;
 //-------------反馈----------------
	eso_in->out_mode=1;	
 //-------liner    0
	eso_in->KP=kp_in ;	
	//-------noliner  1
	eso_in->alfa0=0.25;
  eso_in->alfa1=0.75;
	eso_in->alfa2=1.5;	
  eso_in->tao=eso_in->h0*2;		
	  //-------noliner  2 3
	eso_in->c=0.5;//阻尼因子	
	eso_in->r1=0.5/pow(eso_in->h0,2);
	eso_in->h1=eso_in->h0*5;
	//----------模型增益
  //eso_in->b0=20;		
	}
	ESO_Safe(eso_in,thr);
	#if ESO_PARA_USE_REAL_TIME
	    eso_in->h0=T;
			eso_in->beta0=1/(eso_in->h0+0.000001);
   	  eso_in->beta1=1/(30*pow(eso_in->h0,2)+0.000001);
			eso_in->tao=eso_in->h0*2;
	#endif
	eso_in->KP=kp_in ;
	if(eso_in->use_td)
	OLDX_SMOOTH_IN_ESO(eso_in,v);
	switch(eso_in->level){
		case 1:ESO_2N(eso_in,v, y, u, T, MAX,0);break;
		case 2:ESO_3N(eso_in,v, y, u, T, MAX);break;
	}
	ESO_CONTROL(eso_in,v, y, u, T, MAX,0);
	return eso_in->u;
}

//-----------------------------POS-ESO--------------------------------
float OLDX_ESO_2N_POS(ESO *eso_in,float v,float y,float u,float T,float MAX,u8 for_high)             // v是控制系统的输入，y是控制系统的输出，反馈给ESO，u是ADRC的输出控制量
{
	float e=0,fe,fe1;
	eso_in->h=eso_in->h0;
	e=eso_in->e=my_deathzoom1(eso_in->z[0]-y,eso_in->eso_dead);
	fe=fal(e,0.5,eso_in->h0);
	fe1=fal(e,0.25,eso_in->h0);
	eso_in->z[0]+=eso_in->h0*(eso_in->z[1]-eso_in->beta0*e+eso_in->b0*eso_in->Thr_Weight *u);
	eso_in->z[1]+=-eso_in->h0*eso_in->beta1*e;
	eso_in->z[1]=LIMIT_ESO(eso_in->z[1],-MAX*eso_in->Thr_Weight*(eso_in->b0+1),MAX*eso_in->Thr_Weight*(eso_in->b0+1));
//	if(eso_in->eso_for_z)
//	eso_in->z[1]=0;	
	if(eso_in->n==0)
		eso_in->n=1;
	return eso_in->disturb=LIMIT_ESO(eso_in->z[1]/eso_in->n,-MAX,MAX);
}


float OLDX_ESO_CONTROL_POS(ESO *eso_in,float v,float y,float u,float T,float MAX,u8 for_high)
{static float e0,e1,e2;
e1=my_deathzoom1(v-eso_in->z[0],eso_in->eso_dead);
e2=-eso_in->z[1];

if(eso_in->err_limit>0)	
eso_in->u=eso_in->KP*LIMIT_ESO(e1,-eso_in->err_limit,eso_in->err_limit);
else
eso_in->u=eso_in->KP*e1;;	
		if(eso_in->b0!=0){
		eso_in->disturb_u=eso_in->z[1]/eso_in->b0;
		if(fabs(e1)>eso_in->eso_dead)
		eso_in->disturb_u_reg=eso_in->disturb_u;
		eso_in->u-=eso_in->Thr_Weight *eso_in->disturb_u_reg;	
	}
return  eso_in->u=LIMIT_ESO(eso_in->u,-MAX,MAX);
}

//位置控制内环 
float OLDX_POS_CONTROL_ESO(ESO *eso_in,float v,float y,float u,float T,float MAX,float kp_in,u16 thr_view)
{ if(!eso_in->init)
	{
  eso_in->init=1;
	eso_in->level=1;//系统阶次	
 //-------------跟踪器
	eso_in->use_td=0;		
	eso_in->r0=4000;//跟踪速度
  eso_in->h0=T;//滤波因子
 //-------------观测器
	eso_in->beta0=1/(eso_in->h0);
	eso_in->beta1=1/(30*pow(eso_in->h0,2));
	eso_in->beta2=1000;	
 //-------------反馈----------------
	eso_in->out_mode=1;	
		//-------liner    0
	eso_in->KP=kp_in;
	eso_in->KI=0;
	eso_in->KD=2;
	//-------noliner  1
	eso_in->alfa0=0.25;
  eso_in->alfa1=0.75;
	eso_in->alfa2=1.5;	
  eso_in->tao=eso_in->h0*2;		
	  //-------noliner  2 3
	eso_in->c=0.5;//阻尼因子	
	eso_in->r1=0.5/pow(eso_in->h0,2);
	eso_in->h1=eso_in->h0*5;
	//----------模型增益
	//
	}  
   	ESO_Safe(eso_in,thr_view);	
	 #if ESO_PARA_USE_REAL_TIME
	    eso_in->h0=T;
  		eso_in->beta0=1/(eso_in->h0+0.000001);
   	  eso_in->beta1=1/(30*pow(eso_in->h0,2)+0.000001);
	    eso_in->tao=eso_in->h0*2;	
	 #endif
	eso_in->KP=kp_in;
  OLDX_ESO_2N_POS(eso_in,v, y, u, T, MAX,0);
	OLDX_ESO_CONTROL_POS(eso_in,v, y, u, T, MAX,0);
	return eso_in->u;
}

float np=0.01;
float L1=1.15;
float OLDX_AUTO_B0(ESO *eso_in,float v,float y,float u,float T,float MAX)
{
static u8 init;
if(!init){init=1;
eso_in->b01=eso_in->b0;	
}
static float reg_e,reg_e1;
static float Tv,Tv1,b0;
eso_in->b01=eso_in->b0;	
//Tv=Tv1+L1*sign(fabs(eso_in->e-reg_e)-Tv1*fabs(eso_in->e-2*reg_e+reg_e1));
//Tv1=Tv;

//if(sign(eso_in->e)==sign(reg_e))
//	//eso_in->b01-=np*b0/(Tv+0.00001);
//eso_in->b01+=np*eso_in->e*(eso_in->e-2*reg_e+reg_e1)*Thr_Weight*sign(eso_in->u);
//else
//  eso_in->b01=L1*eso_in->b01;

//b0=eso_in->b01;

//eso_in->b01=LIMIT(eso_in->b01,10,80);
//reg_e=eso_in->e;
//reg_e1=reg_e;
}

