#include "alt_fushion.h"
#include "mymath.h"
#include "filter.h"
#include "imu.h"
#include "quar.h"
#include "mpu6050.h"
#include "usart.h"
#include "height_ctrl.h"
#include "include.h"
#include "rc.h"
#include "ultrasonic.h"
#include "eso.h"
#include "baro_ekf_oldx.h"

		
float baro_compensate(float dT,float kup,float kdw,float vz,float lim)
{
	float z_sin;
	static float com_val,com_tar;
	
	z_sin = my_sqrt(1-my_pow(vz));
	
	//com_tar = (z_sin/0.44f) *lim;
	LPF_1_(2.0f,dT,((z_sin/0.44f) *lim),com_tar);
	com_tar = LIMIT(com_tar,0,lim);
	
	if(com_val<(com_tar-100))
	{
		com_val += 1000 *dT *kup;
	}
	else if(com_val>(com_tar+100))
	{
		com_val -= 1000 *dT *kdw;
	}
	return (com_val);
}

void body_to_NEZ(float *vr, float *v, float *q) {
    float w, x, y, z;

    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];

    vr[0] = w*w*v[0] + 2.0f*y*w*v[2] - 2.0f*z*w*v[1] + x*x*v[0] + 2.0f*y*x*v[1] + 2.0f*z*x*v[2] - z*z*v[0] - y*y*v[0];
    vr[1] = 2.0f*x*y*v[0] + y*y*v[1] + 2.0f*z*y*v[2] + 2.0f*w*z*v[0] - z*z*v[1] + w*w*v[1] - 2.0f*x*w*v[2] - x*x*v[1];
    vr[2] = 2.0f*x*z*v[0] + 2.0f*y*z*v[1] + z*z*v[2] - 2.0f*w*y*v[0] - y*y*v[2] + 2.0f*w*x*v[1] - x*x*v[2] + w*w*v[2];
}

float ALT_POS_BMP,ALT_VEL_BMP;
float ALT_POS_SONAR,ALT_VEL_SONAR,ALT_POS_SONAR2,ALT_POS_SONAR3;
float ALT_POS_SONAR2,ALT_POS_SONAR3;
float ALT_POS_BMP_EKF,ALT_VEL_BMP_EKF;
float ALT_POS_BMP_UKF_OLDX,ALT_VEL_BMP_UKF_OLDX,ALT_ACC_BMP_UKF_OLDX;
//-----------------KF  parameter------------------
double P_kf_baro[9]={100,0,0,0,100,0,0,0,100}; 
double X_kf_baro[4];
//float g_spd=0.05;
float ga=0.01;
float gwa=0.2;

//float gh_bmp=0.1;
//float g_spd=0.168;
//double pos_delay[4]={1,2,0,0};
//u8 en_x2_check=0;
//int bmp_x2=23;//35;//23;

//float gh_bmp=0.1;
//float g_spd=0.168;
//double pos_delay[4]={1,4,0,0};
//int bmp_x2=200;//35;//23;
//u8 en_x2_check=0;

float gh_bmp=0.1; 
float g_spd=0.5;
double pos_delay[4]={1,2,0,4};//4状态修正   5量测修正
int bmp_x2=200;//35;//23;
u8 en_x2_check=0;
float FLT_BARO=1;
float FLT_ACC=20;
//float gh_bmp=0.1;
//float g_spd=0.85;
//double pos_delay[4]={1,4,0,5};
//int bmp_x2=200;//35;//23;
//u8 en_x2_check=1;

float acc_body[3];
float K_SONAR=2.8;
ESO eso_h_acc,eso_h_spd;
float nmda[3];
u8 baro_flt_sel=1;

float flt_namda=0.1;

float X_apo_height[2] = {0.0f, 0.0f};
float P_apo_k_height[4] = {100.0f,0.0f,0.0f,100.0f};
float r_baro = 10;//10; // 10.0f;			
float r_acc =  0.1; // 0.5f;
float off_ekf[2]={0,0};

void ukf_baro_task1(float T)// 气压计加速度计融合
{
static u8 height_ctrl_mode_reg;	
static float off_bmp_sonar,acc_off_baro;		
static u8 init,mode_reg;
if(!init)
{   init=1;
		eso_h_spd.h0=eso_h_acc.h0=0.02;
    eso_h_spd.r0=eso_h_acc.r0=8;
	  sys_init.baro_ekf=1;
}	

#if SONAR_USE_FC||SONAR_USE_FC1
float  tilted_fix_sonar;
tilted_fix_sonar=LIMIT((ultra.relative_height/cos(LIMIT(my_deathzoom_21(Pit_fc,5),-45,45)/57.3)/
								cos(LIMIT(my_deathzoom_21(Rol_fc,5),-45,45)/57.3)-ultra.relative_height),0,0.5);
float posz_sonar=LIMIT(ultra.relative_height+tilted_fix_sonar,0,8);
		if( fabs(posz_sonar - ALT_POS_SONAR2) < 0.1 )
		{			
			ALT_POS_SONAR2 += ( 1 / ( 1 + 1 / ( K_SONAR*4.0f *3.14f *T ) ) ) *(posz_sonar - ALT_POS_SONAR2) ;
		}
		else if( fabs(posz_sonar - ALT_POS_SONAR2) < 0.2 )
		{
			ALT_POS_SONAR2 += ( 1 / ( 1 + 1 / ( K_SONAR*2.2f *3.14f *T ) ) ) *(posz_sonar- ALT_POS_SONAR2) ;
		}
		else if( fabs(posz_sonar - ALT_POS_SONAR2) < 0.4 )
		{
			ALT_POS_SONAR2 += ( 1 / ( 1 + 1 / ( K_SONAR*1.2f *3.14f *T ) ) ) *(posz_sonar- ALT_POS_SONAR2) ;
		}
		else
		{
			ALT_POS_SONAR2 += ( 1 / ( 1 + 1 / ( K_SONAR*0.6f *3.14f *T ) ) ) *(posz_sonar- ALT_POS_SONAR2) ;
		}	
#endif
		
//	if(height_ctrl_mode_reg!=height_ctrl_mode&&height_ctrl_mode==1&&module.sonar==1&&ALT_POS_SONAR2<2)
//	{
//		off_bmp_sonar=-(float)(baro.relative_height)/1000.+ALT_POS_SONAR2;
//	} 		
	
	#define BARO_AV_NUM_FU 10
	static float baro_av_arr_fu[BARO_AV_NUM_FU];
	static  u16 baro_av_cnt_fu;
	baro.h_origin=((float)(baro.relative_height)/1000.+off_bmp_sonar);
	//baro.h_flt=firstOrderFilter((baro.h_origin) ,&firstOrderFilters[BARO_LOWPASS],T);
  DigitalLPF(baro.h_origin,&baro.h_flt,FLT_BARO,T);

float baro_com_val;
#if EN_ATT_CAL_FC
baro_com_val = baro_compensate(T,1.0f,1.0f,reference_vr_imd_down_fc[2],3500);
#else
baro_com_val = baro_compensate(T,1.0f,1.0f,reference_vr_imd_down[2],3500);
#endif
float posz;
posz=(float)(baro.h_flt);	
posz=(float)(baro.h_origin);		
		static float temp_r;
		u8 i,j;
		float acc_temp1,temp;  
		float accIn[3];
		float acc_body_temp[3],acc_body_temp_flt[3];
		accIn[0] =(float) mpu6050_fc.Acc.x/4096.*9.8;
		accIn[1] =(float) mpu6050_fc.Acc.y/4096.*9.8;
		accIn[2] =(float) mpu6050_fc.Acc.z/4096.*9.8;
		body_to_NEZ(acc_body_temp_flt, accIn, ref_q_imd_down_fc);

		static float wz_acc ;
		static u16 ekf_init_cnt;

		//acc_body_temp[2] = firstOrderFilter(acc_body_temp_flt[2]-9.8 ,&firstOrderFilters[ACC_LOWPASS_Z],T);
			 				
		if(NS==0||1)
	  acc_off_baro=0;
		else if((!fly_ready&&NS==2))		
		{
			X_kf_baro[0]=posz;X_kf_baro[1]=X_kf_baro[2]=0;
			for(i=0;i<9;i++)
			P_kf_baro[i]=0;			
			acc_off_baro += ( 1 / ( 1 + 1 / ( 3*0.6f *3.14f *T ) ) ) *(acc_body_temp[2]- acc_off_baro) ;
			acc_off_baro=LIMIT(acc_off_baro,-3,3);
		}
    acc_body[2]=DigitalLPF_NEW(acc_body_temp_flt[2]-9.8,acc_body[2],FLT_ACC,T);	   
   // acc_body[2]=(acc_body_temp[2]-acc_off_baro*0);

		#if defined(USE_KF)
		double A[9]=
		{1,       0,    0,
		T,       1,    0,
		-T*T/2, -T,    1};
		double B[3]={T*T/2,T,0}; 
		double H[9]={
		0,0,0,
		0,0,0,
		0,0,0}; 
	  double H_x2[3]={0,0,0};
		
		if(baro_update==1)//室内
		{
			baro_update=0;
			if(m100.STATUS==3)
				H[4]=1; 	
			H[0]=1;
		}else
			H[0]=0;
		  H_x2[0]=H[0];
		//H[0]=0;H[1]=1;
    float gh_bmp_use;
		if(nmda[2]>bmp_x2/100.   &&  en_x2_check && m100.STATUS==3)
			{H[0]=0;}
//			gh_bmp_use=10;
//		if(nmda[2]>bmp_x2/100.   &&  en_x2_check)
//		   gh_bmp_use=1000;
		else
			 gh_bmp_use=gh_bmp;
	double Z_kf[3]={posz,ALT_VEL_BMP,0};
	if(sys_init.baro_ekf)
		KF_OLDX_NAV( X_kf_baro,  P_kf_baro,  Z_kf,  acc_body[2], A,  B,  H, H_x2, ga,  gwa, gh_bmp_use,  g_spd,  T , 2,pos_delay);
	nmda[2]=LIMIT(X_kf_baro[3]*flt_namda+(1-flt_namda)*nmda[2],0,333);
	#else
	float Z_baro_ekf[2]={posz,acc_body[2]};		
	BARO_EKF_OLDX(X_apo_height,P_apo_k_height, X_apo_height, P_apo_k_height ,Z_baro_ekf,  r_baro,  r_acc, T);
	X_kf_baro[0]=X_apo_height[0]-off_ekf[0];
	X_kf_baro[1]=X_apo_height[1]-off_ekf[1];
	#endif
	
	static u8 fly_readyr;
	static float bmp_off[2];
	if(fly_ready&&!fly_readyr){
		bmp_off[0]=X_kf_baro[0];
	  bmp_off[1]=LIMIT(X_kf_baro[1],-1,1);
	}
	fly_readyr=fly_ready;
	ALT_POS_BMP_UKF_OLDX=X_kf_baro[0]-bmp_off[0];
	#if defined(USE_KF)
	ALT_VEL_BMP_UKF_OLDX=X_kf_baro[1];
	#else
	ALT_VEL_BMP_UKF_OLDX=X_kf_baro[1]-bmp_off[1];
	#endif
	ALT_ACC_BMP_UKF_OLDX=X_kf_baro[2];	
	
}