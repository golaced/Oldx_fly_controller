#include "include.h" 
#include "iic_soft.h"
#include "hml_sample.h"
#include "ms5611.h"
#include "hml5833l.h"
#include "alt_kf.h"
#include "flash.h"
#include "led_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "sonar_avoid.h"
#include "error.h"
#include "circle.h"
#include "filter.h"
#include "gps.h"
#include "dog.h"
#include "ukf_task.h"
#include "ukf_baro.h"
#include "avoid.h"
#include "LIS3MDL.h"
#include "flow.h"
#include "filter.h"
#include "m100.h"
#include "nav_ukf.h"
#include "wifi_ctrl.h"
#include "oldx_ekf_imu2.h"
#include "uwb.h"
//==============================传感器 任务函数==========================
u8 fly_ready,fly_ready_rx;
float inner_loop_time_time;
float inner_loop_time_time_int;

//========================外环  任务函数============================
OS_STK INNER_TASK_STK[INNER_STK_SIZE];
u16 cnt_time[2];
#include "LSM303.h"
void inner_task(void *pdata)
{	static u8 cnt,cnt1,cnt2,cnt3,cnt4,cnt5;			
  static u16 init,cnt_init;
  u8 i,j;	
 	while(1)
	{	
	inner_loop_time_time = Get_Cycle_T(GET_T_MEMS); 	
	if(!init){
	if(cnt_init++>2)
	init=1;
	inner_loop_time_time=0.005;
	}
	else{
	if(inner_loop_time_time<0.006)	
		cnt_time[0]++;
	else 
		cnt_time[1]++;
							//获取内环准确的执行周期
	if(inner_loop_time_time<0.000002)inner_loop_time_time=0.005;
	
	LSM6_readAcc(0);
	LSM6_readGyro(0);
	if(cnt4++>=3){cnt4=0;
		LIS3MDL_read(0);//80hz
		QMC5883_Update();
	}
	LIS_Data_Prepare(inner_loop_time_time)	;
	
	imu_fushion.Acc.x=lis3mdl.Acc_t.x;
	imu_fushion.Acc.y=lis3mdl.Acc_t.y;
	imu_fushion.Acc.z=lis3mdl.Acc_t.z;
	imu_fushion.Gyro.x=lis3mdl.Gyro_t.x;
	imu_fushion.Gyro.y=lis3mdl.Gyro_t.y;
	imu_fushion.Gyro.z=lis3mdl.Gyro_t.z;
	imu_fushion.Gyro_deg.x=lis3mdl.Gyro_deg_t.x;
	imu_fushion.Gyro_deg.y=lis3mdl.Gyro_deg_t.y;
	imu_fushion.Gyro_deg.z=lis3mdl.Gyro_deg_t.z;
	imu_fushion.Mag_Val.x=lis3mdl.Mag_Val_t.x;
	imu_fushion.Mag_Val.y=lis3mdl.Mag_Val_t.y;
	imu_fushion.Mag_Val.z=lis3mdl.Mag_Val_t.z;
	#if USE_M100_IMU
	imu_fushion.Alt = (m100.H);
	#else
	imu_fushion.Alt=lis3mdl.Alt;
	#endif
	
  }
	delay_ms(5);
	}
}		



//========================外环  任务函数============================
OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];
float YawR,PitchR,RollR;
float YawRm,PitchRm,RollRm;
float outer_loop_time;
float k_gyro_z=1.2;
double X_ekf[7]={1,0,0,0}, P_ekf[49]={0};
double n_q=0.0001,  n_w=0.00001,  n_a=0.01,  n_m=1000;
double Att[4];
float z_k[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f, 0.2f, -0.2f, 0.2f};					/**< Measurement vector */
float x_apo[12]={0};		/**< states */
float P_apo[144] = {100.f, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
					 0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
					 0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,
					 0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,
					 0,   0,   0,   0,  100.f,  0,   0,   0,   0,   0,   0,   0,
					 0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,
					 0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,
					 0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,
					 0,   0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,
					 0,   0,   0,   0,   0,   0,   0,   0,  0.0f, 100.0f,   0,   0,
					 0,   0,   0,   0,   0,   0,   0,   0,  0.0f,   0,   100.0f,   0,
					 0,   0,   0,   0,   0,   0,   0,   0,  0.0f,   0,   0,   100.0f};
float zFlag[3]={1,1,1};
float Rot_matrix[9] = {1.f,  0,  0,
			      0,  1.f,  0,
			      0,  0,  1.f
			     };		
float param[7]={ 
0.00005,//qrspeed
0.08,//qaspeed
0.009,//qacc
0.0005,//qmag
0.0008,//0.0008,//rgyro
10000.0,//racc
50.0};//rmag
float eulerAngles[3];
float q_ekf[4];
//#define AHRS_MAD
//#define AHRS_SO3
//#define AHRS_EKF2
#define AHRS_ANO 

float k_gyro_ekf=1;
void outer_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;	
	static u8 cal_sel;
 	while(1)
	{	
	outer_loop_time = Get_Cycle_T(GET_T_OUTTER);								//获取外环准确的执行周期	
	if(!init){if(cnt_init++>2)
		init=1;
	outer_loop_time=0.01;
	}
	else{
	if(outer_loop_time<=0.00002)outer_loop_time=0.01;	
	#if defined(AHRS_ANO)
	IMUupdate(0.5f *outer_loop_time,
		my_deathzoom_2(imu_fushion.Gyro_deg.x,0.5), my_deathzoom_2(imu_fushion.Gyro_deg.y,0.5),
		my_deathzoom_2(imu_fushion.Gyro_deg.z,0.5)*1.1,
		imu_fushion.Acc.x, imu_fushion.Acc.y, imu_fushion.Acc.z,&RollR,&PitchR,&YawR);	
	#else
				#if defined(AHRS_EKF2)
				z_k[0]= imu_fushion.Gyro_deg.x*0.0173*k_gyro_ekf;
				z_k[1]= imu_fushion.Gyro_deg.y*0.0173*k_gyro_ekf;
				z_k[2]= imu_fushion.Gyro_deg.z*0.0173*k_gyro_ekf;
				z_k[3]= imu_fushion.Acc.x/4096.*9.8;
				z_k[4]= imu_fushion.Acc.y/4096.*9.8;
				z_k[5]= imu_fushion.Acc.z/4096.*9.8;
				z_k[6]= -imu_fushion.Mag_Val.x;
				z_k[7]= -imu_fushion.Mag_Val.y;
				z_k[8]= -imu_fushion.Mag_Val.z;
				oldx_ekf_imu2( x_apo,P_apo,zFlag,z_k,param,outer_loop_time,x_apo,P_apo,Rot_matrix, eulerAngles);
				RollRm=eulerAngles[1];
				PitchRm=eulerAngles[0];
				YawRm=eulerAngles[2];
				Quaternion_FromRotationMatrix(Rot_matrix,q_ekf);
				ref_q_imd_down[0]= q_ekf[2];
				ref_q_imd_down[1]= -q_ekf[3];
				ref_q_imd_down[2]= -q_ekf[0];
				ref_q_imd_down[3]= q_ekf[1];
				reference_vr_imd_down[0] = 2*(ref_q_imd_down[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2]);
				reference_vr_imd_down[1] = 2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]);
				reference_vr_imd_down[2] = 1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2]);
				#endif

				#if defined(AHRS_MAD)
				madgwick_update_new(
				imu_fushion.Acc.x, imu_fushion.Acc.y, imu_fushion.Acc.z,
				my_deathzoom_2(imu_fushion.Gyro_deg.x,0.0)*DEG_RAD, my_deathzoom_2(imu_fushion.Gyro_deg.y,0.0)*DEG_RAD, my_deathzoom_2(imu_fushion.Gyro_deg.z,0.0)*DEG_RAD*k_gyro_z,
				imu_fushion.Mag_Val.x, imu_fushion.Mag_Val.y, imu_fushion.Mag_Val.z,
				&RollRm,&PitchRm,&YawRm,outer_loop_time);	
				#endif

				#if defined(AHRS_SO3)
				static u8 init_ekf;
				static u16 cnt_init,offset_count;
				static double gyro_offsets[3]={0};
				float fRealGyro[3] = {0}, fRealAccel[3] = {0}, fRealMag[3] = {0};
				fRealGyro[0]= imu_fushion.Gyro_deg.x*0.0173;
				fRealGyro[1]= imu_fushion.Gyro_deg.y*0.0173;
				fRealGyro[2]= imu_fushion.Gyro_deg.z*0.0173;
				fRealAccel[0]= imu_fushion.Acc.x/4096.*9.8;
				fRealAccel[1]= imu_fushion.Acc.y/4096.*9.8;
				fRealAccel[2]= imu_fushion.Acc.z/4096.*9.8;
				fRealMag[0]= imu_fushion.Mag_Val.x;
				fRealMag[1]= imu_fushion.Mag_Val.y;
				fRealMag[2]= imu_fushion.Mag_Val.z;
				if(!init_ekf){init_ekf=1;
				NonlinearSO3AHRSinit(fRealAccel[0],fRealAccel[1],fRealAccel[2], fRealMag[0],fRealMag[1],fRealMag[2]);
				}
				else if(init_ekf==1){	
				NonlinearSO3AHRSupdate(fRealGyro[0],fRealGyro[1],fRealGyro[2],
				fRealAccel[0],fRealAccel[1],fRealAccel[2],
				fRealMag[0],fRealMag[1],fRealMag[2],
				20, 0.05, outer_loop_time,&RollRm,&PitchRm,&YawRm);
				cnt_init++;
				gyro_offsets[0] += imu_fushion.Gyro_deg.x;
				gyro_offsets[1] += imu_fushion.Gyro_deg.y;
				gyro_offsets[2] += imu_fushion.Gyro_deg.z;
				offset_count++;
				if(cnt_init>5/outer_loop_time){
				init_ekf=2;
				gyro_offsets[0] = LIMIT(gyro_offsets[0]/offset_count,-5,5)*0.0173;
				gyro_offsets[1] = LIMIT(gyro_offsets[1]/offset_count,-5,5)*0.0173;
				gyro_offsets[2] = LIMIT(gyro_offsets[2]/offset_count,-5,5)*0.0173;
				}
				}else{
				NonlinearSO3AHRSupdate(fRealGyro[0]-gyro_offsets[0],fRealGyro[1]-gyro_offsets[1],fRealGyro[2]-gyro_offsets[2],
				fRealAccel[0],fRealAccel[1],fRealAccel[2],
				fRealMag[0],fRealMag[1],fRealMag[2],
				0.3, 0.005, outer_loop_time,&RollRm,&PitchRm,&YawRm);
				}
				YawRm=-YawRm;
				ref_q_imd_down[0]= -q_so3[3];
				ref_q_imd_down[1]= -q_so3[2];
				ref_q_imd_down[2]= q_so3[1];
				ref_q_imd_down[3]= q_so3[0];
				reference_vr_imd_down[0] = 2*(ref_q_imd_down[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2]);
				reference_vr_imd_down[1] = 2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]);
				reference_vr_imd_down[2] = 1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2]);
				#endif
		
		RollR=RollRm;
		PitchR=PitchRm;
		YawR=YawRm;
		reference_vr[0]=reference_vr_imd_down[0];
		reference_vr[1]=reference_vr_imd_down[1]; 
		reference_vr[2]=reference_vr_imd_down[2];
		q_nav[0]=ref_q[0]=ref_q_imd_down[0]; 		
		q_nav[1]=ref_q[1]=ref_q_imd_down[1]; 
		q_nav[2]=ref_q[2]=ref_q_imd_down[2]; 
		q_nav[3]=ref_q[3]=ref_q_imd_down[3]; 	
  #endif
		static float off_yaw; 
    if (m100.connect==1){
		if(m100.m100_data_refresh==1&&m100.Yaw!=0&&fabs(m100.Yaw-YawR)>10)
		off_yaw=m100.Yaw-YawR;	
		}
    
		Yaw_mid_down=Yaw=To_180_degrees(YawR+off_yaw);	
		Pitch_mid_down=Pitch=PitchR;
		Roll_mid_down=Roll=RollR;	
  }
	if(imu_feed_dog==1&&FC_CONNECT==1)
		IWDG_Feed();//喂狗
	#if USE_M100_IMU
	m100_data(1);
	#endif
	SINS_Prepare();
	delay_ms(5);
	}
}		


//========================EKF  任务函数============================
OS_STK EKF_TASK_STK[EKF_STK_SIZE];
float ekf_loop_time;

#include "FastMath.h"
#include "Quaternion.h"
#include "ekf_ins.h"

void ekf_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;	
 	while(1)
	{	
	ekf_loop_time = Get_Cycle_T(GET_T_EKF);			
	if(cnt_init++>2&&!init){cnt_init=101;
		init=1;	
	}
	else{
		if(ekf_loop_time<0.000002)ekf_loop_time=0.02;
		static u8 ekf_gps_cnt;

		if(!lis3mdl.Mag_CALIBRATED)		
		ukf_pos_task_qr(0,0,Yaw,flow_matlab_data[2],flow_matlab_data[3],LIMIT(flow_matlab_data[0],-3,3),LIMIT(flow_matlab_data[1],-3,3),ekf_loop_time);
		delay_ms(10);
		}
	}
}		


//气压计 任务函数
OS_STK BARO_TASK_STK[BARO_STK_SIZE];
void baro_task(void *pdata)
{							  
 	while(1)
	{ static u8 cnt;
		alt_fushion(0.02);
		delay_ms(20);  
	}
}	

//=======================超声波 任务函数==================
OS_STK SONAR_TASK_STK[SONAR_STK_SIZE];
void sonar_task(void *pdata)
{static u16 cnt_ground;							  
 	while(1)
	{
		#if defined(SONAR_USE_SCL) 
			 if(fly_ready||en_ble_debug)
					Ultra_Duty_SCL(); 
				else if(cnt_ground++>1/0.1){cnt_ground=0;
					Ultra_Duty_SCL(); 
				}
		#else
				#if defined(USE_LIDAR)
				#else
				if(fly_ready||en_ble_debug)
					Ultra_Duty();
				#endif
		#endif	  
		#if defined(SONAR_SAMPLE1)
		delay_ms(40);
		#elif defined(SONAR_SAMPLE2)
		delay_ms(100);
		#elif defined(SONAR_SAMPLE3)
		delay_ms(70);
		#endif
	}
}

float focal_length_px = (16) / (4.0f * 6.0f) * 1000.0f; //original focal lenght: 12mm pixelsize: 6um, binning 4 enabled
float accumulated_flow_x = 0;
float accumulated_flow_y = 0;
float accumulated_gyro_x = 0;
float accumulated_gyro_y = 0;
float accumulated_gyro_z = 0;
uint16_t accumulated_framecount = 0;
uint16_t accumulated_quality = 0;
uint32_t integration_timespan = 0;

float k_flow_acc[2]={0.0231,0.0231};
#if FLOW_USE_OPENMV
float k_gro_acc[3]={0.0018,0.0018,0};
float off_flow_origin[2]={-0.001,0.01};
#else
float k_gro_acc[3]={0.1,0.1,0.1};
float off_flow_origin[2]={0,0};
#endif

#if FLOW_USE_OPENMV
float flt_gro=1;//1;
#else
float flt_gro=0.03;//1;
#endif

float k_time_use=4.2;
void flow_sample(void)
{
uint32_t deltatime=Get_Cycle_T(GET_T_FLOW_SAMPLE)*1000000;
float x_rate = imu_fushion.Gyro_deg.y; // change x and y rates
float y_rate = imu_fushion.Gyro_deg.x;
float z_rate = -imu_fushion.Gyro_deg.z; // z is correct

integration_timespan = deltatime*k_time_use;

accumulated_flow_x = qr.spdx  / focal_length_px * 1.0f*k_flow_acc[0]-off_flow_origin[0]; //rad axis swapped to align x flow around y axis
accumulated_flow_y = qr.spdy / focal_length_px * 1.0f*k_flow_acc[1]-off_flow_origin[1];//rad
#if FLOW_USE_OPENMV
accumulated_flow_x = flow_5a.flow_y_integral  / focal_length_px * 1.0f*k_flow_acc[0]-off_flow_origin[0]; //rad axis swapped to align x flow around y axis
accumulated_flow_y = flow_5a.flow_x_integral / focal_length_px * 1.0f*k_flow_acc[1]-off_flow_origin[1];//rad
#endif	
accumulated_gyro_x = x_rate * k_gro_acc[0]*flt_gro+accumulated_gyro_x*(1-flt_gro);
accumulated_gyro_y = y_rate * k_gro_acc[1]*flt_gro+accumulated_gyro_y*(1-flt_gro);
accumulated_gyro_z = z_rate * k_gro_acc[2]*flt_gro+accumulated_gyro_z*(1-flt_gro);	
}

//=======================FLOW 任务函数==================
u8 imu_board_test=1;
OS_STK FLOW_TASK_STK[FLOW_STK_SIZE];
u8 MID_CNT_SPD=5;
float k_flp=1-0.1; 
static double b_IIR_acc[4+1] ={ 0.0004  ,  0.0017  ,  0.0025  ,  0.0017 ,   0.0004};  //系数b
static double a_IIR_acc[4+1] ={ 1.0000   ,-3.1806   , 3.8612  , -2.1122  ,  0.4383};//系数a
static double InPut_IIR_acc[3][4+1] = {0};
static double OutPut_IIR_acc[3][4+1] = {0};
float b[3] = {0.8122  ,  1.6244  ,  0.8122};
float a[3] = {1.0000  ,  1.5888  ,  0.6600};
float xBuf1[3];
float yBuf1[3];
float xBuf2[3];
float yBuf2[3];
float xBuf3[3];
float yBuf3[3];
float acc_neo[3],flow_ground_temp[4];
float flow_matlab_data[4];
float baro_matlab_data[2];
float flow_loop_time;

float k_flow_devide_pi=0.486;
FLOW_RAD flow_rad_use;  
void flow_task1(void *pdata)
{float flow_height_fliter;		
 static float acc_neo_off[3];
 float flow_temp[2];
 	while(1)
	{
	 #if FLOW_USE_IIC
		Read_Px4flow();		
	 #endif	
	 #if SENSOR_FORM_PI_FLOW&&!SENSOR_FORM_PI_FLOW_SONAR_NOT
	 if(!pi_flow.insert)
	 flow_height_fliter=0.666;
	 else if(pi_flow.z>4)
	 flow_height_fliter=4;
	 else
	 flow_height_fliter=pi_flow.z;
   #else
	 if(!ultra_ok)
	 flow_height_fliter=0.666;
	 else if(ALT_POS_SONAR3>4)
	 flow_height_fliter=4;
	 else
	 flow_height_fliter=ALT_POS_SONAR3;	
	 #endif
	 flow_sample();
	 #if FLOW_USE_P5A||FLOW_USE_OPENMV
	 qr.use_spd=1;
	 #endif
	 if(qr.use_spd==0)
	 {
	 flow_rad_use.time_usec=flow_rad_use.integration_time_us=flow_rad.integration_time_us;
	 flow_rad_use.integrated_xgyro=flow_rad.integrated_xgyro;
	 flow_rad_use.integrated_ygyro=flow_rad.integrated_ygyro;
   flow_rad_use.integrated_zgyro=flow_rad.integrated_zgyro;		 
	 flow_rad_use.integrated_x=flow_rad.integrated_x;
	 flow_rad_use.integrated_y=flow_rad.integrated_y;
	 }
	 else
	 {
	 flow_rad_use.integration_time_us=integration_timespan;
	 flow_rad_use.integrated_xgyro=accumulated_gyro_x;
	 flow_rad_use.integrated_ygyro=accumulated_gyro_y;
   flow_rad_use.integrated_zgyro=accumulated_gyro_z;		 
	 flow_rad_use.integrated_x=accumulated_flow_x;
	 flow_rad_use.integrated_y=accumulated_flow_y;
	 }	 
	  flow_loop_time = Get_Cycle_T(GET_T_FLOW);			
	  if(flow_loop_time<0.001)flow_loop_time=0.01;
	  #if SENSOR_FORM_PI_FLOW
	  flow_ground_temp[0]=pi_flow.sensor.spdy;
		flow_ground_temp[1]=-pi_flow.sensor.spdx;
		flow_ground_temp[2]=pi_flow.sensor.spdy;
	  flow_ground_temp[3]=-pi_flow.sensor.spdx;
	  #else 
	  flow_pertreatment_oldx( &flow_rad_use , flow_height_fliter);
		if(module.pi_flow&&!module.flow&&!module.flow_iic){
		flow_ground_temp[0]=pi_flow.sensor.spdy*k_flow_devide_pi;
		flow_ground_temp[1]=-pi_flow.sensor.spdx*k_flow_devide_pi;
		flow_temp[0]=pi_flow.sensor.spdy*k_flow_devide_pi;
		flow_temp[1]=-pi_flow.sensor.spdx*k_flow_devide_pi;
		}else{
		flow_ground_temp[0]=flow_per_out[0];
		flow_ground_temp[1]=flow_per_out[1];

		flow_temp[0]=firstOrderFilter(LIMIT(flow_per_out[2]*k_flow_devide,-1,1),&firstOrderFilters[FLOW_LOWPASS_X],flow_loop_time);
		flow_temp[1]=firstOrderFilter(LIMIT(flow_per_out[3]*k_flow_devide,-1,1),&firstOrderFilters[FLOW_LOWPASS_Y],flow_loop_time);
		#if FLOW_USE_OPENMV
		flow_temp[0]*=-1;
		flow_temp[1]*=-1;
    #endif		

    flow_matlab_data[2]=flow_temp[0]*cos(FLOW_SET_ANGLE1)+flow_temp[1]*sin(FLOW_SET_ANGLE1);//x;//x
    flow_matlab_data[3]=flow_temp[1]*cos(FLOW_SET_ANGLE1)-flow_temp[0]*sin(FLOW_SET_ANGLE1);//x;//y 			
		}
		#endif
		static float a_br[3]={0};	
		static float acc_temp[3]={0};
		a_br[0] =(float) imu_fushion.Acc.x/4096.;
		a_br[1] =(float) imu_fushion.Acc.y/4096.;
		a_br[2] =(float) imu_fushion.Acc.z/4096.;
		// acc
	  if(fabs(a_br[0])<3.5&&fabs(a_br[1])<3.5){
		acc_temp[0] = a_br[1]*reference_vr[2]  - a_br[2]*reference_vr[1] ;
		acc_temp[1] = a_br[2]*reference_vr[0]  - a_br[0]*reference_vr[2] ;
	  acc_temp[2] =(reference_vr[2] *a_br[2] + reference_vr[0] *a_br[0] + reference_vr[1] *a_br[1]);
    if(fabs(acc_temp[0])<1.5&&fabs(acc_temp[1])<1.5){
		static float acc_neo_temp[3]={0};
		#if USE_UKF_FROM_AUTOQUAD
		float accIn[3];
    accIn[0] = IMU_ACCX + UKF_ACC_BIAS_X*1;
    accIn[1] = IMU_ACCY + UKF_ACC_BIAS_Y*1;
    accIn[2] = IMU_ACCZ + UKF_ACC_BIAS_Z*1;
    float acc[3];
    // rotate acc to world frame
    navUkfRotateVectorByQuat(acc, accIn, &UKF_Q1);
		acc_neo_temp[0]=-acc[0];
		acc_neo_temp[1]=-acc[1];
		acc_neo_temp[2]=-(acc[2]+9.87);	
		#else
	  acc_neo_temp[0]=-acc_temp[0]*9.87;
		acc_neo_temp[1]=-acc_temp[1]*9.87;
		acc_neo_temp[2]=(acc_temp[2]-1.0f)*9.87;		
		#endif
	  if(en_ble_debug||imu_board_test)
			;
		else if(!fly_ready){
	  acc_neo_off[0]+= ( 1 / ( 1 + 1 / ( 2.2f *3.14f *0.04 ) ) ) *(acc_neo_temp[0]- acc_neo_off[0]) ;
		acc_neo_off[1]+= ( 1 / ( 1 + 1 / ( 2.2f *3.14f *0.04 ) ) ) *(acc_neo_temp[1]- acc_neo_off[1]) ;
		acc_neo_off[2]+= ( 1 / ( 1 + 1 / ( 2.2f *3.14f *0.04 ) ) ) *(acc_neo_temp[2]- acc_neo_off[2]) ;
		}
		static float acc_neo_temp1[3]={0};
		static float acc_flt[3];
    acc_neo_temp1[0]=Moving_Median(5,5,acc_neo_temp[0]-acc_neo_off[0]);
		acc_neo_temp1[1]=Moving_Median(6,5,acc_neo_temp[1]-acc_neo_off[1]);
		acc_neo_temp1[2]=Moving_Median(7,5,acc_neo_temp[2]-acc_neo_off[2]);	
		acc_flt[0]=firstOrderFilter(acc_neo_temp1[0],&firstOrderFilters[ACC_LOWPASS_X],flow_loop_time);
		acc_flt[1]=firstOrderFilter(acc_neo_temp1[1],&firstOrderFilters[ACC_LOWPASS_Y],flow_loop_time);
		acc_flt[2]=firstOrderFilter(acc_neo_temp1[2],&firstOrderFilters[ACC_LOWPASS_Z],flow_loop_time);			

		if(fabs(acc_neo_temp[0])<8.6&&fabs(acc_neo_temp[1])<8.6){
		acc_neo[0]=acc_flt[0];
		acc_neo[1]=acc_flt[1];
		acc_neo[2]=acc_flt[2];
	 	flow_matlab_data[0]=acc_neo[0];//acc
		flow_matlab_data[1]=acc_neo[1];}
	  }
	 }
	 
		float temp_spd[2];
		temp_spd[0]=Moving_Median(16,MID_CNT_SPD,FLOW_VEL_X);
		temp_spd[1]=Moving_Median(17,MID_CNT_SPD,FLOW_VEL_Y);
		imu_nav.flow.speed.x=imu_nav.flow.speed.x*(1-k_flp)+k_flp*temp_spd[0];
		imu_nav.flow.speed.y=imu_nav.flow.speed.y*(1-k_flp)+k_flp*temp_spd[1];
 
	  GPS_Data_Processing_Task(10);GPS_Data_Processing_Task6(10);
		delay_ms(10); 
	}
}	

#define BLE_FLOW_F 5
#define BLE_GPS  6

u8 UART_UP_LOAD_SEL=BLE_GPS;//<------------------------------UART UPLOAD DATA SEL
float time_uart;
float px4_test[4]={0};
void TIM3_IRQHandler(void)
{
	OSIntEnter();static u16 cnt,cnt1,cnt2,cnt3,cnt_init,init;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
	time_uart = Get_Cycle_T(GET_T_UART); 	
	if(!init){
	if(cnt_init++>2)
	init=1;
	time_uart=0.0025;
	}
	else{
	#if USE_OUTER_LINK
  Send_TO_CAR();	
	#else
	if(m100.control_connect)
		m100_contrl_px4(m100.control_spd[0],m100.control_spd[1],m100.control_spd[2],m100.control_yaw,m100.px4_tar_mode);
		//m100_contrl_px4(px4_test[0],px4_test[1],px4_test[2],px4_test[3],m100.px4_tar_mode);
	#endif
							//获取内环准确的执行周期
	if(time_uart<0.000002)time_uart=0.005;
		#if EN_DMA_UART2 			
			     if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA2_Steam7传输完成
								{ 
							DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//清除DMA2_Steam7传输完成标志
							clear_nrf_uart();		
							GOL_LINK_TASK_DMA();
					    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA1_Stream6,nrf_uart_cnt+2);     //开始一次DMA传输！	
								}							
					#else
							GOL_LINK_TASK();
					#endif
	
	debug_pi_flow[0]=0;  
	en_ble_debug=1;								
	if(debug_pi_flow[0])									
		en_ble_debug=1;
	if(cnt++>2-1){cnt=0;	
								if(en_ble_debug){
								switch(UART_UP_LOAD_SEL)
								{
								case 0://海拔速度
								Send_BLE_DEBUG((int16_t)(baroAlt),ALT_POS_BMP*1000,ALT_POS_BMP_EKF*1000,
								0,-ALT_VEL_BMP*100,ALT_VEL_BMP_EKF*100,
								0,acc_z_view[1],acc_z_view[0]);break;		
								case 1://海拔速度
								Send_BLE_DEBUG((int16_t)(0),flow_rad_use.integrated_x*1000,flow_rad_use.integrated_y*1000,
								(flow_rad_use.integrated_y-flow_rad_use.integrated_ygyro)*1000,flow_rad_use.integrated_xgyro*1000,flow_rad_use.integrated_ygyro*1000,
								0,flow_matlab_data[2]*100,flow_matlab_data[3]*100);break;		
								case 2://海拔速度
								Send_BLE_DEBUG((int16_t)(X_ukf_baro[3]*100),baro_matlab_data[0]*100,X_ukf_baro[0]*100,
								X_ukf_baro[4]*100,X_ukf_baro[1]*100,ALT_VEL_BMP_EKF*100,
								0,ALT_VEL_BMP_UNION*100,X_kf_sonar[1]*100);break;	
								case 3:
								Send_BLE_DEBUG(0,gpsx.pvt.PVT_Down_speed*100,X_kf_baro[1]*100,
								(ultra_distance),m100.H*100,X_kf_baro[0]*100,
								m100.H_Spd*1000,Global_GPS_Sensor.NED_Pos[1]*100,Global_GPS_Sensor.NED_Pos[0]*100);break;
								case 4:
								Send_BLE_DEBUG(i_uwb_lps_tag.position.x*100,i_uwb_lps_tag.position.y*100,uwb_pos[2]*100,
								X_ukf_Pos[0]*100,X_ukf_Pos[1]*100,0,
								ALT_POS*100,-ALT_VEL*100,AQ_PRESSURE*100);break;
								case 5:
								Send_BLE_DEBUG(amf.spd_body[0]*100,amf.spd_body[1]*100,X_ukf_Pos[0]*100,
								X_ukf[1]*100,X_ukf[4]*100,amf.pos[2]*100,
								amf.pos[0]*100,amf.pos[1]*100,X_ukf_Pos[1]*100);break;
								case 6://GPS
								Send_BLE_DEBUG(
								Gps_information.local_pos[Xr]*100,X_ukf_Pos[0]*100,0,
								Gps_information.local_spd_flt[Xr]*100,Gps_information.local_spd_flt[Yr]*100,Gps_information.real_D_vel*100,
								acc_neo[0]*10, acc_neo[1]*10,  acc_neo[2]*10);break;
								default:break;
							 }
								} 
							}								
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
	}
	OSIntExit(); 
}

//=======================故障保护 任务函数==================
OS_STK ERROR_TASK_STK[ERROR_STK_SIZE];
void error_task(void *pdata)
{	static u8 cal_flag;
	static u16 cal_flag_mask;						  
 	while(1)
	{
		if(gps_loss_cnt++>5/0.2)
			gps_good=0;
		if(pi_flow.loss_cnt++>1/0.2)
			pi_flow.insert=0;
		if(pi_flow.sensor.loss_cnt++>1/0.2)
			pi_flow.sensor.connect=0;
	  if(fc_loss_cnt++>3/0.2)
			FC_CONNECT=0;
		if(m100.cnt_m100_data_refresh++>3/0.2)
		 m100.m100_data_refresh=0;
		if(m100.loss_cnt++>3/0.2)
		 m100.connect=0;
		if(m100.control_loss++>3/0.2)
		 m100.control_connect=0;
		if(qr.loss_cnt++>3/0.2)
		 qr.connect=0;
    if(wifiCtrl.loss_cnt++>3/0.2)
		 wifiCtrl.connect=0;
		if(Gps_information.lose_cnt++>4/0.2)
		 Gps_information.connect=0;
		flow.rate=flow.flow_cnt;
	  flow.flow_cnt=0;
		
    if(cal_flag_mask++>5/0.2&&!cal_flag)
			cal_flag=1;
		if(!cal_flag)
			lis3mdl.Mag_CALIBRATED=lis3mdl.Acc_CALIBRATE=lis3mdl.Gyro_CALIBRATE=0;
		delay_ms(200); 
	}
}	

//------------------------------软件定时器--------------------------------//
OS_TMR   * tmr1;			//软件定时器1
OS_TMR   * tmr2;			//软件定时器2
OS_TMR   * tmr3;			//软件定时器3

//软件定时器1的回调函数	  OSCPUUsage
//每100ms执行一次,用于显示CPU使用率和内存使用率		
 u16 cpuusage=0;
void tmr1_callback(OS_TMR *ptmr,void *p_arg) 
{
	static u8 tcnt=0;	    

	if(tcnt==5)
	{	
		cpuusage=0;
		tcnt=0; 
	}
	cpuusage+=OSCPUUsage;
	tcnt++;				    
}

//软件定时器2的回调函数				  	   
void tmr2_callback(OS_TMR *ptmr,void *p_arg) 
{	
static u8 cnt;
	LEDRGB_STATE();
}
