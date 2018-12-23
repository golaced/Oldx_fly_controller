#include "ukf_task.h"
#include "ukf_oldx.h"
#include "my_math.h"
#include "usart_fc.h"
#include "gps.h"
#include "KF_OLDX_NAV.h"
#include "OLDX_MMKF.h"
#include "hml5833l.h"
#include "alt_kf.h"
#include "insgps.h"
#include "time.h"
#include "nav_ukf.h"
#include "matlib.h"

double X_ukf[8],X_ukf_global[8];
double X_ukf_baro[6];
float X_ukf_Pos[2];
float r1,r2;
float posNorth,posEast;
double local_Lat,local_Lon;//GPS局部坐标初始
float velEast,velNorth;
float GPS_J_F,GPS_W_F;//融合GPS
static void CalcEarthRadius(double lat) {
    double sinLat2;

    sinLat2 = sin(lat * (double)DEG_TO_RAD);
    sinLat2 = sinLat2 * sinLat2;

    r1 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD * ((double)1.0 - (double)NAV_E_2) / pow((double)1.0 - ((double)NAV_E_2 * sinLat2), ((double)3.0 / (double)2.0));
    r2 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD / sqrt((double)1.0 - ((double)NAV_E_2 * sinLat2)) * cos(lat * (double)DEG_TO_RAD);
}

static void CalcGlobalDistance(double lat, double lon) {
    posNorth = (lat - local_Lat) * r1;
    posEast =  (lon - local_Lon) * r2;
}

void CalcGlobalDistance1(double lat, double lon,float local_Lat,float local_Lon,float *posNorth,float *posEast ) {
    *posNorth = (lat - local_Lat) * r1;
    *posEast =  (lon - local_Lon) * r2;
}

static void CalcGlobalLocation(float posNorth,float posEast){ 
    GPS_W_F=(float)posNorth/(float)(r1+0.1)+local_Lat;
    GPS_J_F=(float)posEast/(float)(r2+0.1)+local_Lon;
}

#define MODE_FLOW 0
#define MODE_GPS 1
#define MODE_IMU 2
#define MODE_UWB 3
double X_KF_NAV[2][3];
double P_KF_NAV[2][9]={0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001};
float ga_nav= 0.1; 
float gwa_nav=0.1;
//flow
float g_pos_flow= 0.0086*1.2;//0.0051;
float g_spd_flow= 0.00026;//0.0006;
float K_spd_flow=0.86;//1.2;//0.86;
//uwb
float g_pos_uwb= 0.00123;//0.0051;
//gps
//float g_pos_gps= 0.036;
float g_pos_gps= 0.06;//WT G1
//--------------------
//float g_spd_gps= 0.00025/2;
//float g_spd_gps= 0.000252125;//WT G1
float g_spd_gps= 0.0005;//WT G1
//float g_spd_gps= 0.0015;//WT G2
//---------------------------
//double pos_delay[4]={0,4,0,5};	
double pos_delay[4]={0,4,0,0};//WT	G1 
//double pos_delay[4]={0,10,0,5};//WT	G2
u8 position_mode=0;
void ukf_pos_task_qr(float Qr_x,float Qr_y,float Yaw,float flowx,float flowy,float accx,float accy,float T)
{
u8 i;	

u8 sensor_sel=0;
float Posx,Posy;
float Sdpx,Accx;
float Sdpy,Accy;
float g_pos_use,g_spd_use;
float velNorth_gps,velEast_gps;

double A[9]=
			 {1,       0,    0,
				T,       1,    0,
				-T*T/2, -T,    1};

double B[3]={T*T/2,T,0}; 
double H[9]={
			 0,0,0,
       0,0,0,
       0,0,0}; 

#if USE_M100_IMU
gpsx.pvt.PVT_latitude=m100.Lat;
gpsx.pvt.PVT_longitude=m100.Lon;	
if(m100.connect&&m100.m100_data_refresh==1&&m100.Yaw!=0)	
{gpsx.pvt.PVT_numsv=4;gpsx.pvt.PVT_fixtype=3;}
#endif			 

navigaition_mode=0;//clear 
//--------------------------------GPS_KF------------------------------------- 
/*********************************************************************************
**********************************************************************************/	
//--------=-----------------flow in global-----------------------------------------
	 float Yaw_qr=To_180_degrees(Yaw);
	 if (px4.connect&&px4.m100_data_refresh)	
	   	 Yaw_qr=px4.Yaw;
	 //uwb fusion
	 estimate_position_uwb(T);
	 #if USE_UWB_AS_POS
   if(i_uwb_lps_tag.init)
	    Yaw_qr=Yaw_uwb;
	 #endif		
	 float ACCY=flow_matlab_data[1];
   float ACCX=flow_matlab_data[0];
	 float accEast=ACCY*sin(Yaw_qr*0.0173)+ACCX*cos(Yaw_qr*0.0173);
   float accNorth=ACCY*cos(Yaw_qr*0.0173)-ACCX*sin(Yaw_qr*0.0173);
   float SPDY=flowy*K_spd_flow;
	 float SPDX=flowx*K_spd_flow;
	 if(amf.connect)
	 {
	  SPDY=amf.spd_body[1];
		SPDX=amf.spd_body[0];
	 }
   //conver body flow spd to NEZ flow
	 velNorth=SPDY*cos(Yaw_qr*0.0173)-SPDX*sin(Yaw_qr*0.0173);
   velEast=SPDY*sin(Yaw_qr*0.0173)+SPDX*cos(Yaw_qr*0.0173);

	 if (px4.connect&&px4.m100_data_refresh)		
	 { 
		velNorth=LIMIT(px4.spd[1]*cos(Yaw_qr*0.0173)-px4.spd[0]*sin(Yaw_qr*0.0173),-6.3,6.3);
		velEast=LIMIT(px4.spd[1]*sin(Yaw_qr*0.0173)+px4.spd[0]*cos(Yaw_qr*0.0173),-6.3,6.3);	
		position_mode=MODE_IMU;	
		if(px4.GPS_STATUS>=3)
			gps_update[0]=1;	 
	 }
	  
	 Sdpy=velNorth;
	 Accy=accNorth;
	 Sdpx=velEast;
	 Accx=accEast;
	 
	 //UWB
	 #if USE_UWB_AS_POS
	 static u8 state_init_uwb_pos;
	 Posx=i_uwb_lps_tag.position.x;
	 Posy=i_uwb_lps_tag.position.y;
	 if(state_init_uwb_pos==0&&i_uwb_lps_tag.init==1)
				{X_KF_NAV[1][0]=i_uwb_lps_tag.position.y;X_KF_NAV[0][0]=i_uwb_lps_tag.position.x;}
	 state_init_uwb_pos=i_uwb_lps_tag.init;		
	 #endif
				
	 //GPS fusion
		 //init		
		 if(Gps_information.satellite_num>5&&Gps_information.kf_init==0&&
			 ABS(Gps_information.latitude)>10&&ABS(Gps_information.latitude)<180)
			{		
			 Gps_information.home_latitude=Gps_information.latitude;
			 Gps_information.home_longitude=Gps_information.longitude;
			 CalcEarthRadius(Gps_information.home_latitude);
			}		
		 float pos_gps[2]={0,0};
		 float spd_gps[2]={0,0};
		 //measure
     if(Gps_information.connect&&Gps_information.update[0]&&ABS(Gps_information.latitude)>10&&ABS(Gps_information.latitude)<180){	
			 CalcGlobalDistance1(Gps_information.latitude,Gps_information.longitude,
			 Gps_information.home_latitude,Gps_information.home_longitude,
			 &pos_gps[Yr],&pos_gps[Xr]); 
			 
			  spd_gps[Xr]=Gps_information.local_spd_flt[Xr];
			  spd_gps[Yr]=Gps_information.local_spd_flt[Yr];
			 
			  Gps_information.local_pos[Xr]=pos_gps[Xr];
			  Gps_information.local_pos[Yr]=pos_gps[Yr];

			  if(Gps_information.kf_init==0&&Gps_information.satellite_num>5&&(Gps_information.PVT_Vacc<4500||1)){
						Gps_information.off_earth=get_declination(Gps_information.latitude,Gps_information.longitude);
						Gps_information.kf_init=1;
					  position_mode=MODE_GPS;//set gps mode
					  X_KF_NAV[0][0]=pos_gps[Xr];
					  X_KF_NAV[1][0]=pos_gps[Yr];
				 }
			}
		  //reset
		    static int gps_rst_cnt;
		    if(Gps_information.kf_init&&(Gps_information.connect==0||Gps_information.PVT_Vacc>10000)) 
					gps_rst_cnt++;
				else
					gps_rst_cnt=0;
				if(gps_rst_cnt>5/T)
					position_mode=gps_rst_cnt=Gps_information.kf_init=0;
			
				
		 // KF measurement flag	
		 //GPS
			if(Gps_information.kf_init&&Gps_information.update[0]){
				Gps_information.update[0]=0;	
				H[0]=H[4]=1;  
			} 
			//FLOW
      if(flow_update==1&&position_mode==MODE_FLOW)
			{
				H[4]=1; 	
				flow_update=0;
			}
			
		 //UWB
		 #if USE_UWB_AS_POS
			 if(i_uwb_lps_tag.init&&i_uwb_lps_tag.state&&i_uwb_lps_tag.position.x!=0&&i_uwb_lps_tag.position.y!=0
				 &&i_uwb_lps_tag.position.x<8&&i_uwb_lps_tag.position.y<8&&i_uwb_lps_tag.position.x>-0.3&&
			 i_uwb_lps_tag.position.y>-0.3)
			 {H[0]=1;position_mode=sensor_sel=MODE_UWB;}
			 else
			 H[0]=0; 	 
		 #endif

	  //kalman filter		
	 double Zx_fix[3]={Posx,Sdpx,1};
	 double Zy_fix[3]={Posy,Sdpy,1};

		if(position_mode==MODE_GPS&&Gps_information.kf_init&&Gps_information.connect)//gps
		{	
			navigaition_mode=2;
		  Zx_fix[0]=pos_gps[Xr];Zx_fix[1]=spd_gps[Xr];	
		  Zy_fix[0]=pos_gps[Yr];Zy_fix[1]=spd_gps[Yr];	
      sensor_sel=MODE_GPS;						
		}	
		else if(position_mode==MODE_FLOW&&(module.flow||module.flow_iic||module.pi_flow||amf.connect))  //flow
		{
			navigaition_mode=1;
			Zx_fix[0]=Posx; Zy_fix[0]=Posy;
	    Zx_fix[1]=Sdpx; Zy_fix[1]=Sdpy;
			pos_delay[0]=pos_delay[1]=0;
			sensor_sel=MODE_FLOW;
		}else// sensor error
		{
			H[0]=0; H[4]=1; 
	    Zx_fix[1]=0.00001; Zy_fix[1]=0.00001;
			pos_delay[0]=pos_delay[1]=0;
			sensor_sel=MODE_GPS;
		}
		
	 float gps_wqv=gpsx.ubm.sAcc *0.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.nDOP*0.01*gpsx.pvt.nDOP*0.01) * UKF_GPS_VEL_M_N;
	 float gps_wqp=gpsx.ubm.hAcc *1.001* __sqrtf(gpsx.pvt.tDOP*0.01*gpsx.pvt.tDOP*0.01 + gpsx.pvt.nDOP*0.01*gpsx.pvt.nDOP*0.01) * UKF_GPS_POS_M_N;
	 float g_spd,g_pos;
   switch(sensor_sel){
		 case MODE_GPS:g_spd=g_spd_gps+gps_wqv;g_pos=g_pos_gps+gps_wqp;break;
     case MODE_FLOW:g_spd=g_spd_flow;g_pos=g_pos_flow;break;
		 case MODE_UWB:g_spd=g_spd_flow;g_pos=g_pos_uwb;break;
		 default:g_spd=g_spd_flow;g_pos=g_pos_flow;break; 
	 }
	
	 float g_w[2]={1,1};//gps to flow weight
	 if((px4.connect||amf.connect)&&position_mode==MODE_FLOW)
		{g_w[0]=0.1;g_w[1]=0.1;}

		
   KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy_fix,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos*g_w[0],  g_spd*g_w[1],  T, 1, pos_delay);
	 KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx_fix,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos*g_w[0],  g_spd*g_w[1],  T, 0, pos_delay); 
		 
	 X_ukf[0]=X_KF_NAV[0][0];//East pos
	 X_ukf[6]=X_KF_NAV[0][1];//East vel
	 X_ukf[2]=X_KF_NAV[0][2];
	 X_ukf[3]=X_KF_NAV[1][0];//North  pos
	 X_ukf[7]=X_KF_NAV[1][1];//North  vel
	 X_ukf[5]=X_KF_NAV[1][2];	
   //spd convert to body frame 	 
	 X_ukf[1]=-X_KF_NAV[1][1]*sin(Yaw_qr*0.0173)+X_KF_NAV[0][1]*cos(Yaw_qr*0.0173);//X
	 X_ukf[4]= X_KF_NAV[1][1]*cos(Yaw_qr*0.0173)+X_KF_NAV[0][1]*sin(Yaw_qr*0.0173);//Y
 	 X_ukf_Pos[0]=X_ukf[0];//East Pos
   X_ukf_Pos[1]=X_ukf[3];//North Pos
	 //global
	 X_ukf_global[1]=X_ukf[6];//East   vel
	 X_ukf_global[4]=X_ukf[7];//North  vel

	 if(Gps_information.kf_init&&position_mode==MODE_GPS)
   {
		gpsx.pvt.PVT_fixtype=Gps_information.fix_type;
		gpsx.pvt.PVT_numsv=Gps_information.satellite_num;
		local_Lat= Gps_information.home_latitude;
		local_Lon= Gps_information.home_longitude;
	  CalcGlobalLocation(X_ukf[3],X_ukf[0]);
	 }
}
//-------------------------------------Height fusion-----------------------------------
double X_KF_NAVZ[3];
double P_KF_NAVZ[9]={0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001};
static float sonar_values[3] = { 0.0f };
static unsigned insert_index = 0;

static void sonar_bubble_sort(float sonar_values[], unsigned n);

static void sonar_bubble_sort(float sonar_values[], unsigned n)
{
	float t;

	for (unsigned i = 0; i < (n - 1); i++) {
		for (unsigned j = 0; j < (n - i - 1); j++) {
			if (sonar_values[j] > sonar_values[j+1]) {
				/* swap two values */
				t = sonar_values[j];
				sonar_values[j] = sonar_values[j + 1];
				sonar_values[j + 1] = t;
			}
		}
	}
}

static float insert_sonar_value_and_get_mode_value(float insert)
{
	const unsigned sonar_count = sizeof(sonar_values) / sizeof(sonar_values[0]);

	sonar_values[insert_index] = insert;
	insert_index++;
	if (insert_index == sonar_count) {
		insert_index = 0;
	}

	/* sort and return mode */

	/* copy ring buffer */
	float sonar_temp[sonar_count];
	memcpy(sonar_temp, sonar_values, sizeof(sonar_values));

	sonar_bubble_sort(sonar_temp, sonar_count);

	/* the center element represents the mode after sorting */
	return sonar_temp[sonar_count / 2];
}


float g_pos_alt_sonar=0.01;
float g_pos_alt=2;
float g_spd_alt=0.003;
double alt_delay[4]={4,4,0,5};		
void alt_fushion(float T)
{
static int kf_init;	
double A[9]=
			 {1,       0,    0,
				T,       1,    0,
				-T*T/2, -T,    1};

double B[3]={T*T/2,T,0}; 
double H[9]={
			 0,0,0,
       0,0,0,
       0,0,0}; 
	  float temp_sonar;
		#if defined(SONAR_SAMPLE1)
		temp_sonar = (float)(Moving_Median(1,10,ultra_distance))/1000;
		#elif defined(SONAR_SAMPLE2)
		temp_sonar = (float)(Moving_Median(1,5,ultra_distance))/1000;
		#elif defined(SONAR_SAMPLE3)
		temp_sonar = (float)(Moving_Median(1,10,ultra_distance))/1000;
		#endif
		#if SONAR_USE_FLOW
		temp_sonar=flow.hight.originf;
		#endif
	  if(Gps_information.kf_init&&kf_init==0){
		 kf_init=1;	
		 X_KF_NAVZ[0]=Gps_information.PVT_height;
		}
		if(Gps_information.kf_init==0&&kf_init==1)
		 kf_init=0;
		
    float oldx_sonar;
		oldx_sonar=insert_sonar_value_and_get_mode_value(temp_sonar);

		if(amf.connect)
				ultra.relative_height=amf.pos_o[2];
		else
			  ultra.relative_height=oldx_sonar;
		if( fabs(ultra.relative_height - ALT_POS_SONAR3) < 0.1 )
		{
			
			ALT_POS_SONAR3 += ( 1 / ( 1 + 1 / ( 3.6*4.0f *3.14f *T ) ) ) *(ultra.relative_height - ALT_POS_SONAR3) ;
		}
		else if( fabs(ultra.relative_height - ALT_POS_SONAR3) < 0.2 )
		{
			
			ALT_POS_SONAR3 += ( 1 / ( 1 + 1 / ( 3.6*2.2f *3.14f *T ) ) ) *(ultra.relative_height- ALT_POS_SONAR3) ;
		}
		else if( fabs(ultra.relative_height - ALT_POS_SONAR3) < 0.4 )
		{
			ALT_POS_SONAR3 += ( 1 / ( 1 + 1 / ( 3.6*1.2f *3.14f *T ) ) ) *(ultra.relative_height- ALT_POS_SONAR3) ;
		}
		else
		{
			ALT_POS_SONAR3 += ( 1 / ( 1 + 1 / ( 3.6*0.6f *3.14f *T ) ) ) *(ultra.relative_height- ALT_POS_SONAR3) ;
		}	

	  ALT_POS_SONAR2=ALT_POS_SONAR3;
 		 
		 // KF measurement flag	
		 float gh,gv;
		 //GPS
		  float posz,spdz;
			if(Gps_information.kf_init&&Gps_information.update[1]){
				Gps_information.update[1]=0;	
				H[0]=1;
				H[4]=1;  
				posz=Gps_information.PVT_height;
				spdz=Gps_information.local_spd_flt[Zr];
				gh=g_pos_alt;
				gv=g_spd_alt;
			} 
			
			if(Gps_information.kf_init==0){
				H[0]=1;
				posz=ALT_POS_SONAR2;
				gh=g_pos_alt_sonar;
				gv=g_spd_alt;
			}
			
	 float Accz=SINS_Accel_Body[Zr];	
	 double Zz_fix[3]={Gps_information.PVT_height,Gps_information.local_spd_flt[Zr],0};
   KF_OLDX_NAV( X_KF_NAVZ,  P_KF_NAVZ,  Zz_fix,  Accz, A,  B,  H,  ga_nav,  gwa_nav, gh,  gv,  T, 2, alt_delay);
	 
	 X_kf_baro[0]=X_KF_NAVZ[0];//D pos
	 X_kf_baro[1]=X_KF_NAVZ[1];//D vel
	 X_kf_baro[2]=X_KF_NAVZ[2];
}

float pos_f_uwb[3];
float gain_f_uwb=0.5;
float uwb_corr_off=-90;
float SPDYU,SPDXU; 
float Yaw_uwb;
void estimate_position_uwb(float dt)
{
	static int init;
	Yaw_uwb=To_180_degrees(Yaw-uwb_corr_off);
	float SPDY,SPDX;
	static float uwb_corr_off_local;
  if(fly_ready_rx==0)
    uwb_corr_off_local=0-Yaw_uwb;
	else
	  i_uwb_lps_tag.init=0;
	Yaw_uwb+=uwb_corr_off_local;
   if(!i_uwb_lps_tag.init&&i_uwb_lps_tag.state&&i_uwb_lps_tag.position.x!=0&&i_uwb_lps_tag.position.y!=0)
	 {
	   i_uwb_lps_tag.init=1;
	   pos_f_uwb[0]=i_uwb_lps_tag.position.x;
		 pos_f_uwb[1]=i_uwb_lps_tag.position.y;
	 }
   if(amf.connect&&1)
	 {
	  SPDY=amf.spd_body[1];
		SPDX=amf.spd_body[0];
		//conver body flow spd to NEZ
		SPDYU=SPDY*cos(Yaw_uwb*0.0173)-SPDX*sin(Yaw_uwb*0.0173);
		SPDXU=SPDY*sin(Yaw_uwb*0.0173)+SPDX*cos(Yaw_uwb*0.0173);
		pos_f_uwb[0]+=SPDXU*dt;
		pos_f_uwb[1]+=SPDYU*dt;
	 }
	 if(init&&i_uwb_lps_tag.state&&i_uwb_lps_tag.position.x!=0&&i_uwb_lps_tag.position.y!=0)
	 {
	   pos_f_uwb[0]=i_uwb_lps_tag.position.x*gain_f_uwb+(1-gain_f_uwb)*pos_f_uwb[0];
		 pos_f_uwb[1]=i_uwb_lps_tag.position.y*gain_f_uwb+(1-gain_f_uwb)*pos_f_uwb[1];
	 }
}	



float SINS_Accel_Body[3];
float SINS_Accel_Earth[2]={0,0};
void  SINS_Prepare(void)
{
      float Sin_Pitch=sin(Pitch* 0.0173);
      float Cos_Pitch=cos(Pitch* 0.0173);
      float Sin_Roll=sin(-Roll* 0.0173);
      float Cos_Roll=cos(-Roll* 0.0173);
      float Sin_Yaw=sin(Yaw* 0.0173);
      float Cos_Yaw=cos(Yaw* 0.0173);
      float Acce_Control[3];
			Acce_Control[0]=LPButterworth(imu_fushion.Acc.y,
				&Butter_Buffer[0],&Butter_30HZ_Parameter_Acce);
			Acce_Control[1]=LPButterworth(imu_fushion.Acc.x
				,&Butter_Buffer[1],&Butter_30HZ_Parameter_Acce);
			Acce_Control[2]=LPButterworth(imu_fushion.Acc.z
				,&Butter_Buffer[2],&Butter_30HZ_Parameter_Acce);

			float oAcceleration[3];
      oAcceleration[2] =
                            -Sin_Roll* Acce_Control[0]
                              + Sin_Pitch *Cos_Roll * Acce_Control[1]
                                 + Cos_Pitch * Cos_Roll *Acce_Control[2];

      oAcceleration[Xr]=
                         Cos_Yaw* Cos_Roll * Acce_Control[0]
                              +(Sin_Pitch*Sin_Roll*Cos_Yaw-Cos_Pitch * Sin_Yaw) * Acce_Control[1]
                                +(Sin_Pitch * Sin_Yaw+Cos_Pitch * Sin_Roll * Cos_Yaw)*Acce_Control[2];

      oAcceleration[Yr]=
                         Sin_Yaw* Cos_Roll * Acce_Control[0]
                              +(Sin_Pitch * Sin_Roll * Sin_Yaw +Cos_Pitch * Cos_Yaw) * Acce_Control[1]
                                + (Cos_Pitch * Sin_Roll * Sin_Yaw - Sin_Pitch * Cos_Yaw)*Acce_Control[2];
																
      oAcceleration[2]*=9.8/4096.;
      oAcceleration[2]-=9.8;//减去重力加速度

      oAcceleration[Xr]*=9.8/4096.;

      oAcceleration[Yr]*=9.8/4096.;


   /******************************************************************************/
   //将无人机在导航坐标系下的沿着正东、正北方向的运动加速度旋转到当前航向的运动加速度:机头(俯仰)+横滚

      SINS_Accel_Earth[Xr]=-oAcceleration[Xr];//沿地理坐标系，正东方向运动加速度,单位为CM
      SINS_Accel_Earth[Yr]=oAcceleration[Yr];//沿地理坐标系，正北方向运动加速度,单位为CM


      SINS_Accel_Body[Xr]=-(SINS_Accel_Earth[Xr]*Cos_Yaw+SINS_Accel_Earth[Yr]*Sin_Yaw);  //横滚正向运动加速度  X轴正向
      SINS_Accel_Body[Yr]=-(-SINS_Accel_Earth[Xr]*Sin_Yaw+SINS_Accel_Earth[Yr]*Cos_Yaw); //机头正向运动加速度  Y轴正向
      SINS_Accel_Body[Zr]= oAcceleration[2];
			
			float spd_gps[3];
//			spd_gps[Xr]=LPButterworth(Gps_information.real_E_vel
//			,&Butter_Buffer_gps[Xr],&Butter_30HZ_Parameter_Acce);
//			spd_gps[Yr]=LPButterworth(Gps_information.real_N_vel
//			,&Butter_Buffer_gps[Yr],&Butter_30HZ_Parameter_Acce);
//			spd_gps[Zr]=LPButterworth(Gps_information.real_D_vel
//			,&Butter_Buffer_gps[Zr],&Butter_30HZ_Parameter_Acce);
			spd_gps[Xr]=Gps_information.real_E_vel;
			spd_gps[Yr]=Gps_information.real_N_vel;
			spd_gps[Zr]=Gps_information.real_D_vel;
			Gps_information.local_spd_flt[Xr]=spd_gps[Xr];
			Gps_information.local_spd_flt[Yr]=spd_gps[Yr];
			Gps_information.local_spd_flt[Zr]=spd_gps[Zr];
}
