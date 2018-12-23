#ifndef _CIRCLE_H_
#define	_CIRCLE_H_

#include "stm32f4xx.h"

#define NAV_EQUATORIAL_RADIUS	(6378.137 * 1000.0)			    // meters
#define NAV_FLATTENING		(1.0 / 298.257223563)			    // WGS-84
#define NAV_E_2			(NAV_FLATTENING * (2.0 - NAV_FLATTENING))
#define M_PI			3.14159265f
#define M_PI_2			(M_PI / 2.0f)
#define NAV_HF_HOME_DIST_D_MIN	2.0f						// do not compute dynamic bearing when closer than this to home position (zero to never compute)
#define NAV_HF_HOME_DIST_FREQ	4						// update distance to home at this Hz, should be > 0 and <= 400
#define NAV_HF_HOME_BRG_D_MAX	1.0f * DEG_TO_RAD				// re-compute headfree reference angles when bearing to home changes by this many degrees (zero to always re-compute)
#define NAV_HF_DYNAMIC_DELAY	((int)3e6f)					// delay micros before entering dynamic mode after switch it toggled high
#define RAD_TO_DEG		(180.0f / M_PI)
#define DEG_TO_RAD		(M_PI / 180.0f)
#define MAX_POS_ERR 6.5 //m
 void CalcEarthRadius(double lat) ;
 void CalcGlobalDistance(double lat, double lon,double local_Lat,double local_Lon,float *posNorth,float *posEast ) ;
 void CalcGlobalLocation(float posNorth,float posEast,double local_Lat,double local_Lon,double *GPS_W_F,double* GPS_J_F);
extern float yaw_qr_off,r1,r2;
typedef struct
{ float exp;
	float now;
	float err;
	float err_old;
	float err_d;
	float err_i;
	float pid_out;
	float damp;
	float err_weight;
  u8 mode;
}_pos_control;

typedef struct
{ float f_kp;
	float kp;
	float kd;
	float ki;
  float dead;
	float flt_nav_kd;
	float flt_nav;
	float max_exp;
}_pos_pid;


#define SPD_TO_MAX_AS 		2000.0f										
#define ACC_TO_MAX_AS     1000.0f
#define MAX_CTRL_POS_SPEED 	 	1666.0f									//允许的最大控制角速度
#define MAX_CTRL_POS_ACC 	 	9800.0f									//允许的最大控制角速度
#define NAV_POS_INT        500//mm/s  
#define NAV_SPD_INT 		0.5f *MAX_CTRL_POS_SPEED		//内环积分幅度
#define NAV_ACC_INT 		0.5f *MAX_CTRL_POS_ACC		//内环积分幅度
extern float  target_position[2];
extern float  now_position[2];
extern _pos_pid nav_pos_pid;
extern _pos_pid nav_spd_pid,nav_acc_pid;
extern _pos_control nav_pos_ctrl[2];
extern _pos_control nav_spd_ctrl[2];
extern float height_off_api;    
void Positon_control1(float T);// 位置控制 
void reset_nav_pos(u8 sel);

//---------------------
typedef struct{
int x,y,z;
int spdx,spdy,spdz;
int x_flp,y_flp,center_y,center_x,pix_x,pix_y;
u8 check;
u8 connect,lose_cnt;
int control[2];
u16 r;
float control_k_miss,control_k;
float control_yaw;
float forward;
float forward_end_dj_pwm;
u8 dj_fly_line;
float pit,rol,yaw;
u8 use_spd;
float yaw_off;
float gx_vm,gy_vm,gz_vm,gyaw_vm;
float gx,gy,gz,gyaw;
float range,range_xy,range_all;
double lat,lon;
u8 use_circle;	
}CI;


typedef struct{
 int x,y;
 int x_flp,y_flp;
 u8 check;
 u8 connect,lose_cnt;
 int control[2];
 
	int pos_set[2];
	int pos_now[2];
	int angle[3];
	int Yaw_marker,center_y,center_x,pix_x,pix_y;
	float ero_m[2];
	float target_speed_m[2];
}MARKER;

extern CI circle,track,c2c;
extern MARKER marker;

//---------------------
typedef struct{
u8 connect,lose_cnt,reset;
float x,y,z;
float yaw;
}VSLAM;
extern VSLAM vslam;

 typedef struct
{
 float robot_range,forward_spd;
 

 float k_f;	
}ROBOT_LAND;
extern ROBOT_LAND robot_land;


#define MID_Y 152-40+14
#define MID_X 182+60-38
void AUTO_LAND_FLYUP(float T);
void OLDX_MISSION_API(float T);
extern float nav_land[3], nav[2];;
extern u8 state_v;
extern u8 mode_change;
#define WAY_POINT_NUM 10
extern double home_lat,home_lon,way_point[WAY_POINT_NUM][3];
extern u8 way_point_to_go;
extern u8 mission_state,mission_sel;
//state
#define SG_LOW_CHECK 0
#define SG_MID_CHECK 1
#define SU_UP1 2
#define SU_HOLD 3
#define SD_RETRY_UP 4
#define SD_RETRY_UP_HOLD 5
#define TRACK_FAR 6
#define TRACK_CLOSE 7
#define TRACL_PLANNER 8

#define SD_HOLD 13
#define SD_MISS_SEARCH 14
#define SD_HOLD1 15
#define SD_HIGH_FAST_DOWN 16
#define SD_CIRCLE_SLOW_DOWN 17
#define SD_CIRCLE_HOLD 18
#define SD_CIRCLE_MID_DOWN  19
#define SD_CHECK_G 20
#define SD_SHUT_DOWN 21
#define SD_SAFE 22
#define SD_HOME 23
#define SD_WAY_POINT 24
#define SD_WAY_POINT_SINGLE 25
#define SU_MISSION 66
//------------parameter
#define DEAD_NAV_RC 50
#define AUTO_FLY_SPD_Z 0.4 //m/s
#define AUTO_UP_POS_Z 1  //m
#define AUTO_DOWN_POS_Z LIMIT(1.5,0.25,2) //m
#define AUTO_DOWN_SPD_Z 0.4 //m/s
#define GROUND_SPEED_CHECK 0.3  //m/s
#define SONAR_HEIGHT_SET 0.1 //m

#define WAY_POINT_DEAD 0.2 //m
#define MISSION_CHECK_TIME 0.35//s
#define AUTO_HOME_POS_Z 4 //m
//------------
#define SMART_MODE_POS 3
#define SMART_MODE_SPD 2
#define SMART_MODE_RC 1
#define SMART_MODE_SPD_RATE 4


void Nav_pos_set_test(u8 mode_in,float T);
#endif