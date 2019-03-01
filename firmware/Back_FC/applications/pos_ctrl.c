#include "include.h"
#include "rc.h"
#include "imu.h"
#include "ctrl.h"
#include "filter.h"
#include "alt_fushion.h"
#include "eso.h"
#include "m100.h"
#include "quar.h"
#include "oldx_api.h"
#include "oldx_mission.h"
#include "mavl.h"
float r1,r2;
double home_lat=39.9626431,home_lon=116.3040791;
double way_point[WAY_POINT_NUM][3];
double way_point_use[3]={39.9626433,116.3040790,3};
u8 way_point_to_go;
u8 home_set=0;
u8 mission_state=0;
u8 mission_sel=0;
#define WAY_POINT_TEST_NUM 2
double way_point_test1[WAY_POINT_TEST_NUM][3]={39.9626433,116.3040790,3, 
																							 39.9626433,116.3040790,3};
																		 
void way_point_init(u8 sel)
{
	u8 i;
	way_point_to_go=WAY_POINT_TEST_NUM;
  for(i=0;i<WAY_POINT_TEST_NUM;i++)
   {
	   way_point[i][0]=way_point_test1[i][0];
	   way_point[i][1]=way_point_test1[i][1];
	   way_point[i][2]=way_point_test1[i][2];
	 }
}

void set_way_point_single(double lat,double lon,float height)
{
 if(ABS(lat)>5&&ABS(lon)>5&&height>0) 
 {	
 way_point_use[0]=lat;
 way_point_use[1]=lon;
 way_point_use[2]=height;
 }
}

//----------------------------
void Set_home(void)
{
#if PX4_SDK	
 if(px4.connect&&px4.GPS_STATUS>=3&&ABS(m100.Lat)>5&&ABS(m100.Lon)>5)
 { home_set=1;
   way_point_use[0]=home_lat=m100.Lat;
	 way_point_use[1]=home_lon=m100.Lon;
	 way_point_use[2]=2;
 }
#else 
 if(m100.GPS_STATUS>=6&&m100.STATUS>=2&&ABS(m100.Lat)>5&&ABS(m100.Lon)>5)
 { home_set=1;
   way_point_use[0]=home_lat=m100.Lat;
	 way_point_use[1]=home_lon=m100.Lon;
	 way_point_use[2]=ALT_POS_BMP_UKF_OLDX;
 }
#endif
}

void Way_point_push(void)
{u8 i;
	
#if PX4_SDK	
 if(px4.connect&&px4.GPS_STATUS>=3&&px4.Lat!=0&&px4.Lon!=0)
 { 
	 
 if(way_point[0][0]>20&&way_point[0][1]>20)
	{ 
		way_point_use[0]=way_point[0][0];
		way_point_use[1]=way_point[0][1];
		way_point_use[2]=way_point[0][2];
	}
		
	for(i=0;i<WAY_POINT_NUM-1;i++){ 
	 way_point[i][0]=way_point[i+1][0];
	 way_point[i][1]=way_point[i+1][1];
	 way_point[i][2]=way_point[i+1][2];
	}
	way_point_to_go--; 
	
 }
#else 
 if(m100.GPS_STATUS>=6&&m100.STATUS>=2&&ABS(m100.Lat)>5&&ABS(m100.Lon)>5)
 { 
	for(i=0;i<WAY_POINT_NUM;i++){ 
	 way_point[i][0]=way_point[i+1][0];
	 way_point[i][1]=way_point[i+1][1];
	 way_point[i][2]=way_point[i+1][2];
	}
	way_point_to_go--; 
 }
#endif 
}

void CalcEarthRadius(double lat) {
    double sinLat2;

    sinLat2 = sin(lat * (double)DEG_TO_RAD);
    sinLat2 = sinLat2 * sinLat2;

    r1 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD * ((double)1.0 - (double)NAV_E_2) / pow((double)1.0 - ((double)NAV_E_2 * sinLat2), ((double)3.0 / (double)2.0));
    r2 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD / sqrt((double)1.0 - ((double)NAV_E_2 * sinLat2)) * cos(lat * (double)DEG_TO_RAD);
}

 void CalcGlobalDistance(double lat, double lon,double local_Lat,double local_Lon,float *posNorth,float *posEast ) {
    *posNorth = (lat - local_Lat) * r1;
    *posEast =  (lon - local_Lon) * r2;
}


 void CalcGlobalLocation(float posNorth,float posEast,double local_Lat,double local_Lon,double *GPS_W_F,double* GPS_J_F){ 
    *GPS_W_F=(double)posNorth/(double)(r1+0.1)+local_Lat;
    *GPS_J_F=(double)posEast/(double)(r2+0.1)+local_Lon;
}

// input lat/lon in degrees, returns distance in meters
float navCalcDistance(double lat1, double lon1, double lat2, double lon2) {
    float n = (lat1 - lat2) * r1;
    float e = (lon1 - lon2) * r2;
    return __sqrtf(n*n + e*e);
}

// input lat/lon in degrees, returns bearing in radians
float navCalcBearing(double lat1, double lon1, double lat2, double lon2) {
    float n = (float)((lat2 - lat1) * (double)DEG_TO_RAD * r1);
    float e = (float)((lon2 - lon1) * (double)DEG_TO_RAD * r2);
    float ret = atan2f(e, n);

    if (!isfinite(ret))
        ret = 0.0f;

    return ret;
}


CI circle,track,c2c;
MARKER marker;
float nav_circle[2];
/*
        y add (0~256)  1
        |
        |


        -------- x decrease (0~320)  0


*/
float nav[2];
float  target_position[2];
float  now_position[2];

#define NAV_ANGLE_MAX 0.8*MAX_CTRL_ANGLE
#define Mark_Size 0.18
#define Mark_Dis  0.36
#define NUM_HAN 10
#define NUM_LIE 10
//                                    r  an_now   dangle
float exp_center_cycle[5]={1.50,1.50,0.6,0,20};
u8 line_test_num[2]={12,34};
u8 tan_test_num[4]={12,82,88,18};
void Nav_pos_set_test(u8 mode_in,float T)//轨迹规划
{
u8 i;	
static u8 state_plan;
static u16 way_cnt;
static float init_pos[3],time_now;	
static u8 mode_circle_reg,line_flag=0,tangle_flag=0;
float exp_ling[2][2];
float exp_tan[4][2];	
if(mode_oldx.flow_hold_position==0||(
mode_oldx.flow_hold_position==2&&(fabs(CH_filter[ROL])>25||fabs(CH_filter[PIT])>25)))//(mode_in==2&&mode_circle_reg!=2)||mode_oldx.flow_hold_position<=1)
{
 init_pos[Xr]=POS_UKF_X;
 init_pos[Yr]=POS_UKF_Y; 
 init_pos[Zr]=ALT_POS_BMP_UKF_OLDX;
 exp_center_cycle[3]=0;
 state_plan=0;
}
mode_circle_reg=mode_in;	

switch(mode_in){
	case 2://cycle
		  exp_center_cycle[3]+=T*exp_center_cycle[4];
			if(exp_center_cycle[3]<0)exp_center_cycle[3]=360;
			if(exp_center_cycle[3]>360)exp_center_cycle[3]=0;
	        nav_pos_ctrl[X].exp=cos(exp_center_cycle[3]*0.0173)*exp_center_cycle[2]-exp_center_cycle[2]*0+init_pos[X];
					nav_pos_ctrl[Y].exp=sin(exp_center_cycle[3]*0.0173)*exp_center_cycle[2]-exp_center_cycle[2]*0+init_pos[Y];
	break;
	case 1://line
  	for(i=0;i<2;i++)
 {
	  exp_tan[i][X] = (line_test_num[i]%NUM_LIE-1+(line_test_num[i]%NUM_LIE==0?NUM_LIE:0))*Mark_Dis;
	  exp_tan[i][Y] = -(line_test_num[i]/NUM_LIE-(line_test_num[i]%NUM_LIE==0?1:0))*Mark_Dis; 
 }	
	
	nav_pos_ctrl[X].exp=exp_tan[tangle_flag][X];
	nav_pos_ctrl[Y].exp=exp_tan[tangle_flag][Y];
    
	if(fabs(nav_pos_ctrl[X].exp - nav_pos_ctrl[X].now)<WAY_POINT_DEAD&&fabs(nav_pos_ctrl[Y].exp - nav_pos_ctrl[Y].now)<WAY_POINT_DEAD1)
	 { if(tangle_flag<1)tangle_flag+=1;
	   else tangle_flag=0;}
	
	break;
	case 3://way_point
	for(i=0;i<4;i++)
 {
	  exp_tan[i][X] = (tan_test_num[i]%NUM_LIE-1+(tan_test_num[i]%NUM_LIE==0?NUM_LIE:0))*Mark_Dis+init_pos[X];
	  exp_tan[i][Y] = -(tan_test_num[i]/NUM_LIE-(tan_test_num[i]%NUM_LIE==0?1:0))*Mark_Dis+init_pos[Y]; 
 }	
	
	nav_pos_ctrl[X].exp=exp_tan[tangle_flag][X];
	nav_pos_ctrl[Y].exp=exp_tan[tangle_flag][Y];
    
	if(fabs(nav_pos_ctrl[X].exp - nav_pos_ctrl[X].now)<WAY_POINT_DEAD1&&fabs(nav_pos_ctrl[Y].exp - nav_pos_ctrl[Y].now)<WAY_POINT_DEAD1)
	 { 
	   way_cnt++;
	 }else
	 way_cnt=0;
	 if(way_cnt>0.15/T){way_cnt=0;
	 if(tangle_flag<3)tangle_flag+=1;
	   else tangle_flag=0;
	 }
	break;
	case 4://traj_plan_land
			switch(state_plan){
			case 0:			
			traj[0].ps[Xr]=POS_UKF_X;//m
			traj[0].ps[Yr]=POS_UKF_Y;//m
			traj[0].ps[Zr]=ALT_POS_BMP_UKF_OLDX;//m
			traj[0].ps[Xr]=VEL_UKF_X;//m
			traj[0].ps[Yr]=VEL_UKF_Y;//m
			traj[0].ps[Zr]=ALT_VEL_BMP_UKF_OLDX;//m
			traj[0].defined[0]=1;traj[0].defined[1]=1;traj[0].defined[2]=0;
			traj[0].Time=5;//s
			traj[0].pe[Xr]=init_pos[X];//m
			traj[0].pe[Yr]=init_pos[Y]+2;//m
			traj[0].pe[Zr]=0.2;//m

			plan_tra(&traj[0]);
			state_plan=1;
			time_now=0;
			break;
			case 1:
			time_now+=T;
			get_tra(&traj[0],time_now);
			nav_pos_ctrl[Xr].exp=traj[0].pt[Xr];
			nav_pos_ctrl[Yr].exp=traj[0].pt[Yr];
			exp_height=traj[0].pt[Zr];
			if(time_now>traj[0].Time)
				state_plan=2;
			break;
			case 2:
				nav_pos_ctrl[Xr].exp=init_pos[Xr];
				nav_pos_ctrl[Yr].exp=init_pos[Yr];
				exp_height=init_pos[Zr];
			if(fabs(init_pos[Xr] - nav_pos_ctrl[Xr].now)<WAY_POINT_DEAD1
				&&fabs(init_pos[Yr] - nav_pos_ctrl[Yr].now)<WAY_POINT_DEAD1)
				state_plan=0;		 
			break;
			} 
	break;
		case 5://follow car directly
       if(car_pos[0]!=0&&car_pos[1]!=0)
			 {
			   nav_pos_ctrl[Xr].exp=car_pos[0];
	       nav_pos_ctrl[Yr].exp=car_pos[1];
			 }
    break;		
	default:
	break;

}
}
#define FORCE_SPEED_MODE 0
float yaw_qr_off;
float out_timer_nav,in_timer_nav;
float acc_temp[3];
_pos_pid nav_pos_pid;
_pos_pid nav_spd_pid;
_pos_pid nav_acc_pid;
_pos_control nav_pos_ctrl[2];
_pos_control nav_spd_ctrl[2];
_pos_control nav_acc_ctrl[2];



double sigmoidFunction(float x,float max, float off, float s)
    {
        double ex;
        ex = pow(2.718281828,s*x-off);
        return LIMIT(max/(1+ex),-ABS(max),ABS(max));
    }

		
void reset_nav_pos(u8 sel)
{
	if(sel==Y)	
	nav_pos_ctrl[Y].exp=POS_UKF_Y;//mm
	if(sel==X)
	nav_pos_ctrl[X].exp=POS_UKF_X;//mm
}

float nav_spd_pid_flt_nav_rc;
float off_GPS[2]={0,0};//{0.0012,0.0061};

void Positon_control1(float T)// 位置控制 Drone 330
{ u8 i;
	static u8 is_hover=1;
	static u8 cnt[2],init;
	static float spd_reg[2],stop_cnt,stop_exp_spd[2];
	if(!init){init=1;
		//pos
		nav_pos_ctrl[X].mode=2;
		nav_pos_pid.kp=0.36;//3;
		nav_pos_pid.ki=0.0500;
		nav_pos_pid.kd=0.0;
		nav_pos_pid.dead=0.01;
		//adrc
		eso_pos[X].b0=eso_pos[Y].b0=0;//4.5;
		eso_pos[X].err_limit=eso_pos[Y].err_limit=3500;
		eso_pos[X].eso_dead=eso_pos[Y].eso_dead=nav_pos_pid.dead*1000;
	  //spd	
		nav_spd_pid.f_kp=0.2;
		#if defined(HANX6_BIG)
		nav_spd_pid.kp=0.1086;//0.33;//
		#elif defined(HANX6)
		nav_spd_pid.kp=0.17;//0.33;//
		#elif defined(HANX6_BIGX6)
		nav_spd_pid.kp=0.17;//0.33;//
		#else
		nav_spd_pid.kp=0.33;//
		#endif		
		nav_spd_pid.ki=0.000;//0.10;//<------------
		nav_spd_pid.kd=0;
		nav_spd_pid.max_exp=0;
		#if USE_BLDC
			nav_spd_pid.flt_nav_kd=1.0;
		#else
			#if MINI_X4
      nav_spd_pid.flt_nav_kd=1;
			#else
			nav_spd_pid.flt_nav_kd=1;
			#endif
		#endif
		#if defined(HANX6_BIG)
		nav_spd_pid.dead=0.0051*1000;
		#else
		nav_spd_pid.dead=0.005*1000;
		#endif
		//adrc
		eso_pos_spd[X].b0=eso_pos_spd[Y].b0=0;
		eso_pos_spd[X].err_limit=eso_pos_spd[Y].err_limit=MAX_CTRL_POS_SPEED*2;
		eso_pos_spd[X].eso_dead=eso_pos_spd[Y].eso_dead=nav_spd_pid.dead;	
		nav_spd_pid.flt_nav=1;//0.356;//决定刹车手感
		nav_spd_pid_flt_nav_rc=0.68;
		//acc
		nav_acc_pid.f_kp=0.15;
		nav_acc_pid.kp=0.16;//0.15;
		nav_acc_pid.ki=0.00;
		nav_acc_pid.kd=0.0;	
		nav_acc_pid.dead=0.04*1000;
		nav_acc_pid.flt_nav_kd=15;
	}
	static u8 pos_reset_state;
	static u16 pos_reset_cnt;

	u8 flag_temp=0;
	#if defined(POS_TEST)
	flag_temp=1;
	#endif
	stop_exp_spd[Y]=stop_exp_spd[X]=0;
	if(((NS==0&&mode_oldx.rc_loss_return_home==0)||mode_oldx.rc_control_flow_pos_sel||Thr_Low)&&flag_temp&&mode_oldx.flow_hold_position==2
		&&!mode_oldx.rc_loss_return_home)
	Nav_pos_set_test(mode_oldx.rc_control_flow_pos_sel,T);
	else{ 	//smooth for pos control   S函数
	for(i=0;i<2;i++){
			switch(pos_reset_state)
			{
				case 0:
				 if(fabs(CH_filter[ROL])>25||fabs(CH_filter[PIT])>25)
				 {pos_reset_state=1;
					pos_reset_cnt=0;
				  reset_nav_pos(X);reset_nav_pos(Y);
				 }
				break;
				case 1:
				 reset_nav_pos(X);reset_nav_pos(Y);
				 spd_reg[X]=VEL_UKF_X;spd_reg[Y]=VEL_UKF_Y;
				 if(fabs(CH_filter[ROL])<25&&fabs(CH_filter[PIT])<25)
				   pos_reset_cnt++;
				 else
					 pos_reset_cnt=0;
				 
				 if(pos_reset_cnt>0.25/T){
					 pos_reset_cnt=stop_cnt=0;
					 pos_reset_state=2;
				 }					 
				break;
				case 2:
				  stop_cnt+=T;
				  reset_nav_pos(X);reset_nav_pos(Y);
          stop_exp_spd[X]=sigmoidFunction(stop_cnt,LIMIT(spd_reg[X],-2,2),3.86,2.68);
          stop_exp_spd[Y]=sigmoidFunction(stop_cnt,LIMIT(spd_reg[Y],-2,2),3.86,2.68);				
				  if(stop_cnt>2.86||(ABS(stop_exp_spd[X])<0.01&&ABS(stop_exp_spd[Y])<0.01))//3.68)
						 stop_cnt=pos_reset_state=spd_reg[X]=spd_reg[Y]=0;  
				break;
			}	
  }
	
	
	if(((!fly_ready||Thr_Low)&&NS==2))
	{ 
		reset_nav_pos(Y);reset_nav_pos(X);
	  nav_pos_ctrl[X].err_i=nav_pos_ctrl[Y].err_i=nav_acc_ctrl[X].err_i=nav_acc_ctrl[Y].err_i=nav_spd_ctrl[X].err_i=nav_spd_ctrl[Y].err_i=0;
	}
 }
	
	static u8 mode_flow_hold_position_reg;
	if(mode_flow_hold_position_reg!=mode_oldx.flow_hold_position)
	{reset_nav_pos(Y); reset_nav_pos(X);}
	mode_flow_hold_position_reg=mode_oldx.flow_hold_position;
	
	 if(m100.STATUS>1&&m100.STATUS!=99&&ABS(m100.Init_Lat)>5&&ABS(m100.Init_Lon)>5)
		CalcGlobalLocation(POS_UKF_Y,POS_UKF_X,m100.Init_Lat+off_GPS[0],m100.Init_Lon+off_GPS[1],&m100.Lat,&m100.Lon);
	
/*
		north    LAT=1     V_West+                             __________
		|   Y+  y                                              P- R- GPS-
		                                        
		|P+
	
    _____	R	+   x         LON=0  V_East+
	

	   
head  |    1 PIT y-   90d in marker
			| 
		   _____  0 ROL x+

*/
  float pos[2];
	int spd[2],acc[2];
	float a_br[3];	
	static float acc_flt[2];
	a_br[0] =(float) mpu6050_fc.Acc.x/4096.;//16438.;
	a_br[1] =(float) mpu6050_fc.Acc.y/4096.;//16438.;
	a_br[2] =(float) mpu6050_fc.Acc.z/4096.;//16438.;
	acc_temp[0] = a_br[1]*reference_vr_imd_down_fc[2]  - a_br[2]*reference_vr_imd_down_fc[1] ;
	acc_temp[1] = a_br[2]*reference_vr_imd_down_fc[0]  - a_br[0]*reference_vr_imd_down_fc[2] ;

	acc_flt[0] += ( 1 / ( 1 + 1 / ( 10 *3.14f *T ) ) ) *my_deathzoom1( (firstOrderFilter(-acc_temp[0] ,&firstOrderFilters[ACC_LOWPASS_X],T) - acc_flt[0]),0 );
	acc_flt[1] += ( 1 / ( 1 + 1 / ( 10 *3.14f *T ) ) ) *my_deathzoom1( (firstOrderFilter(-acc_temp[1] ,&firstOrderFilters[ACC_LOWPASS_Y],T) - acc_flt[1]),0 );

//输入数据	
  static u8 pos_use_sel;
	pos[Y]=POS_UKF_Y;//m
  pos[X]=POS_UKF_X;//m
	spd[Y]=VEL_UKF_Y*1000;
	spd[X]=VEL_UKF_X*1000;	
	acc_body[Y]=acc[Y]=acc_flt[Y]*9800;
  acc_body[X]=acc[X]=acc_flt[X]*9800;
		
//位置
	//smart
	if((smart.rc.POS_MODE==SMART_MODE_POS&&smart.pos.x!=0)||(smart.rc.POS_MODE==SMART_MODE_RC&&ABS(smart.rc.PITCH-1500)>25)||(smart.rc.POS_MODE==SMART_MODE_RC&&ABS(smart.rc.ROLL-1500)>25))
	nav_pos_ctrl[X].exp=smart.pos.x;
	if((smart.rc.POS_MODE==SMART_MODE_POS&&smart.pos.y!=0)||(smart.rc.POS_MODE==SMART_MODE_RC&&ABS(smart.rc.PITCH-1500)>25)||(smart.rc.POS_MODE==SMART_MODE_RC&&ABS(smart.rc.ROLL-1500)>25))
	nav_pos_ctrl[Y].exp=smart.pos.y;
	
	if(cnt[0]++>1){cnt[0]=0;
	float temp1;
  temp1=(float)Get_Cycle_T(GET_T_OUT_NAV)/1000000.;
	
	if(temp1>0.001)
		out_timer_nav=temp1;
	else
		out_timer_nav=0.02;
	float dis[3],k_size[3];
	for (i=0;i<2;i++)
		dis[i]=ABS(nav_pos_ctrl[i].now-nav_pos_ctrl[i].exp);
	k_size[0]=LIMIT(dis[0]/(dis[1]+dis[0]+0.001),0,1);
	k_size[1]=1-k_size[0];
	if(ABS(dis[0])<2.5)
		k_size[0]=1;
	if(ABS(dis[1])<2.5)
		k_size[1]=1;
	for (i=0;i<2;i++){ 
		nav_pos_ctrl[i].now=pos[i];
	  //OLDX_POS_CONTROL_ESO(&eso_pos[i],nav_pos_ctrl[i].exp*1000,nav_pos_ctrl[i].now*1000,eso_pos[i].u,out_timer_nav,1.5*1000,nav_pos_pid.kp,thr_view );//速度环自抗扰控制	
		nav_pos_ctrl[i].exp=LIMIT(nav_pos_ctrl[i].exp,nav_pos_ctrl[i].now-MAX_POS_ERR*k_size[i],nav_pos_ctrl[i].now+MAX_POS_ERR*k_size[i]);
		
		nav_pos_ctrl[i].err = ( nav_pos_pid.kp*LIMIT(my_deathzoom1(nav_pos_ctrl[i].exp - nav_pos_ctrl[i].now,nav_pos_pid.dead),-MAX_POS_ERR,MAX_POS_ERR)*1000 );
    if(eso_pos[i].b0==0&&nav_pos_pid.ki>0){
		nav_pos_ctrl[i].err_i += nav_pos_pid.ki *nav_pos_ctrl[i].err *out_timer_nav;
		nav_pos_ctrl[i].err_i = LIMIT(nav_pos_ctrl[i].err_i,-Thr_Weight *NAV_POS_INT,Thr_Weight *NAV_POS_INT);
		}
		else
		nav_pos_ctrl[i].err_i=0;
		nav_pos_ctrl[i].err_d =  nav_pos_pid.kd *( 0.6f *(-(float)spd[i]*out_timer_nav) + 0.4f *(nav_pos_ctrl[i].err - nav_pos_ctrl[i].err_old) );
		if(eso_pos[i].b0==0)
		nav_pos_ctrl[i].pid_out = nav_pos_ctrl[i].err +nav_pos_ctrl[i].err_i + nav_pos_ctrl[i].err_d;
		else
	  nav_pos_ctrl[i].pid_out = nav_pos_ctrl[i].err_d +eso_pos[i].u;
    if(nav_spd_pid.max_exp!=0)
			nav_pos_ctrl[i].pid_out = LIMIT(nav_pos_ctrl[i].pid_out,-nav_spd_pid.max_exp*1000,nav_spd_pid.max_exp*1000);//m/s		
		else	
			nav_pos_ctrl[i].pid_out = LIMIT(nav_pos_ctrl[i].pid_out,-1.5*1000,1.5*1000);//m/s
		nav_pos_ctrl[i].err_old = nav_pos_ctrl[i].err;
		}
	}
	
	if(circle.check&&circle.connect)
	yaw_qr_off=circle.yaw-Yaw_fc;
  else if(circle.connect==0)
	yaw_qr_off=0;

	float Yaw_qr=To_180_degrees(Yaw_fc+yaw_qr_off);
	#if defined(USE_UWB)
	if(m100.uwb_f[0]!=0)
		 Yaw_qr=m100.uwb_f[3];
	#endif
	float temp_pos_out[2];
	temp_pos_out[Y]= nav_pos_ctrl[North].pid_out*cos(Yaw_qr*0.0173)+nav_pos_ctrl[East].pid_out*sin(Yaw_qr*0.0173); 
	temp_pos_out[X]=-nav_pos_ctrl[North].pid_out*sin(Yaw_qr*0.0173)+nav_pos_ctrl[East].pid_out*cos(Yaw_qr*0.0173);

 if((smart.rc.POS_MODE==SMART_MODE_SPD||smart.rc.POS_MODE==SMART_MODE_SPD_RATE))//only for smart_spd
	 {
		 if(smart.spd.y==0)
		 nav_spd_ctrl[Y].exp=temp_pos_out[Y]; 
		 else
		 {	nav_spd_ctrl[Y].exp= smart.spd.y*1000;reset_nav_pos(Y);reset_nav_pos(X);}
		 if(smart.spd.x==0)
		 nav_spd_ctrl[X].exp=temp_pos_out[X];
			else
		{   nav_spd_ctrl[X].exp= smart.spd.x*1000;reset_nav_pos(X);reset_nav_pos(Y);}
	 } 
	 else
	 {
	 nav_spd_ctrl[Y].exp=temp_pos_out[Y];
	 nav_spd_ctrl[X].exp=temp_pos_out[X];
	 } 
	
  if(mode_oldx.flow_hold_position!=2){
	 nav_spd_ctrl[Y].exp*=0.0;
	 nav_spd_ctrl[X].exp*=0.0;}
	
//----------------------------------速度阶越测试
	static u8 state_tune_spd;
	static u8 flag_way;
	static u16 cnt_s1;
	switch(state_tune_spd){
	case 0:	
	if(mode_oldx.trig_flow_spd)
	{state_tune_spd=1;cnt_s1=0;flag_way=!flag_way;}
	break;
	case 1:
	if(mode_oldx.trig_flow_spd)
	{	if(flag_way)
	nav_spd_ctrl[X].exp=0.35*1000;//mm
	else
	nav_spd_ctrl[X].exp=-0.35*1000;		
	}
	else
	state_tune_spd=0;	
	if(cnt_s1++>2.25/T)
	{cnt_s1=0;state_tune_spd=2;}
	break;
	case 2:
	nav_spd_ctrl[X].exp=0;			
	if(cnt_s1++>2.25/T)	
	state_tune_spd=0;
	if(!mode_oldx.trig_flow_spd)
	state_tune_spd=0;
	break;
	}
//--------------------------------------------------速度环-----------------------------------------------	

	static float spd_old[2],acc_old[2];
	float temp;
  temp=(float)Get_Cycle_T(GET_T_IN_NAV)/1000000.;
	
	if(temp>0.001)
		in_timer_nav=temp;
	else
		in_timer_nav=0.01;
	static float nav_spd_ctrl_reg[2];
  #if FORCE_SPEED_MODE
		nav_spd_ctrl[X].exp=nav_spd_ctrl[Y].exp=0;//force spd control
	#endif
	//smooth
  if(ABS(stop_exp_spd[X])>0.01)
		nav_spd_ctrl[X].exp=stop_exp_spd[X];
	if(ABS(stop_exp_spd[Y])>0.01)
		nav_spd_ctrl[Y].exp=stop_exp_spd[Y];
	 
	for (i=0;i<2;i++){	
	nav_spd_ctrl[i].now=(spd[i]);
	if(nav_spd_pid.max_exp!=0)
	nav_spd_ctrl[i].exp = LIMIT(nav_spd_ctrl[i].exp, -nav_spd_pid.max_exp*1000,nav_spd_pid.max_exp*1000 );	
	else
	nav_spd_ctrl[i].exp = LIMIT(nav_spd_ctrl[i].exp, -MAX_CTRL_POS_SPEED,MAX_CTRL_POS_SPEED );
		
  //OLDX_POS_CONTROL_ESO(&eso_pos_spd[i],nav_spd_ctrl[i].exp,nav_spd_ctrl[i].now,eso_pos_spd[i].u,in_timer_nav,250,nav_spd_pid.kp,thr_view );//速度环自抗扰控制		
	
	nav_spd_ctrl[i].err =  nav_spd_pid.kp*my_deathzoom1( nav_spd_ctrl[i].exp - nav_spd_ctrl[i].now ,nav_spd_pid.dead);
	nav_spd_ctrl[i].err_weight = (float)ABS(nav_spd_ctrl[i].err)/MAX_CTRL_POS_SPEED;
		
	if(!mode_oldx.test4)
		nav_spd_ctrl[i].err_d = 0.01f/in_timer_nav *10*nav_spd_pid.kd * 
		((nav_spd_pid.flt_nav_kd)*-my_deathzoom1( (LIMIT(-acc[i],-9800*0.8,9800*0.8) ) ,25) 
		+(1-nav_spd_pid.flt_nav_kd)*(nav_spd_ctrl[i].err - nav_spd_ctrl[i].err_old))*in_timer_nav;
	else 
		nav_spd_ctrl[i].err_d=0;
	
	if((fabs(nav_spd_ctrl[i].err)<eso_pos_spd[i].eso_dead||eso_pos_spd[i].b0==0)&&nav_spd_pid.ki>0){
	nav_spd_ctrl[i].err_i += nav_spd_pid.ki  *(nav_spd_ctrl[i].exp - nav_spd_ctrl[i].now) *in_timer_nav;
	nav_spd_ctrl[i].err_i = LIMIT( nav_spd_ctrl[i].err_i, -NAV_SPD_INT,NAV_SPD_INT );
	}
	else
	nav_spd_ctrl[i].err_i=0;	

	if(eso_pos_spd[i].b0==0){
	nav_spd_ctrl[i].pid_out  =  ( nav_spd_pid.f_kp *LIMIT((0.45f + 0.55f*nav_spd_ctrl[i].err_weight),0,1)*nav_spd_ctrl[i].exp + 
	(1 - nav_spd_pid.f_kp ) *( nav_spd_ctrl[i].err + nav_spd_ctrl[i].err_d + nav_spd_ctrl[i].err_i ) );}
	else
	{
	nav_spd_ctrl[i].pid_out  =  ( nav_spd_pid.f_kp *LIMIT((0.45f + 0.55f*nav_spd_ctrl[i].err_weight),0,1)*nav_spd_ctrl[i].exp + 
	(1 - nav_spd_pid.f_kp ) *( nav_spd_ctrl[i].err_d +eso_pos_spd[i].u) );}
	
  if(mode_oldx.test4)//acc
	nav_spd_ctrl[i].pid_out =(float)(nav_spd_ctrl[i].pid_out);	\
  else
	nav_spd_ctrl[i].pid_out =(float)LIMIT((nav_spd_ctrl[i].pid_out),-250,250)/10.;	\
	nav_spd_ctrl[i].err_old= nav_spd_ctrl[i].err;
	spd_old[i] = nav_spd_ctrl[i].now  ;
	
	
//---------------------------------------------加速度环---------------------------------------------	
		if(mode_oldx.test4)
		{ 
			nav_acc_ctrl[i].now=acc_body[i];
			nav_acc_ctrl[i].exp=nav_spd_ctrl[i].pid_out*10*(8./12.);
			nav_acc_ctrl[i].exp = LIMIT(nav_acc_ctrl[i].exp, -MAX_CTRL_POS_ACC,MAX_CTRL_POS_ACC );
			nav_acc_ctrl[i].damp = ( nav_acc_ctrl[i].now - acc_old[i]) *( 0.02f/in_timer_nav );
			nav_acc_ctrl[i].err =  my_deathzoom1(( nav_acc_ctrl[i].exp - nav_acc_ctrl[i].now ),nav_acc_pid.dead) *(1000.0f/MAX_CTRL_POS_ACC);
			nav_acc_ctrl[i].err_d = ( nav_acc_pid.kd  *( -10 *nav_acc_ctrl[i].damp) *( 0.02f/in_timer_nav ) );
			if((fabs(nav_acc_ctrl[i].err)<eso_pos_spd[i].eso_dead||eso_pos_spd[i].b0==0)&&nav_acc_pid.ki>0){
			nav_acc_ctrl[i].err_i += nav_acc_pid.ki  *(nav_acc_ctrl[i].err) *in_timer_nav;
			nav_acc_ctrl[i].err_i = LIMIT( nav_acc_ctrl[i].err_i, -NAV_ACC_INT,NAV_ACC_INT );
			}
			else
			nav_acc_ctrl[i].err_i=0;	
			
			if(eso_pos_spd[i].b0==0||1){
			nav_acc_ctrl[i].pid_out  =  ( nav_acc_pid.f_kp*nav_acc_ctrl[i].exp + 
			(1 - nav_acc_pid.f_kp ) *nav_acc_pid.kp  *( nav_acc_ctrl[i].err + nav_acc_ctrl[i].err_d + nav_acc_ctrl[i].err_i ) );}
			else
			{	nav_acc_ctrl[i].pid_out  =  (nav_acc_pid.f_kp*nav_acc_ctrl[i].exp + 
			(1 - nav_acc_pid.f_kp ) *(  nav_acc_pid.kp  *nav_acc_ctrl[i].err_d +eso_pos_spd[i].u) );}

			nav_acc_ctrl[i].pid_out =(float)LIMIT((float)(nav_acc_ctrl[i].pid_out)/10.,-25,25);	
			nav_acc_ctrl[i].err_old= nav_acc_ctrl[i].err;
			acc_old[i] = nav_acc_ctrl[i].now  ;
		}
  }
		
	if(mode_oldx.test4){
  nav[PITr]=nav_spd_pid.flt_nav*nav_acc_ctrl[Y].pid_out+(1-nav_spd_pid.flt_nav)*nav[PITr];
	nav[ROLr]=nav_spd_pid.flt_nav*nav_acc_ctrl[X].pid_out+(1-nav_spd_pid.flt_nav)*nav[ROLr];
	}
	else
	{
  nav[PITr]=nav_spd_pid.flt_nav*nav_spd_ctrl[Y].pid_out+(1-nav_spd_pid.flt_nav)*nav[PITr];
	nav[ROLr]=nav_spd_pid.flt_nav*nav_spd_ctrl[X].pid_out+(1-nav_spd_pid.flt_nav)*nav[ROLr];
	}	
	// rc_first_filter(nav_acc_ctrl[Y].pid_out, &nav[PITr],30,in_timer_nav);
	// rc_first_filter(nav_acc_ctrl[X].pid_out, &nav[ROLr],30,in_timer_nav);
}

//--------------------------------------自动起飞降落 视觉导航状态机
u8 mode_change;
u8 state_v=0;
u8 force_pass;
float height_off_api=0;
u8 test_mission=0;
float pan_set=0;
float tar_Yaw1;
u8 state_home;
void OLDX_MISSION_API(float T)
{ 
	static u8 init;
	static u8 cnt[10];
	static u8 state_buf=0;
	static u8 mode_reg[3],rc_middle_reg[4];
	static u8 mission_sel_lock;
	static u16 AUX2_REG,fly_readyr,NS_REG;
	if(!init)
	{
	target_track_downward_task(&c2c,0.25*0,0.1,MODE_SPD,T);
	est_target_state(&c2c, &three_wheel_car,T);
	target_track_pan_task(&c2c,0.2,1,1,0,T);
	}
	u8 rc_middle[4]={0};
	u8 mission_flag=0;
	u8 px4_mode_position=0;
//------------------------------state switch---------------------------------	
	smart.rc.ATT_MODE=smart.rc.HEIGHT_MODE=smart.rc.POS_MODE=0;
	smart.pos.x=smart.pos.y=smart.pos.z=0;
	smart.spd.x=smart.spd.y=smart.spd.z=0;
	smart.att_rate.x=smart.att_rate.y=smart.att_rate.z=0;
	smart.att.x=smart.att.y=smart.att.z=0;
	mission_switch=PX4_MODE_OFFBOARD;
	
	//limit
	nav_spd_pid.max_exp=0;
	ctrl_1.zrate_limit=0;
	//
	if(my_deathzoom( CH_filter[PIT] ,0,88 )==0)
	rc_middle[PIT]=1;
	if(my_deathzoom( CH_filter[ROL] ,0,88 )==0)
	rc_middle[ROL]=1;
	if(my_deathzoom( CH_filter[YAW] ,0,88 )==0)
	rc_middle[YAW]=1;
	if(my_deathzoom( CH_filter[THR] ,0,88 )==0)
	rc_middle[THR]=1;
	
	#if PX4_SDK
	if(px4.connect)
	{
	  mode_oldx.auto_fly_up=px4.Rc_gear>1400;
		fly_ready=px4.Rc_mode>1400;
		px4_mode_position=px4.Rc_mode<1600;
	}
	#endif
	float tar_lat=navData.missionLegs[LIMIT(navData.Leg_num-1,0,99)].targetLat;
	float tar_lon=navData.missionLegs[LIMIT(navData.Leg_num-1,0,99)].targetLon;
	 tar_Yaw1=To_180_degrees(navCalcBearing(m100.Lat, m100.Lon,tar_lat,tar_lon));	 
	//down_ward_search_task_tangle(6,6,3,2,T);
	//target_track_downward_task_search(&c2c,0.1,3,T);
	//mode_oldx.sdk_flag=1;
  //pan_set_task(90,T);
	//target_track_downward_task(&c2c,0.25*0,0.1,MODE_SPD,T);
	//test
	 //if(test_mission)
	//	 way_point_mission(0.8,EN_HEAD_WAY,T);
	   //set_drone_yaw_task(90,FLT_YAW,T);
	//est_target_state(&c2c, &three_wheel_car,T);
	//pan_set_task(pan_set,T);
	//target_track_pan_task(&c2c,0.2,1,1,0,T);
	//pan_search_task(5,35,20,2,0,T);
	//est_target_state(&c2c, &three_wheel_car, T);
 // target_track_downward_task(&c2c,0,0.5,1,T);//pos error
  //pan_search_task(35,90,45,2,1,T);
	//end
	
		
  switch(state_v){
	   case SG_LOW_CHECK:
			 height_off_api=0;
		   //state_home=0;
		   mode_oldx.led_flag=0;
		   #if defined(USE_CARGO)
				aux_open(2,0);
		   #endif
		   mission_sel_lock=mission_sel;
			if(mode_oldx.auto_fly_up&&CH_filter[THR]<-500+DEAD_NAV_RC)
		    cnt[0]++;
			else
				cnt[0]=0;
			
			if(mode_oldx.flow_hold_position==2&&ALT_POS_BMP_UKF_OLDX>SONAR_HEIGHT_SET&&fly_ready&&fabs(CH_filter[THR])<DEAD_NAV_RC)//空中切换
			{
			 init_mission_reg();way_point_init(0);track_init(c2c);
			 Set_home();
			 set_way_point_single(home_lat,home_lon,ALT_POS_BMP_UKF_OLDX);
			 height_off_api=ALT_POS_SONAR2-px4.H;
			 state_v=SD_HOLD;
			}	

			if(height_ctrl_mode!=0&&mode_oldx.auto_sdk&&mode_oldx.auto_fly_up&&mode_oldx.flow_hold_position==2&&fly_ready&&fabs(CH_filter[THR])<DEAD_NAV_RC)//直接起飞
			{
			 init_mission_reg();way_point_init(0);track_init(c2c);
			 Set_home();
			 set_way_point_single(home_lat,home_lon,ALT_POS_BMP_UKF_OLDX);
			 height_off_api=ALT_POS_SONAR2-px4.H;
			 state_v=SU_MISSION;	
			}
			
			if(force_pass||(px4.Rc_mode>1600&&px4.Rc_gear>1600&&fly_ready&&fabs(CH_filter[THR])<DEAD_NAV_RC))//直接起飞
			{
				force_pass=0;
				init_mission_reg();way_point_init(0);track_init(c2c);
			  Set_home();
			  set_way_point_single(home_lat,home_lon,ALT_POS_BMP_UKF_OLDX);
			  height_off_api=ALT_POS_SONAR2-px4.H;
				state_v=SU_MISSION;	
			}
		break;
		//--------------------------------------------	
		case SG_MID_CHECK:
	    if(mode_oldx.auto_fly_up&&!px4_mode_position&&fabs(CH_filter[THR])<DEAD_NAV_RC)
		    cnt[0]++;
			else
				cnt[0]=0;
			if(cnt[0]>1.5/T)
			{cnt[0]=0;state_v=SU_UP1;Set_home();}
			else if(!mode_oldx.auto_fly_up||(ALT_POS_SONAR2>SONAR_HEIGHT_SET*1.25&&!mode_oldx.auto_fly_up)||fly_ready)
			{cnt[0]=0;state_v=SG_LOW_CHECK;}	
	  break;
		//--------------------------------------------	
		case SU_UP1:
			if(cnt[0]++>5/T||ALT_POS_SONAR2>AUTO_UP_POS_Z)
				{cnt[0]=0;state_v=SD_HOLD;}	
				if(mode_oldx.flow_hold_position==0||!fly_ready ){if(cnt[3]++>0.25/T){state_v=SD_SAFE;cnt[3]=0;}}
		break;
		//---------------------system IDLE--------------------		
		case SD_HOLD:
			if(mode_oldx.auto_sdk)
		     cnt[0]++;
			else
				 cnt[0]=0;
			if(cnt[0]>2/T||force_pass)
			{cnt[0]=0;force_pass=0;
			 state_v=SU_MISSION;	
			}
    if(mode_oldx.flow_hold_position==0||!fly_ready||px4_mode_position  ){if(cnt[3]++>0.25/T){state_v=SD_SAFE;cnt[3]=0;}}
    break;
		//--------------------------------------------
		case SU_MISSION:	
			//mode_oldx.led_flag=1;
			if(Rc_Get_PWM.AUX2>1500) {
			three_wheel_car.car_mode=3;//3circle 2forwad
			three_wheel_car.gain[0]=0.5;//spd
			three_wheel_car.gain[1]=2;//r
			three_wheel_car.gain[2]=2;//length fb
			}else{
			three_wheel_car.car_mode=1;//3circle 2forwad
			three_wheel_car.gain[0]=1;//spd
			three_wheel_car.gain[1]=1;//r
			three_wheel_car.gain[2]=2;//length fb}
			}
			m100.save_data=m100.save_video=1;//使能存储数据
					 
//----------------------------任务指派接口------------------------------//	
	   if(fabs(CH_filter[THR])<DEAD_NAV_RC*1.2&&fabs(CH_filter[ROL])<DEAD_NAV_RC
			 &&fabs(CH_filter[PIT])<DEAD_NAV_RC&&fabs(CH_filter[YAW])<DEAD_NAV_RC&&
		   state_home==0&&mode_oldx.auto_sdk)
		    switch(mission_sel_lock)
				{
				  case 0:
								switch(id_chip)
								{
								case IMAV1: mission_flag=mission_test_gps(T); break;//MAP1
								//case IMAV2: mission_flag=mission_test_gps(T); break;//Serach2mission_search
							  case IMAV2: mission_flag=mission_search(T); break;//Serach2
								case IMAV3: mission_flag=mission_test_gps(T); break;//MAP2
								default:mission_flag=mission_test_gps(T); break;
								}	
					break;
				  case 1:mission_flag=mission_landing1(T); break;
					case 2:mission_flag=mission_test_gps(T); break;//GPS航点
					case 3:mission_flag=mission_test5(T); break;//跟踪逼近观测估计 最后直接用位置降落
					case 4:mission_flag=mission_test8(T); break;//存储
					case 11:mission_flag=mission_light_draw(T); break;//光绘
					default:mission_flag=mission_landing(T); break;
				}	
		   
		
		if(mode_oldx.flow_hold_position==0||!fly_ready||px4_mode_position){if(cnt[3]++>0.25/T){state_v=SD_SAFE;cnt[3]=0;}}
    break;
		
    //------------------------------------SAFE------------------------------------------------
		case SD_SAFE://safe out
			if((!mode_oldx.auto_sdk&&!mode_oldx.flow_hold_position&&(CH_filter[THR]<-500+150))&&!fly_ready )
			state_v=SG_LOW_CHECK;	
		break;
  }		
	//auto return home

	switch(state_home)
	{
		case 0:
			   //one key retrun home
			   if(fly_ready&&ALT_POS_BMP_UKF_OLDX>SONAR_HEIGHT_SET&&mode_oldx.flow_hold_position==2&&height_ctrl_mode>0
				 &&mode_oldx.en_auto_home)
					 if(AUX2_REG>=1500&&Rc_Get_PWM.AUX2<1500)
						state_home=1;
				 
				 //rc loss
				 if(fly_ready&&height_ctrl_mode>0//&&ALT_POS_BMP_UKF_OLDX>0.05
					 &&mode_oldx.rc_loss_return_home&&NS_REG==2&&NS==0)
					{
					  if(mode_oldx.rc_loss_return_home==1&&mode_oldx.flow_hold_position>0)
						{init_mission_reg();state_home=1;}//return home
						else if(mode_oldx.rc_loss_return_home==2)
						{init_mission_reg();state_home=2;}//landing directly
					}
				break;
	  case 1:
			  height_ctrl_mode=1;mode_oldx.flow_hold_position=2;
				if(return_home(10,1,EN_LAND,NEN_HEAD_WAY,T))
					fly_ready=state_home=cnt_mission[HOME_CNT]=cnt_mission[WAY_INIT]=cnt_mission[WAY_CNT]=0;
				if((!rc_middle[PIT]||!rc_middle[ROL]||!rc_middle[YAW]||!rc_middle[THR])&&NS==2)
					state_home=cnt_mission[HOME_CNT]=cnt_mission[WAY_INIT]=cnt_mission[WAY_CNT]=0;
				break;
		case 2:
			  height_ctrl_mode=1;mode_oldx.flow_hold_position=2;
				if(land_task1(0.4,T))
					fly_ready=state_home=cnt_mission[HOME_CNT]=cnt_mission[WAY_INIT]=cnt_mission[WAY_CNT]=0;
				if((!rc_middle[PIT]||!rc_middle[ROL]||!rc_middle[YAW]||!rc_middle[THR])&&NS==2)
					state_home=cnt_mission[HOME_CNT]=cnt_mission[WAY_INIT]=cnt_mission[WAY_CNT]=0;
				break;
	}
	
	//led test
	if(!fly_ready&&Rc_Get_PWM.AUX2<1500){
		mode_oldx.led_flag=1;
		#if defined(USE_CARGO)
	   aux_open(2,1);
		#endif
	}
	if(fly_ready&&!fly_readyr){
		Set_home();
		set_way_point_single(home_lat,home_lon,ALT_POS_BMP_UKF_OLDX);
		height_off_api=ALT_POS_SONAR2-px4.H;
	}
	fly_readyr=fly_ready;
	AUX2_REG=Rc_Get_PWM.AUX2;
  NS_REG=NS;
	if(smart.rc.ATT_MODE!=0||state_v!=SU_MISSION||rc_middle[YAW]!=1)
	smart.rc.SWITCH_MODE[0]=0;
	if(smart.rc.HEIGHT_MODE!=0||state_v!=SU_MISSION||rc_middle[THR]!=1)
	smart.rc.SWITCH_MODE[1]=0;
	if(smart.rc.POS_MODE!=0||state_v!=SU_MISSION||rc_middle[PIT]!=1||rc_middle[ROL]!=1)
	smart.rc.SWITCH_MODE[2]=0;
	
	if(((smart.rc.ATT_MODE!=0&&mode_reg[0]==0)||(rc_middle_reg[YAW]!=1&&rc_middle[YAW]==1))&&state_v==SU_MISSION)
	{
   smart.rc.SWITCH_MODE[0]=1;	
	 smart.rc.pos_switch[3]=Yaw_fc;
	}
	if(((smart.rc.HEIGHT_MODE!=0&&mode_reg[1]==0)||(rc_middle_reg[THR]!=1&&rc_middle[THR]==1))&&state_v==SU_MISSION)
	{
   smart.rc.SWITCH_MODE[1]=1;	
	 smart.rc.pos_switch[2]=ALT_POS_BMP_UKF_OLDX;
	}
	if(((smart.rc.POS_MODE!=0&&mode_reg[2]==0)
		||(rc_middle_reg[PIT]!=1&&rc_middle[PIT]==1)
		||(rc_middle_reg[ROL]!=1&&rc_middle[ROL]==1))
	   &&state_v==SU_MISSION)
	{
   smart.rc.SWITCH_MODE[2]=1;	
	 smart.rc.pos_switch[0]=POS_UKF_X;
	 smart.rc.pos_switch[1]=POS_UKF_Y;
	}
	
	smart.rc.SWITCH_MODE[0]=smart.rc.SWITCH_MODE[1]=smart.rc.SWITCH_MODE[2]=0;
	
	mode_reg[0]=smart.rc.ATT_MODE;
	mode_reg[1]=smart.rc.HEIGHT_MODE;
	mode_reg[2]=smart.rc.POS_MODE;
  rc_middle_reg[PIT]=rc_middle[PIT];
	rc_middle_reg[ROL]=rc_middle[ROL];
	rc_middle_reg[YAW]=rc_middle[YAW];
	rc_middle_reg[THR]=rc_middle[THR];
}	