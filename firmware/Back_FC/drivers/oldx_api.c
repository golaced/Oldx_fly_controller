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
#include "mavl.h"

u32 cnt_mission[MAX_CNT_NUM];
u16 cnt_task[MAX_CNT_NUM];
// input lat/lon in degrees, returns bearing in radians
float navCalcBearing1(double lat1, double lon1, double lat2, double lon2) {
    float n = (float)((lat2 - lat1) * (double)DEG_TO_RAD * r1);
    float e = (float)((lon2 - lon1) * (double)DEG_TO_RAD * r2);
    float ret = atan2f(e, n);

    if (!isfinite(ret))
        ret = 0.0f;

    return ret;
}

//--------------------------------------new  SDK api----------------------------------------

void track_init(CI tracker)
{
  tracker.lat=tracker.lon=tracker.gx=tracker.gy=tracker.gz=tracker.gyaw=0;
	tracker.gx_vm=tracker.gy_vm=tracker.gz_vm=tracker.gyaw=0;
	tracker.range=tracker.range_xy=tracker.range_all=0;
	tracker.pix_x=tracker.pix_y=0;
}	
					

void init_mission_reg(void)
{ u16 i;
  mission_state=0;
  for(i=0;i<MAX_CNT_NUM;i++)
   cnt_mission[i]=cnt_task[i]=0;
}

u8 set_pos_task(float x,float y,float z,float dt)
{ 
	static float pos_reg[3];
	double tar_gps[3];

  if(x!=0||y!=0){
		smart.rc.POS_MODE=SMART_MODE_POS;	
		smart.pos.x=x;			
		smart.pos.y=y;
	}
	
	if(z>0){
		smart.rc.HEIGHT_MODE=SMART_MODE_POS;
		smart.pos.z=z;	
	}
	
	smart.rc.RST=3;	


  if(ABS(x - nav_pos_ctrl[X].now)<WAY_POINT_DEAD*3.5
		&&ABS(y - nav_pos_ctrl[Y].now)<WAY_POINT_DEAD*3.5
		&&ABS(ultra_ctrl.now-z*1000)<WAY_POINT_DEAD*3.5*1000)
		 cnt_mission[SET_POS_CNT]++;
	else
		 cnt_mission[SET_POS_CNT]=0;
	
	if(cnt_mission[SET_POS_CNT]>MISSION_CHECK_TIME/dt){
	  cnt_mission[SET_POS_CNT]=0;
		return 1;
		}
	else
    return 0;			
}	

u8 way_point_task(double lat,double lon, float height,float loter_time,u8 mode,float dt)
{ 
	static float pos_reg[3];
	double tar_gps[3];
	if(cnt_mission[WAY_INIT]==0){
		pos_reg[0]=m100.Lat;			
		pos_reg[1]=m100.Lon;					
		pos_reg[2]=ALT_POS_BMP_UKF_OLDX;			
		cnt_mission[WAY_INIT]=1;
  }
	if(ABS(m100.Init_Lat)<5|| ABS(m100.Init_Lon)<5)
		return 2;
	
  if(ABS(lat)>5)
		tar_gps[0]=lat;
	else
	  tar_gps[0]=pos_reg[0];
	if(ABS(lon)>5)
		tar_gps[1]=lon;
	else
	  tar_gps[1]=pos_reg[1];
	if(height>0)
		tar_gps[2]=height;
	else
	  tar_gps[2]=pos_reg[2];
	
	float tar_Y,tar_X,tar_Yaw;			
	CalcGlobalDistance(tar_gps[0], tar_gps[1], m100.Init_Lat, m100.Init_Lon, &tar_Y,&tar_X );
	tar_Yaw=navCalcBearing1(m100.Lat, m100.Lon,tar_gps[0], tar_gps[1])*57.3;	
  if(ABS(lat)>5&&ABS(lon)>5){
		smart.pos.x=tar_X;			
		smart.pos.y=tar_Y;
		smart.rc.POS_MODE=SMART_MODE_POS;	
	}

	if(height>0){
		smart.pos.z=tar_gps[2];	
		smart.rc.HEIGHT_MODE=SMART_MODE_POS;
	}
	
	smart.rc.RST=3;	

	float dead_use=WAY_POINT_DEAD1;
	float check_time=MISSION_CHECK_TIME;
	if(mode==MODE_FAST_WAY){
		dead_use=WAY_POINT_DEAD1*6.6;
		check_time=MISSION_CHECK_TIME;
	}
	if(loter_time>1)
		check_time=loter_time;
  if(ABS(tar_X - nav_pos_ctrl[X].now)<dead_use
		&&ABS(tar_Y - nav_pos_ctrl[Y].now)<dead_use
		&&ABS(height*1000-ultra_ctrl.now)<dead_use*1000*1.618)
		 cnt_mission[WAY_CNT]++;
	else
		 cnt_mission[WAY_CNT]=0;
	
	if(cnt_mission[WAY_CNT]>LIMIT(check_time,0.05,3600)/dt){
		cnt_mission[WAY_INIT]=cnt_mission[WAY_CNT]=0;
		return 1;
		}
	else
    return 0;			
}	


u8 way_point_mission(float spd_limit,u8 head_to_waypoint,float dt)
{
	u8 flag;
  double lat,lon,alt;
	lat=navData.missionLegs[cnt_mission[MISSION_CNT]].targetLat;
	lon=navData.missionLegs[cnt_mission[MISSION_CNT]].targetLon;
	alt=LIMIT(navData.missionLegs[cnt_mission[MISSION_CNT]].targetAlt,0,MAX_WAYP_Z);
	
	float tar_Yaw=To_180_degrees(navCalcBearing1(m100.Lat, m100.Lon,lat,lon)*57.3);	

	if(navData.Leg_num<1)
		 return 2;
	float dis=sqrt(pow(nav_pos_ctrl[X].exp - nav_pos_ctrl[X].now,2)+pow(nav_pos_ctrl[Y].exp - nav_pos_ctrl[Y].now,2));
	if(ABS(lat)>5&&ABS(lon)>5&&alt>0&&navData.missionLegs[cnt_mission[MISSION_CNT]].type!=0){
		if(navData.missionLegs[cnt_mission[MISSION_CNT]].maxHorizSpeed!=0)
		nav_spd_pid.max_exp=LIMIT(navData.missionLegs[cnt_mission[MISSION_CNT]].maxHorizSpeed,0,3);
		else
		nav_spd_pid.max_exp=spd_limit;
					
		if(navData.missionLegs[cnt_mission[MISSION_CNT]].poiHeading!=0)
		  set_drone_yaw_task(To_180_degrees(navData.missionLegs[cnt_mission[MISSION_CNT]].poiHeading*57.3)
		  ,YAW_LIMIT_RATE,dt);
    else if(head_to_waypoint&&dis>3){
			set_drone_yaw_task(tar_Yaw,YAW_LIMIT_RATE,dt);
    }	
		
	  float loiterTime;
		if(navData.missionLegs[cnt_mission[MISSION_CNT]].loiterTime>1)
			loiterTime=navData.missionLegs[cnt_mission[MISSION_CNT]].loiterTime;
		if(way_point_task(lat,lon,alt,loiterTime,MODE_FAST_WAY,dt))
		  {cnt_mission[F_YAW_INIT]=0;cnt_mission[MISSION_CNT]++;}
	}
	else
		cnt_mission[MISSION_CNT]++;
	
	if(MISSION_CNT>0&&navData.missionLegs[cnt_mission[MISSION_CNT]].loiterTime==DRAW_TIME)
		mode_oldx.led_flag=1;
	if(cnt_mission[MISSION_CNT]>navData.Leg_num){
		cnt_mission[MISSION_CNT]=0;
		mode_oldx.led_flag=0;
		return 1;}
	else 
		return 0;
}

u8 return_home(float set_z, float spd_limit, u8 en_land, u8 head_to_home, float dt)
{
	 u8 flag=0;
	 float tar_Yaw=To_180_degrees(navCalcBearing1(m100.Lat, m100.Lon,home_lat,home_lon));	
	 float dis=sqrt(pow(nav_pos_ctrl[X].exp - nav_pos_ctrl[X].now,2)+pow(nav_pos_ctrl[Y].exp - nav_pos_ctrl[Y].now,2));
	 switch(cnt_mission[HOME_CNT]){
			case 0:
			flag=set_drone_z_task(set_z,dt);	
			if(flag)
				cnt_mission[HOME_CNT]++;
			break;
			case 1:
			if(ABS(home_lat)>5&&ABS(home_lon)>5){
			flag=way_point_task(home_lat,home_lon,set_z,0,NMODE_FAST_WAY,dt);
			
			if(dis>2)
				nav_spd_pid.max_exp=spd_limit;
			if(head_to_home&&dis>3)
			  set_drone_yaw_task(tar_Yaw,YAW_LIMIT_RATE,dt);
			 
		  }
			if(flag)
			{cnt_mission[HOME_CNT]++;cnt_mission[F_YAW_INIT]=0;}
			break;
			case 2:
				if(en_land)
					spd_move_task(0,0,-LAND_SPD,99,dt);	
				else
					cnt_mission[HOME_CNT]++;	

				if(ALT_POS_BMP_UKF_OLDX<2)
					cnt_mission[HOME_CNT]++;
			break;
			case 3:
        if(en_land)
			    flag=land_task1(LAND_SPD,dt);
				else
					cnt_mission[HOME_CNT]++;
				if(flag)
					cnt_mission[HOME_CNT]++;
			break;
   }
	 
	if(cnt_mission[HOME_CNT]>3){
		cnt_mission[HOME_CNT]=0;
		return 1;}
	else 
		return 0;
}


u8 spd_move_task(float x,float y,float z,float time,float dt)
{	
	if(x!=0||y!=0){
		smart.spd.x=x;			
		smart.spd.y=y;
		smart.rc.POS_MODE=SMART_MODE_SPD;
  }		
	
	if(z!=0){
		smart.spd.z=z;			
		smart.rc.HEIGHT_MODE=SMART_MODE_SPD;
  }		

	if(cnt_mission[SPD_MOVE_CNT]++>time/dt){
		cnt_mission[SPD_MOVE_CNT]=0;
		return 1;
		}
	else
    return 0;			
}	


u8 pos_move_global_task(float x,float y,float z,float dt)
{	
	static float pos_reg[3];
	if(cnt_mission[BODY_MOVE_INIT]==0){
		pos_reg[0]=POS_UKF_X;			
		pos_reg[1]=POS_UKF_Y;					
		pos_reg[2]=ALT_POS_BMP_UKF_OLDX;			
		cnt_mission[BODY_MOVE_INIT]=1;
  }

	smart.pos.x=pos_reg[0]+x;			
	smart.pos.y=pos_reg[1]+y;				
	smart.pos.z=pos_reg[2]+z;	
	
	smart.rc.RST=3;	
	smart.rc.POS_MODE=SMART_MODE_POS;
	smart.rc.HEIGHT_MODE=SMART_MODE_POS;

  if(ABS(smart.pos.x - nav_pos_ctrl[X].now)<WAY_POINT_DEAD
		&&ABS(smart.pos.y - nav_pos_ctrl[Y].now)<WAY_POINT_DEAD
		&&ABS(smart.pos.z*1000-ultra_ctrl.now)<WAY_POINT_DEAD*1000)
		 cnt_mission[BODY_MOVE_CNT]++;
	else
		 cnt_mission[BODY_MOVE_CNT]=0;
	
	if(cnt_mission[BODY_MOVE_CNT]>MISSION_CHECK_TIME/dt){
		cnt_mission[BODY_MOVE_INIT]=cnt_mission[BODY_MOVE_CNT]=0;
		return 1;
		}
	else
    return 0;		
}	


u8 pos_move_body_task(float x,float y,float z,float dt)
{	
	static float pos_reg[4];
	float pos_add_body[2];
	if(cnt_mission[BODY_MOVE_INIT]==0){
		pos_reg[0]=POS_UKF_X;			
		pos_reg[1]=POS_UKF_Y;					
		pos_reg[2]=ALT_POS_BMP_UKF_OLDX;		
    pos_reg[3]=Yaw_fc;		
		cnt_mission[BODY_MOVE_INIT]=1;
  }
	
  pos_add_body[0]=cos(pos_reg[3]/57.3)*x+sin(pos_reg[3]/57.3)*y;
	pos_add_body[1]=cos(pos_reg[3]/57.3)*y-sin(pos_reg[3]/57.3)*x;
	smart.pos.x=pos_reg[0]+pos_add_body[0];			
	smart.pos.y=pos_reg[1]+pos_add_body[1];				
	smart.pos.z=pos_reg[2]+z;	
	
	smart.rc.RST=3;	
	smart.rc.POS_MODE=SMART_MODE_POS;
	smart.rc.HEIGHT_MODE=SMART_MODE_POS;

  if(ABS(smart.pos.x - nav_pos_ctrl[X].now)<WAY_POINT_DEAD
		&&ABS(smart.pos.y - nav_pos_ctrl[Y].now)<WAY_POINT_DEAD
		&&ABS(ultra_ctrl.now-smart.pos.z*1000)<WAY_POINT_DEAD*1000)
		 cnt_mission[BODY_MOVE_CNT]++;
	else
		 cnt_mission[BODY_MOVE_CNT]=0;
	
	if(cnt_mission[BODY_MOVE_CNT]>MISSION_CHECK_TIME/dt){
		cnt_mission[BODY_MOVE_INIT]=cnt_mission[BODY_MOVE_CNT]=0;
		return 1;
		}
	else
    return 0;		
}	


_st_height_pid_v tarck_ctrl[2];
_st_height_pid track_pid[10];
float flt_p=0.1;
float flt_pr=0.1;
void Est_target_gpos(float x,float y,float z,float yaw,
										 float xd,float yd,float zd,float yawd,
										 float *xg,float *yg,float *zg,float *yawg)
{	
*yawg=To_180_degrees(yawd-yaw);
float cy=cos(-*yawg*0.0173);	
float sy=sin(-*yawg*0.0173);	
float x_temp= x*cy - y*sy;
float y_temp=	x*sy + y*cy;
*xg=xd-x_temp;	
*yg=yd-y_temp;
*zg=zd-z;
}


// 云台设置API：输入（俯仰轴：输入0则复位云台）
u8 pan_set_task(float pitch,float dt)
{
	static u8 init;
	static float off[3];
   if(!init)
	{ init=1;
		off[0]=aux.att_off[0];
	}
  if(pitch==0)
		aux.att_ctrl[0]=0;
	else
		#if defined(M_DRONE_330)
	  aux.att_ctrl[0]=pitch-aux.att_off[0];
	  #else
		aux.att_ctrl[0]=pitch+aux.att_off[0];
		#endif
	
		#if defined(BLDC_PAN)
	  aux.att_ctrl[0]=pitch;
	  #endif
}	

//前进搜索
void pan_search_task(float min,float max,float da,u8 en_yaw,u8 en_forward,float dt)
{
	static u8 init;
	static u16 cnt;
	static float off[3];
	static u8 flag=0,flagy=0;
	static float angle;
   if(!init)
	{ init=1;
		off[0]=aux.att_off[0];
	}
	
	if(flag)
		angle+=da*dt;
	else 
		angle-=da*dt;
	#if defined(M_DRONE_330)
	 aux.att_ctrl[0]=angle-aux.att_off[0];
	#else
	 aux.att_ctrl[0]=angle+aux.att_off[0];
	#endif
	#if defined(BLDC_PAN)
	  aux.att_ctrl[0]=angle;
	#endif
   if(angle>max)
	 {
	   flag=0;
		 angle=max*0.96;
	 }
	 
	 if(angle<min)
	 {
	   flag=1;
		 angle=min*1.04;
	 }
	 
	 if(en_yaw==1)
	 {
	    smart.rc.ATT_MODE=SMART_MODE_SPD; 
			smart.att_rate.z=40;
	 } else if(en_yaw==2)
	 {
			 switch(cnt_mission[PAN_SEARCH_FLAG]) 
			 {
				 case 0:
					if(flagy)
					smart.att_rate.z=15;
					else 
					smart.att_rate.z=-15;
					smart.rc.ATT_MODE=SMART_MODE_SPD; 
					cnt++;
					if(cnt>2.5/dt)
						{cnt=0;flagy=!flagy;cnt_mission[PAN_SEARCH_FLAG]=1;}
				 break;
				 case 1:
					if(flagy)
					smart.att_rate.z=15;
					else 
					smart.att_rate.z=-15;
					smart.rc.ATT_MODE=SMART_MODE_SPD; 
					cnt++;
					if(cnt>2.5/dt)
						{cnt=0;cnt_mission[PAN_SEARCH_FLAG]=2;}
				 break;
				 case 2://middle
						if(en_forward)
						cnt_mission[PAN_SEARCH_FLAG]=101;
						else
						cnt_mission[PAN_SEARCH_FLAG]=3;	
						cnt=0;
				 break;
				 case 3://end
					if(flagy)
					smart.att_rate.z=15;
					else 
					smart.att_rate.z=-15;
					smart.rc.ATT_MODE=SMART_MODE_SPD; 
					cnt++;
					if(cnt>2.5/dt)
						{cnt=0;flagy=!flagy;cnt_mission[PAN_SEARCH_FLAG]=1;}
				 break;
				 case 101:
					 	smart.spd.y=0.3;	
						smart.rc.POS_MODE=SMART_MODE_SPD;
					if(cnt++>2/dt)
						{cnt=0;cnt_mission[PAN_SEARCH_FLAG]=3;}
				 break;
			 }	 
	}
	  
}	

//圆环搜索
void down_ward_search_task(float r,float z,float d_angle,float d_r,float dt)
{
	 static u8 init,flag;
	 static float pos_reg[2];
	 static float angle,rad;
		if(cnt_mission[C_SEARCH_INT]==0){
		pos_reg[0]=POS_UKF_X;			
		pos_reg[1]=POS_UKF_Y;					
		cnt_mission[C_SEARCH_INT]=1;
  }
   pan_set_task(90,dt);
   if(z!=0){
		 smart.pos.z=z;
		 smart.rc.HEIGHT_MODE=SMART_MODE_POS;
   }
	 angle+=dt*d_angle;
	
		if(flag)
			rad+=d_r*dt;
		else 
			rad-=d_r*dt;
	
   if(rad>r)
	 {
	   flag=0;
		 rad=r*0.96;
	 }
	 
	 if(rad<0.2*r)
	 {
	   flag=1;
		 rad=0.24*r;
	 }
	 if(d_r==0)
	   rad=r;
		if(angle<0)angle=360;
		if(angle>360)angle=0;
		smart.pos.x=cos(angle*0.0173)*rad+pos_reg[X];
		smart.pos.y=sin(angle*0.0173)*rad+pos_reg[Y];
		smart.rc.POS_MODE=SMART_MODE_POS;
}
	
//矩形搜索
u8 down_ward_search_task_tangle(float w,float h,int wn,int hn,float spd_limit,float z,float dt)
{
	 int i,j,temp;
	 float dx,dy;
	 static u8 init,flag;
	 static float pos_reg[2];
	 static float way_points[60][2];
		if(cnt_mission[C_SEARCH_T_INT]==0){
			pos_reg[0]=POS_UKF_X;			
			pos_reg[1]=POS_UKF_Y;					
			cnt_mission[C_SEARCH_T_INT]=1;
			cnt_mission[C_SEARCH_T_INT1]=0;
			flag=1;
			dx=w/LIMIT(wn-1,1,8);
			dy=h/LIMIT(hn-1,1,8);			
				for(i=0;i<wn;i++){
					if(flag){
						for(j=0;j<hn;j++){
							way_points[i*hn+j][0]=i*dx+pos_reg[0]-w/2;
							way_points[i*hn+j][1]=j*dy+pos_reg[1]-h/2;
						}
						flag=0;
					}
					else{
						for(j=0;j<hn;j++){
							way_points[(i+1)*hn-j-1][0]=i*dx+pos_reg[0]-w/2;
							way_points[(i+1)*hn-j-1][1]=j*dy+pos_reg[1]-h/2;
						}
						flag=1;
					}	
				}
			pan_set_task(90,dt);
	  }
		
    if(spd_limit>0)
			nav_spd_pid.max_exp=spd_limit;
		if(cnt_mission[C_SEARCH_T_INT1]<wn*hn)
			temp=set_pos_task(way_points[cnt_mission[C_SEARCH_T_INT1]][0],way_points[cnt_mission[C_SEARCH_T_INT1]][1],z,dt);
		if(temp==1)
			cnt_mission[C_SEARCH_T_INT1]++;

		if(cnt_mission[C_SEARCH_T_INT1]>wn*hn-1){
			cnt_mission[C_SEARCH_T_INT]=cnt_mission[C_SEARCH_T_INT1]=0;
		  return 1;
		}
		else 
			return 0;
}
	

//估计目标位置
float flt_car_est[2]={0.1,0.25};//visual UWB
float drone_test_pos[4]={0,0,2,0};
float est_target_state(CI* target,_CAR *target_b, float dt)
{
	float Yaw_d;
	float  xg=0,yg=0,zg,yawg;
	static int init_cnt;
	float off[3]={0,0,0.3};
	u8 en_uwb=0,en_tag=1,en_encoder=1;
	#if defined(USE_UWB)
	if(m100.uwb_f[0]!=0&&m100.uwb_f[1]!=0)
	Yaw_d=m100.uwb_f[3];
	else
	Yaw_d=Yaw_fc;
	#else
	Yaw_d=Yaw_fc;
	#endif
	off[0]=sin(Yaw_fc*0.0173)*off[2];
	off[1]=cos(Yaw_fc*0.0173)*off[2];
	//visual measurement
	if(target->connect&&target->check&&target->x!=0&&target->y!=0){
	Est_target_gpos(target->x/100.,target->y/100.,target->z/100.,target->yaw,
										POS_UKF_X+off[0],POS_UKF_Y+off[1],ALT_ACC_BMP_UKF_OLDX,Yaw_d,
										 &xg,&yg,&zg,&yawg);
//	drone_test_pos[3]=Yaw_d;	
//	Est_target_gpos(target->x/100.,target->y/100.,target->z/100.,target->yaw,
//										drone_test_pos[0],drone_test_pos[1],drone_test_pos[2],drone_test_pos[3],
//										 &xg,&yg,&zg,&yawg);	
	target->gx_vm=xg;target->gy_vm=yg;target->gz_vm=zg;target->gyaw_vm=yawg;}
	else
	init_cnt=target->gx_vm=target->gy_vm=target->gz_vm=target->gyaw_vm=0;	
  	//kalman估计里程计
	if(car_spd[0]!=0&&car_spd[1]!=0&&cnt_mission[EST_INT]&&en_encoder)	
	{
	 target->gx+=car_spd[0]*dt*cos(target->gyaw*0.0173)+car_spd[1]*dt*sin(target->gyaw*0.0173);
	 target->gy+=car_spd[1]*dt*cos(target->gyaw*0.0173)-car_spd[0]*dt*sin(target->gyaw*0.0173);
	 //target->gyaw+=car_spd[2]*dt*cos(target->gyaw*0.0173);
	}	
	//marker init visual
	if(target->connect&&target->check&&cnt_mission[EST_INT]==0&&en_tag){
		init_cnt++;
		if(init_cnt>3&&ABS(xg)>0.02&&ABS(yg)>0.025){
		cnt_mission[EST_INT]=1;
		target->gx=xg;
		target->gy=yg;
		target->gz=zg;
		target->gyaw=yawg;
		}
	}
	//marker init uwb
	if(target_b->pos[0]!=0&&target_b->pos[1]!=0&&cnt_mission[EST_INT]==0&&en_uwb){
		cnt_mission[EST_INT]=2;
		target->gx=target_b->pos[0]+target_b->pos_off[0];
		target->gy=target_b->pos[1]+target_b->pos_off[1];
	}
	
	//measurement update
	if(target->connect&&target->check&&cnt_mission[EST_INT]&&ABS(xg)>0.025&&ABS(yg)>0.025 &&en_tag){//visual
    target->gx_vm=xg;target->gy_vm=yg;target->gz_vm=zg;target->gyaw_vm=yawg;
		target->gx=xg*flt_car_est[0]+(1-flt_car_est[0])*target->gx;
		target->gy=yg*flt_car_est[0]+(1-flt_car_est[0])*target->gy;
		target->gz=zg*flt_car_est[0]+(1-flt_car_est[0])*target->gz;
		target->gyaw=To_180_degrees(yawg);//*flt_car_est[0]+(1-flt_car_est[0])*target->gyaw);
    }
	
	if(target_b->pos[0]!=0&&target_b->pos[1]!=0&&en_uwb){//&&cnt_mission[EST_INT]){//uwb
		target->gx=(target_b->pos[0]+target_b->pos_off[0])*flt_car_est[1]+(1-flt_car_est[1])*target->gx;
		target->gy=(target_b->pos[1]+target_b->pos_off[1])*flt_car_est[1]+(1-flt_car_est[1])*target->gy;
    }
	
	//转换二维码局部位置到全球坐标系
	if(ABS(m100.Init_Lat)>5&&ABS(m100.Init_Lon)>5&&cnt_mission[EST_INT])
		CalcGlobalLocation(target->gy,target->gx,m100.Init_Lat,m100.Init_Lon,&target->lat,&target->lon);
	
	if(target->gx!=0&&target->gy!=0&&cnt_mission[EST_INT]!=0)
		return sqrt(pow(POS_UKF_X+off[0]-target->gx,2)+pow(POS_UKF_Y+off[1]-target->gy,2));
	else
		return 0;
}	

float flt_range_pan=0.618;//云台计算距离权重
//下置摄像头对准API： 输入（目标，下降速度，最终高度，模式）
//mode 0->pos 1->spd 2 auto
u8 target_track_downward_task(CI *target,float spdz,float end_z,u8 mode,float dt)
{
  float  xg,yg,zg,yawg;
	float ero_qr_pix[2];
	static float ero_qr_pixr[2];
	static u8 init;
	static u16 cnt[3]={0};
	static float reg_x,reg_y,reg_z;
	int pix_off[2]={0,-15*0};
	if(!init){init=1;
		#if PX4_SDK
		track_pid[0].kp=0.0055;
		track_pid[0].kd=0.002;
		#else
		track_pid[0].kp=0.0015;//0.0015;//0.0015;
		track_pid[0].kd=0;
		#endif
	}	
	
	if(target->connect&&target->check&&target->z!=0&&target->x!=0&&target->z!=0){
		cnt_mission[D_DOWN_INT]=1;//marker init
		
		#if defined(M_DRONE_330)
		float pan_pitch_angle=LIMIT(90-aux.att[0],0,90);
		#else
		float pan_pitch_angle=LIMIT(-aux.att[0],0,90);
		#endif
		//由云台和高度计算目标距离
		target->range=LIMIT(target->z/100.*tan(pan_pitch_angle*0.0173),0,5)*flt_pr+(1-flt_pr)*target->range;
		//由图像位置计算目标距离
		target->range_xy=LIMIT(sqrt(target->x/100.*target->x/100.+target->y/100.*target->y/100.),0,5)*flt_p
		+(1-flt_p)*target->range_xy;
		//融合距离
		target->range_all=target->range*flt_range_pan+(1-flt_range_pan)*target->range_xy;
	  } 
	
		if(mode==0&&target->gx!=0&&target->gy!=0){
			smart.pos.x=target->gx;			
			smart.pos.y=target->gy;	
			smart.rc.POS_MODE=SMART_MODE_POS;
			//control z		
			if(target->check){
					smart.spd.z=spdz;
					smart.rc.HEIGHT_MODE=SMART_MODE_SPD;		
			}else{				
					smart.spd.z=spdz*0.68;
					smart.rc.HEIGHT_MODE=SMART_MODE_SPD; 
			  }			
    }else if(mode==MODE_SPD)//spd
		{		  
				float erox,eroy;	
				if(target->check&&target->pix_x!=0&&target->pix_y!=0){	
						cnt[0]=0;
						cnt[1]=1;
						if(target->z==0)
						 target->z=66;
						reg_x=LIMIT(target->pix_x,0,320);
						reg_y=LIMIT(target->pix_y,0,320);
						reg_z=LIMIT(((float)target->z/100.),0,3);
					}

					if(cnt[0]++>0.5/dt)//delay the control
						cnt[0]=cnt[1]=0;
		 
				 if(cnt[1]){	
					 ero_qr_pix[X]= my_deathzoom1(reg_x-(160+pix_off[X]),6)*2;// *reg_z;
					 ero_qr_pix[Y]=-my_deathzoom1(reg_y-(120+pix_off[Y]),6)*2;// *reg_z;	
					 #if PX4_SDK
					 smart.spd.x=LIMIT(ero_qr_pix[ROLr]*track_pid[0].kp+(ero_qr_pix[X] - ero_qr_pixr[X])*track_pid[0].kd,-1.6,1.6);
					 smart.spd.y=LIMIT(ero_qr_pix[PITr]*track_pid[0].kp+(ero_qr_pix[Y] - ero_qr_pixr[Y])*track_pid[0].kd,-1.6,1.6);
					 #else
					 smart.spd.x=LIMIT(ero_qr_pix[ROLr]*track_pid[0].kp+(ero_qr_pix[X] - ero_qr_pixr[X])*track_pid[0].kd,-0.8,0.8);
					 smart.spd.y=LIMIT(ero_qr_pix[PITr]*track_pid[0].kp+(ero_qr_pix[Y] - ero_qr_pixr[Y])*track_pid[0].kd,-0.8,0.8);
					 #endif
					 ero_qr_pixr[X]=ero_qr_pix[X];	
					 ero_qr_pixr[Y]=ero_qr_pix[Y]; 
					 if(target->z>55)
						smart.rc.POS_MODE=SMART_MODE_SPD;
				 //control z	
					if(ALT_POS_BMP_UKF_OLDX>end_z-WAY_POINT_DEAD1){
						smart.spd.z=-ABS(spdz);
						if(ABS(spdz)>0.01)
							smart.rc.HEIGHT_MODE=SMART_MODE_SPD; 		
					 }
				 }else
				 {
					 if(ALT_POS_BMP_UKF_OLDX>end_z-WAY_POINT_DEAD1){
						smart.spd.z=-ABS(spdz)*0.5;
						if(ABS(spdz)>0.01)
							smart.rc.HEIGHT_MODE=SMART_MODE_SPD; 		
					 }
				 }
		 
		}
		
	if(smart.rc.HEIGHT_MODE==SMART_MODE_SPD)	
		smart.pos.z=ALT_POS_BMP_UKF_OLDX;
	
  if(mode==MODE_POS){	
		if(ABS(smart.pos.x - nav_pos_ctrl[X].now)<WAY_POINT_DEAD
		&&ABS(smart.pos.y - nav_pos_ctrl[Y].now)<WAY_POINT_DEAD
		&&target->z<end_z*1.1&&mode==0)//pos
			 cnt_mission[D_DOWN_CNT]++;
		else
			 cnt_mission[D_DOWN_CNT]=0;
  }
  else{//SPD
//		if((target->check
//		&&ABS(ero_qr_pix[X])<160*0.25
//		&&ABS(ero_qr_pix[Y])<120*0.25
//		&&ALT_POS_BMP_UKF_OLDX<end_z+WAY_POINT_DEAD1*2)||ALT_POS_BMP_UKF_OLDX<end_z+WAY_POINT_DEAD1)//pix
		if((target->check
		&&ABS(ero_qr_pix[X])<160*0.25
		&&ABS(ero_qr_pix[Y])<120*0.25
		&&ALT_POS_BMP_UKF_OLDX<end_z+WAY_POINT_DEAD1*1.5)
		||ALT_POS_BMP_UKF_OLDX<end_z+WAY_POINT_DEAD1)
			 cnt_mission[D_DOWN_CNT]++;
		//else
		//	 cnt_mission[D_DOWN_CNT]=0;
  }
	//reset
	if(target->check)
		cnt[2]=0;
	else if(cnt[2]++>4/dt){
		cnt_mission[D_DOWN_CNT]=0;
	}
	
	if(cnt_mission[D_DOWN_CNT]>MISSION_CHECK_TIME/dt){
		cnt_mission[D_DOWN_INT]=cnt_mission[D_DOWN_CNT]=0;
		return 1;
		}
	else
    return 0;
	
	smart.rc.RST=3;	
}

//下置摄像头直接跟踪  搜寻投递用
u8 target_track_downward_task_search(CI *target,float spdz,float end_z,float dt)
{
  float  xg,yg,zg,yawg;
	float ero_qr_pix[2];
	static float ero_qr_pixr[2];
	static u8 init;
	static u16 cnt[3]={0};
	static int reg_x,reg_y;
	int pix_off[2]={0,0};
	if(!init){init=1;
		track_pid[0].kp=0.0018;
		track_pid[0].kd=0;
	}	
	
		 float erox,eroy;	
		 if(target->connect&&target->x!=0&&target->y!=0){	
			 cnt[0]=0;
			 cnt[1]=1;
			 reg_x=LIMIT(target->x,0,320);reg_y=LIMIT(target->y,0,320);
		 }
		 
		 if(cnt[0]++>0.5/dt)//delay the control
			  cnt[0]=cnt[1]=0;
		 
		 if(cnt[1]){
			 ero_qr_pix[X]=my_deathzoom1(reg_x-(160+pix_off[X]),6)*2 ;
			 ero_qr_pix[Y]=-my_deathzoom1(reg_y-(120+pix_off[Y]),6)*2;	
			 smart.spd.x=LIMIT(ero_qr_pix[ROLr]*track_pid[0].kp+(ero_qr_pix[X] - ero_qr_pixr[X])*track_pid[0].kd,-0.8,0.8);
			 smart.spd.y=LIMIT(ero_qr_pix[PITr]*track_pid[0].kp+(ero_qr_pix[Y] - ero_qr_pixr[Y])*track_pid[0].kd,-0.8,0.8);
			 ero_qr_pixr[X]=ero_qr_pix[X];	
			 ero_qr_pixr[Y]=ero_qr_pix[Y]; 
			 smart.rc.POS_MODE=SMART_MODE_SPD;
		 //control z	
			if(ALT_POS_BMP_UKF_OLDX>end_z-WAY_POINT_DEAD1){
				if(ABS(spdz)>0.01){
					smart.spd.z=-ABS(spdz);
					smart.rc.HEIGHT_MODE=SMART_MODE_SPD; 
				 }					
			 }
		 }else{
		  if(ALT_POS_BMP_UKF_OLDX>end_z-WAY_POINT_DEAD1){
				if(ABS(spdz)>0.01){
					smart.spd.z=-ABS(spdz)/2;
					smart.rc.HEIGHT_MODE=SMART_MODE_SPD; 
				 }					
			 }
		 
		 }
		 
		if(smart.rc.HEIGHT_MODE==SMART_MODE_SPD)	
			smart.pos.z=ALT_POS_BMP_UKF_OLDX;

//		if((cnt[1]
//		&&ABS(ero_qr_pix[X])<160*0.25
//		&&ABS(ero_qr_pix[Y])<120*0.25
//		&&ALT_POS_BMP_UKF_OLDX<end_z+WAY_POINT_DEAD1*2)||ALT_POS_BMP_UKF_OLDX<end_z+WAY_POINT_DEAD1)//pix
		
		if((target->x!=0&&target->y!=0&&ABS(ero_qr_pix[X])<160*0.25
				&&ABS(ero_qr_pix[Y])<120*0.25
				&&ALT_POS_BMP_UKF_OLDX<end_z+WAY_POINT_DEAD1*2.5)||
		    ALT_POS_BMP_UKF_OLDX<end_z+WAY_POINT_DEAD1)
			 cnt_mission[D_DOWN_CNT]++;
		//else
			// cnt_mission[D_DOWN_CNT]=0;
		
			//reset
	if(target->x!=0&&target->y!=0)
		cnt[2]=0;
	if(cnt[2]++>3/dt)
		cnt_mission[D_DOWN_CNT]=0;
	
	if(cnt_mission[D_DOWN_CNT]>MISSION_CHECK_TIME/dt){
		cnt_mission[D_DOWN_INT]=cnt_mission[D_DOWN_CNT]=0;
		return 1;
		}
	else
    return 0;
	
	smart.rc.RST=3;	
}
// 云台跟踪API： 输入（目标，使能俯仰轴，使能航向）
//close_param 逼近参数 spd模式下为逼近速度  pos模式下为逼近距离
//close_mode=0 spd逼近  1 pos逼近 99仅云台跟踪
float flt_t=0.6;

u8 target_track_pan_task(CI* target,float close_param,u8 en_pitch,u8 en_yaw,u8 close_mode,float dt)
{
	float  xg,yg,zg,yawg;
  static float px,py;
	float ero_qr_pix[2];
	float pid_out[2];
	static float ero_qr_pixr[2];
	static u8 init;
	if(!init){init=1;
		
		#if defined(W2C_MARK)
		#if USE_CIRCLE
			track_pid[1].kp=0.55/10.23;;//yaw  
			track_pid[2].kp=0.012/2.23;//pitch
			track_pid[9].kp=1.124/2.23;//close pos
		#else
			track_pid[1].kp=0.55/1.8;//yaw  
			track_pid[2].kp=0.01;//0.012;//pitch
			track_pid[9].kp=0.4;//0.6;//close pos
		#endif
		
		#else
		track_pid[1].kp=0.55;//yaw  
		track_pid[2].kp=0.012;//pitch
		track_pid[9].kp=1.124;//close pos
		#endif
	}	
	
		
	if(target->connect&&target->check&&target->z!=0&&target->x!=0&&target->y!=0){
		//目标位置全局测量值
		float Yaw_d;
	
		#if defined(M_DRONE_330)
		float pan_pitch_angle=LIMIT(90-aux.att[0],0,90);
		#else
		float pan_pitch_angle=LIMIT(-aux.att[0],0,90);
		#endif
		//云台高度
		target->range=LIMIT(target->z/100.*tan(pan_pitch_angle*0.0173),0,5)*flt_pr+(1-flt_pr)*target->range;
		//图像信息
		target->range_xy=LIMIT(sqrt(target->x/100.*target->x/100.+target->y/100.*target->y/100.),0,5)*flt_p
		+(1-flt_p)*target->range_xy;
		//融合
		target->range_all=target->range*flt_range_pan+(1-flt_range_pan)*target->range_xy;
	  }
	
	if(target->connect&&target->check&&target->pix_x&&target->pix_x!=0&&target->pix_y!=0){
		//kalman
		px=target->pix_x*flt_t+(1-flt_t)*px;
		py=target->pix_y*flt_t+(1-flt_t)*py;
		ero_qr_pix[X]=my_deathzoom1(px-160,10);// *LIMIT(((float)target.z/100.),0,3);
		ero_qr_pix[Y]=my_deathzoom1(py-120,5);// *LIMIT(((float)target.z/100.),0,3);	
		pid_out[0]=LIMIT(ero_qr_pix[X]*track_pid[1].kp+(ero_qr_pix[X] - ero_qr_pixr[X])*track_pid[1].kd,-100,100);
		pid_out[1]=LIMIT(ero_qr_pix[Y]*track_pid[2].kp+(ero_qr_pix[Y] - ero_qr_pixr[Y])*track_pid[2].kd,-30,30);
		ero_qr_pixr[X]=ero_qr_pix[X];	
		ero_qr_pixr[Y]=ero_qr_pix[Y]; 
		
		 if(en_pitch){
			 aux.att_ctrl[0]+=pid_out[1];
		 }
		 if(en_yaw){
			 smart.rc.ATT_MODE=SMART_MODE_SPD; 
			 smart.att_rate.z=pid_out[0];
		 }
	  } 
  else{
		smart.att_rate.z=0;
	}
	
	u8 close_enough=0;
	//close control
	if(close_param>0&&target->check)
	{
		if(close_mode==0){//spd
		 smart.spd.x=0;
		 if(LIMIT(-aux.att[0],0,90)>10)
		 smart.spd.y=ABS(close_param);
		 else
		 smart.spd.y=LIMIT(-0.3*ABS(close_param),-0.2,0);
		 
		 if(-aux.att[0]<20&&-aux.att[0]>0)
			 close_enough=1;
		 smart.rc.POS_MODE=SMART_MODE_SPD;
		}
		else if(close_mode==1&&target->range_all!=0)//pos
		{
		 smart.spd.x=0;
		 smart.spd.y=-LIMIT((close_param-target->range_all)*track_pid[9].kp,-0.68,0.68);
		 smart.rc.POS_MODE=SMART_MODE_SPD;
		  if(ABS(close_param-target->range_all)<0.223)
			 close_enough=1;
//			if(-aux.att[0]<20&&-aux.att[0]>0)
//			 close_enough=1;
		}
	}
	
	
	if(close_enough&&target->check
	&&ABS(ero_qr_pix[X])<160*0.15
	&&ABS(ero_qr_pix[Y])<120*0.15)
	return 2;
	else if(target->check
	&&ABS(ero_qr_pix[X])<160*0.15
	&&ABS(ero_qr_pix[Y])<120*0.15)
		 return 1;
	else
		 return 0;
}



u8 avoid_task(CI* target,float set_x,float set_y,float set_z, float dead, float max_spd, float dt)
{
	static u16 cnt[3];
	static float x_r,y_r;
  float x=LIMIT(target->pit,-60,60)/60.; 
	float y=LIMIT(target->rol,-60,60)/60.; 
	float x_t,y_t;
 if(cnt[0]++>0.5/dt)//delay the control
		cnt[0]=cnt[1]=0;
  if(target->connect)
	{
		x_t=my_deathzoom1(set_x+x*max_spd,dead);
		y_t=my_deathzoom1(set_y+y*max_spd,dead);
		
		if(ABS(x_t)>0)
		{x_r=x_t;cnt[1]=1;}
		if(ABS(y_t)>0)
		{y_r=y_t;cnt[1]=1;}
		
		if(cnt[1])
		{  
		  smart.spd.x=x_r;
			smart.spd.y=y_r;
		}
		if(ABS(smart.spd.x)>0||ABS(smart.spd.y)>0)
			smart.rc.POS_MODE=SMART_MODE_SPD;
	}
}


u8 avoid_way_point_task(CI* target,float tar_lat,float tar_lon,float spd, float dead, float max_spd,u8 mode, float dt)
{
   float tar_Yaw;	
	 float x_t,y_t,dis;
	 if(ABS(tar_lat)>5&&ABS(tar_lon)>5){
	 tar_Yaw=To_180_degrees(navCalcBearing1(m100.Lat, m100.Lon,tar_lat,tar_lon)*57.3);	 
		if(mode)
			tar_Yaw=To_180_degrees(tar_Yaw+180);
   CalcGlobalDistance(tar_lat, tar_lon, m100.Lat, m100.Lon, &y_t,&x_t );
	 dis=sqrt(pow(x_t,2)+pow(y_t,2));	 
	 }
	 else
	 dis=99;
	 
		set_drone_yaw_task(tar_Yaw,YAW_LIMIT_RATE,dt); 
	  avoid_task(target,0,spd,0, dead, max_spd, dt) ;
	 
	 if(dis<3)
		 return 1;
	 else 
		 return 0;
}


u8 set_drone_yaw_task(float yaw,float rate_limit,float dt)
{
  static float px,py;
	float ero_qr_pix[2];
	float pid_out[2];
	static float ero_qr_pixr[2],yawf;
	static u8 init;

	smart.rc.ATT_MODE=SMART_MODE_POS; 
	smart.att.z= yaw;

	ctrl_1.zrate_limit=rate_limit;
  if(ABS(To_180_degrees(yaw-Yaw_fc))<5)
		 cnt_mission[F_YAW_CNT]++;
	else
		 cnt_mission[F_YAW_CNT]=0;
	
	if(cnt_mission[F_YAW_CNT]>MISSION_CHECK_TIME/dt){
		init=yawf=cnt_mission[F_YAW_CNT]=cnt_mission[F_YAW_INIT]=0;
		return 1;
		}
	else
    return 0;
}


u8 set_drone_z_task(float z,float dt)
{
  static float px,py;
	float ero_qr_pix[2];
	float pid_out[2];
	static float ero_qr_pixr[2];
	static u8 init;

	smart.rc.HEIGHT_MODE=SMART_MODE_POS; 
	smart.pos.z=z;

  if(ABS(z*1000-ultra_ctrl.now)<WAY_POINT_DEAD*1000)
		 cnt_mission[DRONE_Z_CNT]++;
	else
		 cnt_mission[DRONE_Z_CNT]=0;
	
	if(cnt_mission[DRONE_Z_CNT]>MISSION_CHECK_TIME/dt){
		cnt_mission[DRONE_Z_CNT]=0;
		return 1;
		}
	else
    return 0;
}
//SEL need bigger than 1
u8 traj_init_task(float ps[3],float pe[3],float T,u8 sel)
{		
			traj[sel].ps[Xr]=ps[0];//m
			traj[sel].ps[Yr]=ps[1];//m
			traj[sel].ps[Zr]=ps[2];//m
			traj[sel].defined[0]=1;traj[sel].defined[1]=1;traj[sel].defined[2]=0;
			traj[sel].Time=T;//s
			traj[sel].pe[Xr]=pe[0];//m
			traj[sel].pe[Yr]=pe[1];//m
			traj[sel].pe[Zr]=pe[2];//m
			traj[sel].time_now=0;
			plan_tra(&traj[sel]);
	    return 1;
}

//mode 1->pos  2->spd
u8 follow_traj(float yaw,u8 sel,u8 mode,float dt)
{
			traj[sel].time_now+=dt;
	    traj[sel].time_now=LIMIT(traj[sel].time_now,0,traj[sel].Time);
			get_tra(&traj[sel],traj[sel].time_now);
	if(mode==1){
		  smart.rc.POS_MODE=SMART_MODE_POS;
			smart.rc.HEIGHT_MODE=SMART_MODE_POS; 
		  smart.pos.x=traj[sel].pt[Xr];
		  smart.pos.y=traj[sel].pt[Yr];
	    smart.pos.z=traj[sel].pt[Zr];
	}else 
	{
			smart.rc.POS_MODE=SMART_MODE_SPD;
			smart.rc.HEIGHT_MODE=SMART_MODE_SPD; 
			smart.spd.y= traj[sel].vt[Yr]*cos(yaw*0.0173)+traj[sel].vt[Xr]*sin(yaw*0.0173); 
			smart.spd.x=-traj[sel].vt[Yr]*sin(yaw*0.0173)+traj[sel].vt[Xr]*cos(yaw*0.0173);
	    smart.spd.z=traj[sel].vt[Zr];
	}
	float dis=sqrt(pow(POS_UKF_X-traj[sel].pe[Xr],2)+pow(POS_UKF_Y-traj[sel].pe[Yr],2)+pow(ALT_POS_BMP_UKF_OLDX-traj[sel].pe[Zr],2));
	if(traj[sel].time_now>=traj[sel].Time||dis<WAY_POINT_DEAD1/2)
		 return 1;
	else 
		 return 0;
}

u8 take_off_task(u8 auto_diarm,float dt)
{
static float init_h;
  if(cnt_mission[COMMON_INT]==0)
	{
	   cnt_mission[COMMON_INT]=1;
		 init_h=ALT_POS_SONAR2;
	}
  if(cnt_mission[COMMON_CNT1]<10&&auto_diarm&&cnt_mission[COMMON_CNT2]==0) 
		mission_switch=PX4_MODE_DISARM;
	else if(cnt_mission[COMMON_CNT1]<20)
	{mission_switch=PX4_MODE_TAKEOFF;cnt_mission[COMMON_CNT2]=1;}
	else
	 mission_switch=PX4_MODE_OFFBOARD;
	
	cnt_mission[COMMON_CNT1]++;
 if(ALT_POS_SONAR2-init_h>0.8)
		 cnt_mission[COMMON_CNT]++;
	else
		 cnt_mission[COMMON_CNT]=0;
	
	if(cnt_mission[COMMON_CNT]>MISSION_CHECK_TIME/dt){
		cnt_mission[COMMON_CNT2]=cnt_mission[COMMON_CNT1]=cnt_mission[COMMON_CNT]=cnt_mission[COMMON_INT]=0;
		return 1;
		}
	else
    return 0;		
}

u8 land_task(float dt)
{
  static float init_h;
	
  if(cnt_mission[COMMON_CNT1]<10) 
	  mission_switch=PX4_MODE_OFFBOARD;
	else if(cnt_mission[COMMON_CNT1]<20)
	  mission_switch=PX4_MODE_LAND;
	else
	  mission_switch=PX4_MODE_OFFBOARD;

	cnt_mission[COMMON_CNT1]++;
 if(ALT_POS_SONAR2<0.3)
		 cnt_mission[COMMON_CNT]++;
	else
		 cnt_mission[COMMON_CNT]=0;
	
	if(cnt_mission[COMMON_CNT]>MISSION_CHECK_TIME/dt){
		cnt_mission[COMMON_CNT1]=cnt_mission[COMMON_CNT]=0;
		mission_switch=PX4_MODE_OFFBOARD;
		return 1;
		}
	else
    return 0;		
}



u8 take_off_task1(float set_z,float set_spd,float dt)
{
  static float init_h;
  
	smart.rc.HEIGHT_MODE=SMART_MODE_SPD; 
	smart.spd.z=set_spd;

	if(cnt_mission[COMMON_CNT]++>(set_z/set_spd)*1.123/dt&&ALT_POS_BMP_UKF_OLDX>set_z){
		cnt_mission[COMMON_CNT2]=cnt_mission[COMMON_CNT1]=cnt_mission[COMMON_CNT]=cnt_mission[COMMON_INT]=0;
		return 1;
		}
	else
    return 0;		
}

void aux_open(u8 sel,u8 open)
{
	if(open)
  aux.att_ctrl[sel]=-55;
	else
	aux.att_ctrl[sel]=10;	
}

u8 land_task1(float spd,float dt)
{
  static float init_h;
	
	if(spd!=0){
		smart.spd.z=-ABS(spd);			
		smart.rc.HEIGHT_MODE=SMART_MODE_SPD;
  }		

	if(ABS(acc_body[2])>LAND_CHECK_G*9.8&&ALT_POS_BMP_UKF_OLDX<3)
	   cnt_mission[LAND_CNT]+=4;
	else if(height_ctrl_out<350&&ALT_POS_BMP_UKF_OLDX<2)
	   cnt_mission[LAND_CNT1]++;
	else
		 cnt_mission[LAND_CNT1]=0;
	
	if(cnt_mission[LAND_CNT]>LAND_CHECK_TIME/dt||cnt_mission[LAND_CNT1]>0.35/dt){
		fly_ready=cnt_mission[LAND_CNT]=cnt_mission[LAND_CNT1]=0;
		return 1;
		}
	else
    return 0;			
}

u8 delay_task(float delay,float dt)
{
  static float init_h;
	
	if(cnt_mission[DELAY_CNT]++>delay/dt){
		cnt_mission[DELAY_CNT]=0;
		return 1;
		}
	else
    return 0;		
}


// light draw simple
u8 draw_heart(float time,float size, float dt)
{
	int divid=100;
	static float pos_st[3];
	float x,y,z,t;
	char flag=0;
  switch(cnt_mission[LIGHT_DRAW_CNT1])
	{
	  case 0:
		  pos_st[0]=POS_UKF_X;			
		  pos_st[1]=POS_UKF_Y;					
		  pos_st[2]=ALT_POS_BMP_UKF_OLDX;		
		  cnt_mission[LIGHT_DRAW_CNT1]++;
		  cnt_mission[LIGHT_DRAW_CNT3]=cnt_mission[LIGHT_DRAW_CNT2]=0;
		break;
		case 1:
			t=0;
			x=16 * pow(sin(t), 3);
		  z=13 * cos(t) - 5 * cos(2 * t) - 3 * cos(3 * t) - 1*cos(4 * t);
		  x=x*size+pos_st[0];
		  z=z*size+pos_st[2];
		  flag=set_pos_task(x,pos_st[1],z,dt);
		  if(flag)
				cnt_mission[LIGHT_DRAW_CNT1]++;
		break;
		case 2:
			mode_oldx.led_flag=1;
			t=cnt_mission[LIGHT_DRAW_CNT2]*360./divid;
			x=16 * pow(sin(t*0.0173), 3);
      z=13 * cos(t*0.0173) - 5 * cos(2 * t*0.0173) - 3 * cos(3 * t*0.0173) - 1*cos(4 * t*0.0173);
		  x=x*size+pos_st[0];
		  z=z*size+pos_st[2];
		  set_pos_task(x,pos_st[1],z,dt);
		  if(cnt_mission[LIGHT_DRAW_CNT3]++>(time/divid)/dt)
				{cnt_mission[LIGHT_DRAW_CNT2]++;cnt_mission[LIGHT_DRAW_CNT3]=0;}
		  if(t>360+360/divid)
				cnt_mission[LIGHT_DRAW_CNT1]++;
		break;
	  case 3:
			cnt_mission[LIGHT_DRAW_CNT3]=cnt_mission[LIGHT_DRAW_CNT2]=cnt_mission[LIGHT_DRAW_CNT1]=0;
			mode_oldx.led_flag=0;
		  return 1;
		break;
	}
	
	return 0;
}