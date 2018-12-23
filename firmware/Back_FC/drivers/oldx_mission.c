#include "oldx_mission.h"
#include "oldx_api.h"
#include "rc.h"
#include "imu.h"
#include "ctrl.h"
#include "filter.h"
#include "alt_fushion.h"
#include "eso.h"
#include "m100.h"
#include "quar.h"
#include "mavl.h"
#include "usart.h"
//跟踪对准降落
u8 mission_test4(float dt)
{
	u8 flag=0,mission_finish=0,temp=0;
	switch(mission_state){
	case 0:pan_set_task(45,dt);
				 flag=set_drone_z_task(0.88,dt);
				 if(cnt_task[0]++>1.5/dt)
				 {mission_state=101;cnt_task[0]=0;}	 	
		     break;	
  case 1:				 
				 est_target_state(&c2c, &three_wheel_car,dt);
	       temp=target_track_pan_task(&c2c,0.2,1,1,1,dt);//pos逼近
	       set_drone_z_task(0.88,dt);//高度保持
	       if(temp==2)
					cnt_task[1]++;
				 else
					cnt_task[1]=0; 
				 if(cnt_task[1]>3)
					{cnt_task[1]=0;flag=1;}
				
				 if(c2c.check&&c2c.connect)cnt_task[0]=0;
				 else if(cnt_task[0]++>3.5/dt)
				 {mission_state=101;cnt_task[0]=cnt_task[1]=cnt_mission[9]=0;}	 
		     break;
  case 2:est_target_state(&c2c, &three_wheel_car,dt);
		     pan_set_task(90,dt);
				 if(cnt_task[0]++>2/dt)
				 {flag=1;cnt_task[0]=0;}
	       break;
	case 3:flag=spd_move_task(0,0.4,0,2,dt);
				 break;
	case 4:				
					est_target_state(&c2c, &three_wheel_car,dt); 
					target_track_downward_task(&c2c,-0.2,0.2,1,dt);
	       break;
	case 9:flag=land_task(dt);
		     if(flag)mission_finish=1;
				 break;
	case 101:pan_search_task(20,80,45,2,1,dt);
				   set_drone_z_task(0.88,dt);//高度保持
    		 if(c2c.check&&c2c.connect)cnt_task[0]++;
				 else cnt_task[0]=0;
				 if(cnt_task[0]>3)
				 {cnt_task[0]=0;mission_state=1;cnt_mission[9]=3;}
				 break;			 
	default:break;
	}
	if(flag)
		mission_state++;
	
	if(mission_finish)
	  return 1;		
  else
    return 0;		
}	



//跟踪逼近观测估计 最后直接用位置降落
u8 mission_test5(float dt)
{ three_wheel_car.pos_off[0]=0;
	three_wheel_car.pos_off[1]=0;
	float Yaw_d;
	float platform_height=0.13;//0.2;
	static u8 flag1;
	static u16 cnt_delay=0;
	static float last_pos[2];
	#if defined(USE_UWB)
	if(m100.uwb_f[0]!=0&&m100.uwb_f[1]!=0)
	Yaw_d=m100.uwb_f[3];
	else
	Yaw_d=Yaw_fc;
	#else
	Yaw_d=Yaw_fc;
	#endif

	u8 flag=0,mission_finish=0,temp=0;
	float dis=0;
	switch(mission_state){
	case 0:
				 //pan_set_task(45,dt);
				 pan_set_task(25,dt);
				 //flag=set_drone_z_task(0.88,dt);
				 flag=set_drone_z_task(0.73,dt);
				 if(cnt_task[0]++>1.5/dt)
				 {mission_state=101;cnt_task[0]=0;
				 }	 	
		     break;	
  case 1:	//远距离逼近	
				 est_target_state(&c2c, &three_wheel_car,dt); 
	       temp=target_track_pan_task(&c2c,0.5,1,1,1,dt);//pos逼近
	       set_drone_z_task(0.6+platform_height,dt);
	       if(temp==2)
					cnt_task[1]++;
				 else
					cnt_task[1]=0; 
				 if(cnt_task[1]>3)
					{cnt_task[0]=cnt_task[1]=0;flag=1;}
				 if(c2c.check&&c2c.connect)cnt_task[0]=0;
				 else if(cnt_task[0]++>3.5/dt)
				 {mission_state=101;cnt_task[0]=cnt_task[1]=cnt_mission[9]=0;}	 
		     break;
  case 2: //近距离观测
		     c2c.use_circle=0;
				 est_target_state(&c2c, &three_wheel_car,dt); 
	       temp=target_track_pan_task(&c2c,0.3,1,1,1,dt);//pos逼近
				 //set_drone_z_task(0.66,dt);//高度保持
				 set_drone_z_task(0.45+platform_height,dt);//高度保持
					if(temp==2)
					cnt_task[1]++;
				 if((cnt_task[0]++>4.5/dt&&cnt_task[1]>1)||(cnt_task[0]>2/dt&&cnt_task[1]>10))
				 {flag=1;cnt_task[0]=cnt_task[1]=0;}
	       break;
	case 3:	//轨迹规划降落
				  dis=est_target_state(&c2c, &three_wheel_car,dt); 
		      target_track_pan_task(&c2c,0.1,1,1,99,dt);	 
					//set_drone_z_task(0.4,dt);//高度保持
					if(c2c.gx!=0&&c2c.gy!=0){
						float off[2];
						off[0]=sin(Yaw_d*0.0173)*0.15;
						off[1]=cos(Yaw_d*0.0173)*0.15;
						set_pos_task(c2c.gx+off[0],c2c.gy+off[1],0.3,dt);
						if(dis<0.3&&dis!=0)
						spd_move_task(0,0,-0.3,10,dt);	
						else
						spd_move_task(0,0,-0.2,10,dt);
					}
	       break;
	case 101:
		       pan_search_task(5,35,20,2,1,dt);//pan_search_task(20,70,20,2,0,dt);
				   set_drone_z_task(0.73,dt);
    		 if(c2c.check&&c2c.connect)cnt_task[0]++;
				 else cnt_task[0]=0;
				 if(cnt_task[0]>3)
				 {cnt_task[0]=0;mission_state=1;last_pos[0]=POS_UKF_X;last_pos[1]=POS_UKF_Y;}
				 break;			 
	default:break;
	}
	
	static u8 smooth_state;
	static float spd_reg;
	switch(smooth_state)
	{
		case 0:
			if(mission_state==1&&smart.spd.y!=0)
			{smooth_state=1;spd_reg=smart.spd.y;}
		break;
		case 1:
			if(mission_state==1&&smart.spd.y!=0)
			{smooth_state=2;cnt_delay=0;}
		break;
		case 2:
			if(smart.spd.y==0){
			 smart.rc.POS_MODE=SMART_MODE_SPD;
		   smart.spd.y=0.2;}
			if(cnt_delay++>2/dt||smooth_state!=1)
				smooth_state=0;
		break;
	}
	
	if(flag)
		mission_state++;
	
	if(mission_finish)
	  return 1;		
  else
    return 0;		
}	


//数据存储
u8 mission_test8(float dt)
{ three_wheel_car.pos_off[0]=0;
	three_wheel_car.pos_off[1]=0;
	float Yaw_d;
	float platform_height=0.13;//0.2;
	static u8 flag1;
	static u16 cnt_delay=0;
	static float last_pos[2];
	#if defined(USE_UWB)
	if(m100.uwb_f[0]!=0&&m100.uwb_f[1]!=0)
	Yaw_d=m100.uwb_f[3];
	else
	Yaw_d=Yaw_fc;
	#else
	Yaw_d=Yaw_fc;
	#endif

	u8 flag=0,mission_finish=0,temp=0;
	float dis=0;
	switch(mission_state){
	case 0:
				 pan_set_task(25,dt);
				 flag=set_drone_z_task(0.73,dt);
				 if(cnt_task[0]++>1.5/dt)
				 {mission_state=101;cnt_task[0]=0;
				 }	 	
		     break;	
  case 1:	//远距离逼近	
				 est_target_state(&c2c, &three_wheel_car,dt); 
	       temp=target_track_pan_task(&c2c,0.5,1,1,1,dt);//pos逼近     
		     break;
	default:break;
	}
	
	if(flag)
		mission_state++;
	
	if(mission_finish)
	  return 1;		
  else
    return 0;		
}	

u8 mission_landing1(float dt)
{ 
	u8 flag=0,mission_finish=0,temp=0;
	float dis=0;
	switch(mission_state){
	case 0:
			   flag=take_off_task1(2,0.5,dt);
		     break;
	case 1:
				 pan_set_task(90,dt);
				 flag=set_drone_z_task(3.5,dt);
		     break;
	case 2:
				 flag=spd_move_task(0,0.4,0,6,dt);
		     break;
	case 3:
		     flag=return_home(8,1,NEN_LAND,NEN_HEAD_WAY,dt);
	       break;
	case 4:
				 flag=target_track_downward_task(&c2c,0.25,0.45,MODE_SPD,dt);
	       break;
	case 5:
		     flag=land_task1(LAND_SPD,dt);
	       break;
	case 6:mission_finish=1;
	       break;
	default:break;
	}
	
	if(flag)
		mission_state++;
	
	if(mission_finish)
	  return 1;		
  else
    return 0;		
}	

u8 mission_landing(float dt)
{ 
	u8 flag=0,mission_finish=0,temp=0;
	float dis=0;
	switch(mission_state){
	case 0:
			   flag=take_off_task1(2,0.5,dt);
		    break;
	case 1:
				 pan_set_task(90,dt);
				 flag=set_drone_z_task(3.5,dt);
		     break;
	case 2:
				 flag=target_track_downward_task(&c2c,0.25*1,0.45,MODE_SPD,dt);
	       break;
	case 3:
		     flag=land_task1(LAND_SPD,dt);
	       break;
	case 4:mission_finish=1;
	       break;
	default:break;
	}
	
	if(flag)
		mission_state++;
	
	if(mission_finish)
	  return 1;		
  else
    return 0;		
}	

u8 mission_light_draw(float dt)
{ 
	u8 flag=0,mission_finish=0,temp=0;
	float dis=0;
	switch(mission_state){
	case 0:
			   flag=take_off_task1(2,0.5,dt);
		    break;
	case 1:
				 pan_set_task(90,dt);
				 flag=set_drone_z_task(8,dt);
		     break;	
  case 2:			
	       flag=draw_heart(60, 0.25, dt);
		     break;
	case 3:
				 flag=return_home(8,1.1,EN_LAND,NEN_HEAD_WAY,dt);
	       break;
	case 4:mission_finish=1;
	       break;
	default:break;
	}
	
	if(flag)
		mission_state++;
	
	if(mission_finish)
	  return 1;		
  else
    return 0;		
}	



u8 mission_test_gps(float dt)
{ 
	u8 flag=0,mission_finish=0,temp=0;
	static float mission_time;
	pan_set_task(90,dt);
	float dis=0;
	switch(mission_state){
	case 0:
			   flag=take_off_task1(2,1.5,dt);
		    break;
	case 1:
				 pan_set_task(90,dt);
				 flag=set_drone_z_task(3,dt);
	       mission_time=0;
		     break;	
  case 2:			
	       flag=way_point_mission(1,NEN_HEAD_WAY,dt);
		     break;
	case 3:
				 flag=return_home(10,1,NEN_LAND,NEN_HEAD_WAY,dt);
	       break;
	case 4:
				 flag=set_drone_z_task(5.5,dt);
	       break;
	case 5:
		     mode_oldx.sdk_flag=0;//en_land
				 flag=target_track_downward_task(&c2c,0.3,0.15,MODE_SPD,dt);
				 if(c2c.check)
				  cnt_task[0]=0;
				 else
				  cnt_task[0]++;
				 if(cnt_task[0]>30/dt)
				 {mission_state=101;cnt_task[0]=cnt_task[1]=cnt_task[2]=0;}
				
				 if(cnt_task[1]++>40/dt)
				 {flag=1;cnt_task[0]=cnt_task[1]=cnt_task[2]=0;}
				 break;
	case 6:
				 flag=target_track_downward_task(&c2c,0.2,0.15,MODE_SPD,dt);
	       break;
	case 7:
		     flag=land_task1(LAND_SPD,dt);
	       break;
	case 8:mission_finish=1;
	       break;
	//
	case 101://搜索降落标志
		    // r, z, d_angle, d_r, dt
				//down_ward_search_task(3,6.6,10,0,dt);//circle
	      mode_oldx.sdk_flag=0;//
	      flag=down_ward_search_task_tangle(6,6,3,3,0.4,5,dt);//tangle
			  if(c2c.check)
					  cnt_task[0]++;
				if(cnt_task[0]>5)
				{ mission_state=6;cnt_task[2]=cnt_task[0]=cnt_task[1]=0;}
				if(cnt_task[2]++>60/dt||flag)
				{mission_state=102;cnt_task[2]=cnt_task[0]=cnt_task[1]=0;flag=0;}
	      break;
	case 102://搜索失败返航降落
				 if(return_home(4,0.8,NEN_LAND,NEN_HEAD_WAY,dt))
						mission_state=6;
				 if(c2c.check)
					  cnt_task[0]++;
				 if(cnt_task[0]>5)
						{ mission_state=6;cnt_task[2]=cnt_task[0]=cnt_task[1]=0;}
	       break;
	default:break;
	}
	
	if(flag)
		mission_state++;
	
	mission_time+=dt;
	if(mission_time>7*60&&(mission_state>100||mission_state==2))//超时强制返航
		mission_state=102;
	
	if(mission_finish)
	  return 1;		
  else
    return 0;		
}	
float lat_human,lon_human;
u8 mission_search1(float dt)
{ 
	u8 flag=0,mission_finish=0,temp=0;
	float dis=0;
	static float mission_time;
	switch(mission_state){
	case 0:flag=take_off_task1(2,1.5,dt);
		    break;
	case 1:pan_set_task(90,dt);
				 flag=set_drone_z_task(6,dt);
		     mission_time=0;
		     break;	
  case 2:mode_oldx.sdk_flag=1;//en_search      
			   flag=way_point_mission(1.6,NEN_HEAD_WAY,dt);
	       if(c2c.connect&&					 
					 c2c.x<160+90&&c2c.x>160-90&&
				   c2c.y<120+90&&c2c.y>120-90)
					 cnt_task[5]++;
				 
				 if(cnt_task[5]>20&&cnt_task[6]==0)//to track target
					{mission_state=102;cnt_task[0]=cnt_task[1]=cnt_task[2]=cnt_task[5]=flag=0;}//to search
		     break;
	case 3://航点寻找目标
		    //w, h, wn, hn, spd, z,dt;
	       //mission_state=4;
	       mode_oldx.sdk_flag=1;//en_search
				 flag=down_ward_search_task_tangle(10,10,3,3,0.6,7.5,dt);
				 if(c2c.connect&&
					 c2c.x<160+90&&c2c.x>160-90&&
				   c2c.y<120+90&&c2c.y>120-90)
					 cnt_task[5]++;
				 
				 if(cnt_task[5]>6&&cnt_task[6]==0)//to track target
					{mission_state=102;cnt_task[0]=cnt_task[1]=cnt_task[2]=flag=0;}//to search
					
				 if(flag&&cnt_task[6]==0)//repeat
				  {mission_state=2;cnt_mission[MISSION_CNT]=cnt_task[5]=cnt_task[0]=cnt_task[1]=cnt_task[2]=flag=0;}
					
				 if(cnt_task[6]==1)//drop finish check
					 flag=1;
	       break;
  //------------------------------------------------------
	case 4:flag=return_home(12,2,NEN_LAND,NEN_HEAD_WAY,dt);
				 if(flag)
					cnt_task[0]=cnt_task[1]=cnt_task[2]=0;
	       break;
	case 5:flag=set_drone_z_task(6,dt);
				 break;
	case 6:mode_oldx.sdk_flag=0;//en_land
				 flag=target_track_downward_task(&c2c,0.4,0.66,MODE_SPD,dt);
				 if(c2c.check)
				  cnt_task[0]=0;
				 else
				  cnt_task[0]++;
				 
				 if(cnt_task[0]>10/dt)//超时搜索
				 {mission_state=11;cnt_task[0]=cnt_task[1]=cnt_task[2]=0;
				 cnt_mission[C_SEARCH_T_INT]=cnt_mission[C_SEARCH_T_INT1]=flag=0;}
				
				 if(cnt_task[1]++>15/dt)//超时降落
				 {flag=1;cnt_task[0]=cnt_task[1]=0;}
				 break;
	case 7:mode_oldx.sdk_flag=0;//en_land
				 flag=target_track_downward_task(&c2c,0.25,0.66,MODE_SPD,dt);
	       break;
	case 8:flag=land_task1(LAND_SPD,dt);
	       break;
	case 9:mission_finish=1;
	       break;
	//-----------------------AUX MISSION----------------------------
	case 11://搜索 降落标志
		    // r, z, d_angle, d_r, dt
	      mode_oldx.sdk_flag=0;//
	      flag=down_ward_search_task_tangle(8,8,4,4,0.4,5,dt);//tangle
			  if(c2c.check)
					 cnt_task[0]++;
				
				if(cnt_task[0]>5)
					{cnt_task[0]=0;mission_state=6;flag=0;}
					
				if(cnt_task[2]++>60/dt||flag)
				{mission_state=12;cnt_task[2]=0;flag=0;}
	      break;
	case 12://搜索失败返航
			 if(return_home(4,0.8,NEN_LAND,NEN_HEAD_WAY,dt))
					mission_state=7;
			 break;		 	 			
	case 13://超时直接投递			
				flag=way_point_mission(1,NEN_HEAD_WAY,dt);	
        if(flag)
				{flag=0;aux_open(2,1);mission_state=4;}					
	break;			
	//--------------------
	case 102://搜索 投递目标
		    mode_oldx.sdk_flag=1;//
		    flag=target_track_downward_task_search(&c2c,0.25,4.5,dt);
				if(cnt_task[2]++>30/dt)
				{mission_state=3;cnt_task[2]=flag=cnt_task[5]=0;}
				
				if(flag)//drop and return 
				{mission_state=4;//to return home
				 cnt_task[2]=flag=0;
				 aux_open(2,1);
				 cnt_task[6]=1;//drop flag
				}
	      break;
				
	default:break;
	}
	

	if(flag)
		mission_state++;
	
	mission_time+=dt;
	if(mission_time>3*60&&(mission_state>=100||mission_state==3))//超时强制返航
	{mission_state=13;cnt_mission[C_SEARCH_T_INT]=cnt_mission[C_SEARCH_T_INT1]=0;}
	
	if(mission_finish)
	  return 1;		
  else
    return 0;		
}	



u8 mission_search(float dt)
{ 
	u8 flag=0,mission_finish=0,temp=0;
	float dis=0;
	static float mission_time;
	switch(mission_state){
	case 0:flag=take_off_task1(2,1.5,dt);
		     lat_human=lon_human=0;
		    break;
	case 1:pan_set_task(90,dt);
				 flag=set_drone_z_task(15,dt);
		     mission_time=0;
		     break;	
  case 2:mode_oldx.sdk_flag=1;//en_search      
			   flag=way_point_mission(2,NEN_HEAD_WAY,dt);
	       if(c2c.connect&&					 
					 c2c.x<160+90&&c2c.x>160-90&&
				   c2c.y<120+90&&c2c.y>120-90)
					 cnt_task[5]++;
				 
				 if(cnt_task[5]>20&&cnt_task[6]==0)//to track target
					{mission_state=102;cnt_task[0]=cnt_task[1]=cnt_task[2]=cnt_task[5]=flag=0;}//to search
		     break;
	case 3://航点寻找目标
		    //w, h, wn, hn, spd, z,dt;
	       //mission_state=4;
	       mode_oldx.sdk_flag=1;//en_search
				 flag=down_ward_search_task_tangle(10,10,3,3,1,15-3,dt);
				 if(c2c.connect&&
					 c2c.x<160+90&&c2c.x>160-90&&
				   c2c.y<120+90&&c2c.y>120-90)
					 cnt_task[5]++;
				 
				 if(cnt_task[5]>6&&cnt_task[6]==0)//to track target
					{mission_state=102;cnt_task[0]=cnt_task[1]=cnt_task[2]=flag=0;}//to search
					
				 if(flag&&cnt_task[6]==0)//repeat
				  //{mission_state=2;cnt_mission[MISSION_CNT]=cnt_task[5]=cnt_task[0]=cnt_task[1]=cnt_task[2]=flag=0;}
					{mission_state=13;cnt_mission[MISSION_CNT]=cnt_task[5]=cnt_task[0]=cnt_task[1]=cnt_task[2]=flag=0;}
					
				 if(cnt_task[6]==1)//drop finish check
					 flag=1;
	       break;
  //------------------------------------------------------
	case 4:flag=return_home(15,2,NEN_LAND,NEN_HEAD_WAY,dt);
				 aux_open(2,1);
				 if(flag)
					cnt_task[0]=cnt_task[1]=cnt_task[2]=0;
	       break;
	case 5:flag=set_drone_z_task(6,dt);
				 break;
	case 6:mode_oldx.sdk_flag=0;//en_land
				 flag=target_track_downward_task(&c2c,0.45,0.66,MODE_SPD,dt);
				 if(c2c.check)
				  cnt_task[0]=0;
				 else
				  cnt_task[0]++;
				 
				 if(cnt_task[0]>10/dt)//超时搜索
				 {mission_state=11;cnt_task[0]=cnt_task[1]=cnt_task[2]=0;
				 cnt_mission[C_SEARCH_T_INT]=cnt_mission[C_SEARCH_T_INT1]=flag=0;}
				
				 if(cnt_task[1]++>15/dt)//超时降落
				 {flag=1;cnt_task[0]=cnt_task[1]=0;}
				 break;
	case 7:mode_oldx.sdk_flag=0;//en_land
				 flag=target_track_downward_task(&c2c,0.25,0.66,MODE_SPD,dt);
	       break;
	case 8:flag=land_task1(LAND_SPD,dt);
	       break;
	case 9:mission_finish=1;
	       break;
	//-----------------------AUX MISSION----------------------------
	case 11://搜索 降落标志
		    // r, z, d_angle, d_r, dt
	      mode_oldx.sdk_flag=0;//
	      flag=down_ward_search_task_tangle(9,9,3,3,0.68,6.6,dt);//tangle
			  if(c2c.check)
					 cnt_task[0]++;
				
				if(cnt_task[0]>5)
					{cnt_task[0]=0;mission_state=6;flag=0;}
					
				if(cnt_task[2]++>60/dt||flag)
				{mission_state=12;cnt_task[2]=0;flag=0;}
	      break;
	case 12://搜索失败返航
			 if(return_home(4,0.8,NEN_LAND,NEN_HEAD_WAY,dt))
					mission_state=7;
			 break;		 	 			
	case 13://超时直接投递			
				flag=way_point_mission(1,NEN_HEAD_WAY,dt);	
        if(flag)
				{flag=0;aux_open(2,1);mission_state=4;
				  lat_human=m100.Lat;lon_human=m100.Lon;
				}					
	break;			
	//--------------------
	case 102://搜索 投递目标
		    mode_oldx.sdk_flag=1;//
		    flag=target_track_downward_task_search(&c2c,0.25,15-3,dt);
				if(cnt_task[2]++>30/dt)
				{mission_state=3;cnt_task[2]=flag=cnt_task[5]=0;}
				
				if(flag)//drop and return 
				{mission_state=4;//to return home
				 cnt_task[2]=flag=0;
				 aux_open(2,1);
				 cnt_task[6]=1;//drop flag
				}
	      break;
				
	default:break;
	}
	

	if(flag)
		mission_state++;
	
	mission_time+=dt;
	if(mission_time>5*60&&(mission_state>=100||mission_state==3))//超时强制返航
	{mission_state=13;cnt_mission[C_SEARCH_T_INT]=cnt_mission[C_SEARCH_T_INT1]=0;}
	
	if(mission_finish)
	  return 1;		
  else
    return 0;		
}	

u8 mission_avoid(float dt)
{ 
	u8 flag=0,mission_finish=0,temp=0;
	float lat,lon;
	static float mission_time;
	pan_set_task(90,dt);
	float dis=0;
	switch(mission_state){
	case 0:
			   flag=take_off_task1(1.68,1.5,dt);
		    break;
	case 1:
				 pan_set_task(90,dt);
				 flag=set_drone_z_task(1.68,dt);
	       mission_time=0;
		     break;	
	case 2:
		     mode_oldx.sdk_flag=1;//en_search
		     set_drone_z_task(1.5,dt);
		     if(Rc_Get_PWM.AUX1>1500){
					//avoid_task(&c2c,0,0.35,0,0.068,0.68,dt);
				 	lat=navData.missionLegs[LIMIT(navData.Leg_num-1,0,99)].targetLat;
					lon=navData.missionLegs[LIMIT(navData.Leg_num-1,0,99)].targetLon;
				  flag=avoid_way_point_task(&c2c,lat,lon,0.25,0.2,0.5,0,dt);
				  }
				 	if(c2c.check)
				  cnt_task[0]++;
				 
				  if(cnt_task[0]>5){
						aux_open(2,1);
					  flag=1;
					}
	       break;	
	case 3:mode_oldx.sdk_flag=0;//en_search
		     set_drone_z_task(1.5,dt);
		     if(Rc_Get_PWM.AUX1>1500){
				 	lat=home_lat;
					lon=home_lon;
				  avoid_way_point_task(&c2c,lat,lon,-0.25,0.25,0.5,1,dt);
				  }
	       break;					
	default:break;
	}
	
	if(flag)
		mission_state++;
	
	mission_time+=dt;
	if(mission_time>7*60&&(mission_state>100||mission_state==2))//超时强制返航
		mission_state=3;
	
	if(mission_finish)
	  return 1;		
  else
    return 0;		
}	
