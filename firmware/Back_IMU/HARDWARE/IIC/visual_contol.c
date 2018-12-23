#include "circle.h"
#include "usart_fc.h"
#include "alt_kf.h"
#include "filter.h"
CIRCLE circle;
float nav_circle[2],nav_circle_last[2];
/*
        y add (0~256)  1
        |
        |


        -------- x decrease (0~320)  0


*/
float circle_check=0.01;
float circle_lfp=1;

void circle_control(float T)
{static u8 state;
 u8 i;
 static int circle_reg[2],circle_use[2];
 float p,d,intit,rate_error[2],derivative[2];
 static float last_error[2],last_derivative[2];
 float  distance_use;
 static u16 cnt[3];	
 float out[2];
//state	
	switch(state){
		case 0:if(circle.check&&circle.connect)
			       cnt[0]++;
		       else
						 cnt[0]=0;
		       if(cnt[0]>circle_check/T)
					 {state=1;cnt[0]=0;}
		  break;
		case 1:
			     if(!circle.check||!circle.connect)
						 state=0;
					 
		      break;
		default:state=0;break;
	}
//output	
	switch(state){
		case 0:circle_use[0]=MID_X;circle_use[1]=MID_Y;break;
		case 1:circle_use[0]=circle.x;circle_use[1]=circle.y;break;
		default:circle_use[0]=MID_X;circle_use[1]=MID_Y;break;
	}
	 //if(circle.check&&circle.connect){
	 circle_use[0]=circle.x_flp;circle_use[1]=circle.y_flp; //}
//	 else{
//	 circle_use[0]=MID_X;circle_use[1]=MID_Y;}
//	if(ALT_POS_SONAR2<0.15)
//		 distance_use=0.8;
//	else
//		distance_use=ALT_POS_SONAR2;
		//circle_use[0]=circle.x_flp-160;
	   circle_use[0]-=MID_X;
		//circle_use[1]=circle.y_flp-128;
	   circle_use[1]-=MID_Y;
		rate_error[0]=circle_use[0]*distance_use;
		rate_error[1]=circle_use[1]*distance_use;
	static float integrator_circle[2];
 if(pid.circle.in.i==0)
 {
 integrator_circle[0]=integrator_circle[1]=0;
 }
	 for(i=0;i<=1;i++){
			p = rate_error[i]*pid.circle.in.p;
			derivative[i] = (rate_error[i] - last_error[i]) ;/// DT;
			derivative[i]=0.3* derivative[i]+ 0.7*last_derivative[i];
			// update state
			last_error[i] = rate_error[i] ;
			last_derivative[i]= derivative[i];
			// add in derivative component
			d = derivative[i]*pid.circle.in.d;//d
			  integrator_circle[i] += ((float) rate_error[i]* pid.circle.in.i);// *DT;
        intit = LIMIT(integrator_circle[i],-10,10) ;
			//nav_circle[i] = p + d;
  		out[i] =circle_lfp*(p+d+intit)+(1-circle_lfp)*nav_circle_last[i];
		  nav_circle_last[i]=out[i]; 
		  //nav_circle[i]=out[i];
		  //nav_circle[i] += 0. *T *3.14f * ( -nav_circle[i] + out[i] );
 	 }
	 
	 
	 int flag[2];
		 
	 for(i=0;i<2;i++)
   {
	 if(circle_use[i]>0)
	 flag[i]=1;
	 else if(circle_use[i]<0)
	 flag[i]=-1;
	 else
	 flag[i]=0; 
	 }	 
	 if(SPID.YP==1) //2
	 flag[0]=1;//nav_circle[0]=flag[0]*(float)SPID.YD/10;
	 else  if(SPID.YP==2)
	 flag[0]=-1;//*(float)SPID.YD/10;
	 else
		flag[0]=0; 
	 nav_circle[0]=LIMIT(flag[0]*circle_use[0]*pid.circle.in.p,-10,10);
 
	 if(SPID.YI==1) //2
	 flag[1]=1;//nav_circle[1]=flag[1]*(float)SPID.YD/10;
	 else  if(SPID.YI==2)
	 flag[1]=-1;//nav_circle[1]=-flag[1]*(float)SPID.YD/10;
	 else
		flag[1]=0; 
	 nav_circle[1]=LIMIT(flag[1]*circle_use[1]*pid.circle.in.p,-10,10);
	 
	 if(!circle.check)
	 nav_circle[0]=nav_circle[1]=0;
// nav_circle[0] =Moving_Median(18,10,out[0]);
// nav_circle[1] =Moving_Median(19,10,out[1]);	 
}


#define NAV_ANGLE_MAX 0.8*MAX_CTRL_ANGLE
float nav[2];
float GPS_angle[2];
float  target_position[2];
float  now_position[2];
float actual_speed[2];
float tar_speed[2],tar_speed_avoidc[2];
float d_flow_watch[2];
float  integrator[2];
void GPS_calc_poshold(float T)//  
{
	float p, i, d;
	float output;
	float target_speed;
	static 	float last_derivative[2];
	float derivative[2];
	static int32_t last_error[2];
	int axis;
	float error_pos[2], rate_error[2];
	float cos_yaw,sin_yaw;	
			/*
		north    LAT=1     V_West+                             __________
		|   Y+  y                                              P- R- GPS-
		                                        
		|P+
	
    _____	R	+   x         LON=0  V_East+
	

	   
head  |    1 PIT y-
			| 
		   _____  0 ROL x+
	
		*/
		if(pid.nav.in.i==0)
		{integrator[1]=integrator[0]=0;}
	if(mode.flow_hold_position_use_global){	
		 actual_speed[LON]=my_deathzoom_2(imu_nav.flow.speed.east,pid.nav.in.dead);//imu_nav.flow.speed.x;//mm
	   actual_speed[LAT]=my_deathzoom_2(imu_nav.flow.speed.west,pid.nav.in.dead);//imu_nav.flow.speed.y;
		 now_position[LON]=imu_nav.flow.position.east;
	   now_position[LAT]=imu_nav.flow.position.west;
	}
	else
	{ if(!mode.flow_sel){
   	 actual_speed[LON]=my_deathzoom_2(imu_nav.flow.speed.y_f,pid.nav.in.dead);//imu_nav.flow.speed.x;//mm
	   actual_speed[LAT]=my_deathzoom_2(imu_nav.flow.speed.x_f,pid.nav.in.dead);//imu_nav.flow.speed.y;
		//if(ultra_dis_lpf>650)
		//	{
		 //pid.nav.in.dead2=10*pid.nav.in.dead;
	   now_position[LON]+=my_deathzoom_2(imu_nav.flow.speed.y_f,pid.nav.in.dead2)*T;
	   now_position[LAT]+=my_deathzoom_2(imu_nav.flow.speed.x_f,pid.nav.in.dead2)*T;
	}
	else
	{
	   actual_speed[LON]=my_deathzoom_2(imu_nav.flow.speed.east,pid.nav.in.dead);//imu_nav.flow.speed.x;//mm
	   actual_speed[LAT]=my_deathzoom_2(imu_nav.flow.speed.west,pid.nav.in.dead);//imu_nav.flow.speed.y;
		//if(ultra_dis_lpf>650)
		//	{
		 //pid.nav.in.dead2=10*pid.nav.in.dead;
	   now_position[LON]+=my_deathzoom_2(imu_nav.flow.speed.east,pid.nav.in.dead2)*T;
	   now_position[LAT]+=my_deathzoom_2(imu_nav.flow.speed.west,pid.nav.in.dead2)*T;
	}
		//}
		 //pid.nav.in.p=0.55  
	}
	  
    for (axis = 0; axis < 2; axis++) {
			//loc p
			  error_pos[axis]=my_deathzoom_2(target_position[axis]-now_position[axis],pid.nav.out.dead);//tar-now 
			//if(mode.sonar_avoid&&need_avoid)
      //  tar_speed[axis]=target_speed =tar_speed_avoid[axis]*pid.nav.out.p; 
			//else  
			if((fabs(tar_speed_avoidc[0])>10||fabs(tar_speed_avoidc[1])>10)&&mode.sonar_avoid){
				switch(axis){
				case 0:
				tar_speed[1]=target_speed=  tar_speed_avoidc[1];break;
				case 1:
				tar_speed[0]=target_speed=  tar_speed_avoidc[0];break;
			}}
			else
				tar_speed[axis]=target_speed = error_pos[axis]*pid.nav.out.p;       // calculate desired speed from lon error
			  
        rate_error[axis] = target_speed - actual_speed[axis];   // calc the speed error
       //-----------rad
        p = rate_error[axis]*pid.nav.in.p;
	     if(rate_error[axis]<0.150)
			  integrator[axis] += ((float) rate_error[axis]* pid.nav.in.i);// *DT;

        i = LIMIT(integrator[axis],-MAX_FIX_ANGLE,MAX_FIX_ANGLE) ;
		
		
		 derivative[axis] = (rate_error[axis] - last_error[axis]) ;/// DT;
    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    // derivative[axis] = last_derivative[axis] + (DT / (AC_PID_FILTER + DT)) * (derivative[axis] - last_derivative[axis]);
		 derivative[axis]=0.3* derivative[axis]+ 0.7*last_derivative[axis];
    // update state
    last_error[axis] = rate_error[axis] ;
    last_derivative[axis]= derivative[axis];
    // add in derivative component
       d_flow_watch[axis]= d = derivative[axis]*pid.nav.in.d;//d
		
        //d = constrain_int16(d, -2000, 2000);
        // get rid of noise
        if (fabs(actual_speed[axis]) < pid.nav.in.dead)
            d = 0;
        output = p + i + d;

        GPS_angle[axis] = limit_mine(output,NAV_ANGLE_MAX);//µ¼º½¿ØÖÆ½Ç¶È
     
    }
	
		if(mode.flow_hold_position_use_global){	
		cos_yaw=cos(Yaw*0.017);
		sin_yaw=sin(Yaw*0.017);
			nav[PITr] = -(GPS_angle[LAT] * cos_yaw + GPS_angle[LON] * sin_yaw);// / 10;
			nav[ROLr] = +(GPS_angle[LON] * cos_yaw - GPS_angle[LAT] * sin_yaw);/// 10;
		}
		else{
			nav[PITr] = -GPS_angle[LAT];
			nav[ROLr] = +GPS_angle[LON];
		}
		
		
		 limit_mine( (nav[PITr]) ,NAV_ANGLE_MAX );
		 limit_mine( (nav[ROLr]) ,NAV_ANGLE_MAX );
		/*
		north    LAT=1     V_West+                             __________
		|   Y+  y                                              P- R- GPS-
		                                        
		|P+
	
    _____	R	+   x         LON=0  V_East+
	   
head  |    1 PIT y-
			| 
		   _____  0 ROL x+
	
		*/
}



