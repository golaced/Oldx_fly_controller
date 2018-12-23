
#include "att.h"
#include "height_ctrl.h"
#include "ultrasonic.h"
#include "flash.h"
#include "pwm_out.h"
#include "alt_kf.h"
#include "eso.h"
#include "neuron_pid.h"
#include "circle.h"
ctrl_t ctrl_1;
ctrl_t ctrl_2;
ctrl_t ctrl_2_fuzzy;
// Calculate nav_lat and nav_lon from the x and y error and the speed

float dj_angle,dj_angle_offset[3]={-2,3,9},dj_angle_set;
#define MAX_FIX_ANGLE_DJ 13
void DJ_offset_save(void)
{
//static u8 moder;

//if(mode.en_dj_cal)
//{
//dj_angle_offset[0]=(float)(Rc_Get.AUX1-500)/1000.*MAX_FIX_ANGLE_DJ*2;
//dj_angle_offset[1]=(float)(Rc_Get.AUX2-500)/1000.*MAX_FIX_ANGLE_DJ*2;
//dj_angle_offset[2]=(float)(Rc_Get.AUX3-500)/1000.*MAX_FIX_ANGLE_DJ*4;
//}
//else if(mode.en_dj_cal==0&&moder==1)
//	WRITE_PARM();
//moder=mode.en_dj_cal;
}



void Ctrl_Para_Init()		//设置默认参数
{
//====================================
	ctrl_1.PID[PIDROLL].kdamp  = 1;
	ctrl_1.PID[PIDPITCH].kdamp = 1;
	ctrl_1.PID[PIDYAW].kdamp 	 = 1;
	
	ctrl_1.FB = 0.20;   //内环  0<fb<1
}
float w_neuro[2]={1/(1.618+1),1/(1.618+1)};//1.618∶1
xyz_f_t except_A = {0,0,0};
xyz_f_t except_AR = {0,0,0};
xyz_f_t ctrl_angle_offset = {0,0,0};

xyz_f_t compensation;
#define YAW_ERO_MAX 45
float YAW_NO_HEAD;	
float except_A_SB[2],except_A_SB_lft[2],nav_angle[2];
float scale_lf_sb=5.5;//感度
float scale_lf_nav=6;
float px_v,ix_v;
void CTRL_2(float T)//角度环
{ float px,py,ix,iy,d;
	static xyz_f_t acc_no_g;
	static xyz_f_t acc_no_g_lpf;
	float nav_angle_lft[2]={0,0};
  float dj_temp;
	static float dj_sb,dj_angle_set_out;
  static u8 flag_yaw_out=0;
  float cos1,sin1;
	float temp,temp_yaw;
	static u8 no_head;
/*   head  |    1 PIT y-    AUX2
	         | 
	         _____  0 ROL x+   AUX1
	
 ROL= 0,
 PIT=1
	*/
//=========================== 期望角度 ========================================
	 except_A_SB_lft[PITr] = -my_deathzoom_2(MAX_CTRL_ANGLE  *( my_deathzoom( (CH_filter[ROLr]) ,30 )/500.0f ),1);  
	 except_A_SB_lft[ROLr] =  my_deathzoom_2(MAX_CTRL_ANGLE  *( my_deathzoom( (CH_filter[PITr]) ,30 )/500.0f ),2);  
	
	
	 except_A_SB[ROLr]  += scale_lf_sb *T *3.14f * ( except_A_SB_lft[ROLr] - except_A_SB[ROLr] );
//---------------------------NAV_angle------------------------------------	
if(!mode.dj_lock)	{
	if(mode.flow_hold_position&&((fabs(except_A_SB[PITr])<2)&&(fabs(except_A_SB[ROLr])<3.5))
			&&NAV_BOARD_CONNECT==1)//add by gol 2015.10.22
			{
				if(mode.en_circle_control){
				if(circle.check){	
					if(ALT_POS_SONAR2>0.2){
					nav_angle_lft[PITr]=-my_deathzoom_2(nav_circle[PITr],0.1);//  -nav_ukf_g[PIT];//nav[PIT];
					nav_angle_lft[ROLr]=-my_deathzoom_2(nav_circle[ROLr],0.1);// nav_ukf_g[ROL];//nav[ROL];	
					}
					else
					{
					nav_angle_lft[PITr]=LIMIT(-my_deathzoom_2(nav_circle[PITr],0.1),-3,3);//  -nav_ukf_g[PIT];//nav[PIT];
					nav_angle_lft[ROLr]=LIMIT(-my_deathzoom_2(nav_circle[ROLr],0.1),-3,3);// nav_ukf_g[ROL];//nav[ROL];	
					}	
				}
				else
					{if(ALT_POS_SONAR2>0.2){
					nav_angle_lft[PITr]=-my_deathzoom_2(nav[PITr],0.1);//  -nav_ukf_g[PIT];//nav[PIT];
					nav_angle_lft[ROLr]=-my_deathzoom_2(nav[ROLr],0.1);// nav_ukf_g[ROL];//nav[ROL];	
					}
					else if(ALT_POS_SONAR2>0.11)
					{
					nav_angle_lft[PITr]=LIMIT(-my_deathzoom_2(nav[PITr],0.1),-3,3);//  -nav_ukf_g[PIT];//nav[PIT];
					nav_angle_lft[ROLr]=LIMIT(-my_deathzoom_2(nav[ROLr],0.1),-3,3);// nav_ukf_g[ROL];//nav[ROL];	
					}
					else
					{
					nav_angle_lft[PITr]=0;//LIMIT(-my_deathzoom(nav[PITr],0.5),-3,3);//  -nav_ukf_g[PIT];//nav[PIT];
					nav_angle_lft[ROLr]=0;//LIMIT(-my_deathzoom(nav[ROLr],0.5),-3,3);// nav_ukf_g[ROL];//nav[ROL];	
					}
					}
				}//---------------
				else{
				if(ALT_POS_SONAR2>0.2){
				nav_angle_lft[PITr]=-my_deathzoom_2(nav[PITr],0.1);//  -nav_ukf_g[PIT];//nav[PIT];
				nav_angle_lft[ROLr]=-my_deathzoom_2(nav[ROLr],0.1);// nav_ukf_g[ROL];//nav[ROL];	
				}
				else if(ALT_POS_SONAR2>0.1)
				{
				nav_angle_lft[PITr]=LIMIT(-my_deathzoom_2(nav[PITr],0.1),-3,3);//  -nav_ukf_g[PIT];//nav[PIT];
				nav_angle_lft[ROLr]=LIMIT(-my_deathzoom_2(nav[ROLr],0.1),-3,3);// nav_ukf_g[ROL];//nav[ROL];	
				}
        else
				{
				nav_angle_lft[PITr]=0;//LIMIT(-my_deathzoom(nav[PITr],0.5),-3,3);//  -nav_ukf_g[PIT];//nav[PIT];
				nav_angle_lft[ROLr]=0;//LIMIT(-my_deathzoom(nav[ROLr],0.5),-3,3);// nav_ukf_g[ROL];//nav[ROL];	
				}
				}
			}
}
else {//DJ mode
	if(mode.flow_hold_position&&
			((fabs(dj_sb)<2)&&(fabs(except_A_SB[ROLr])<3.5))
			&&NAV_BOARD_CONNECT==1)//add by gol 2015.10.22
			{
			nav_angle_lft[PITr]=nav[PITr];
			nav_angle_lft[ROLr]=nav[ROLr];	 
			}
}
//	//scale_lf_nav=pid.nav.out.i;
//	if(mode.sb_smooth){
//if(!mode.en_circle_control){
//	nav_angle[PITr] += scale_lf_nav *T *3.14f * ( nav_angle_lft[PITr] - nav_angle[PITr] );
//	nav_angle[ROLr] += scale_lf_nav *T *3.14f * ( nav_angle_lft[ROLr] - nav_angle[ROLr] );
//	}
//	else{
	nav_angle[PITr] = nav_angle_lft[PITr] ;
	nav_angle[ROLr] = nav_angle_lft[ROLr];
//	}
	//-----------------ATT_PROTECTOR----------------------------------------
	if(fabs(Pitch)>MAX_CTRL_ANGLE*1.5||fabs(Roll)>MAX_CTRL_ANGLE*1.5||ultra_distance<400)
	dj_angle_set=0;
	
	
	//---------DJ_CONTROL
	if(mode.dj_lock){
		dj_sb  += 6 *T *3.14f * ( except_A_SB_lft[PITr] - dj_sb );//fix by gol 2015.11.8
	
	  except_A_SB[PITr]  += 6 *T *3.14f * ( dj_angle_set - except_A_SB[PITr] );//fix by gol 2015.11.8
		if(fabs(Pitch-dj_angle_set)>0.6)
		dj_angle_set_out+= 10 *T *3.14f * ( LIMIT(Pitch,-SCALE_DJ*MAX_DJ_ANGLE,SCALE_DJ*MAX_DJ_ANGLE) - dj_angle_set_out);//fix by gol 2015.11.8  
		dj_temp=-dj_sb+dj_angle_set_out-nav_angle[PITr];
	}
	else
	{ 
		except_A_SB[PITr]  += scale_lf_sb *T*3.14f * ( except_A_SB_lft[PITr] - except_A_SB[PITr] );//fix by gol 2015.11.8
		dj_angle_set_out=0;dj_temp=0;
	}
	
	//--------------
//	if((fabs(except_A_SB_lft[PITr]>2)||fabs(except_A_SB_lft[ROLr])>2)||ALT_POS_SONAR2<0.04)//||(ultra_dis_lpf<300))//NAV_RESET
//	{
////		if(RX_CH[AUX3] >500 ){//add by gol 2015.10.21 (WT)
////		;
////		}
////		else
////		{
//	integrator[0]=integrator[1]=0;
//	target_position[LON]=now_position[LON];//imu_nav.flow.position.east;
//	target_position[LAT]=now_position[LAT];//imu_nav.flow.position.west;
//	//	}
//	}
  static u8 flow_pos_set_state;
	static u16 cnt_flow_set_pos;
	switch(flow_pos_set_state)
	{
		case 0:
			if((fabs(except_A_SB_lft[PITr]>2)||fabs(except_A_SB_lft[ROLr])>3.5)&&ALT_POS_SONAR2>0.04)
			{flow_pos_set_state=1;cnt_flow_set_pos=0;}
			
		
		break;
		case 1:
			 if((fabs(except_A_SB_lft[PITr]<2)&&fabs(except_A_SB_lft[ROLr])<3.5)&&ALT_POS_SONAR2>0.04)
				cnt_flow_set_pos++;
			 else
				 cnt_flow_set_pos=0;
				 
			 if(cnt_flow_set_pos>3.5/T)
			 {flow_pos_set_state=2;cnt_flow_set_pos=0;} 
		break;
	  case 2:
			flow_pos_set_state=0;
		break;
	}
	//out
	switch(flow_pos_set_state)
	{
		case 0:
		
		break;
		case 1:
			integrator[0]=integrator[1]=0;
			target_position[LON]=now_position[LON];//imu_nav.flow.position.east;
			target_position[LAT]=now_position[LAT];//imu_nav.flow.position.west; 
		break;
	  case 2:
		 	integrator[0]=integrator[1]=0;
			target_position[LON]=now_position[LON];//imu_nav.flow.position.east;
			target_position[LAT]=now_position[LAT];//imu_nav.flow.position.west; 
		break;
	}
		if(mode.en_circle_control&&circle.check)	
		{
			integrator[0]=integrator[1]=0;
			target_position[LON]=now_position[LON];//imu_nav.flow.position.east;
			target_position[LAT]=now_position[LAT];//imu_nav.flow.position.west; 
		}
	
	
	
	
	
	
	
	
//--------------------------超声波 壁障----------------------------------------------
//	if(mode.sonar_avoid&&((fabs(except_A_SB_lft[PIT])<2)&&(fabs(except_A_SB_lft[ROL])<2))&&	(ultra_dis_lpf>800))//&&need_avoid)
//	{
//	//target_position[LON]=tar_position_avoid[LON];//imu_nav.flow.position.east;
//	//target_position[LAT]=tar_position_avoid[LAT];//imu_nav.flow.position.west;
//		tar_speed_avoidc[LON]=tar_speed_avoid[LON];
//		tar_speed_avoidc[LAT]=tar_speed_avoid[LAT];
//	}
//	else
//	{
//		tar_speed_avoidc[LON]=0;
//		tar_speed_avoidc[LAT]=0;
//	}

	
	/*YAW -180~180     -90  135
		-	 0  + 
			 |
			 |
		-	180 +
	*/
//----------------------------------no head mode -------------------------------
	
	switch(no_head)//add by gol 11.8 (WT)
	{
		case 0:if(Thr_Low == 0 && ultra_distance>350)
		{no_head=1;YAW_NO_HEAD=Yaw;}
		else
			{
			except_AR.x=except_A_SB[ROLr];
			except_AR.y=except_A_SB[PITr];	
			}
		break;
		case 1:
			if(mode.no_head)
			{ 
			temp=Yaw-YAW_NO_HEAD;
			cos1=cos(temp*0.017);
			sin1=sin(temp*0.017);
			except_AR.x=except_A_SB[ROLr]*cos1+except_A_SB[PITr]*sin1;
			except_AR.y=except_A_SB[PITr]*cos1-except_A_SB[ROLr]*sin1;
			}	
			else
			{
			except_AR.x=except_A_SB[ROLr];
			except_AR.y=except_A_SB[PITr];	
			}
			if(Thr_Low == 1 && ultra_distance<200)
				no_head=0;
			break;
	}
//------------------------angle output	--------------------------------------------
	
		//DJ Smooth
	
	dj_angle += ( 1 / ( 1 + 1 / ( 6 *3.14f*0.02 ) ) ) *((dj_temp)- dj_angle) ;
	
	DJ_offset_save();
	//DJ Out
	Set_DJ(LIMIT(dj_angle,-MAX_DJ_ANGLE,MAX_DJ_ANGLE)+dj_angle_offset[0],
	LIMIT(Pitch,-90,90),
	-LIMIT(Roll,-90,90)+dj_angle_offset[2],
	-LIMIT(dj_angle,-MAX_DJ_ANGLE,MAX_DJ_ANGLE)-dj_angle_offset[1]
	);
	//---------END DJ

	except_A.x=limit_mine(nav_angle[ROLr]+except_AR.x,MAX_CTRL_ANGLE);	
	if(mode.dj_lock)
	except_A.y=limit_mine(except_AR.y,MAX_CTRL_ANGLE);	
	else
	except_A.y=limit_mine(nav_angle[PITr]+except_AR.y,MAX_CTRL_ANGLE);
	
	
	//--------------------------IMU Control Yaw
	 if( !Thr_Low && ultra_distance>SONAR_HEIGHT+20 )//fix by gol 2015.11.7 && ultra_distance>150)
	{
		if((flag_yaw_out==2&&CH_filter[YAWr]>100)||(flag_yaw_out==1&&CH_filter[YAWr]<-100)||flag_yaw_out==0)
		except_AR.z += (s16)( MAX_CTRL_YAW_SPEED *( my_deathzoom_2( (CH_filter[YAWr]) ,40 )/500.0f ) ) *T ;  //50
	}
	else
	{
		except_AR.z += 1 *3.14 *T *( Yaw - except_AR.z );
	}

  except_A.z  = To_180_degrees(except_AR.z);
	
	//-----------------------------------ATT PID  TUNING--------------------------
	if(mode.att_pid_tune)
	{
	if(KEY_SEL[0])//TRIG
	except_A.y=-15;
	else
  except_A.y=LIMIT(except_A.y,-15,15);
		
	ctrl_2.err.x =  my_deathzoom_2(To_180_degrees( ctrl_angle_offset.x + except_A.x - Roll  ),0.5);
	ctrl_2.err.y =  my_deathzoom_2(To_180_degrees( ctrl_angle_offset.y + except_A.y - Pitch ),0.5);
	ctrl_2.err.z =  0;//my_deathzoom_2(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);
	}
	else{
  /* 得到角度误差 */
	ctrl_2.err.x =  my_deathzoom_2(To_180_degrees( ctrl_angle_offset.x + except_A.x - Roll  ),0.5);
	ctrl_2.err.y =  my_deathzoom_2(To_180_degrees( ctrl_angle_offset.y + except_A.y - Pitch ),0.5);
	ctrl_2.err.z =  my_deathzoom_2(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);
	}
	
	
	//------------------YAW PROTECTOR--------------------------------
	if(fabs(To_180_degrees(except_A.z - Yaw))>80){
		except_A.z  = To_180_degrees(Yaw);
	  ctrl_2.err.z=0;
	}
	
	my_deathzoom(except_A.x,0.01);
	my_deathzoom(except_A.y,0.01);
	my_deathzoom(except_A.z,0.01);
	
	
  if((ctrl_2.err.z)>=YAW_ERO_MAX)
		flag_yaw_out=1;
	else  if((ctrl_2.err.z)<=-YAW_ERO_MAX)
		flag_yaw_out=2;
	else
		flag_yaw_out=0;

float Cpmin=0.3	;//P cut rate 0.2~0.4
float M1=5;//angle _dead
if(mode.hunman_pid)
{
if(ctrl_2.err.x<M1)
px=(1-(1-Cpmin)*exp(-1*fabs(ctrl_2.err.x)))*ctrl_2.PID[PIDROLL].kp;
else
px=	ctrl_2.PID[PIDROLL].kp;
if(
	((ctrl_2.err.x - ctrl_2.err_old.x)*ctrl_2.err.x)>0
||((ctrl_2.err.x - ctrl_2.err_old.x)==0&&(ctrl_2.err.x)!=0)
)
ix=ctrl_2.PID[PIDROLL].ki;
else
ix=0;	


if(ctrl_2.err.y<M1)
py=(1-(1-Cpmin)*exp(-1*fabs(ctrl_2.err.y)))*ctrl_2.PID[PIDROLL].kp;
else
py=	ctrl_2.PID[PIDROLL].kp;
if(((ctrl_2.err.y - ctrl_2.err_old.y)*ctrl_2.err.y)>0
||((ctrl_2.err.y - ctrl_2.err_old.y)==0&&(ctrl_2.err.y)!=0))
iy=ctrl_2.PID[PIDROLL].ki;
else
iy=0;	

d=ctrl_2.PID[PIDROLL].kd;
ix_v=ix;
ix=iy=ctrl_2.PID[PIDROLL].ki;
#if PLANE_IS_BIG
px_v=px=LIMIT(px,0.6,1);
py=LIMIT(py,0.6,1);
#else
px_v=px=LIMIT(px,0.15,0.5);
py=LIMIT(py,0.15,0.5);
#endif
}
else
{
px=py=ctrl_2.PID[PIDROLL].kp;
ix=iy=ctrl_2.PID[PIDROLL].ki;
d=ctrl_2.PID[PIDROLL].kd;	
}	
//----------------------------------------ESO FORWARD FEEDBACK-------------------------------
	
	
	ESO_3N(&eso_att_outter[PITr],except_A.y,Pitch,0,0.01,6); 
	ESO_3N(&eso_att_outter[ROLr],except_A.x,Roll,0,0.01,6); 
	ESO_3N(&eso_att_outter[YAWr],except_A.z,Yaw,0,0.01,6);
if(0){//mode.en_eso){
	if(SPID.YI!=0)eso_att_outter[PITr].n=eso_att_outter[ROLr].n=eso_att_outter[YAWr].n=SPID.YI;
  ctrl_2.err.x -=eso_att_outter[ROLr].disturb;  
	ctrl_2.err.y -=eso_att_outter[PITr].disturb;    
	//if(!mode.att_pid_tune)
	ctrl_2.err.z -=eso_att_outter[YAWr].disturb;    
}

	/* 计算角度误差权重 */
	ctrl_2.err_weight.x = ABS(ctrl_2.err.x)/ANGLE_TO_MAX_AS;
	ctrl_2.err_weight.y = ABS(ctrl_2.err.y)/ANGLE_TO_MAX_AS;
	ctrl_2.err_weight.z = ABS(ctrl_2.err.z)/ANGLE_TO_MAX_AS;
	/* 角度误差微分（跟随误差曲线变化）*/
	if(mode.en_fuzzy_angle_pid){
	ctrl_2.err_d.x = 10 *eso_att_outter_c[PITr].KD *(ctrl_2.err.x - ctrl_2.err_old.x) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.x );
	ctrl_2.err_d.y = 10 *eso_att_outter_c[PITr].KD *(ctrl_2.err.y - ctrl_2.err_old.y) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.y );
	ctrl_2.err_d.z = 10 *ctrl_2.PID[PIDYAW].kd 	 *(ctrl_2.err.z - ctrl_2.err_old.z) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.z );	
	}else{	
	ctrl_2.err_d.x = 10 *d *(ctrl_2.err.x - ctrl_2.err_old.x) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.x );
	ctrl_2.err_d.y = 10 *d *(ctrl_2.err.y - ctrl_2.err_old.y) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.y );
	ctrl_2.err_d.z = 10 *ctrl_2.PID[PIDYAW].kd 	 *(ctrl_2.err.z - ctrl_2.err_old.z) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.z );}
	/* 角度误差积分 */
	ctrl_2.err_i.x +=ix  *ctrl_2.err.x *T;
	ctrl_2.err_i.y +=iy  *ctrl_2.err.y *T;
	ctrl_2.err_i.z += ctrl_2.PID[PIDYAW].ki 	*ctrl_2.err.z *T;
	/* 角度误差积分分离 *///Thr_Weight=1;
	ctrl_2.eliminate_I.x = Thr_Weight *CTRL_2_INT_LIMIT;
	ctrl_2.eliminate_I.y = Thr_Weight *CTRL_2_INT_LIMIT;
	ctrl_2.eliminate_I.z = Thr_Weight *CTRL_2_INT_LIMIT;
	/* 角度误差积分限幅 */
	ctrl_2.err_i.x = LIMIT( ctrl_2.err_i.x, -ctrl_2.eliminate_I.x,ctrl_2.eliminate_I.x );
	ctrl_2.err_i.y = LIMIT( ctrl_2.err_i.y, -ctrl_2.eliminate_I.y,ctrl_2.eliminate_I.y );
	ctrl_2.err_i.z = LIMIT( ctrl_2.err_i.z, -ctrl_2.eliminate_I.z,ctrl_2.eliminate_I.z );
	/* 对用于计算比例项输出的角度误差限幅 */
	ctrl_2.err.x = LIMIT( ctrl_2.err.x, -90, 90 );
	ctrl_2.err.y = LIMIT( ctrl_2.err.y, -90, 90 );
	ctrl_2.err.z = LIMIT( ctrl_2.err.z, -90, 90 );
	//---------------------NEURO PID------------------------------
	//NEURON_PID(&neuron_pid_outter[PITr],ctrl_2.err.y ,except_A.y,10, T);
	//NEURON_PID(&neuron_pid_outter[ROLr],ctrl_2.err.x ,except_A.x,10, T);
	//NEURON_PID_LQ(&neuron_pid_outter[PITr],ctrl_2.err.y ,except_A.y,10, T);NEURON_PID(&neuron_pid_outter[ROLr],ctrl_2.err.y ,except_A.y,7.5, T);
	//NEURON_PID_LQ(&neuron_pid_outter[ROLr],ctrl_2.err.x ,except_A.x,10, T);
	//---------------------SELF DISTURB------------------------------
	ATT_CONTRL_OUTER_ESO_3(&eso_att_outter_c[PITr],except_A.y,Pitch,ctrl_2.out.y,0.01,10);
	ATT_CONTRL_OUTER_ESO_3(&eso_att_outter_c[ROLr],except_A.x,Roll,ctrl_2.out.x,0.01,10);
	/* 角度PID输出 */
	#define MAX_W_NEURO 1/(1.618+1)
	float x_out,y_out;
	x_out=px*( ctrl_2.err.x + ctrl_2.err_d.x + ctrl_2.err_i.x );
	y_out=py*( ctrl_2.err.y + ctrl_2.err_d.y + ctrl_2.err_i.y );
	w_neuro[0]=1;//LIMIT(AWDF_R(neuron_pid_outter[ROLr].u,x_out),0,0.5);
	w_neuro[1]=1;//LIMIT(AWDF_P(neuron_pid_outter[PITr].u,y_out),0,0.5);
//-----------------------------------CONTROL OUT SEL---------------------	
	if(mode.en_fuzzy_angle_pid){
	//ctrl_2.out.x= neuron_pid_outter[ROLr].u*w_neuro[0]+(1-w_neuro[0])*x_out;
	//ctrl_2.out.y=	neuron_pid_outter[PITr].u*w_neuro[1]+(1-w_neuro[1])*y_out;
	ctrl_2.err_i.x=ctrl_2.err_i.y=ctrl_2.err_i.z=0;
	ctrl_2.out.x= (eso_att_outter_c[ROLr].u+px  *( ctrl_2.err_d.x ))*w_neuro[0]+(1-w_neuro[0])*x_out;
	ctrl_2.out.y=	(eso_att_outter_c[PITr].u+py  *( ctrl_2.err_d.y ))*w_neuro[1]+(1-w_neuro[1])*y_out;
	}
	else{
	ctrl_2.out.x = x_out;	//rol
	ctrl_2.out.y = y_out;  //pit
  }
	
	ctrl_2.out.z = ctrl_2.PID[PIDYAW].kp   *( ctrl_2.err.z + ctrl_2.err_d.z + ctrl_2.err_i.z );
	/* 记录历史数据 */	
	ctrl_2.err_old.x = ctrl_2.err.x;
	ctrl_2.err_old.y = ctrl_2.err.y;
	ctrl_2.err_old.z = ctrl_2.err.z;

  
}

//----------------------------------------INNER------------------------------------------------
xyz_f_t except_AS;

float g_old[7];
 
void CTRL_1(float T)  //x roll,y pitch,z yaw 角速度  内环  2ms
{float ctrl_angle_out[3],ctrl_angle_weight[3];
	xyz_f_t EXP_LPF_TMP;
	

	ctrl_angle_out[0]=ctrl_2.out.x;
	ctrl_angle_out[1]=ctrl_2.out.y;
	ctrl_angle_out[2]=ctrl_2.out.z;
	ctrl_angle_weight[0]=ctrl_2.err_weight.x;
	ctrl_angle_weight[1]=ctrl_2.err_weight.y;
	ctrl_angle_weight[2]=ctrl_2.err_weight.z;
	
	
	/* 给期望（目标）角速度 */
	EXP_LPF_TMP.x = MAX_CTRL_ASPEED *(ctrl_angle_out[0]/ANGLE_TO_MAX_AS);//*( (CH_filter[0])/500.0f );//
	EXP_LPF_TMP.y = MAX_CTRL_ASPEED *(ctrl_angle_out[1]/ANGLE_TO_MAX_AS);//*( (CH_filter[1])/500.0f );//
	EXP_LPF_TMP.z = MAX_CTRL_ASPEED *(ctrl_angle_out[2]/ANGLE_TO_MAX_AS);
	
	except_AS.x = EXP_LPF_TMP.x;//20 *3.14 *T *( EXP_LPF_TMP.x - except_AS.x );//
	except_AS.y = EXP_LPF_TMP.y;//20 *3.14 *T *( EXP_LPF_TMP.y - except_AS.y );//
	except_AS.z = EXP_LPF_TMP.z;//20 *3.14 *T *( EXP_LPF_TMP.z - except_AS.z );//
	/* 期望角速度限幅 */
	except_AS.x = LIMIT(except_AS.x, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
	except_AS.y = LIMIT(except_AS.y, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
	except_AS.z = LIMIT(except_AS.z, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );

	/* 角速度直接微分（角加速度），负反馈可形成角速度的阻尼（阻碍角速度的变化）*/
	ctrl_1.damp.x = ( mpu6050.Gyro_deg.x - g_old[A_X]) *( 0.002f/T );//ctrl_1.PID[PIDROLL].kdamp
	ctrl_1.damp.y = (-mpu6050.Gyro_deg.y - g_old[A_Y]) *( 0.002f/T );//ctrl_1.PID[PIDPITCH].kdamp *
	ctrl_1.damp.z = (-mpu6050.Gyro_deg.z - g_old[A_Z]) *( 0.002f/T );//ctrl_1.PID[PIDYAW].kdamp	 *
	/* 角速度误差 */
	ctrl_1.err.x =  ( except_AS.x - mpu6050.Gyro_deg.x ) *(300.0f/MAX_CTRL_ASPEED);
	ctrl_1.err.y =  ( except_AS.y + mpu6050.Gyro_deg.y ) *(300.0f/MAX_CTRL_ASPEED);  //-y
	ctrl_1.err.z =  ( except_AS.z + mpu6050.Gyro_deg.z ) *(300.0f/MAX_CTRL_ASPEED);	 //-z
//----------------------------------ESO-----------------------------		
	ESO_2N(&eso_att_inner[PITr],except_AS.y,mpu6050.Gyro_deg.y,0,0.005,50); 
	ESO_2N(&eso_att_inner[ROLr],except_AS.x,-mpu6050.Gyro_deg.x,0,0.005,50); 
	ESO_2N(&eso_att_inner[YAWr],except_AS.z,mpu6050.Gyro_deg.z,0,0.005,50);
  
if(0){//mode.en_fuzzy_angle_pid){//mode.en_eso){
	if(SPID.YI!=0)eso_att_inner[PITr].n=eso_att_inner[ROLr].n=eso_att_inner[YAWr].n=SPID.YI;
  ctrl_1.err.x +=eso_att_inner[ROLr].disturb;  
	ctrl_1.err.y +=eso_att_inner[PITr].disturb;    
	//if(!mode.att_pid_tune)
	ctrl_1.err.z +=eso_att_inner[YAWr].disturb;    
}
	
	/* 角速度误差权重 */
	ctrl_1.err_weight.x = ABS(ctrl_1.err.x)/MAX_CTRL_ASPEED;
	ctrl_1.err_weight.y = ABS(ctrl_1.err.y)/MAX_CTRL_ASPEED;
	ctrl_1.err_weight.z = ABS(ctrl_1.err.z)/MAX_CTRL_YAW_SPEED;
	/* 角速度微分 */	
	ctrl_1.err_d.x = ( ctrl_1.PID[PIDROLL].kd  *( -10 *ctrl_1.damp.x) *( 0.002f/T ) );
	ctrl_1.err_d.y = ( ctrl_1.PID[PIDPITCH].kd *( -10 *ctrl_1.damp.y) *( 0.002f/T ) );
	ctrl_1.err_d.z = ( ctrl_1.PID[PIDYAW].kd   *( -10 *ctrl_1.damp.z) *( 0.002f/T ) );

//	ctrl_1.err_d.x += 40 *3.14 *0.002 *( 10 *ctrl_1.PID[PIDROLL].kd  *(ctrl_1.err.x - ctrl_1.err_old.x) *( 0.002f/T ) - ctrl_1.err_d.x);
//	ctrl_1.err_d.y += 40 *3.14 *0.002 *( 10 *ctrl_1.PID[PIDPITCH].kd *(ctrl_1.err.y - ctrl_1.err_old.y) *( 0.002f/T ) - ctrl_1.err_d.y);
//	ctrl_1.err_d.z += 40 *3.14 *0.002 *( 10 *ctrl_1.PID[PIDYAW].kd   *(ctrl_1.err.z - ctrl_1.err_old.z) *( 0.002f/T ) - ctrl_1.err_d.z);

	/* 角速度误差积分 */
	ctrl_1.err_i.x += ctrl_1.PID[PIDROLL].ki  *(ctrl_1.err.x - ctrl_1.damp.x) *T;
	ctrl_1.err_i.y += ctrl_1.PID[PIDPITCH].ki *(ctrl_1.err.y - ctrl_1.damp.y) *T;
	ctrl_1.err_i.z += ctrl_1.PID[PIDYAW].ki 	*(ctrl_1.err.z - ctrl_1.damp.z) *T;
	/* 角速度误差积分分离 */
	ctrl_1.eliminate_I.x = Thr_Weight *CTRL_1_INT_LIMIT ;
	ctrl_1.eliminate_I.y = Thr_Weight *CTRL_1_INT_LIMIT ;
	ctrl_1.eliminate_I.z = Thr_Weight *CTRL_1_INT_LIMIT ;
	/* 角速度误差积分限幅 */
	ctrl_1.err_i.x = LIMIT( ctrl_1.err_i.x, -ctrl_1.eliminate_I.x,ctrl_1.eliminate_I.x );
	ctrl_1.err_i.y = LIMIT( ctrl_1.err_i.y, -ctrl_1.eliminate_I.y,ctrl_1.eliminate_I.y );
	ctrl_1.err_i.z = LIMIT( ctrl_1.err_i.z, -ctrl_1.eliminate_I.z,ctrl_1.eliminate_I.z );
	//-----------------------------------------ESO
	ATT_CONTRL_INNER_ESO_3(&eso_att_inner_c[PITr],except_AS.y,-mpu6050.Gyro_deg.y,eso_att_inner_c[PITr].u,0.005,200);
	ATT_CONTRL_INNER_ESO_3(&eso_att_inner_c[ROLr],except_AS.x,mpu6050.Gyro_deg.x,eso_att_inner_c[ROLr].u,0.005,200);
	if(0){//(mode.en_fuzzy_angle_pid){
	ctrl_1.out.x = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.x),0,1)*except_AS.x+( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDPITCH].kp *( ctrl_1.err_d.x+ ctrl_1.err_i.x )) 
								+eso_att_inner_c[ROLr].u;
	ctrl_1.out.y = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.y),0,1)*except_AS.y+( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDPITCH].kp *( ctrl_1.err_d.y+ ctrl_1.err_i.y ))
								+eso_att_inner_c[PITr].u;
	ctrl_1.out.z = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.z),0,1)*except_AS.z + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDYAW].kp   *( ctrl_1.err.z + ctrl_1.err_d.z + ctrl_1.err_i.z ) );
	}else{	
	/* 角速度PID输出 */
	ctrl_1.out.x = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.x),0,1)*except_AS.x + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDROLL].kp  *( ctrl_1.err.x + ctrl_1.err_d.x + ctrl_1.err_i.x ) );
	ctrl_1.out.y = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.y),0,1)*except_AS.y + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDPITCH].kp *( ctrl_1.err.y + ctrl_1.err_d.y + ctrl_1.err_i.y ) );
	ctrl_1.out.z = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.z),0,1)*except_AS.z + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDYAW].kp   *( ctrl_1.err.z + ctrl_1.err_d.z + ctrl_1.err_i.z ) );
	}
	Thr_Ctrl(T);// 油门控制
	
	if(mode.att_pid_tune)
	{	
	ctrl_1.out.z=0;
	}
	All_Out(ctrl_1.out.x,ctrl_1.out.y,ctrl_1.out.z);


	ctrl_1.err_old.x = ctrl_1.err.x;
	ctrl_1.err_old.y = ctrl_1.err.y;
	ctrl_1.err_old.z = ctrl_1.err.z;

	g_old[A_X] =  mpu6050.Gyro_deg.x ;
	g_old[A_Y] = -mpu6050.Gyro_deg.y ;
	g_old[A_Z] = -mpu6050.Gyro_deg.z ;
}


//----------------------------------------------THR --------------------------------

u16 att_tuning_thr_limit;
int baro_to_ground,baro_ground_off;
float thr_value;
u8 Thr_Low,force_Thr_low=0;
float Thr_Weight;
float thr_test;
void Thr_Ctrl(float T)
{	float delta_thr;
	static float thr;
	static float Thr_tmp;
		static u8 cnt_thr_add,fly_ready_r;
	if(!fly_ready)
	   thr=0;		 
	else
    thr = 500 + CH_filter[THRr]; //油门值 0 ~ 1000
//----------Drop protector-----------------
	if(!fly_ready&&500 + CH_filter[THRr]<100)
	force_Thr_low=0;
	if((fabs(Pitch)>45||fabs(Roll)>45)&&fly_ready)
		force_Thr_low=1;
//protect flag init	
	if(fly_ready_r==0&&fly_ready==1&&500 + CH_filter[THRr]>100)
		force_Thr_low=1;
		fly_ready_r=fly_ready;
	
	if(force_Thr_low)
		thr=0;
	
	
	Thr_tmp += 10 *3.14f *T *(thr/400.0f - Thr_tmp); //低通滤波
	Thr_Weight = LIMIT(Thr_tmp,0,1);    							//后边多处分离数据会用到这个值
	
	if( thr < 100 )
	{
		Thr_Low = 1;
	
	}
	else
	{ 
		Thr_Low = 0;
	}
	
	if( 500 + CH_filter[THRr]<100)	baro_ground_off=ALT_POS_BMP*1000;
	baro_to_ground=LIMIT(ALT_POS_BMP*1000-baro_ground_off,10,8000);
	
	Height_Ctrl(T,thr);
	thr_value = Thr_Weight *height_ctrl_out;   //实际使用值	
	if(mode.att_pid_tune)
	{	
	if(att_tuning_thr_limit==0)att_tuning_thr_limit=550;
	thr_test=thr_value = LIMIT(thr,0,LIMIT(att_tuning_thr_limit,0,800));
	}		
	else
	thr_test=thr_value = LIMIT(thr_value,0,10 *MAX_THR *MAX_PWM/100);
}


float motor[MAXMOTORS];
float posture_value[MAXMOTORS];
float curve[MAXMOTORS];
#if defined(ZHOU_550)
float scale_thr_fix=2;//油门补偿系数
#endif
#define MAX_THR_FIX_ANGLE MAX_CTRL_ANGLE
int thr_value_fix;
void All_Out(float out_roll,float out_pitch,float out_yaw)
{
	s16 motor_out[MAXMOTORS];
	u8 i;
	float posture_value[MAXMOTORS];
  float curve[MAXMOTORS];
	static float motor_last[MAXMOTORS];

	out_yaw = LIMIT( out_yaw , -5*MAX_THR ,5*MAX_THR ); //50%
	
	posture_value[0] = - out_roll + out_pitch + out_yaw ;
	posture_value[1] = + out_roll + out_pitch - out_yaw ;
	posture_value[2] = + out_roll - out_pitch + out_yaw ;
	posture_value[3] = - out_roll - out_pitch - out_yaw ;
	
	for(i=0;i<4;i++)
	{
		posture_value[i] = LIMIT(posture_value[i], -1000,1000 );
	}
	
	curve[0] = (0.55f + 0.45f *ABS(posture_value[0])/1000.0f) *posture_value[0] ;
	curve[1] = (0.55f + 0.45f *ABS(posture_value[1])/1000.0f) *posture_value[1] ;
	curve[2] = (0.55f + 0.45f *ABS(posture_value[2])/1000.0f) *posture_value[2] ;
	curve[3] = (0.55f + 0.45f *ABS(posture_value[3])/1000.0f) *posture_value[3] ;
	
	//if(SPID.YP!=0)
	//scale_thr_fix=SPID.YP*0.001;
	//qscale_thr_fix=LIMIT(scale_thr_fix,0,10);
	int date_throttle;
	if(!mode.att_pid_tune){//add by gol 16.3.28 (WT)
	  //date_throttle	= (thr_value)/cos(LIMIT(my_deathzoom_2(Pitch,5),-MAX_THR_FIX_ANGLE,MAX_THR_FIX_ANGLE)/57.324841*scale_thr_fix	)/cos(LIMIT(my_deathzoom_2(Roll,5),-MAX_THR_FIX_ANGLE,MAX_THR_FIX_ANGLE)/57.324841*scale_thr_fix	);//add  12.9
    
		thr_value_fix=LIMIT((thr_value/cos(LIMIT(my_deathzoom_2(Pitch,5),-MAX_THR_FIX_ANGLE,MAX_THR_FIX_ANGLE)/57)/
							      cos(LIMIT(my_deathzoom_2(Roll,5),-MAX_THR_FIX_ANGLE,MAX_THR_FIX_ANGLE)/57)-thr_value),0,100)*scale_thr_fix;
		date_throttle=thr_value+thr_value_fix;
	}
		else{
		date_throttle	= thr_value;
	}
	
	
	motor[0] = date_throttle + Thr_Weight *curve[0] ;
	motor[1] = date_throttle + Thr_Weight *curve[1] ;
	motor[2] = date_throttle + Thr_Weight *curve[2] ;
	motor[3] = date_throttle + Thr_Weight *curve[3] ;
	
	mode.en_moto_smooth=0;
	  if(mode.en_moto_smooth){
     for(i=0;i<MAXMOTORS;i++){
        if(motor[i] > motor_last[i]) 
					motor[i] = (1 * (int16_t) motor_last[i] + motor[i]) / 2;  //mean of old and new
        else                                         
					motor[i] = motor[i] - (motor_last[i] - motor[i]) * 1; // 2 * new - old
			}
			 for(i=0;i<MAXMOTORS;i++)
					motor_last[i] = motor[i];  //mean of old and new
	    }
			
	/* 是否解锁 */
	if(fly_ready)
	{
		if( !Thr_Low )  //油门拉起
		{
			for(i=0;i<4;i++)
			{
				motor[i] = LIMIT(motor[i], (10 *READY_SPEED),(10*MAX_PWM) );
			}
		}
		else						//油门低
		{
			for(i=0;i<4;i++)
			{
				motor[i] = LIMIT(motor[i], 0,(10*MAX_PWM) );
			}
		}
	}
	else
	{
		for(i=0;i<4;i++)
		{
			motor[i] = 0;
		}
	}
	/* xxx */
	#if NEW_FLY_BOARD
	motor_out[0] = (s16)(motor[0]);  
	motor_out[1] = (s16)(motor[1]);	 
	motor_out[2] = (s16)(motor[2]);
	motor_out[3] = (s16)(motor[3]);
	#else
  motor_out[0] = (s16)(motor[0]);  
	motor_out[1] = (s16)(motor[1]);	 
	motor_out[2] = (s16)(motor[2]);
	motor_out[3] = (s16)(motor[3]);
	#endif
	SetPwm(motor_out,0,1000); //
}
//



