#include "height_ctrl.h"
#include "ultrasonic.h"
#include "my_math.h"
#include "filter.h"
#include "att.h"
#include "ms5611.h"
#include "rc.h"
#include "alt_kf.h"
#include "eso.h"

#if PLANE_IS_BIG
#define HOLD_THR 500 
#define ALT_HOLD_THR_RANGE_SCALE 2
#else  ///use now
#define HOLD_THR 450+100//425 // 5030 M=600g/310g*4=0.48 *1.1=520 | 6030 M=600g/410*4=0.34 *1.1= 0.37  PA 570
float ALT_HOLD_THR_RANGE_SCALE=2;//LIMIT((float)(1000-HOLD_THR)/230,1,2.5);//PA 250
#endif

#define EXP_Z_SPEED  ( 2.7f *my_deathzoom( (thr-500), 50 ) )
float exp_spd_zv,thr_use,thr_in_view;
_st_height_pid_v wz_speed_pid_v;
_st_height_pid_v wz_speed_pid_v_safe;
_st_height_pid_v wz_speed_pid_v_eso;

_st_height_pid_v ultra_ctrl,ultra_ctrl_safe;

_st_height_pid wz_speed_pid,wz_speed_pid_safe;
_st_height_pid wz_speed_pid_use;
_st_height_pid ultra_pid_safe,ultra_pid;
 _st_height_pid ultra_pid_use;

#define MAX_HEIGH_ERO 1000
float exp_height_speed,exp_height;
float exp_height_speed_safe,exp_height_safe;
float ultra_speed,ultra_speed_safe;
float ultra_dis_lpf,ultra_dis_lpf_safe;
float ultra_ctrl_out_safe, ultra_ctrl_out,ultra_ctrl_out_use;
float baro_speed;
float height_ctrl_out;
float wz_acc;
float adrc_out;
void Ultra_PID_Init()
{//use 
	#if defined(ZHOU_550)//-----------------------------------------550----------------------------------	
	ultra_pid.kp = 1.25;//1 //0.4;//1.8;//1.65;//1.5;   WT
	ultra_pid.ki = 0.00;//1;//101;//add
	ultra_pid.kd = 0.25;//0;
	#endif
	//
  ultra_pid_safe.kp = 0;//50.0;//1.8;//1.65;//1.5;
	ultra_pid_safe.ki = 0.0;//1;//add
	ultra_pid_safe.kd = 0;
}

  
void WZ_Speed_PID_Init()
{//use
	#if defined(ZHOU_550)//-----------------------------------------550----------------------------------	
	wz_speed_pid.kp = 0.3;//1.25;//0.3;//0.25;//0.5;//0.3; 
	wz_speed_pid.ki = 0.2;//1.4;//0.08;//0.5; 
	wz_speed_pid.kd = 1.0;//0.5;//8;//1.5; 
	#endif
	//
	wz_speed_pid_safe.kp = 0.3;//120.4;//0.5;//0.3; 
	wz_speed_pid_safe.ki = 0.2;//45.5; 
	wz_speed_pid_safe.kd = 1.0; 
}

#define BARO_SPEED_NUM 10
float baro_speed_arr[BARO_SPEED_NUM + 1];
u16 baro_cnt;
u8 switch_r;
int wz_acc_ukf;
void height_mode_switch(void)
{
static u8 state;
static u16 cnt,cnt1;	

 if(height_ctrl_mode==2){
	 
	 switch(state)
			{ 
			case 0:
			if(ultra_ok==1){
			if(ALT_POS_BMP*1000>BMP_MAX_HEIGHT)
			{state=1;height_ctrl_mode_use=1;cnt=0;}
			else
				height_ctrl_mode_use=2;
			}
			else
			height_ctrl_mode_use=1;
			break;
			case 1:
			if(ALT_POS_SONAR2*1000<ULTRA_MAX_HEIGHT-100)
			cnt++;
			else
			cnt=0;

			if(cnt>800)
			{state=0;height_ctrl_mode_use=2;cnt=0;}

			break;
			}

}
 else
 {	height_ctrl_mode_use=height_ctrl_mode;state=0;}
}


#define BARO_HIHG_NUM 100
float baro_h_arr[BARO_HIHG_NUM + 1];
u16 baro_h_cnt[2];

u8 hs_ctrl_cnt_max[2]={20,1},hold_alt_flag;
float out_timer_high,in_timer_high;
float baro_only_move;
void Height_Ctrl(float T,float thr)
{			static u8 hs_ctrl_cnt;
	static float wz_speed_t;
	static u8 height_ctrl_start_f,height_mode_reg;
	static u16 hc_start_delay;
	float t_sonar_out;
	float temp;
	
	switch( height_ctrl_start_f )
	{		
		case 0:
		temp=(reference_vr[2] *mpu6050.Acc.z + reference_vr[0] *mpu6050.Acc.x + reference_vr[1 ] *mpu6050.Acc.y - 4096  );
		wz_acc_ukf += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *my_deathzoom( (temp - wz_acc_ukf),50 );
		wz_acc += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *my_deathzoom( (temp - wz_acc),50);//100 );
		Moving_Average( (float)( baroAlt),baro_h_arr,BARO_HIHG_NUM, baro_h_cnt ,&baro_only_move ); //单位mm/s
	if(mode.en_h_mode_switch)
				height_mode_switch();
		 if( height_ctrl_mode_use!=0)
		{ 
	
			//if(KEY[5])
				//hs_ctrl_cnt_max=1;//10ms
			//else hs_ctrl_cnt_max=3;
		  //hs_ctrl_cnt = hs_ctrl_cnt%10;
			if(hs_ctrl_cnt++>=hs_ctrl_cnt_max[1])//3
			{  //----------------------mode switch----------------------
				//in_timer_high=Get_Cycle_T(GET_T_HIGH_CONTROL_I);hs_ctrl_cnt=0;
				if(height_mode_reg==1&&height_ctrl_mode_use==2)//SONAR<-BMP
				{exp_height=ultra_dis_lpf=ALT_POS_SONAR*1000;}
				else if(height_mode_reg==2&&height_ctrl_mode_use==1)//SONAR->BMP
				{exp_height=ultra_dis_lpf=ALT_POS_BMP*1000;}
				 height_mode_reg=height_ctrl_mode_use;
				//---------------------safe height--------------------------------
				static u8 safe_mode_reg;
				 if(safe_mode_reg==0&&mode.height_safe==1)//SONAR->BMP
				{exp_height_safe=ultra_dis_lpf_safe=ALT_POS_BMP*1000;}
				else if(safe_mode_reg==1&&mode.height_safe==0)
				{
				if(height_ctrl_mode_use==2)//SONAR<-BMP
				{exp_height=ultra_dis_lpf=ALT_POS_SONAR2*1000;}
				else if(height_ctrl_mode_use==1)//SONAR->BMP
				{exp_height=ultra_dis_lpf=ALT_POS_BMP*1000;}
				
				}safe_mode_reg=mode.height_safe;
				//heigh thr test
				static u8 state_thr,cnt_down;
				static float thr_reg;
				thr_in_view=thr;
				//-----------------UP MIN thr Protector---------------
				if(0)//mode.thr_fix_test)//FA
				{  
					switch(state_thr)
					{
						case 0:
							if(thr>525)
							{state_thr=1;thr_reg=500;cnt_down=0;}
							else
							{thr_use=thr;}	
						break;
						case 1:
							if(thr>525){
								if(thr<thr_reg*0.85)
									cnt_down++;
								else
								{thr_use=thr;cnt_down=0;}
								if(cnt_down>2)state_thr=2;
								if(thr>=thr_reg)
								 thr_reg=thr;
							}
							else
								state_thr=0;
							
							
							break;
						case 2:
							thr_use=500;
							if(thr<550)
								state_thr=0;
						break;
					}
				}
				else
				{thr_use=thr;state_thr=0;thr_reg=500;}
				//----------------------DOWN THR MINING---------------------
				// thr_use=thr_down_min_portect(thr_use);
				if(mode.height_safe)
				{
				mode.height_in_speed=1;
				ultra_pid_use.kp =ultra_pid_safe.kp;ultra_pid_use.ki =ultra_pid_safe.ki; ultra_pid_use.kd =ultra_pid_safe.kd;  
				wz_speed_pid_use.kp =wz_speed_pid_safe.kp;wz_speed_pid_use.ki =wz_speed_pid_safe.ki; wz_speed_pid_use.kd =wz_speed_pid_safe.kd;  
				}
				else
				{
				ultra_pid_use.kp =ultra_pid.kp;ultra_pid_use.ki =ultra_pid.ki; ultra_pid_use.kd =ultra_pid.kd;  
				wz_speed_pid_use.kp =wz_speed_pid.kp;wz_speed_pid_use.ki =wz_speed_pid.ki; wz_speed_pid_use.kd =wz_speed_pid.kd; 
				}
				 //------------------------SPD CONTOLLER------------------
					//	adrc_out=ADRC( exp_height,ultra_dis_lpf, adrc_out,0.02,500)  ;          // v是控制系统的输入，y是控制系统的输出，反馈给ESO，u是ADRC的输出控制量
						
//				 applyMultirotorAltHold(thr_use,0.02);
				   exp_spd_zv=EXP_Z_SPEED;
//				 if(KEY[6])
//					 height_speed_ctrl(in_timer_high,thr_use,0.4*ultra_ctrl_out,ultra_speed);	
//				 else
//						{if(EXP_Z_SPEED!=0){
//				    	height_speed_ctrl(in_timer_high,thr_use,( EXP_Z_SPEED ),ultra_speed);//baro_alt_speed *10);///
//						//	height_speed_ctrl_Safe(0.02f,thr,EXP_Z_SPEED,ultra_speed_safe); 
//						}
//							else
				   //if(EXP_Z_SPEED!=0){
//				  if(mode.height_in_speed){
//					   ultra_ctrl_out=EXP_Z_SPEED;
//						 height_speed_ctrl(in_timer_high,thr_use,LIMIT( ultra_ctrl_out,-400,400),-LIMIT(ALT_VEL_BMP,-1.6,1.6)*1000);}
//				   else
//					 {
				
//							if( fabs(-LIMIT(ALT_VEL_BMP,-1.6,1.6)*1000) < 100 )
//							{
//								ultra_speed += ( 1 / ( 1 + 1 / ( 4 *3.14f *T*1 ) ) ) * ( (float)(-LIMIT(ALT_VEL_BMP,-1.6,1.6)*1000) - ultra_speed );
//							}
//							else
//							{
//								ultra_speed += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T*1.5 ) ) ) * ( (float)(-LIMIT(ALT_VEL_BMP,-1.6,1.6)*1000) - ultra_speed );
//							}
							ultra_speed=-LIMIT(ALT_VEL_BMP,-1.6,1.6)*1000;
				     if(!hold_alt_flag)
						 ultra_ctrl_out_use=EXP_Z_SPEED;
						 else
						 ultra_ctrl_out_use=ultra_ctrl_out; 
             if(!mode.height_safe)
                //if(mode.thr_fix_test)
							height_speed_ctrl(in_timer_high,thr_use,LIMIT(ultra_ctrl_out_use,-888,888),ultra_speed);
									//else  
						 //height_speed_ctrl(in_timer_high,thr_use,LIMIT(ultra_ctrl_out_use,-660,660),ultra_speed);		
             else						 
						 height_speed_ctrl(in_timer_high,thr_use,LIMIT(ultra_ctrl_out_use,-660,660),ultra_speed);		 
					 //}
						//	height_speed_ctrl_Safe(0.02f,thr,0.4*ultra_ctrl_out_safe,ultra_speed_safe);
							//}
						//}
					
			}//---end of speed control
			static u8 cnt_100ms;
			//if(KEY[5])
			#if  defined(SONAR_SAMPLE1)
			hs_ctrl_cnt_max[0]=20;//50Hz  WT
			#endif
		 //else
			if(mode.height_safe)
		  hs_ctrl_cnt_max[0]=20;//10Hz
			if( cnt_100ms++>=hs_ctrl_cnt_max[0] )//PA
			{
	      out_timer_high=Get_Cycle_T(GET_T_HIGH_CONTROL_O);
				cnt_100ms=0;

				Ultra_Ctrl(out_timer_high,thr_use);//超声波周期100ms
				//Ultra_Ctrl_Safe(0.1f,thr);//超声波周期100ms
				//ultra_start_f = -1;
			}
		}

			if(height_ctrl_mode_use)//定高模式
		{	
			//if(mode.height_safe)
			//height_ctrl_out =  wz_speed_pid_v_safe.pid_out;//LIMIT(thr-50*LIMIT(Moving_Median(9,5,wz_acc_ukf)/4096,-0.2,0.2)*9.8,0,500);	
			//else
			height_ctrl_out = wz_speed_pid_v.pid_out;
		}
		else//手动模式
		{		
		  	height_ctrl_out = thr-50*LIMIT(wz_acc_ukf/4096,-0.2,0.2)*9.8;   
		}
		
		break; //case 1
		
		default: break;
		
	} //switch
}
#define MIN_THR_DOWN HOLD_THR-50
#define MIN_THR_RC_CHECK 200 //RC MIN_THR_CHECK
int Thr_down_min_portect(int thr_in,float T){
static u8 state;
int out;
//state
	switch(state)	
	{
		case 0:if(!Thr_Low&&thr_in>MIN_THR_DOWN)//Check RC
						state=1;
			break;
		case 1:if(500 + CH_filter[THRr]<MIN_THR_RC_CHECK||!fly_ready)
						state=0;
			break;
		default:state=0; break;
	}
//output
	switch(state)	
	{
		case 0:out=thr_in;break;
		case 1:out=LIMIT(thr_in,MIN_THR_DOWN,HOLD_THR);break;
		default:out=thr_in;break;
	}
return out;
}	


///test 
float p1=0.4,p2=0.1;//0.35;//0.3;//WT
u8 speed_ctrl_sel=1,speed_spd_sel=1;
float wz_speed,wz_speed_old,wz_acc_mms2;
float height_thrv,wz_speed_pid_v_view;
void height_speed_ctrl(float T,float thr,float exp_z_speed,float h_speed)
{
static float thr_lpf;
float height_thr;
static float lpf_tmp,hc_speed_i,hc_speed_i_2,wz_speed_0,wz_speed_1,wz_speed_2,hc_acc_i;
	
	if(Thr_Low)wz_speed_pid_v.err_i=0;
	height_thr = LIMIT( ALT_HOLD_THR_RANGE_SCALE * thr , 0, HOLD_THR );
	height_thr = Thr_down_min_portect(height_thr,T);//add by gol 16.3.28 (WT)
	thr_lpf += ( 1 / ( 1 + 1 / ( 2.0f *3.14f *T ) ) ) *( height_thr - thr_lpf );
	height_thrv=thr_lpf;
switch(speed_spd_sel){
	case 0:	
//=================================================
	wz_acc_mms2 = (wz_acc/4096.0f) *10000 + hc_acc_i;//9800 *T;
	wz_speed_0 += my_deathzoom( (wz_acc_mms2 ) ,50) *T;
	wz_speed_0 = 0.99	*wz_speed_0 + 0.01*h_speed;
	hc_acc_i += p1 *T *(h_speed - wz_speed);
	hc_acc_i = LIMIT( hc_acc_i, -500, 500 );	
	wz_speed_0 += ( 1 / ( 1 + 1 / ( p2 *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;
	wz_speed=wz_speed_0 + hc_acc_i;
	if( ABS( wz_speed ) < 50 )
	{
		wz_speed = 0;
	}
	break;
	case 1:
//=================================================	
  wz_acc_mms2 = (wz_acc/4096.0f) *9800;//10000 ;//9800 *T;
	wz_speed_0 += my_deathzoom( wz_acc_mms2 ,100) *T;
	
	hc_speed_i += 0.4f *T *( h_speed - wz_speed_1 );
	hc_speed_i = LIMIT( hc_speed_i, -1500, 1500 );	
	
	wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.4f *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;
	wz_speed_1 = wz_speed_0 + hc_speed_i;
	
	if( ABS( wz_speed_1 ) < 25 )
	{
		wz_speed_1 = 0;
	}
	
	wz_speed_2 += my_deathzoom( wz_acc_mms2 ,100) *T;
	

		lpf_tmp += 0.4f *T *( wz_speed_1 - wz_speed ); 
		lpf_tmp = LIMIT( lpf_tmp, -1500, 1500 ); 

	hc_speed_i_2 += 0.01f *T *( wz_speed_1 - wz_speed_2 ); 
	hc_speed_i_2 = LIMIT( hc_speed_i_2, -500, 500 );	
	
	wz_speed_2 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( wz_speed_1 - wz_speed_2 + hc_speed_i_2 ) ;//*(baro_speed - wz_speed);
	wz_speed = wz_speed_2 + lpf_tmp;
	break;
}
//===============================	
 //	if(KEY[5])(PA)
	wz_speed=my_deathzoom_2( (h_speed) ,25);
  switch(speed_ctrl_sel){case 0:
	//--------------------------------------------------------------------------------
	wz_speed_pid_v.err = wz_speed_pid_use.kp *( exp_z_speed - wz_speed );
	//wz_speed_pid_v.err_d = 0.002f/T *10*wz_speed_pid.kd * (-wz_acc_mms2) *T;//(wz_speed_pid_v_safe.err - wz_speed_pid_v_safe.err_old);
	wz_speed_pid_v.err_d = wz_speed_pid_use.kd * (- my_deathzoom( (wz_acc_mms2 ) ,100)) *T;
	wz_speed_pid_v.err_i += wz_speed_pid_use.ki *( exp_z_speed - h_speed ) *T;//*wz_speed_pid.kp 
	wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i,-Thr_Weight *300,Thr_Weight *300);//-100~100 //(WT)
//	if(KEY[5])
//	wz_speed_pid_v.pid_out = thr_lpf + Thr_Weight *LIMIT(( wz_speed_pid.kp*0.0 *LIMIT(exp_z_speed,-300,300)+wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-400,400);	
//	else
	wz_speed_pid_v.pid_out = thr_lpf + Thr_Weight *LIMIT(( LIMIT(wz_speed_pid_use.kp *exp_z_speed,-200,200)+wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-400,400);
	wz_speed_pid_v.err_old = wz_speed_pid_v.err; 
	wz_speed_old=wz_speed;
		break;
	case 1://use
  //-----------------------------------------------------------------------------------
	wz_speed_pid_v.err = wz_speed_pid_use.kp *( exp_z_speed - wz_speed );
	wz_speed_pid_v.err_d = 0.002f/T *10*wz_speed_pid_use.kd * (-my_deathzoom( (wz_acc_mms2 ) ,100)) *T;
	//wz_speed_pid_v.err_i += wz_speed_pid.ki *wz_speed_pid.kp *( exp_z_speed - h_speed ) *T;
	wz_speed_pid_v.err_i += wz_speed_pid_use.ki *( exp_z_speed - h_speed ) *T;
	wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i,-Thr_Weight *300,Thr_Weight *300);
	
	wz_speed_pid_v_view=thr_lpf + Thr_Weight *LIMIT((LIMIT(wz_speed_pid_use.kp *exp_z_speed,-200,200) + wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-400,400);

	if(mode.en_eso_h_in&&!mode.height_safe){
	HIGH_CONTROL_SPD_ESO(&eso_att_inner_c[THRr],exp_z_speed,wz_speed,eso_att_inner_c[THRr].u,T,400);
	wz_speed_pid_v.pid_out=thr_lpf + Thr_Weight *LIMIT(eso_att_inner_c[THRr].u + wz_speed_pid_v.err_d ,-400,400);
	}
	else{
	//if(KEY[7])//(WT)
	wz_speed_pid_v.pid_out = thr_lpf + Thr_Weight *LIMIT(( wz_speed_pid_use.kp*0.0 *LIMIT(exp_z_speed,-300,300)+wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-400,400);	
	//else
	//wz_speed_pid_v.pid_out = thr_lpf + Thr_Weight *LIMIT((LIMIT(wz_speed_pid.kp *exp_z_speed,-200,200) + wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-400,400);
	}
	wz_speed_pid_v.err_old = wz_speed_pid_v.err; 
	break;
   }
}

u8 baro_ctrl_start;
float baro_height,baro_height_old;

float ultra_sp_test[2];


void Ultra_Ctrl(float T,float thr)
{
	float ultra_sp_tmp,ultra_dis_tmp;	
	#define MID_THR 500 //摇杆中位PWM
	exp_height_speed = ULTRA_SPEED *my_deathzoom_2(thr - MID_THR,50)/300.0f; //+-ULTRA_SPEEDmm / sEXP_Z_SPEED;//
	exp_height_speed = LIMIT(exp_height_speed ,-ULTRA_SPEED,ULTRA_SPEED);

//	  if( exp_height <= 0 )
//		{
//			if( exp_height_speed < 0 )
//			{
//				exp_height_speed = 0;
//			}
//		}
//	 if( thr < 100 )
//	{
//		exp_height += ( 1 / ( 1 + 1 / ( 0.2f *3.14f *T ) ) ) *( -exp_height);
//	}	
//	 if((exp_height-ultra_dis_lpf)>MAX_HEIGH_ERO&&exp_height_speed>0)//add by golaced 2015.11.4(WT)
//	{
//		exp_height_speed = 0;
//	}
//   if((exp_height-ultra_dis_lpf)<-MAX_HEIGH_ERO&&exp_height_speed<0)//add by golaced 2015.11.4(WT)
//	{
//		exp_height_speed = 0;
//	}
  // 	exp_height += exp_height_speed *T;

	
 if(mode.flow_hold_position_high_fix&&height_ctrl_mode_use==2)//add by gol 2015.11.8(PA)
	  exp_height =1000;//+= ( 1 / ( 1 + 1 / ( 0.4f *3.14f *T ) ) ) *(1000- exp_height) ;	
 static int bmp_expr;
  if(mode.flow_hold_position_high_fix&&height_ctrl_mode_use==1)//add by gol 2015.11.8(PA)
	  exp_height = bmp_expr+800 ;
	else
		bmp_expr=ALT_POS_BMP*1000;
	
//if(mode.height_in_speed){		
	 static u16 cnt_in_mid;
	 static u8 state_high_set=0;
	
	 #define CHECK_CNT_MID 0.35
	switch(state_high_set){
		case 0:
			if(EXP_Z_SPEED==0){
	     	if(cnt_in_mid++>=CHECK_CNT_MID/out_timer_high)	
				{state_high_set=1;cnt_in_mid=0;}
			}else
     	cnt_in_mid=0;		
		  break;
	  case 1:
			if(EXP_Z_SPEED!=0){
	     	if(cnt_in_mid++>=CHECK_CNT_MID/out_timer_high)	
				{state_high_set=0;cnt_in_mid=0;}
			}else cnt_in_mid=0;
		break;
	}
		switch(state_high_set){
		case 0:	
		if(height_ctrl_mode_use==1&&ALT_POS_SONAR2>0.04)
		{exp_height= LIMIT(ALT_POS_BMP,0.04,10)*1000;}
		if(height_ctrl_mode_use==2&&ALT_POS_SONAR2>0.04)
		{exp_height=LIMIT(ALT_POS_SONAR2,0.04,10)*1000;}
		hold_alt_flag=0;
		break;
	  case 1:hold_alt_flag=1;break;
	
	}
	
//	if(EXP_Z_SPEED==0)
//	{  
//		if(cnt_in_mid++>=CHECK_CNT_MID/out_timer_high)	
//		cnt_in_mid=CHECK_CNT_MID/out_timer_high+1;
//		else{
//			if(height_ctrl_mode_use==1)
//			{exp_height=ALT_POS_BMP*1000;}
//			if(height_ctrl_mode_use==2&&ALT_POS_SONAR2>100/1000)
//			{exp_height=ALT_POS_SONAR2*1000;}
//			}	
//		}
//	else{	cnt_in_mid=0;
//		if(height_ctrl_mode_use==1)
//		{exp_height= ALT_POS_BMP*1000;}
//		if(height_ctrl_mode_use==2)
//		{exp_height=ALT_POS_SONAR2*1000;}
//	}
//		
		

 //}else
//	exp_height += exp_height_speed *T;
	
	//mode.flow_hold_position_high_fix=1;
	 
		
//    if(height_ctrl_mode_use==1){
//		//	if(KEY[7]==1)
//    ultra_sp_tmp=my_deathzoom_2(-LIMIT(ALT_VEL_BMP,-0.8*2,0.8*2)*1000,0);	
//		//else
//		//	ultra_sp_tmp=Moving_Median(2,5,my_deathzoom_2(-ALT_VEL_BMP*1000,0));//ALT_VEL_BMP*1000;//WT
//		}
//    else
//		//if(KEY[5])//WT
//		ultra_sp_tmp=my_deathzoom_2(-LIMIT(ALT_VEL_BMP,-0.8*2,0.8*2)*1000,0);	
		//else
	  //ultra_sp_tmp=my_deathzoom_2(-LIMIT(ALT_VEL_SONAR,-0.8,0.8)*1000,0);//Moving_Median(2,5,my_deathzoom_2(-ALT_VEL_SONAR*1000,0));

//		if( fabs(ultra_sp_tmp) < 100 )
//		{
//			ultra_speed += ( 1 / ( 1 + 1 / ( 4 *3.14f *T*1 ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
//		}
//		else
//		{
//			ultra_speed += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T*1.5 ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
//		}
	
	 
		if(height_ctrl_mode_use==1)
		ultra_dis_tmp = ALT_POS_BMP*1000;//ALT_POS_BMP*1000;//ALT_POS_BMP*1000;	WTbaro_only_move;//
		else
		ultra_dis_tmp=  ALT_POS_SONAR2*1000;//ultra_distance;
		
		
		if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 100 )
		{
			
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 4.0f *3.14f *T ) ) ) *(ultra_dis_tmp - ultra_dis_lpf) ;
		}
		else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 200 )
		{
			
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 2.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
		}
		else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 400 )
		{
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 1.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
		}
		else
		{
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 0.6f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
		}
	ultra_dis_lpf=  ultra_dis_tmp;
		
	if(ultra_pid.ki==0)ultra_ctrl.err_i=0;

  ultra_ctrl.err = ( ultra_pid_use.kp*my_deathzoom_2(exp_height - ultra_dis_lpf,10) );
	
	ultra_ctrl.err_i += ultra_pid_use.ki *ultra_ctrl.err *T;
	
	ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i,-Thr_Weight *ULTRA_INT,Thr_Weight *ULTRA_INT);
	
	ultra_ctrl.err_d = ultra_pid_use.kd *( 1.f *(-wz_speed*T) +0.0f *(ultra_ctrl.err - ultra_ctrl.err_old) );
	
	ultra_ctrl.pid_out = ultra_ctrl.err + ultra_ctrl.err_i + ultra_ctrl.err_d;
	
	ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-1000,1000);
		
	ultra_ctrl_out = ultra_ctrl.pid_out;
	
	ultra_ctrl.err_old = ultra_ctrl.err;
//===========		
// 	ultra_ctrl.err = ( ultra_pid.kp*my_deathzoom_2(exp_height - ultra_dis_lpf,10) );
//	
//	ultra_ctrl.err_i += ultra_pid.ki *ultra_ctrl.err *T;
//	
//	ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i,-Thr_Weight *ULTRA_INT,Thr_Weight *ULTRA_INT);
//	
//	ultra_ctrl.err_d = ultra_pid.kd *( 0.6f *(-wz_speed*T)+0.4*(ultra_ctrl.err - ultra_ctrl.err_old)) ;
//	
//	ultra_ctrl.pid_out = ultra_ctrl.err + ultra_ctrl.err_i + ultra_ctrl.err_d;
//	
//	ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-400,400);//(PA)
//		
//	ultra_ctrl_out = ultra_ctrl.pid_out;
//	
//	ultra_ctrl.err_old = ultra_ctrl.err;
}







//----------------------------------------------------------height PID test-----------------------------------------------------
float setVelocity;
u8 velocityControl;
int errorVelocityI;

int32_t calculateAltHoldThrottleAdjustment(int32_t vel_tmp)
{
    int32_t result = 0;
    int32_t error;
    int32_t setVel;
    if (!velocityControl) {
        error = LIMIT(exp_height_safe - ultra_dis_lpf_safe, -500, 500);
        error = my_deathzoom_2(error, 10); // remove small P parameter to reduce noise near zero position
        setVel = LIMIT((ultra_pid_safe.kp* error / 128), -300, +300); // limit velocity to +/- 3 m/s
    } else {
        setVel = setVelocity;
    }
    // Velocity PID-Controller

    // P
    error = setVel - vel_tmp;
    result = LIMIT((wz_speed_pid_safe.kp * error / 32), -300, +300);

    // I
    errorVelocityI += (wz_speed_pid_safe.ki* error);
    errorVelocityI = LIMIT(errorVelocityI, -(8192 * 200), (8192 * 200));
    result += errorVelocityI / 8192;     // I in range +/-200
		int accZ_tmp=((float)wz_acc_ukf/4096.0f) *9.8*100;
		static int accZ_old;
    // D
    result -= LIMIT(wz_speed_pid_safe.kd * (accZ_tmp + accZ_old) / 512, -150, 150);
    accZ_old=accZ_tmp;
    return result;
}

void applyMultirotorAltHold(u16 thr,float T)
{
static uint8_t isAltHoldChanged = 0;
static float thr_lpf;
float height_thr;
static float lpf_tmp,hc_speed_i,hc_speed_i_2,wz_speed_0,wz_speed_1,wz_speed_2,hc_acc_i;
	  if(thr<100)
			errorVelocityI=0;
	
	  height_thr = LIMIT( ALT_HOLD_THR_RANGE_SCALE * thr , 0, HOLD_THR );
	
		thr_lpf += ( 1 / ( 1 + 1 / ( 1.5f *3.14f *T ) ) ) *( height_thr - thr_lpf );

        // slow alt changes, mostly used for aerial photography
        if (fabs(thr - 500) > 50) {
            // set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
            setVelocity = (thr-500) / 2;
            velocityControl = 1;
            isAltHoldChanged = 1;
        } else if (isAltHoldChanged) {
            exp_height_safe = ultra_dis_lpf_safe;
            velocityControl = 0;
            isAltHoldChanged = 0;
        }
				
				ultra_dis_lpf_safe = ALT_POS_BMP*100;
				float ultra_sp_tmp;
				ultra_sp_tmp=my_deathzoom_2(-ALT_VEL_BMP*100,0);//ALT_VEL_BMP*1000;//
				ultra_sp_tmp=LIMIT(ultra_sp_tmp,-150,150);//cm
				if( fabs(ultra_sp_tmp) < 10 ){
					ultra_speed_safe += ( 1 / ( 1 + 1 / ( 4 *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed_safe );
				}
				else
				{
					ultra_speed_safe += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed_safe );
				}
	
				float altHoldThrottleAdjustment;
				altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment(ultra_speed_safe);
        wz_speed_pid_v_safe.pid_out = LIMIT(thr_lpf + altHoldThrottleAdjustment, 0,800);
}


































//---------------------------------------------------------------------Safe height con------------------------------------------------------------

float wz_speed_safe,wz_speed_old_safe,wz_acc_mms2_safe;
void height_speed_ctrl_Safe(float T,float thr,float exp_z_speed,float h_speed)
{static float lpf_tmp,hc_speed_i,hc_speed_i_2,wz_speed_0,wz_speed_1,wz_speed_2,hc_acc_i;
	static float thr_lpf;
	float height_thr;

	
	height_thr = LIMIT( ALT_HOLD_THR_RANGE_SCALE * thr , 0, HOLD_THR );
	
	thr_lpf += ( 1 / ( 1 + 1 / ( 1.5f *3.14f *T ) ) ) *( height_thr - thr_lpf );
	wz_acc_mms2_safe = (wz_acc/4096.0f) *10000 + hc_acc_i;//9800 *T;
	wz_speed_0 += my_deathzoom( (wz_speed_old_safe ) ,100) *T;
	hc_acc_i += 0.4 *T *( (wz_speed_safe - wz_speed_old_safe)/T - wz_acc_mms2_safe );
	hc_acc_i = LIMIT( hc_acc_i, -500, 500 );	
	wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.35 *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;
	wz_speed_1 = wz_speed_0;
	if( ABS( wz_speed_1 ) < 50 )
	{
		wz_speed_1 = 0;
	}
	wz_speed_safe = wz_speed_1;
	
	

	
	wz_speed_pid_v_safe.err = wz_speed_pid_safe.kp *( exp_z_speed - wz_speed_safe );
	wz_speed_pid_v_safe.err_d = 0.002f/T *10*wz_speed_pid_safe.kd * (-wz_acc_mms2_safe) *T;//(wz_speed_pid_v_safe.err - wz_speed_pid_v_safe.err_old);
	wz_speed_pid_v_safe.err_i += wz_speed_pid_safe.ki *wz_speed_pid_safe.kp *( exp_z_speed - h_speed ) *T;
	wz_speed_pid_v_safe.err_i = LIMIT(wz_speed_pid_v.err_i,-Thr_Weight *800,Thr_Weight *800);//-30~30 //(WT)
	wz_speed_pid_v_safe.pid_out = thr_lpf + Thr_Weight *LIMIT(( wz_speed_pid_safe.kp*0.65 *LIMIT(exp_z_speed,-150,150)+wz_speed_pid_v_safe.err + wz_speed_pid_v_safe.err_d + wz_speed_pid_v_safe.err_i),-300,300);
	wz_speed_pid_v_safe.err_old = wz_speed_pid_v_safe.err; 
	wz_speed_old_safe=wz_speed_safe;
}

void Ultra_Ctrl_Safe(float T,float thr)
{
	float ultra_sp_tmp,ultra_dis_tmp;	
		if(mode.height_safe==0||EXP_Z_SPEED!=0)
		{exp_height_safe=ALT_POS_BMP*1000;}

    ultra_sp_tmp=my_deathzoom_2(-ALT_VEL_BMP*1000,0);//ALT_VEL_BMP*1000;//
		ultra_sp_tmp=LIMIT(ultra_sp_tmp,-1000,1000);
		if( fabs(ultra_sp_tmp) < 100 ){
			ultra_speed_safe += ( 1 / ( 1 + 1 / ( 4 *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed_safe );
		}
		else
		{
			ultra_speed_safe += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed_safe );
		}
	
	
	ultra_dis_lpf_safe = ALT_POS_BMP*1000;//ALT_POS_BMP*1000;	
			
		
 	ultra_ctrl_safe.err = ( ultra_pid_safe.kp*my_deathzoom_2(exp_height_safe - ultra_dis_lpf_safe,10) );
	
	ultra_ctrl_safe.err_i += ultra_pid_safe.ki *ultra_ctrl_safe.err *T;
	
	ultra_ctrl_safe.err_i = LIMIT(ultra_ctrl_safe.err_i,-Thr_Weight *ULTRA_INT,Thr_Weight *ULTRA_INT);
	
	ultra_ctrl_safe.err_d = ultra_pid_safe.kd *( (ultra_ctrl_safe.err - ultra_ctrl_safe.err_old) );
	
	ultra_ctrl_safe.pid_out = ultra_ctrl_safe.err + ultra_ctrl_safe.err_i + ultra_ctrl_safe.err_d;
	
	ultra_ctrl_safe.pid_out = LIMIT(ultra_ctrl_safe.pid_out,-750,750);//
		
	ultra_ctrl_out_safe = ultra_ctrl_safe.pid_out;
	
	ultra_ctrl_safe.err_old = ultra_ctrl_safe.err;
 
}
