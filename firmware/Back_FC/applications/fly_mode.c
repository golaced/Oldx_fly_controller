#include "fly_mode.h"
#include "rc.h"
#include "ak8975.h"
#include "rc_mine.h"
#include "alt_fushion.h"
#include "mpu6050.h"
ERO ero;
u8 mode_value[10];
u8 mode_state,mode_state_old;
u8 height_ctrl_mode_rc;
_MODE mode_oldx;
MOUDLE module;
//#NS
void mode_check(float *ch_in,u8 *mode_value)
{
	 #if USE_RECIVER_MINE		 //使用我的手柄   未使用
		if( RX_CH[AUX4r] < 100 )
		{
			height_ctrl_mode_rc = 0;
		}
		else if(RX_CH[AUX4r] >800  )
		{
			height_ctrl_mode_rc = 1;//气压计
		}
		else
		{
			if(ultra_ok == 1)
			{
				height_ctrl_mode_rc = 2;//超声波
			}
			else
			{
				height_ctrl_mode_rc = 1;
			}
		}	
	#else
    //定高模式判断
		if(Rc_Get_PWM.HEIGHT_MODE <1200 )
		{
				if(module.sonar == 1)
			{
				height_ctrl_mode_rc = 2;//超声波
			}
			else
			{
				height_ctrl_mode_rc = 1;//气压计
			}
		}
		else if(Rc_Get_PWM.HEIGHT_MODE>1400 &&Rc_Get_PWM.HEIGHT_MODE <1600 )
		{
			  height_ctrl_mode_rc = 1;//气压计
		}
		else if(Rc_Get_PWM.HEIGHT_MODE >1800 )
		{
			
				height_ctrl_mode_rc = 0;//手动
		
		}
static u16 cnt_sonar_mask,cnt_loss_pos,mode_check_cnt[3];		
//auto switch for height feedback		
//	if(height_ctrl_mode_rc==2&&NS==2&&0 ){
//    if(ALT_POS_SONAR2<2&&cnt_sonar_mask>2/0.05&&module.sonar ==1){
//			height_ctrl_mode=2;
//			cnt_sonar_mask=65530;
//		}
//		else{
//			height_ctrl_mode=1;
//		}
//		
//		if(ALT_POS_SONAR2<1.5)
//			cnt_sonar_mask++;
//		else
//			cnt_sonar_mask=0;
//	}
//	else
	if(height_ctrl_mode_rc==2){
		if(ALT_POS_SONAR2<3&&ALT_POS_BMP_UKF_OLDX<3.5)	
			height_ctrl_mode=2;
		else
			height_ctrl_mode=1;
	}
	else
		height_ctrl_mode=height_ctrl_mode_rc;
	//height_ctrl_mode=height_ctrl_mode_rc;
	#if PX4_SDK	
	if(px4.connect&&module.sonar)
	  height_ctrl_mode=2;
	#endif
	  u8 mode_oldx_flow_hold_position;
		//定点模式判断
		if(Rc_Get_PWM.POS_MODE>1800)	
    {mode_check_cnt[0]++;mode_oldx_flow_hold_position=2;}
    else
    mode_check_cnt[0]=0;
    if(Rc_Get_PWM.POS_MODE<1400)
		{mode_check_cnt[1]++;mode_oldx_flow_hold_position=0;}
		else
	  mode_check_cnt[1]=0;
		if(Rc_Get_PWM.POS_MODE>=1400&&Rc_Get_PWM.POS_MODE<=1800)	
		{mode_check_cnt[2]++;mode_oldx_flow_hold_position=1;}
    else
    mode_check_cnt[2]=0;
    if(mode_check_cnt[0]>0.25/0.05){mode_check_cnt[0]=0;
		mode_oldx.flow_hold_position=2;	}//智能
		if(mode_check_cnt[1]>0.25/0.05){mode_check_cnt[1]=0;
		mode_oldx.flow_hold_position=0; }//手动
    if(mode_check_cnt[2]>0.25/0.05){mode_check_cnt[2]=0; 
		mode_oldx.flow_hold_position=1;	}//速度
    
		if(px4.connect&&px4.m100_data_refresh)
		{
			if(px4.Rc_gear<1400)
		  mode_oldx.flow_hold_position=1;
			else
			mode_oldx.flow_hold_position=2;	
		}
		static u8 state_cal_en,mode_reg;
		static u16 cnt1,cnt_time;
		switch(state_cal_en)
		{
			case 0:
				if(mode_oldx_flow_hold_position!=mode_reg)
				{cnt1=0;state_cal_en=1;cnt_time=0;}
				break;
		  case 1:
				if(mode_oldx_flow_hold_position!=mode_reg)
				{cnt1=0;state_cal_en=1;cnt_time++;cnt1=0;}
				if(cnt1++>2.22/0.05)
				{cnt1=0;state_cal_en=0;}
				else if(cnt_time>8*2)
				{cnt1=0;state_cal_en=0;cnt_time=0;
				//if(KEY[0]&&KEY[1]==1)
				if(height_ctrl_mode_rc>0)
					mpu6050_fc.Acc_CALIBRATE=mpu6050_fc.Gyro_CALIBRATE=1;	
				else
					ak8975_fc.Mag_CALIBRATED=1;
				
				}	
			break;
		}
		
		
		if((m100.navigation_mode==2&&m100.GPS_STATUS<6)||(m100.navigation_mode==1&&m100.GPS_STATUS<50)||m100.navigation_mode==0)//no gps or flow
		  cnt_loss_pos++;
    else 
      cnt_loss_pos=0;
    if(cnt_loss_pos>3/0.05)		
			mode_oldx.flow_hold_position=cnt_loss_pos=0;
	  mode_reg=mode_oldx_flow_hold_position;	
    #if  USE_M100_IMU
		mode_oldx.flow_hold_position=mode_oldx.flow_hold_position*m100.m100_connect;	
    #endif		
#endif	
		
    if(height_ctrl_mode!=0){
			if(height_ctrl_mode==1){
				if(!mode_oldx.flow_hold_position)	
				drone.fly_mode=2;
				else
				drone.fly_mode=3;
			}
			else{
				if(!mode_oldx.flow_hold_position)	
				drone.fly_mode=4;
				else
				drone.fly_mode=5;
			}
		}
		else
		drone.fly_mode=1;	
}
