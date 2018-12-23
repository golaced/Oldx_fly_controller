#include "include.h"
#include "usart.h"
#include "mymath.h"
#include "m100.h"
#include "ctrl.h"
#include "alt_fushion.h"
void px4_control_publish(float x,float y,float z,float yaw,u8 mode,u8 save_data, u8 save_video);
	
u8 dji_rst;
u8 en_vrc;
u8 m100_control_mode = PX4_MODE_OFFBOARD;//0x66 mission
u8 mission_switch;
u8 force_mode_m100=0;
u8 px4_rc_hover=0;
float k_m100[5]={1,1,1,1,1};//pit rol thr yaw avoid
float k_px4[4]={1,1,0.8,0.7};
float k_px4_rc[4]={0.008,0.008,0.005,1};
float tar_px4[4]={0};
float tar_px4_rc[4]={0};
u16 Rc_Pwm_Inr_mines[8]={1500,1500,1500,1500};
u16 Rc_Pwm_Out_mine_USE[8]={1500,1500,1500,1500},Rc_Pwm_Inr_mine[8]={1500,1500,1500,1500};
u8 force_save[2]={0};
void sdk_task(float dt)
{		
	static u8 cnt_m100;
	static u8 en_vrcr,flag1;
	static int m100_Rc_gr;

	Rc_Pwm_Inr_mine[RC_PITCH]=-(float)LIMIT((px4.Rc_pit-1514)*1.46,-500,500)/1.*1+1500;	
	Rc_Pwm_Inr_mine[RC_ROLL] =(float)LIMIT((px4.Rc_rol-1514)*1.46,-500,500)/1.*1+1500;	
	Rc_Pwm_Inr_mine[RC_YAW]=  (float)LIMIT((px4.Rc_yaw-1514)*1.46,-500,500)/1.*1+1500;	
	Rc_Pwm_Inr_mine[RC_THR]=  (float)LIMIT((px4.Rc_thr-1514)*1.46,-500,500)/1.*1+1500;	
	Rc_Pwm_Inr_mine[RC_MODE]= (float)LIMIT((px4.Rc_mode-1514)*1.46,-500,500)/1.*1+1500;	
	Rc_Pwm_Inr_mine[RC_GEAR]= (float)LIMIT((px4.Rc_gear-1514)*1.46,-500,500)/1.*1+1500;	

  u8 en_px4_control[3]={0};

	if(smart.rc.ATT_MODE!=0&&smart.rc.pos_switch[0]!=0)
	en_px4_control[0]=1;
	if(smart.rc.HEIGHT_MODE!=0&&smart.rc.pos_switch[1]!=0)
	en_px4_control[1]=1;
	if(smart.rc.POS_MODE!=0&&smart.rc.pos_switch[2]!=0)
	en_px4_control[2]=1;
	
	tar_px4[0]=LIMIT((float)nav_spd_ctrl[X].exp/1000.*k_px4[0],-10,10)*en_px4_control[2];//x
	tar_px4[1]=LIMIT((float)nav_spd_ctrl[Y].exp/1000.*k_px4[1],-10,10)*en_px4_control[2];//y
	tar_px4[2]=LIMIT((float)ultra_ctrl.pid_out/1000.*k_px4[2],-10,10)*en_px4_control[1];
	tar_px4[3]=LIMIT(ctrl_2.out.z*k_px4[3],-1680,1680)*en_px4_control[0];
	tar_px4_rc[0]=LIMIT((my_deathzoom_21((float)Rc_Pwm_Inr_mine[RC_ROLL]-1500,88))*k_px4_rc[0],-10,10);
	tar_px4_rc[1]=LIMIT((my_deathzoom_21((float)Rc_Pwm_Inr_mine[RC_PITCH]-1500,88))*k_px4_rc[1],-10,10);
	tar_px4_rc[2]=LIMIT((my_deathzoom_21((float)Rc_Pwm_Inr_mine[RC_THR]-1500,88))*k_px4_rc[2],-10,10);
	tar_px4_rc[3]=LIMIT((my_deathzoom_21((float)Rc_Pwm_Inr_mine[RC_YAW]-1500,88))*k_px4_rc[3],-1000,1000);
	if(tar_px4_rc[0]==0&&tar_px4_rc[1]==0&&tar_px4_rc[2]==0&&tar_px4_rc[3]==0)
	  px4_rc_hover=1;
	else
		px4_rc_hover=0;
		
	#if DEBUG_IN_ROOM
	en_vrc=1;	
	#else
	if(px4.Rc_gear>1400)//
	en_vrc=1;
	else
	en_vrc=0;
	#endif
	
	
	  if(px4.Rc_gear>1600){
		m100_control_mode =mission_switch; 
		}
    else{
		m100_control_mode =PX4_MODE_OFFBOARD; 	
		}
		
		if(force_save[1]){
		  m100.save_video=1;
		  m100.save_data=1;
	  }else if(state_v!=SU_MISSION)
		{ 
			//c2c.use_circle=1; 
			three_wheel_car.car_mode=0;
			three_wheel_car.gain[0]=0;
			m100.save_data=m100.save_video=0;	
		}
		
		if(force_mode_m100!=0)
		m100.px4_tar_mode=force_mode_m100;
		else
	  m100.px4_tar_mode=m100_control_mode;
		
		m100_data_save_publish();
		
		if(en_vrc&&px4.Rc_mode>1600&&px4.connect&&px4_rc_hover)
		px4_control_publish(tar_px4[0]*1,tar_px4[1]*1,tar_px4[2],tar_px4[3],m100.px4_tar_mode,m100.save_data,m100.save_video);
		//px4_control_publish(1,0,0,0,m100_control_mode);//float x,float y,float z,float yaw,u8 mode
		else
		px4_control_publish(tar_px4_rc[0],tar_px4_rc[1],tar_px4_rc[2],tar_px4_rc[3],m100.px4_tar_mode,m100.save_data,m100.save_video);
		static u8 cnt2;
		if(cnt2++>4){cnt2=0;
		send_to_px4();
		}
}

int m100_save_buf[11];

void m100_data_save_publish(void)
{
  m100_save_buf[0]=POS_UKF_X*100;
  m100_save_buf[1]=POS_UKF_Y*100;
	m100_save_buf[2]=ALT_POS_BMP_UKF_OLDX*100;
	m100_save_buf[3]=Yaw_fc*10;

  m100_save_buf[4]=c2c.gx*100;
  m100_save_buf[5]=c2c.gy*100;
	m100_save_buf[6]=c2c.check*c2c.connect;
	m100_save_buf[7]=c2c.yaw*10;

	m100_save_buf[8]=three_wheel_car.body_spd[0]*100;
  m100_save_buf[9]=three_wheel_car.body_spd[1]*100;
	m100_save_buf[10]=three_wheel_car.body_spd[2]*100;
}	

void send_to_px4(void){
u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x77;
	data_to_send[_cnt++]=0;
	
	_temp = (90-(aux.att_off[0]+aux.att_ctrl[0]))*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (aux.att_off[1]+aux.att_ctrl[1])*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Yaw_fc*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

		//save
	for(i=0;i<11;i++){
	_temp=m100_save_buf[i];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	}
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK_NAV(data_to_send, _cnt);
}

void m100_contrl_px4(float x,float y,float z,float yaw,u8 mode,u8 save_data, u8 save_video)
{
	vs16 _temp;
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	u8 data_to_send[50];

	UsartSend_GOL_LINK_NAV(0xFA);
	UsartSend_GOL_LINK_NAV(0xFB);
	UsartSend_GOL_LINK_NAV(0x04);
	UsartSend_GOL_LINK_NAV(0x01);

	UsartSend_GOL_LINK_NAV(mode);	
	_temp=x*1000;
	UsartSend_GOL_LINK_NAV(BYTE1(_temp));
	UsartSend_GOL_LINK_NAV(BYTE0(_temp));
	_temp=y*1000;
	UsartSend_GOL_LINK_NAV(BYTE1(_temp));
	UsartSend_GOL_LINK_NAV(BYTE0(_temp));
	_temp=z*1000;
	UsartSend_GOL_LINK_NAV(BYTE1(_temp));
	UsartSend_GOL_LINK_NAV(BYTE0(_temp));
	_temp=yaw*100;
	UsartSend_GOL_LINK_NAV(BYTE1(_temp));
	UsartSend_GOL_LINK_NAV(BYTE0(_temp));
	
	UsartSend_GOL_LINK_NAV(BYTE0(save_data));
	UsartSend_GOL_LINK_NAV(BYTE0(save_video));
  UsartSend_GOL_LINK_NAV(BYTE0(c2c.use_circle));
	
	UsartSend_GOL_LINK_NAV(0xFE);	
}

void px4_control_publish(float x,float y,float z,float yaw,u8 mode,u8 save_data, u8 save_video)
{
	m100.control_spd[0]=x;
	m100.control_spd[1]=y;
	m100.control_spd[2]=z;
	m100.control_yaw=yaw;
}



void V_SLAM_CONTROL(void)
{ u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	//Pit_fc=11;Rol_fc=22;Yaw_fc=-33;
	_temp = (int)(Pit_fc*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Rol_fc*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Yaw_fc*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp=vslam.reset;
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK_NAV(data_to_send, _cnt);
}  