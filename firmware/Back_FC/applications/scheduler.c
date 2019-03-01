
#include "scheduler.h"
#include "include.h"
#include "time.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "led.h"
#include "rc.h"
#include "imu.h"
#include "pwm_in.h"
#include "ctrl.h"
#include "bmp.h"
#include "parameter.h"
#include "ultrasonic.h"
#include "height_ctrl.h"
#include "fly_mode.h"
#include "anotc_baro_ctrl.h"
#include "rc_mine.h"
#include "alt_fushion.h"
#include "sbus.h"
#include "iic_hml.h"
#include "ms5611_spi.h"
#include "bat.h"
#include "m100.h"
#include "beep.h"
#include "mavl.h"
float pos_time;
float baro_task_time;
u16 Rc_Pwm_In[8];
s16 loop_cnt;
loop_t loop;
u8 yaw_use_fc=0;
void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;
	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
	 #if USE_VER_3
	 LED_1ms_DRV();
	 #endif
}
float outer_loop_time;
float inner_loop_time,inner_loop_time_yaw;
float test[5];	
float Rol_fc1,Pit_fc1,Yaw_fc1;
void Duty_1ms()
{
	  #if USE_VER_3&&!USE_VER_6
	  MS5611_ThreadNew_SPI();baroAlt_fc=ms5611Alt*1000;MS5611_Pressure=ms5611Press;
	  #else
	  MS5611_ThreadNew();
	  #endif
	
	  baro.relative_height = baroAlt_fc;baro.height=MS5611_Pressure;  
}


void Duty_2ms()
{ 

	static u8 init,init_beep[2];
	static u8 cnt;
	static u16 cnt_init;
  float temp;
	temp = Get_Cycle_T(GET_T_INNER)/1000000.0f; 						//获取内环准确的执行周期
	if(temp<0.001)
		inner_loop_time=0.002;
	else
		inner_loop_time=temp;
	
	#if EN_ATT_CAL_FC
	MPU6050_Read(); 															//读取mpu6轴传感器

	MPU6050_Data_Prepare( inner_loop_time );			//mpu6轴传感器数据处理

	/*IMU更新姿态。输入：半个执行周期，三轴陀螺仪数据（转换到度每秒），三轴加速度计数据（4096--1G）；输出：ROLPITYAW姿态角*/
	if(init_beep[0]==1&&!fly_ready){init_beep[0]=2;
	Play_Music_Direct(MEMS_RIGHT_BEEP);
	}
	
	if((init_beep[1]==0&&init_beep[0]==2&&m100.GPS_STATUS>7&&m100.STATUS==3&&!fly_ready)||task_play_flag[0]==1){
		init_beep[1]=1;
		task_play_flag[0]=1;
	  Play_Music_Task(MEMS_GPS_RIGHT,inner_loop_time);
	}
	if((init_beep[1]==1&&init_beep[0]==2&&(m100.GPS_STATUS<4||m100.STATUS!=3||m100.navigation_mode==0)&&!fly_ready)||
		task_play_flag[0]==2){
		init_beep[1]=2;
		task_play_flag[0]=2;
	  Play_Music_Task(MEMS_GPS_LOSS,inner_loop_time);
	}
	
	if(cnt_init++>2/0.002){cnt_init=65530;
  if(!init_beep[0])init_beep[0]=1;
	module.system=1;
 	IMUupdate(0.5f *inner_loop_time,mpu6050_fc.Gyro_deg.x, mpu6050_fc.Gyro_deg.y, mpu6050_fc.Gyro_deg.z, mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z
	,&Rol_fc1,&Pit_fc1,&Yaw_fc1);
		#if PX4_SDK
		if(!px4.connect){
		#else
		if(1){
		#endif
		Pit_fc=Pit_fc1-mpu6050_fc.att_off[0]*mpu6050_fc.Cali_3d;	
		Rol_fc=Rol_fc1-mpu6050_fc.att_off[1]*mpu6050_fc.Cali_3d;		
			
		yaw_use_fc=YAW_FC;
		if(NAV_BOARD_CONNECT&&!yaw_use_fc)
			Yaw_fc=Yaw;
		else
			Yaw_fc=Yaw_fc1;
		}
		else
		{
		Pit_fc=px4.Pit;
		Rol_fc=px4.Rol;
		Yaw_fc=px4.Yaw;
		}
  }	
	#endif
	CTRL_1( inner_loop_time ); 							//内环角速度控制
	
	RC_Duty( inner_loop_time , Rc_Pwm_In );		// 遥控器通道数据处理 ，输入：执行周期，接收机pwm捕获的数据。
	if(motor_test)
	  motorsTest400(0.2);
}


void Duty_5ms()
{ 
	static u8 cnt;
	float temp;
	temp = Get_Cycle_T(GET_T_OUTTER)/1000000.0f;								//获取外环准确的执行周期
	if(temp<0.001)
		outer_loop_time=0.005;
	else
		outer_loop_time=temp;
	
 	CTRL_2( outer_loop_time ); // 外环角度控制
}


u8 UART_UP_LOAD_SEL=0;//<------------------------------上传数据选择
u8 UART_UP_LOAD_SEL_FORCE=0;//<--上位机强制选择
u8 force_flow_ble_debug;
u8 flow_debug_stop=1;
void Duty_10ms()
{
	static u8 cnt_bmp;
	static u8 cnt[4];					 		
//--------------------------------------To  IMU模块--------------------------------------------	
				if(cnt[1]++>2){cnt[1]=0;	
				  #if EN_DMA_UART2 					
					if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA2_Steam7传输完成
								{ 
							DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//清除DMA2_Steam7传输完成标志
							data_per_uart2();
					    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);     //开始一次DMA传输！	
								}	
					#else				
								#if PX4_SDK
                if(!px4.connect)
								#endif
								 GOL_LINK_TASK();	
					#endif
							}					
							
//-----------------------------------------------BLE UPLOAD---------------------------蓝牙调试
			  flow_debug_stop=0;//取消注释则 强制FC蓝牙输出			
			 #if defined(HEIGHT_TEST) 
        UART_UP_LOAD_SEL=2;
				flow_debug_stop=0;	
	     #elif defined(POS_SPD_TEST)	
				UART_UP_LOAD_SEL=3;
				flow_debug_stop=0;
       #elif defined(POS_TEST)
				if(mode_oldx.flow_hold_position==1&&px4.connect==0)		
        UART_UP_LOAD_SEL=3;
        else				
				UART_UP_LOAD_SEL=5;
				if(mode_oldx.flow_hold_position>0)		
				flow_debug_stop=0;
        else
        flow_debug_stop=1;
			 #elif defined(AUTO_DOWN)
				UART_UP_LOAD_SEL=2;
				flow_debug_stop=0;								
       #endif							
			 if(UART_UP_LOAD_SEL_FORCE!=0)
          UART_UP_LOAD_SEL=UART_UP_LOAD_SEL_FORCE;				 
 
							if(cnt[2]++>1){cnt[2]=0;
								    if(mode_oldx.att_pid_tune){//PID TUNING
											{	
										 #if !TUNING_Z									
												if(ctrl_2.PID[PIDROLL].kp!=0&&KEY[7])//OUTTER
												data_per_uart1(
												#if TUNING_X
												0,-except_A.x*10,0,
												#else
												0,except_A.y*10,0,
												#endif
												#if EN_ATT_CAL_FC
													#if TUNING_X
													0,-Rol_fc*10,0,
													#else
													0,Pit_fc*10,0,
													#endif
												#else
												0,-Roll*10,0,
												#endif
												-ctrl_2.err.y*10,0*10,0,
												(int16_t)(0*100),(int16_t)(0*10.0),(int16_t)(0*100.0),0/10,0,0/10,0*0);
												else//INNER
												data_per_uart1(
												#if TUNING_X
												0,-except_AS.x,0,
												#else
												0,except_AS.y,0,
												#endif
												#if TUNING_X
												0,-mpu6050_fc.Gyro_deg.x,0,
												#else
												0,-mpu6050_fc.Gyro_deg.y,0,
												#endif 
												-ctrl_1.err.y,0,0,
												(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0/10,0,0/10,0*0);
												}
											#else
											  if(ctrl_2.PID[PIDYAW].kp!=0&&KEY[7])//OUTTER
												data_per_uart1(
												0,-except_A.z*10,0,
												0,-Yaw_fc*10,0,
												-ctrl_2.err.z*10,0*10,0,
												(int16_t)(0*100),(int16_t)(0*10.0),(int16_t)(0*100.0),0/10,0,0/10,0*0);
												else//INNER
												data_per_uart1(
												0,-except_AS.z,0,
												0,-mpu6050_fc.Gyro_deg.z,0,
												-ctrl_1.err.z,0,0,
												(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0/10,0,0/10,0*0);
												}				
											#endif
										}
										else if((flow_debug.en_ble_debug||force_flow_ble_debug)&&flow_debug_stop)//DEBUG  FLOW
											data_per_uart1(
										#if USE_MINI_FC_FLOW_BOARD
										  debug_pi_flow[1],debug_pi_flow[2],debug_pi_flow[3],
										  debug_pi_flow[4],debug_pi_flow[5],debug_pi_flow[6],
										  debug_pi_flow[7],debug_pi_flow[8],debug_pi_flow[9],
										#else
											flow_debug.ax,flow_debug.ay,flow_debug.az,
										  flow_debug.gx,flow_debug.gy,flow_debug.gz,
										  flow_debug.hx,flow_debug.hy,flow_debug.hz,
										#endif
											(int16_t)(inner_loop_time*10000.0),(int16_t)(outer_loop_time*10000.0),(int16_t)(0*10.0),0/10,0,0/10,0*0);
										else{//DEBUG-------------------------Normal mode_oldx--------------------------------
								    switch(UART_UP_LOAD_SEL)
											{
											case 0://气压计融合
											data_per_uart1(
											ALT_POS_BMP_UKF_OLDX*100,hc_value.fusion_height/10,baro.h_flt*100,
											ALT_VEL_BMP_UKF_OLDX*100,acc_body[2]*10,wz_speed_pid_v.exp/10,
											ALT_VEL_BMP*100,nmda[2]*100,ALT_POS_SONAR2*100,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_value,0,0/10,0);break;	
											case 1://速度，加速度，高度和控制
											data_per_uart1(
											nav_spd_ctrl[Y].pid_out*10,nav_spd_ctrl[X].pid_out*10,ALT_POS_SONAR2*100,
											VEL_UKF_Y*100,VEL_UKF_X*100,ultra_speed/10,
											acc_body[Y]/100.,acc_body[X]/100.,0,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_value,0,0/10,0);break;	
											case 2://高度控制，速度z控制和速度xy
											data_per_uart1(
											ALT_POS_BMP_UKF_OLDX*1000,ultra_ctrl.exp,baro.h_flt*100,
											ALT_VEL_BMP_UKF_OLDX*1000, wz_speed_pid_v.exp,wz_speed_pid_v.now,
											VEL_UKF_Y*100,VEL_UKF_X*100,0,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_value,0,0/10,0);break;	
											case 3://速度xy测试
											data_per_uart1(
											0,0,0,
											nav_spd_ctrl[X].now,nav_spd_ctrl[X].exp,0,
											nav_spd_ctrl[Y].now,nav_spd_ctrl[Y].exp,0,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_value,0,0/10,0);break;	
											case 4://接收机信号
											data_per_uart1(
											Rc_Get_PWM.ROLL,Rc_Get_PWM.PITCH,Rc_Get_PWM.YAW,
											Rc_Get_PWM.THROTTLE, 0,0,
											VEL_UKF_Y*100,VEL_UKF_X*100,0,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_value,0,0/10,0);break;	
											case 5://位置xy测试
											data_per_uart1(
											ultra_ctrl.now/10,exp_height/10,0,
											nav_pos_ctrl[X].now*100,nav_pos_ctrl[X].exp*100,0,
											nav_pos_ctrl[Y].now*100,nav_pos_ctrl[Y].exp*100,0,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_value,0,0/10,0);break;	
											case 6:
											data_per_uart1(
											baro.h_flt*100,0,ALT_POS_BMP_UKF_OLDX*100,
											baro.v_flt*100,ALT_VEL_BMP_UKF_OLDX*100,ultra_speed/10,
											baro.acc_flt*100,0*100,hc_value.fusion_acc*100,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_value,0,0/10,0);break;		
											case 7:
											data_per_uart1(
											smart.spd.x*100,smart.spd.y*100,0,
											c2c.gx*100,c2c.gy*100,c2c.range_all*1000,
											c2c.pix_x-160,c2c.pix_y-120,c2c.check*10,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_value,0,0/10,0);break;		
											case 8:
											data_per_uart1(
											POS_UKF_X*100,POS_UKF_Y*100,Yaw_fc*10,
											c2c.x,c2c.y,c2c.yaw,
											0,0,c2c.check*10,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_value,0,0/10,0);break;		
										  case 9:
											data_per_uart1(
											POS_UKF_X*100,POS_UKF_Y*100,ALT_POS_BMP_UKF_OLDX*100,
											c2c.gx*100,c2c.gy*100,c2c.gyaw*10,
											c2c.gx_vm*100,c2c.gy_vm*100,c2c.gyaw_vm*10,
											(int16_t)(car_spd[0]*100),(int16_t)(car_spd[1]*100),(int16_t)(car_spd[2]*100),thr_value,0,0/10,0);break;	
											case 10:
											data_per_uart1(
											c2c.gx_vm*100,c2c.gy_vm*100,c2c.gyaw_vm*10,
											c2c.gx*100,c2c.gy*100,c2c.gyaw*10,
											c2c.pix_x*c2c.check,c2c.pix_y*c2c.check,0,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_value,0,0/10,0);break;													
											case 11://姿态PID
												if(Rc_Get_PWM.AUX2>1500)//OUTTER
													data_per_uart1(
													-except_A.x*10,-except_A.y*10,-except_A.z*10,
													-Rol_fc*10,Pit_fc*10,Yaw_fc*10,
													ctrl_2.err.x*10,-ctrl_2.err.y*10,-ctrl_2.err.z*10,
													(int16_t)(0*100),(int16_t)(0*10.0),(int16_t)(0*100.0),0/10,0,0/10,0*0);
												else//INNER
													data_per_uart1(
													-except_AS.x,except_AS.y,except_AS.z,
													-mpu6050_fc.Gyro_deg.x,-mpu6050_fc.Gyro_deg.y,-mpu6050_fc.Gyro_deg.z,
													-ctrl_1.err.x,-ctrl_1.err.y,-ctrl_1.err.z,
													(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0/10,0,0/10,0*0);		
											break;
											default:break;
											}
										}
									}
						
						
				//To  SD卡
				static u8 sd_sel;
				if(cnt[3]++>0){cnt[3]=0;
				
			     #if EN_DMA_UART4 			
					if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)
								{ 
							DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);
							
							clear_nrf_uart();		
							nrf_uart_cnt=0;
								
							switch(sd_sel){//SD卡存储
							case 0:sd_sel=1;		
						  sd_publish();		
							data_per_uart4(SEND_SD_SAVE1);	
							data_per_uart4(SEND_SD_SAVE2);	
							data_per_uart4(SEND_SD_SAVE3);	
							data_per_uart4(SEND_M100);	
							break;
							case 1:sd_sel=0;
							data_per_uart4(SEND_M100);	
							data_per_uart4(SEND_ALT);	
							data_per_uart4(SEND_FLOW);	
							data_per_uart4(SEND_PID);	
							data_per_uart4(SEND_QR);				
							break;				
							}
							USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);    
							MYDMA_Enable(DMA1_Stream4,nrf_uart_cnt+2);   
								}		
					#else
							SD_LINK_TASK2(SEND_IMU);	
					#endif
							}	
				
		update_mavlink();		
		if(module.nrf){
		  Nrf_Check_Event();
		  RC_Send_Task();
		}		
}

float att_test[2]={0,0};
#if defined(M_DRONE_330)
float k_fp_dj[2]={0.068,0.068*1.68};
#elif defined(M_DRONE_330X6)
float k_fp_dj[2]={0.068,0.089};
#else
float k_fp_dj[2]={0.068,0.068};
#endif
void Duty_20ms()
{   u16 temps;
	  #if USE_VER_8
	  aux.att[0]=aux.att_ctrl[0]+aux.att_off[0];
	  aux.att[1]=aux.att_ctrl[1]+aux.att_off[1];
	  #else
	   #if defined(BLDC_PAN)
			aux.att[0]=aux.att_ctrl[0]+aux.att_off[0];
	   #else
			aux.att[0]=Pit_fc1+aux.att_ctrl[0]+aux.att_off[0]-LIMIT(mpu6050_fc.Gyro_deg.y,-180,180)*k_fp_dj[0];
	   #endif
	  aux.att[1]=Rol_fc1+aux.att_ctrl[1]*0+aux.att_off[1]+LIMIT(mpu6050_fc.Gyro_deg.x,-180,180)*k_fp_dj[1];
	  #endif
	  #if FAN_IS_3AXIS
	  SetPwm_AUX(att_test[0],att_test[1]);
	  #else
	  SetPwm_AUX(aux.att[0],aux.att[1]);
	  #endif
	  aux34(aux.att_ctrl[2],aux.att_ctrl[3]); 
 		float temp =(float) Get_Cycle_T(GET_T_OUT_NAV)/1000000.;							
		if(temp<0.001)
		pos_time=0.02;	
		else
		pos_time=temp;
			
		#if defined(AUTO_MISSION)
			OLDX_MISSION_API(pos_time);//new api
		#endif
    #if defined(PX4_LINK)
		sdk_task(0.02);			
		m100_contrl_px4(m100.control_spd[0],m100.control_spd[1],m100.control_spd[2],
					m100.control_yaw,m100.px4_tar_mode,m100.save_data,m100.save_video);	
		#else
		if(vslam.connect)
			 V_SLAM_CONTROL();
		else
		{
		if(mode_oldx.sdk_flag)
			 UsartSend_GOL_LINK_NAV(0x01);//search
    else
			 UsartSend_GOL_LINK_NAV(0x00);//land
		}
		#endif	
	 	#if PX4_SDK

			  RX_CH_PWM[THRr]=	Rc_Get_PWM.THROTTLE=LIMIT(Rc_Pwm_Inr_mine[RC_THR],1000,2000)	;
				RX_CH_PWM[ROLr]=  Rc_Get_PWM.ROLL=my_deathzoom_rc(Rc_Pwm_Inr_mine[RC_ROLL],2)	;
				RX_CH_PWM[PITr]=  Rc_Get_PWM.PITCH=my_deathzoom_rc(Rc_Pwm_Inr_mine[RC_PITCH],2)	;
				RX_CH_PWM[YAWr]=  Rc_Get_PWM.YAW=my_deathzoom_rc(Rc_Pwm_Inr_mine[RC_YAW],2)	;
				Rc_Get_PWM.update=Rc_Get_PWM.connect=px4.connect;
				RX_CH_PWM[AUX3r]=Rc_Get_PWM.POS_MODE=Rc_Get_SBUS.AUX3;
				RX_CH_PWM[AUX4r]=Rc_Get_PWM.HEIGHT_MODE=Rc_Get_SBUS.AUX4;
		#else
			#if USE_MINI_FC_FLOW_BOARD||USE_VER_3
				#if USE_MINI_FC_FLOW_BOARD_BUT_USB_SBUS
				temps=((channels[0])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.ROLL=		 temps;
				temps=((channels[1])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.PITCH=		 temps;
				temps=((channels[2])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.THROTTLE=		 temps;
				temps=((channels[3])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.YAW=		 temps;
				temps=((channels[4])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.AUX1=		 temps;
				temps=((channels[5])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.AUX2=		 temps;
				temps=((channels[6])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.AUX3=		 temps;
				temps=((channels[7])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.AUX4=		 temps;
				Rc_Get_PWM.THROTTLE=Rc_Get_PWM.ROLL=Rc_Get_PWM.PITCH=Rc_Get_PWM.YAW=1500;
				RX_CH_PWM[THRr]=	Rc_Get_PWM.THROTTLE=LIMIT(Rc_Get_SBUS.THROTTLE-RX_CH_FIX_PWM[THRr],1000,2000)	;
				RX_CH_PWM[ROLr]=  Rc_Get_PWM.ROLL=my_deathzoom_rc(Rc_Get_SBUS.ROLL-RX_CH_FIX_PWM[ROLr],2)	;
				RX_CH_PWM[PITr]=  Rc_Get_PWM.PITCH=my_deathzoom_rc(Rc_Get_SBUS.PITCH-RX_CH_FIX_PWM[PITr],2)	;
				RX_CH_PWM[YAWr]=  Rc_Get_PWM.YAW=my_deathzoom_rc(Rc_Get_SBUS.YAW-RX_CH_FIX_PWM[YAWr],2)	;
				Rc_Get_PWM.AUX1=Rc_Get_SBUS.AUX1;
				Rc_Get_PWM.AUX2=Rc_Get_SBUS.AUX2;
				Rc_Get_PWM.AUX3=Rc_Get_SBUS.AUX3;
				Rc_Get_PWM.AUX4=Rc_Get_SBUS.AUX4;
				Rc_Get_PWM.connect=Rc_Get_SBUS.connect;
				Rc_Get_PWM.update=Rc_Get_SBUS.update;
				RX_CH_PWM[AUX3r]=Rc_Get_PWM.POS_MODE=Rc_Get_SBUS.AUX3;
				RX_CH_PWM[AUX4r]=Rc_Get_PWM.HEIGHT_MODE=Rc_Get_SBUS.AUX4;
				
				if(mode_oldx.rc_loss_return_home>0&&NS==0&&fly_ready)//fail safe
				{
					RX_CH_PWM[THRr]=RX_CH_PWM[YAWr]=RX_CH_PWM[ROLr]=RX_CH_PWM[PITr]=1500;
					RX_CH_PWM[AUX3r]=Rc_Get_PWM.POS_MODE=2000;
				  RX_CH_PWM[AUX4r]=Rc_Get_PWM.HEIGHT_MODE=1500;
				}		
	
				#endif
			Nrf_Check_Event();
			RC_Send_Task();
			#endif
		#endif

		float temp1 =(float) Get_Cycle_T(GET_T_BARO)/1000000.;							
		if(temp1<0.001)
		baro_task_time=0.02;	
		else
		baro_task_time=temp1;

		WindEstimate(baro_task_time);
		baro_ctrl(baro_task_time,&hc_value);//高度融合		
		Positon_control1(baro_task_time);//位置控制	
		//OLDX_REMOTE
		#if USE_OLDX_REMOTE	
		static u8 auto_fly_state,auto_fly;
		static u16 cnt_sel,cnt_hml_cal;
		static u16 cnt_out_auto;
		switch(auto_fly_state)
		{
		case 0:
		if(KEY[4])
		cnt_sel++;
		if(cnt_sel>0.05/pos_time)
		{cnt_sel=0;
		auto_fly=!auto_fly;
		auto_fly_state=1;
		}
		break;
		case 1:
		if(cnt_sel++>1/pos_time)
		{cnt_sel=0;
		auto_fly_state=0;
		}
		break;			
		}
    
		if(KEY[4])
		  cnt_hml_cal++;
		else 
			cnt_hml_cal=0;
		
		if(cnt_hml_cal>5/0.02&&KEY[1]&&KEY[0]&&ak8975_fc.Mag_CALIBRATED==0)
			ak8975_fc.Mag_CALIBRATED=1;
		if((ABS(Rc_Get.PITCH-1500)>400||ABS(Rc_Get.ROLL-1500)>400)&&auto_fly==1)
			 cnt_out_auto++;
		if(cnt_out_auto>10)
		{
			cnt_out_auto=0;
			auto_fly=0;
		}

		if(module.flow==0)
		auto_fly=0;
		
		if(auto_fly){
			if(KEY[1])
			 Rc_Get_PWM.POS_MODE=2000;	
			else
		  Rc_Get_PWM.POS_MODE=1500;
		}
		else
		Rc_Get_PWM.POS_MODE=1000;
		
		if(auto_fly){
		RX_CH_PWM[THRr]=	LIMIT(my_deathzoom_rc1(Rc_Get.PITCH,160,170),1500-155,1500+155);
		RX_CH_PWM[YAWr]=  my_deathzoom_rc1(Rc_Get.ROLL,268,268);
			
		RX_CH_PWM[ROLr]=  Rc_Get.YAW;
		RX_CH_PWM[PITr]=  Rc_Get.THROTTLE;
			
		}else{
		RX_CH_PWM[THRr]=	Rc_Get.THROTTLE;
		RX_CH_PWM[ROLr]=  Rc_Get.ROLL;
		RX_CH_PWM[PITr]=  Rc_Get.PITCH;
		RX_CH_PWM[YAWr]=  Rc_Get.YAW;
		}
		if(!KEY[0])
			Rc_Get_PWM.HEIGHT_MODE=1000;	
		else 
			Rc_Get_PWM.HEIGHT_MODE=1500;	
		
		 if(KEY[1])
				Rc_Get_PWM.AUX1=2000;
		 else
				Rc_Get_PWM.AUX1=0;			
		#endif	
			
		//------------------------Smart UPDATE--------------------
		if((ABS(Rc_Get_PWM.ROLL-1500)<50&&ABS(Rc_Get_PWM.PITCH-1500)<50)&&mode_oldx.flow_hold_position==2)
				switch(smart.rc.POS_MODE)
				{
					case SMART_MODE_RC://rc
						RX_CH_PWM[THRr]=	smart.rc.THROTTLE;
						RX_CH_PWM[ROLr]=  smart.rc.ROLL;
						RX_CH_PWM[PITr]=  smart.rc.PITCH;
						RX_CH_PWM[YAWr]=  smart.rc.YAW;
						break;
					case SMART_MODE_SPD://spd
						nav_spd_ctrl[X].exp=smart.spd.x*1000;
						nav_spd_ctrl[Y].exp=smart.spd.y*1000;
						ultra_ctrl_out_use=smart.spd.z*1000;
						break;
					case SMART_MODE_SPD_RATE://spd
						nav_spd_ctrl[X].exp=smart.spd.x*1000;
						nav_spd_ctrl[Y].exp=smart.spd.y*1000;
						ultra_ctrl_out_use=smart.spd.z*1000;
						break;
					case SMART_MODE_POS://pos
						nav_pos_ctrl[X].exp=smart.pos.x;
						nav_pos_ctrl[Y].exp=smart.pos.y;
						exp_height=smart.pos.z*1000;
						break;
				}
				
		static u8 mav_state;
		static u16 cnt[3];
		if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
		{ 	
			DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
			for	(SendBuff1_cnt=0;SendBuff1_cnt<SEND_BUF_SIZE1;SendBuff1_cnt++)
				SendBuff1[SendBuff1_cnt]=0;
			SendBuff1_cnt=0;
      if(cnt[0]++>1/0.02){cnt[0]=0;			
      mavlink_send_message(0, MSG_HEARTBEAT, 0);
			mavlink_msg_sys_status_send(MAVLINK_COMM_0, 0, 0, 0, 1000-mavlinkData.idlePercent,
				bat.average * 1000, -1, bat.percent*100, 0, mavlinkData.packetDrops, 0, 0, 0, 0);
			u8 RADIO_QUALITY;
			//if()
			//mavlink_msg_radio_status_send(MAVLINK_COMM_0, RADIO_QUALITY, 0, 0, 0, 0, 0, 0);	
			}		
	    if(cnt[1]++>0.5/0.02){cnt[1]=0;			
      mavlink_send_message(0, MSG_RADIO_IN, 0);
			mavlink_msg_scaled_imu_send(MAVLINK_COMM_0, GetSysTime_us(), mpu6050_fc.Acc.x,  mpu6050_fc.Acc.y,  mpu6050_fc.Acc.z, 
				 mpu6050_fc.Gyro_deg.x*10.0f, mpu6050_fc.Gyro_deg.y*10.0f, mpu6050_fc.Gyro_deg.z*10.0f,0,0,0);
   		}						

			if(cnt[2]++>2/0.02){cnt[2]=0;			
			uint8_t satellites_visible=0,fix_tpy=0;
			if(m100.STATUS==3&&1){
			if(m100.STATUS>8)
			fix_tpy=3;
			else
			fix_tpy=2;
			satellites_visible=m100.GPS_STATUS;}
			mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, GetSysTime_us(), fix_tpy, m100.Lat*(double)1e7, m100.Lon*(double)1e7, 
			0,0,0,0,0, satellites_visible);
			}
						
			mavlink_send_message(0, MSG_ATTITUDE, 0);
			mavlink_send_message(0, MSG_LOCATION, 0);
		
			// request announced waypoints from mission planner
			if (mavlinkData.wpCurrent < mavlinkData.wpCount && mavlinkData.wpAttempt <= AQMAVLINK_WP_MAX_ATTEMPTS && mavlinkData.wpNext < GetSysTime_us() ) {
				mavlinkData.wpAttempt++;
				mavlink_msg_mission_request_send(MAVLINK_COMM_0, mavlinkData.wpTargetSysId, mavlinkData.wpTargetCompId, mavlinkData.wpCurrent);
				mavlinkData.wpNext = GetSysTime_us() + AQMAVLINK_WP_TIMEOUT;
			}
			// or ack that last waypoint received
			else if (mavlinkData.wpCurrent == mavlinkData.wpCount) {
				mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlinkData.wpTargetSysId, mavlinkData.wpTargetCompId, 0);
				mavlinkData.wpCurrent++;
			//AQ_PRINTF("%u waypoints loaded.", mavlinkData.wpCount);
			} else if (mavlinkData.wpAttempt > AQMAVLINK_WP_MAX_ATTEMPTS) {
			//AQ_NOTICE("Error: Waypoint request timeout!");
			}
		
			USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
			MYDMA_Enable(DMA2_Stream7,SendBuff1_cnt+2);     //开始一次DMA传输！	  
		}		
	

}

float bat_o;
void Duty_50ms()//遥控 模式设置
{  
    u8 i;	
	  static u8 way_point_update,count_reg=1;
	  static u16 cnt=0;
	  switch(way_point_update)
		{
			case 0:
				 if(mavlinkData.wpCurrent!=count_reg)
					 way_point_update=1;
			break;
			case 1:
				 if(mavlinkData.wpCount==mavlinkData.wpCurrent-1)
					 way_point_update=2;
			break;
			case 2:
				 if(cnt++>2/0.05){
					 cnt=0;
				   way_point_update=3;
				 }
			case 3:			
			if(!fly_ready){	
			way_point_update=0;
			navGetWaypointCount();	
			#if !FLASH_USE_STM32
				WRITE_PARM_WAY_POINTS();
			#endif
			mavlinkData.wpNext=1;	
			Play_Music_Direct(MEMS_WAY_UPDATE);
			
			}				
			break;
		}
	  count_reg=mavlinkData.wpCurrent;
		
		
	  #if USE_VER_3
		#if defined(USE_LED)
			LED_ON(mode_oldx.led_flag);
		#endif
	  LED_Duty(0.05);
	  #endif
	  if(ak8975_fc.Mag_CALIBRATED==1)
			mode_oldx.mems_state=31;	
	  else
			mode_oldx.mems_state=0; 
		
		mode_oldx.baro_f_use_ukfm=0;//1->use gps height
		//---------------use now
		//------------0 1   |   2 3       KEY_SEL
		#if USE_RECIVER_MINE		
		mode_oldx.flow_hold_position=KEY_SEL[0];
		mode_oldx.height_safe=KEY_SEL[1];
		mode_oldx.en_pid_sb_set=KEY_SEL[2];
    #else
	  mode_oldx.show_qr_origin=KEY_SEL[0];
    mode_oldx.en_sd_save=KEY_SEL[1];		
    #endif	

//-------------------------------------------------	
		#if !USE_RECIVER_MINE
			#if !USE_TOE_IN_UNLOCK
			if(Rc_Get_PWM.RST>1500&&Rc_Get_PWM.update&&Rc_Get_PWM.THROTTLE>1000)
			fly_ready=1;
			else
			fly_ready=0;
			#endif
		#else
			#if  DEBUG_WITHOUT_SB
			if(cnt2++>200)//
			{fly_ready=1;cnt2=200+1;}
			#else
				#if !USE_RC_GROUND&&!USE_TOE_IN_UNLOCK
					if(Rc_Get_PWM.RST>1500)
						fly_ready=1;
						else
						fly_ready=0;
				#else
				fly_ready=KEY_SEL[3];//解锁
				#endif
			#endif
		#endif
		//-----------------------------constant parameter--------------------------
		if(mode_oldx.flow_hold_position==2&&(state_v==SD_HOLD||state_v==SD_HOLD1))
		mode_oldx.h_is_fix=1;		
		else
		mode_oldx.h_is_fix=0;	
		

	  if(mode_oldx.flow_hold_position==2&&Rc_Get_PWM.AUX1>1500)
		mode_oldx.rc_control_flow_pos_sel=TRAJ1_1;//---------------------------------------------------
		else
		mode_oldx.rc_control_flow_pos_sel=0; 

	  //------------7 6 5 4  |  3 2 1 0  KEY
		#if USE_M100_IMU//2 -> origin 1-> KF mine
		mode_oldx.flow_f_use_ukfm=1;
		#else
		mode_oldx.flow_f_use_ukfm=1;
		#endif
    //-------------------飞控测试模式汇总----------------
	  #if defined(POS_SPD_TEST)
	  if(Rc_Get_PWM.AUX1>1500||(px4.connect&&px4.Rc_gear>1400))
		mode_oldx.trig_flow_spd=1;
		else
		mode_oldx.trig_flow_spd=0;	
		#elif defined(POS_TEST)
	  if(Rc_Get_PWM.AUX1>1500&&mode_oldx.flow_hold_position==1)
		mode_oldx.trig_flow_spd=1;
		else
		mode_oldx.trig_flow_spd=0;			
		#elif defined(HEIGHT_TEST) 
	  if(Rc_Get_PWM.AUX1>1500)
	  mode_oldx.fc_test1=1;
	  else
		mode_oldx.fc_test1=0;
		#elif defined(AUTO_MAPPER)		
		if(Rc_Get_PWM.AUX1>1500) 
   	mode_oldx.px4_map=1;
		else
		mode_oldx.px4_map=0;
		#elif defined(AUTO_HOME)		
		if(Rc_Get_PWM.AUX1>1500) 
   	mode_oldx.return_home=1;
		else
		mode_oldx.return_home=0;
		#else
		if(Rc_Get_PWM.AUX1>1500) 
   	mode_oldx.auto_sdk=1;
		else
		mode_oldx.auto_sdk=0;
		#endif
	
		mode_oldx.test4=1;//1-->position control with acc_loop
//		if(Rc_Get_PWM.AUX2>1500) 
//   	mode_oldx.use_uwb_as_pos=1;// bmp use height
//		else
//		mode_oldx.use_uwb_as_pos=0;		
//		
//		mode_oldx.height_in_speed=1;
//		if(Rc_Get_PWM.AUX2>1500) 
//   	mode_oldx.height_in_speed=0;// bmp use height
//		else
//		mode_oldx.height_in_speed=1;
		
//	  if(Rc_Get_PWM.AUX1>1500)
//		mode_oldx.show_qr_origin=1;
//		else
//		mode_oldx.show_qr_origin=0;	
#if defined(SD_SAVER)	
//sd save
		if(Rc_Get_PWM.AUX2>1500) 
   		mode_oldx.en_sd_save=1;
		else
		  mode_oldx.en_sd_save=0;
#endif		
//-------------------------------------------------------------------------------------------		
	#if defined(PID_TUNNING)	
	mode_oldx.att_pid_tune=1;
  #else		
	mode_oldx.att_pid_tune=KEY[6]&&KEY[5]&&KEY[3]&&KEY[2]&&KEY[1]&&KEY[0];
	#endif
	mode_check(CH_filter,mode_value);
//------------------------磁力计 超声波 采集
	static u8 hml_cnt;	
	#if !USE_MINI_FC_FLOW_BOARD
	#if SONAR_USE_FC||SONAR_USE_FC1
	static u16 cnt_sonar_idle;
	#if !USE_ZIN_BMP	
	if(!Thr_Low||NS==0)
	Ultra_Duty();	
	else if(cnt_sonar_idle++>2/0.05){cnt_sonar_idle=0;
	Ultra_Duty();}
	#endif
	#endif
	#endif
	
//-------------------------超时判断-----------------------------------	
	if(imu_loss_cnt++>1500/50){NAV_BOARD_CONNECT=0;}
		
	if(robot.loss_cnt++>4/0.05)
	robot.connect=0;
	if(circle.lose_cnt++>4/0.05)
	circle.connect=0;
	if(c2c.lose_cnt++>4/0.05)
	c2c.check=c2c.connect=0;
	if(marker.lose_cnt++>4/0.05)
	marker.connect=0;
	if(px4.loss_cnt++>4/0.05)
	px4.connect=0;
	if(vslam.lose_cnt++>4/0.05)
	vslam.connect=0;
	circle.use_spd=circle.connect&&mode_oldx.flow_sel;
	if(Rc_Get_SBUS.lose_cnt_rx++>5/0.05)
		Rc_Get_SBUS.update=0;
	static u8 led_cnt;
	if(led_cnt++>0.68/0.05){led_cnt=0;
	if(NAV_BOARD_CONNECT)
	LEDRGB();
	else{
	module.sonar=module.laser=module.gps=module.flow=0;	
	#if USE_MINI_BOARD
	GPIO_ResetBits(GPIOC,GPIO_Pin_1);
	#else
	GPIO_ResetBits(GPIOD,GPIO_Pin_12);
	#endif	
	}
	}
	#if USE_VER_3
	 Bat_protect(0.05);
	#endif
	
	if(time_fly.temp_cnt++>1/0.05)
	{
	 time_fly.temp_cnt=0;
		
	 switch(time_fly.state)	
	 {
		 case 0:
			   if(fly_ready&&RX_CH_PWM[THRr]>1100&&Rc_Get_PWM.update){
					  time_fly.cnt[time_fly.sel]=1;
  					time_fly.state=1; 
				 }else if(time_fly.rst)
				 {
				   time_fly.rst=0;
					 for(i=0;i<10;i++)
					   time_fly.cnt[i]=0;
					 time_fly.sel=time_fly.longe=time_fly.shorte=0;
					 //WRITE_PARM();
				 }				 
		 break;
	   case 1:
			   time_fly.cnt[time_fly.sel]+=1;
			   if(!fly_ready){
				    time_fly.state=1; 
					 
					 if(time_fly.cnt[time_fly.sel]>1.5*60)
					 {
					 time_fly.sel++;
						 
						for(i=0;i<10;i++){
               if(time_fly.cnt[i]>time_fly.longe)						 
						    time_fly.longe=time_fly.cnt[i];
							 if(time_fly.cnt[i]<time_fly.shorte&&time_fly.cnt[i]!=0)						 
						    time_fly.shorte=time_fly.cnt[i];
						 }
						//WRITE_PARM();
					 }
					 else
					 time_fly.cnt[time_fly.sel]=0;	 
					 if(time_fly.sel>9)
						 time_fly.sel=0;
					 time_fly.state=0;
				 } 
		 break;
	 
	 
	 }
	}
	
}


void Duty_Loop()   					//最短任务周期为1ms，总的代码执行时间需要小于1ms。
{

	if( loop.check_flag == 1 )
	{
		loop_cnt = time_1ms;
		
		Duty_1ms();							//周期1ms的任务
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//周期2ms的任务
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//周期5ms的任务
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		loop.check_flag = 0;		//循环运行完毕标志
	}
}