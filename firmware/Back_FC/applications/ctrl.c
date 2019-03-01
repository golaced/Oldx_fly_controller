

#include "flash.h"
#include "ctrl.h"
#include "height_ctrl.h"
#include "fly_mode.h"
#include "quar.h"
#include "eso.h"
ctrl_t ctrl_1;
ctrl_t ctrl_2;
FAN fan;
void Ctrl_Para_Init()		//设置默认参数
{
//====================================
	ctrl_1.PID[PIDROLL].kdamp  = 1;
	ctrl_1.PID[PIDPITCH].kdamp = 1;
	ctrl_1.PID[PIDYAW].kdamp 	 = 1;
	
	ctrl_1.FB = 0.20;   //外  0<fb<1
}

xyz_f_t except_A = {0,0,0};

xyz_f_t ctrl_angle_offset = {0,0,0};

xyz_f_t compensation;

void CTRL_2(float T)
{static u16 init_yaw;
 int RC_DEAD=30;
	if(mode_oldx.flow_hold_position==0)
		RC_DEAD=0;
//=========================== 期望角度 ========================================
	#if defined(INDOOR_FPV_SERCH)
	except_A.x  = -MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[ROL]) ,0,RC_DEAD )/500.0f );   //30
	except_A.y  = -MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[PIT]) ,0,RC_DEAD )/500.0f );  //30
	#else
	#if USE_RC_INVERT
	except_A.x  = -MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[ROL]) ,0,RC_DEAD )/500.0f );   //30
	except_A.y  = -MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[PIT]) ,0,RC_DEAD )/500.0f );  //30
	#else
	except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[ROL]) ,0,RC_DEAD )/500.0f );   //30
	except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[PIT]) ,0,RC_DEAD )/500.0f );  //30
	#endif
	#endif
	if( !Thr_Low )
	{
    if(smart.rc.ATT_MODE==SMART_MODE_POS&&smart.att.z!=0)
			except_A.z =  smart.att.z ;
		#if PX4_SDK
		else if(px4.connect&&my_deathzoom_2( (CH_filter[YAW]) ,0,20 )!=0)
		except_A.z=Yaw_fc;  
		#endif
		else
		except_A.z += (s16)( MAX_CTRL_YAW_SPEED_RC *( my_deathzoom_2( (CH_filter[YAW]) ,0,20 )/500.0f ) ) *T ;  //50
	}
	else
	{
		#if EN_ATT_CAL_FC
		except_A.z += 1 *3.14 *T *( Yaw_fc - except_A.z );
		#else
		except_A.z += 1 *3.14 *T *( Yaw - except_A.z );
		#endif
	}
	if(init_yaw++>500){init_yaw=502;
	except_A.z = To_180_degrees(except_A.z);
	}else
	except_A.z = Yaw_fc;
	if(mode_oldx.flow_hold_position>0&&except_A.y==0&&except_A.x==0)
	{
			if((module.gps||module.flow||module.pi||module.pi_flow)==1){
				except_A.y=LIMIT(nav[PITr],-MAX_CTRL_ANGLE,MAX_CTRL_ANGLE);
				except_A.x=LIMIT(nav[ROLr],-MAX_CTRL_ANGLE,MAX_CTRL_ANGLE);
			}
	}
  if(smart.rc.ATT_MODE==SMART_MODE_SPD&&smart.att_rate.z!=0)
	{except_A.z  +=  smart.att_rate.z*T; except_A.z = To_180_degrees(except_A.z);}
	static u8 yaw_trig;
	if(mode_oldx.att_pid_tune)
	{
	{	
	if(KEY_SEL[0])//TRIG for tuning
	#if !TUNING_Z	
		#if TUNING_X	
		except_A.x=-15;
		#else
		except_A.y=15;
		#endif
	#else
	{if(yaw_trig==0){except_A.z=Yaw_fc+15;yaw_trig=1;}}
	#endif
	else
	#if !TUNING_Z		
		#if TUNING_X	
		except_A.x=LIMIT(except_A.x,-15,15);	
		#else
		except_A.y=LIMIT(except_A.y,-15,15);	
		#endif
	#else
	 { except_A.x=except_A.y=yaw_trig=0;}
	#endif
  }

  if(mode_oldx.use_px4_err){
	cal_ero_outter_so3(); 
  ctrl_2.err.x =  my_deathzoom_21(ero_angle_px4[0],0.0);
	ctrl_2.err.y =  my_deathzoom_21(ero_angle_px4[1],0.0);
	#if EN_ATT_CAL_FC
	ctrl_2.err.z =  my_deathzoom_21(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw_fc	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#else	
	ctrl_2.err.z =  my_deathzoom_21(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#endif
	}else{	
	ctrl_2.err.x =  my_deathzoom_21(To_180_degrees( ctrl_angle_offset.x + except_A.x - Rol_fc  ),0.1);
	ctrl_2.err.y =  my_deathzoom_21(To_180_degrees( ctrl_angle_offset.y + except_A.y - Pit_fc ),0.1);}
	#if EN_ATT_CAL_FC
	ctrl_2.err.z =  my_deathzoom_21(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw_fc	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#else	
	ctrl_2.err.z =  my_deathzoom_21(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#endif	
	}//normal 
	else{	//使用SO3下误差计算
	
	cal_ero_outter_so3(); 
  /* 得到角度误差 */
	if(mode_oldx.use_px4_err){
  ctrl_2.err.x =  my_deathzoom_21(ero_angle_px4[0],0.0);
	ctrl_2.err.y =  my_deathzoom_21(ero_angle_px4[1],0.0);
	#if EN_ATT_CAL_FC
	ctrl_2.err.z =  my_deathzoom_21(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw_fc	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#else	
	ctrl_2.err.z =  my_deathzoom1(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#endif
	}else{	
	ctrl_2.err.x =  my_deathzoom_21(To_180_degrees( ctrl_angle_offset.x + except_A.x - Rol_fc  ),0.1);
	ctrl_2.err.y =  my_deathzoom_21(To_180_degrees( ctrl_angle_offset.y - except_A.y - Pit_fc ),0.1);}
	#if EN_ATT_CAL_FC
	ctrl_2.err.z =  my_deathzoom_21(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw_fc	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#else	
	ctrl_2.err.z =  my_deathzoom_21(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.5);//*LIMIT(ero_angle_px4[3],0.5,1);
	#endif
	}

	/* 计算角度误差权重 */
	ctrl_2.err_weight.x = ABS(ctrl_2.err.x)/ANGLE_TO_MAX_AS;
	ctrl_2.err_weight.y = ABS(ctrl_2.err.y)/ANGLE_TO_MAX_AS;
	ctrl_2.err_weight.z = ABS(ctrl_2.err.z)/ANGLE_TO_MAX_AS;
//	/* 角度误差微分（跟随误差曲线变化）*/
	ctrl_2.err_d.x = 10 *ctrl_2.PID[PIDROLL].kd  *(ctrl_2.err.x - ctrl_2.err_old.x) *( 0.005f/T ) ;
	ctrl_2.err_d.y = 10 *ctrl_2.PID[PIDPITCH].kd *(ctrl_2.err.y - ctrl_2.err_old.y) *( 0.005f/T ) ;
	ctrl_2.err_d.z = 10 *ctrl_2.PID[PIDYAW].kd 	 *(ctrl_2.err.z - ctrl_2.err_old.z) *( 0.005f/T ) ;
//	/* 角度误差积分 */
//	ctrl_2.err_i.x += ctrl_2.PID[PIDROLL].ki  *ctrl_2.err.x *T;
//	ctrl_2.err_i.y += ctrl_2.PID[PIDPITCH].ki *ctrl_2.err.y *T;
 	ctrl_2.err_i.z += ctrl_2.PID[PIDYAW].ki 	*ctrl_2.err.z *T;
//	/* 角度误差积分分离 */
//	ctrl_2.eliminate_I.x = Thr_Weight *CTRL_2_INT_LIMIT;
//	ctrl_2.eliminate_I.y = Thr_Weight *CTRL_2_INT_LIMIT;
	ctrl_2.eliminate_I.z = Thr_Weight *CTRL_2_INT_LIMIT;
//	/* 角度误差积分限幅 */
//	ctrl_2.err_i.x = LIMIT( ctrl_2.err_i.x, -ctrl_2.eliminate_I.x,ctrl_2.eliminate_I.x );
//	ctrl_2.err_i.y = LIMIT( ctrl_2.err_i.y, -ctrl_2.eliminate_I.y,ctrl_2.eliminate_I.y );
	ctrl_2.err_i.z = LIMIT( ctrl_2.err_i.z, -ctrl_2.eliminate_I.z,ctrl_2.eliminate_I.z );
//	/* 记录历史数据 */	
	ctrl_2.err_old.x = ctrl_2.err.x;
	ctrl_2.err_old.y = ctrl_2.err.y;
	ctrl_2.err_old.z = ctrl_2.err.z;	
	
	/* 对用于计算比例项输出的角度误差限幅 */
	ctrl_2.err.x = LIMIT( ctrl_2.err.x, -90, 90 );
	ctrl_2.err.y = LIMIT( ctrl_2.err.y, -90, 90 );
	if(ctrl_1.zrate_limit!=0)
	 ctrl_2.err.z = LIMIT( ctrl_2.err.z, -ctrl_1.zrate_limit,ctrl_1.zrate_limit );
	else
	 ctrl_2.err.z = LIMIT( ctrl_2.err.z, -90, 90 );
	/* 角度PID输出 */
	ctrl_2.out.x = ctrl_2.PID[PIDROLL].kp  *( ctrl_2.err.x + ctrl_2.err_d.x + ctrl_2.err_i.x );	//rol
	ctrl_2.out.y = ctrl_2.PID[PIDPITCH].kp *( ctrl_2.err.y + ctrl_2.err_d.y + ctrl_2.err_i.y );  //pit
	
//	if(smart.rc.ATT_MODE==SMART_MODE_SPD)
//		ctrl_2.out.z =  smart.att_rate.z ;  
//	else 
		ctrl_2.out.z = ctrl_2.PID[PIDYAW].kp   *( ctrl_2.err.z + ctrl_2.err_d.z + ctrl_2.err_i.z );
}

xyz_f_t except_AS;
int inner_set=0;
u8 en_h_inf=0;
float k_rc_gyro_spd=0.4;
float g_old[ITEMS];

void CTRL_1(float T)  //x roll,y pitch,z yaw
{
 float ctrl_angle_out[3]={0},ctrl_angle_weight[3]={0};
	xyz_f_t EXP_LPF_TMP;
	#if TUNING_ONE_AXIX
	if(ctrl_2.PID[PIDROLL].kp==0||inner_set){//速度环调参给定
	#if !TUNING_Z	
		#if TUNING_X
		if(fabs(Rol_fc)<40)
		ctrl_angle_out[0]=except_A.x*k_rc_gyro_spd;
		else if((Rol_fc)>40)
		ctrl_angle_out[0]=LIMIT(except_A.x*k_rc_gyro_spd,-MAX_CTRL_ANGLE*k_rc_gyro_spd,0);	
		else if((Rol_fc)<-40)
		ctrl_angle_out[0]=LIMIT(except_A.x*k_rc_gyro_spd,0,MAX_CTRL_ANGLE*k_rc_gyro_spd);	
		#else
		if(fabs(Pit_fc)<40)
		ctrl_angle_out[1]=except_A.y*k_rc_gyro_spd;
		else if((Pit_fc)>40)
		ctrl_angle_out[1]=LIMIT(except_A.y*k_rc_gyro_spd,-MAX_CTRL_ANGLE*k_rc_gyro_spd,0);	
		else if((Pit_fc)<-40)
		ctrl_angle_out[1]=LIMIT(except_A.y*k_rc_gyro_spd,0,MAX_CTRL_ANGLE*k_rc_gyro_spd);	
		#endif
	#endif
	}else{
	ctrl_angle_out[0]=ctrl_2.out.x;
	ctrl_angle_out[1]=ctrl_2.out.y;
	}
	#else
	if(ctrl_2.PID[PIDROLL].kp==0||inner_set){//速度环调参给定
		if(fabs(Rol_fc)<40)
		ctrl_angle_out[0]=except_A.x*k_rc_gyro_spd;
		else if((Rol_fc)>40)
		ctrl_angle_out[0]=LIMIT(except_A.x*k_rc_gyro_spd,-MAX_CTRL_ANGLE*k_rc_gyro_spd,0);	
		else if((Rol_fc)<-40)
		ctrl_angle_out[0]=LIMIT(except_A.x*k_rc_gyro_spd,0,MAX_CTRL_ANGLE*k_rc_gyro_spd);	
		if(fabs(Pit_fc)<40)
		ctrl_angle_out[1]=except_A.y*k_rc_gyro_spd;
		else if((Pit_fc)>40)
		ctrl_angle_out[1]=LIMIT(except_A.y*k_rc_gyro_spd,-MAX_CTRL_ANGLE*k_rc_gyro_spd,0);	
		else if((Pit_fc)<-40)
		ctrl_angle_out[1]=LIMIT(except_A.y*k_rc_gyro_spd,0,MAX_CTRL_ANGLE*k_rc_gyro_spd);	
		}
		else{
		 ctrl_angle_out[0]=ctrl_2.out.x;
		 ctrl_angle_out[1]=ctrl_2.out.y;
		}
	#endif
	if(ctrl_2.PID[PIDYAW].kp==0)
		ctrl_angle_out[2]=LIMIT((s16)( MAX_CTRL_YAW_SPEED *k_rc_gyro_spd *( my_deathzoom_2( (CH_filter[YAW]) ,0,20 )/500.0f ) )
		,-MAX_CTRL_YAW_SPEED,MAX_CTRL_YAW_SPEED);	
	else
		ctrl_angle_out[2]=ctrl_2.out.z;
	/* 给期望（目标）角速度 */
	if(force_Thr_low||Thr_Low||!fly_ready)
	{
	ctrl_1.err_i.x =ctrl_1.err_i.y =ctrl_1.err_i.z =0;
	ctrl_2.err_i.x =ctrl_2.err_i.y =ctrl_2.err_i.z =0;
	}
	EXP_LPF_TMP.x = MAX_CTRL_ASPEED *(ctrl_angle_out[0]/ANGLE_TO_MAX_AS);
	EXP_LPF_TMP.y = MAX_CTRL_ASPEED *(ctrl_angle_out[1]/ANGLE_TO_MAX_AS);
	EXP_LPF_TMP.z = MAX_CTRL_YAW_SPEED *(ctrl_angle_out[2]/ANGLE_TO_MAX_AS);
	
	except_AS.x = EXP_LPF_TMP.x;
	except_AS.y = EXP_LPF_TMP.y;
	except_AS.z = EXP_LPF_TMP.z;
	/* 期望角速度限幅 */
	except_AS.x = LIMIT(except_AS.x, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
	except_AS.y = LIMIT(except_AS.y, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
	except_AS.z = LIMIT(except_AS.z, -MAX_CTRL_YAW_SPEED,MAX_CTRL_YAW_SPEED );

	/* 角速度直接微分（角加速度），负反馈可形成角速度的阻尼（阻碍角速度的变化）*/
	ctrl_1.damp.x = ( mpu6050_fc.Gyro_deg.x - g_old[A_X]) *( 0.002f/T );
	ctrl_1.damp.y = (-mpu6050_fc.Gyro_deg.y - g_old[A_Y]) *( 0.002f/T );
	ctrl_1.damp.z = (-mpu6050_fc.Gyro_deg.z - g_old[A_Z]) *( 0.002f/T );
	/* 角速度误差 */
	ctrl_1.err.x =  ( except_AS.x - mpu6050_fc.Gyro_deg.x ) *(300.0f/MAX_CTRL_ASPEED);
	ctrl_1.err.y =  ( except_AS.y + mpu6050_fc.Gyro_deg.y ) *(300.0f/MAX_CTRL_ASPEED);  //-y
	ctrl_1.err.z =  ( except_AS.z + mpu6050_fc.Gyro_deg.z ) *(300.0f/MAX_CTRL_ASPEED);	 //-z
	
	/* 角速度误差权重 */
	ctrl_1.err_weight.x = ABS(ctrl_1.err.x)/MAX_CTRL_ASPEED;
	ctrl_1.err_weight.y = ABS(ctrl_1.err.y)/MAX_CTRL_ASPEED;
	ctrl_1.err_weight.z = ABS(ctrl_1.err.z)/MAX_CTRL_YAW_SPEED;
	/* 角速度微分 */
	ctrl_1.err_d.x = ( ctrl_1.PID[PIDROLL].kd  *( -10 *ctrl_1.damp.x) *( 0.002f/T ) );
	ctrl_1.err_d.y = ( ctrl_1.PID[PIDPITCH].kd *( -10 *ctrl_1.damp.y) *( 0.002f/T ) );
	ctrl_1.err_d.z = ( ctrl_1.PID[PIDYAW].kd   *( -10 *ctrl_1.damp.z) *( 0.002f/T ) );
	//自抗扰
	if(eso_att_inner_c[PITr].b0!=0){
    OLDX_ATT_CONTRL_INNER_ESO(&eso_att_inner_c[PITr],except_AS.y,-mpu6050_fc.Gyro_deg.y,eso_att_inner_c[PITr].u,T,400,ctrl_1.PID[PIDPITCH].kp*k_sensitivity[0],thr_view);
	  OLDX_ATT_CONTRL_INNER_ESO(&eso_att_inner_c[ROLr],except_AS.x,mpu6050_fc.Gyro_deg.x,eso_att_inner_c[ROLr].u,T,400,ctrl_1.PID[PIDROLL].kp*k_sensitivity[0],thr_view);
  }
	if(eso_att_inner_c[YAWr].b0!=0)
		OLDX_ATT_CONTRL_INNER_ESO(&eso_att_inner_c[YAWr],except_AS.z,-mpu6050_fc.Gyro_deg.z,eso_att_inner_c[YAWr].u,T,400,ctrl_1.PID[PIDYAW].kp*k_sensitivity[1],thr_view);

if(eso_att_inner_c[YAWr].b0==0)	{
	ctrl_1.err_i.z += ctrl_1.PID[PIDYAW].ki 	*(ctrl_1.err.z - ctrl_1.damp.z) *T;
  ctrl_1.eliminate_I.z = Thr_Weight *CTRL_1_INT_LIMIT ;	
	ctrl_1.err_i.z = LIMIT( ctrl_1.err_i.z, -ctrl_1.eliminate_I.z,ctrl_1.eliminate_I.z );
}

if(eso_att_inner_c[PITr].b0==0){
	/* 角速度误差积分 */
	ctrl_1.err_i.x += ctrl_1.PID[PIDROLL].ki  *(ctrl_1.err.x - ctrl_1.damp.x) *T;
	ctrl_1.err_i.y += ctrl_1.PID[PIDPITCH].ki *(ctrl_1.err.y - ctrl_1.damp.y) *T;
	
	/* 角速度误差积分分离 */
	ctrl_1.eliminate_I.x = Thr_Weight *CTRL_1_INT_LIMIT ;
	ctrl_1.eliminate_I.y = Thr_Weight *CTRL_1_INT_LIMIT ;
	
	/* 角速度误差积分限幅 */
	ctrl_1.err_i.x = LIMIT( ctrl_1.err_i.x, -ctrl_1.eliminate_I.x,ctrl_1.eliminate_I.x );
	ctrl_1.err_i.y = LIMIT( ctrl_1.err_i.y, -ctrl_1.eliminate_I.y,ctrl_1.eliminate_I.y );
}


	if(eso_att_inner_c[PITr].b0!=0){//ADRC
	ctrl_1.err_i.x=ctrl_1.err_i.y=0;	
	ctrl_1.out.x = 2 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.x),0,1)*except_AS.x + 
		( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDROLL].kp  *( ctrl_1.err_d.x*k_sensitivity[0] +eso_att_inner_c[ROLr].u ) );
	ctrl_1.out.y = 2 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.y),0,1)*except_AS.y + 
		( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDPITCH].kp *( ctrl_1.err_d.y*k_sensitivity[0] +eso_att_inner_c[PITr].u ) );
	}else{	/* 角速度PID输出 */
	ctrl_1.out.x = 2 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.x),0,1)*except_AS.x + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDROLL].kp*k_sensitivity[0]  *( ctrl_1.err.x + ctrl_1.err_d.x + ctrl_1.err_i.x ) );
	ctrl_1.out.y = 2 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.y),0,1)*except_AS.y + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDPITCH].kp*k_sensitivity[0] *( ctrl_1.err.y + ctrl_1.err_d.y + ctrl_1.err_i.y ) );
	}	
  if(eso_att_inner_c[YAWr].b0==0)	
		ctrl_1.out.z = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.z),0,1)*except_AS.z 
	+ ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDYAW].kp*k_sensitivity[1]   *( ctrl_1.err.z + ctrl_1.err_d.z + ctrl_1.err_i.z  ) );
  else
	  ctrl_1.out.z = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.z),0,1)*except_AS.z 
	+ ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDYAW].kp*( eso_att_inner_c[YAWr].u + ctrl_1.err_d.z*k_sensitivity[1] ) );
	Thr_Ctrl(T);// 高度控制
	
	if(k_sensitivity[1]==0)
		ctrl_1.out.z=0;
	if(mode_oldx.att_pid_tune)
	{	
	#if !TUNING_Z		
		#if TUNING_X	
		ctrl_1.out.y=ctrl_1.out.z=0;	
		#else
		ctrl_1.out.x=ctrl_1.out.z=0;		
		#endif
  #endif		
	}
	#if YAW_INVER
	All_Out(ctrl_1.out.x,ctrl_1.out.y,-ctrl_1.out.z,T);	
	#else
	All_Out(ctrl_1.out.x,ctrl_1.out.y,ctrl_1.out.z,T);
  #endif

	ctrl_1.err_old.x = ctrl_1.err.x;
	ctrl_1.err_old.y = ctrl_1.err.y;
	ctrl_1.err_old.z = ctrl_1.err.z;

	g_old[A_X] =  mpu6050_fc.Gyro_deg.x ;
	g_old[A_Y] = -mpu6050_fc.Gyro_deg.y ;
	g_old[A_Z] = -mpu6050_fc.Gyro_deg.z ;
}

float thr_view;
float thr_value;
u8 Thr_Low;
float Thr_Weight;
u8 force_Thr_low;
u16 cnt_for_low=0;
void Thr_Ctrl(float T)
{ static u8 fly_ready_r;
	static float thr;
	static float Thr_tmp;
	thr = 500 + CH_filter[THR]; //油门值 0 ~ 1000
	
	if(!fly_ready&&500 + CH_filter[THRr]<200)
	force_Thr_low=0;
	#if USE_OLDX_REMOTE
	if(((fabs(ctrl_2.err.x)>1.68*MAX_CTRL_ANGLE||fabs(ctrl_2.err.y)>1.68*MAX_CTRL_ANGLE||fabs(Pit_fc)>70||fabs(Rol_fc)>70)&&
	fly_ready&&mode_oldx.att_pid_tune==0&&height_ctrl_mode==0
	  )||(NS==0&&fly_ready&&!mode_oldx.rc_loss_return_home))
	#else
	if(((fabs(ctrl_2.err.x)>1.68*MAX_CTRL_ANGLE||fabs(ctrl_2.err.y)>1.68*MAX_CTRL_ANGLE||fabs(Pit_fc)>70||fabs(Rol_fc)>70)&&
    fly_ready&&mode_oldx.att_pid_tune==0&&height_ctrl_mode==0
	  )||(fly_ready&&!Rc_Get_PWM.update&&mode_oldx.rc_loss_return_home==0))
	#endif
		cnt_for_low++;
	else
		cnt_for_low=0;
	
	u8 init_flag;
	init_flag=sys_init.baro_ekf;
	if(cnt_for_low>2.5/T)
		force_Thr_low=1;
//protect flag init	
//	if(fly_ready_r==0&&fly_ready==1&&CH_filter[THRr]>0)
//		force_Thr_low=1;
		fly_ready_r=fly_ready;
	
	if(px4.connect)
	  force_Thr_low=0;
	
	if(mode_oldx.use_dji)
		force_Thr_low=1;
	if(force_Thr_low)
		thr=0;
	
	Thr_tmp += 10 *3.14f *T *(thr_value/400.0f - Thr_tmp); //低通滤波
	Thr_Weight = LIMIT(Thr_tmp,0,1);    							//后边多处分离数据会用到这个值
	
	if( thr < 100 )
	{
		Thr_Low = 1;
	}
	else
	{
		Thr_Low = 0;
	}
	
/////////////////////////////////////////////////////////////////

	if(height_ctrl_mode&&mode_oldx.rc_loss_return_home==0)
	{
		if(NS==0) //丢失信号
		{
			thr = LIMIT(thr,0,500);
		}
		
	}
	else
	{
		if(NS==0&&mode_oldx.rc_loss_return_home==0) //丢失信号
		{
			thr = LIMIT(thr,0,350);
		}
		
	}
	  thr_view=thr;
	  Height_Ctrl1(T,thr);   //高度控制
	  thr_value =height_ctrl_out;
////////////////////////////////////////////////////////////////
	
	thr_value = LIMIT(thr_value,0,10 *MAX_THR *MAX_PWM/100);
}


float motor[MAXMOTORS];
float posture_value[MAXMOTORS];
s16 motor_out[MAXMOTORS];
s16 motor_out_test1[MAXMOTORS];
float view_out[3];
float k_pitch=PIT_GAIN_INIT;
void All_Out(float out_roll,float out_pitch,float out_yaw,float T)
{

	u8 i;
	float posture_value[MAXMOTORS];

	out_yaw = LIMIT( out_yaw , -5*MAX_THR ,5*MAX_THR ); //50%
	
#if (MAXMOTORS == 4)	
	
	posture_value[0] = - out_roll + out_pitch + out_yaw ;
	posture_value[1] = + out_roll + out_pitch - out_yaw ;
	posture_value[2] = + out_roll - out_pitch + out_yaw ;
	posture_value[3] = - out_roll - out_pitch - out_yaw ;
	
#elif (MAXMOTORS == 6)
	//0.866 == sqrt(3)/2    4/6 == 0.667f
	#if defined(HANX6)||defined(HANX6_BIG)||defined(HANX6_BIGX6)
	posture_value[0] = - 1 *out_roll*0.5 + out_pitch* k_pitch + 0.5 *out_yaw ;
	posture_value[1] = + 1 *out_roll*0.5 + out_pitch* k_pitch - 0.5 *out_yaw ;
	posture_value[2] = + 1 *out_roll*0.5             + 1  *out_yaw ;
	posture_value[3] = + 1 *out_roll*0.5 - out_pitch* k_pitch - 0.5 *out_yaw ;
	posture_value[4] = - 1 *out_roll*0.5 - out_pitch* k_pitch + 0.5 *out_yaw ;
	posture_value[5] = - 1 *out_roll*0.5             - 1  *out_yaw ;
	#else
	posture_value[0] = - 0.866f *out_roll + out_pitch + 0.667f *out_yaw ;
	posture_value[1] = + 0.866f *out_roll + out_pitch - 0.667f *out_yaw ;
	posture_value[2] = + 0.866f *out_roll             + 0.667f *out_yaw ;
	posture_value[3] = + 0.866f *out_roll - out_pitch - 0.667f *out_yaw ;
	posture_value[4] = - 0.866f *out_roll - out_pitch + 0.667f *out_yaw ;
	posture_value[5] = - 0.866f *out_roll             - 0.667f *out_yaw ;
	#endif
#elif (MAXMOTORS == 8)
	posture_value[0] = - 0.5f *out_roll + 0.5f *out_pitch + 0.5f *out_yaw ;
	posture_value[1] = + 0.5f *out_roll + 0.5f *out_pitch - 0.5f *out_yaw ;
	posture_value[2] = + 0.5f *out_roll + 0.5f *out_pitch + 0.5f *out_yaw ;
	posture_value[3] = + 0.5f *out_roll - 0.5f *out_pitch - 0.5f *out_yaw ;
	posture_value[4] = + 0.5f *out_roll - 0.5f *out_pitch + 0.5f *out_yaw ;
	posture_value[5] = - 0.5f *out_roll - 0.5f *out_pitch - 0.5f *out_yaw ;
	posture_value[6] = - 0.5f *out_roll - 0.5f *out_pitch + 0.5f *out_yaw ;
	posture_value[7] = - 0.5f *out_roll + 0.5f *out_pitch - 0.5f *out_yaw ;	
#elif (MAXMOTORS == 12)
//	posture_value[0] = - 1 *out_roll*0.5 + out_pitch + out_yaw ;//UP 1
//  posture_value[1] = - 1 *out_roll*0.5 + out_pitch - out_yaw ;//D 1
//	
//	posture_value[2] = + 1 *out_roll*0.5 + out_pitch + out_yaw ;//2
//	posture_value[3] = + 1 *out_roll*0.5 + out_pitch - out_yaw ;
//	
//	posture_value[4] = + 1 *out_roll*0.5             + out_yaw ;//3
//	posture_value[5] = + 1 *out_roll*0.5             - out_yaw ;
//	
//	posture_value[6] = + 1 *out_roll*0.5 - out_pitch + out_yaw ;//4
//	posture_value[7] = + 1 *out_roll*0.5 - out_pitch - out_yaw ;
//	
//	posture_value[8] = - 1 *out_roll*0.5 - out_pitch + out_yaw ;//5
//	posture_value[9] = - 1 *out_roll*0.5 - out_pitch - out_yaw ;
//	
//	posture_value[10] = - 1 *out_roll*0.5            + out_yaw ;//6
//	posture_value[11] = - 1 *out_roll*0.5            - out_yaw ;
	
	posture_value[0] = - 1 *out_roll*0.5 + out_pitch + 0.5 *out_yaw ;//UP 1
  posture_value[1] = - 1 *out_roll*0.5 + out_pitch - 0.5 *out_yaw ;//D 1
	
	posture_value[2] = + 1 *out_roll*0.5 + out_pitch - 0.5 *out_yaw ;//2
	posture_value[3] = + 1 *out_roll*0.5 + out_pitch + 0.5 *out_yaw ;
	
	posture_value[4] = + 1 *out_roll*0.5             + 1  *out_yaw ;//3
	posture_value[5] = + 1 *out_roll*0.5             - 1  *out_yaw ;
	
	posture_value[6] = + 1 *out_roll*0.5 - out_pitch - 0.5 *out_yaw ;//4
	posture_value[7] = + 1 *out_roll*0.5 - out_pitch + 0.5 *out_yaw ;
	
	posture_value[8] = - 1 *out_roll*0.5 - out_pitch + 0.5 *out_yaw ;//5
	posture_value[9] = - 1 *out_roll*0.5 - out_pitch - 0.5 *out_yaw ;
	
	posture_value[10] = - 1 *out_roll*0.5             - 1  *out_yaw ;//6
	posture_value[11] = - 1 *out_roll*0.5             + 1  *out_yaw ;
	
#endif	
 	#if EN_ATT_CAL_FC
	view_out[0]=out_pitch;
	view_out[1]=out_roll;
	view_out[2]=out_yaw;
	int en[3]={1,1,1};
	motor[0] = + out_roll*en[0] + out_pitch*en[1] + out_yaw*en[2] ;
	motor[1] = + out_roll*en[0] - out_pitch*en[1] + out_yaw*en[2] ;
	motor[2] = - out_roll*en[0] - out_pitch*en[1] + out_yaw*en[2] ;
	motor[3] = - out_roll*en[0] + out_pitch*en[1] + out_yaw*en[2] ;	
	#endif
  
  static float tilted_fix;//补偿油门
	float temp;
	#if EN_ATT_CAL_FC
	temp=LIMIT((thr_value/cos(LIMIT(my_deathzoom_21(Pit_fc,5),-45,45)/57.3)/
									cos(LIMIT(my_deathzoom_21(Rol_fc,5),-45,45)/57.3)-thr_value),0,200);
	#else
  temp=LIMIT((thr_value/cos(LIMIT(my_deathzoom_21(Pitch,5),-45,45)/57.3)/
									cos(LIMIT(my_deathzoom_21(Roll,5),-45,45)/57.3)-thr_value),0,200);
  #endif
	tilted_fix += ( 1 / ( 1 + 1 / ( 1.2f *3.14f *T ) ) ) *(temp- tilted_fix) ;
	for(i=0;i<MAXMOTORS;i++)
	{
		#if USE_FAN_AS_BLDC
		motor[4] = thr_value+tilted_fix ;
		#else
		posture_value[i] = LIMIT(posture_value[i], -1000,1000 );
		
		motor[i] = thr_value+tilted_fix + LIMIT(1.2*Thr_Weight,0,1) *posture_value[i] ;
		#endif
		
	}
	
	/* 是否解锁 */
	if(fly_ready&&force_Thr_low==0)
	{
		if( !Thr_Low )  //油门拉起
		{
			for(i=0;i<MAXMOTORS;i++)
			{
				#if USE_FAN_AS_BLDC
				motor[4] = LIMIT(motor[4], (10 *READY_SPEED),(10*MAX_PWM) );
				#else
				motor[i] = LIMIT(motor[i], (10 *READY_SPEED),(10*MAX_PWM) );
				#endif
			}
		}
		else						//油门低
		{
			for(i=0;i<MAXMOTORS;i++)
			{
				#if USE_FAN_AS_BLDC
				motor[4] = LIMIT(motor[4], 0,(10*MAX_PWM) );
				#else
				motor[i] = LIMIT(motor[i], 0,(10*MAX_PWM) );
				#endif
			}
		}
	}
	else
	{
		for(i=0;i<MAXMOTORS;i++)
		{
			motor[i] = 0;
		}
	}
	static u8 state;
	static u16 cnt;
	switch(state)
	{
		case 0:
			  if(fly_ready)
				{state=1;cnt=0;}
		break;
	  case 1:
				for(i=0;i<MAXMOTORS;i++)
				motor[i] = LIMIT(motor[i], (10 *READY_SPEED),(10*MAX_PWM) );
		    if(!fly_ready)
					state=0;
				if(cnt++>1.68/LIMIT(T,0.001,1))
					state=2;
		break;
		case 2:
			  if(!fly_ready)
					state=0;
		break;
	}	
	/* xxx */
	for(i=0;i<MAXMOTORS;i++)
	{
		motor_out[i] = (s16)(motor[i]);
	}
	
	
	#if USE_FAN_AS_BLDC
  SetPwm_Fan(motor_out);	
	#else
	SetPwm(motor_out,0,1000); //1000
	//SetPwm(motor_out_test1,0,1000); //1000
	#endif
}


