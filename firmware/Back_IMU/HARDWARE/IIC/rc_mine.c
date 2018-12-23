
#include "rc.h"
#include "led_fc.h"
#include "ms5611.h"
#include "height_ctrl.h"
#include "hml5833l.h"
#include "alt_kf.h"
#include "eso.h"
#include "neuron_pid.h"
#include "circle.h"
#include "error.h"
#define RX_DR			6		//????
#define TX_DS			5
#define MAX_RT		4

vs16 QH,ZY,XZ;
u8 is_lock=1;
u8 EN_FIX_GPSF=0;
u8 EN_FIX_LOCKWF=0;
u8 EN_CONTROL_IMUF=0;
u8 EN_FIX_INSF=0;
u8 EN_FIX_HIGHF=0;
u8 tx_lock=1;
u8 EN_FIX_GPS=0;
u8 EN_FIX_LOCKW=0;
u8 EN_CONTROL_IMU=0;
u8 EN_FIX_INS=0;
u8 EN_FIX_HIGH=0;
u8 EN_TX_GX=1;
u8 EN_TX_AX=1;
u8 EN_TX_HM=1;
u8 EN_TX_YRP=1;
u8 EN_TX_GPS=1;
u8 EN_TX_HIGH=1;
u8 up_load_set=0;
u8 up_load_pid=0;
u8 key_rc[6]={1,1,1,1,1,1};
u16 Yaw_sb_rc=0;

u8 cnt_rst=0,delta_pitch=0,delta_roll=0,delta_yew=0;



//中值滤波
float GetMedianNum(float * bArray, u16 iFilterLen)
{  
    int i,j;// 循环变量  
    float bTemp;  
      
    // 用冒泡法对数组进行排序  
    for (j = 0; j < iFilterLen - 1; j ++)  
    {  
        for (i = 0; i < iFilterLen - j - 1; i ++)  
        {  
            if (bArray[i] > bArray[i + 1])  
            {  
                // 互换  
                bTemp = bArray[i];  
                bArray[i] = bArray[i + 1];  
                bArray[i + 1] = bTemp;  
            }  
        }  
    }  
      
    // 计算中值  
    if ((iFilterLen & 1) > 0)  
    {  
        // 数组有奇数个元素，返回中间一个元素  
        bTemp = bArray[(iFilterLen + 1) / 2];  
    }  
    else  
    {  
        // 数组有偶数个元素，返回中间两个元素平均值  
        bTemp = (bArray[iFilterLen / 2] + bArray[iFilterLen / 2 + 1]) / 2;  
    }  
  
    return bTemp;  
}  
u16 data_rate;
#define MID_RC_KEY 15
 u8 key_rc_reg[7][MID_RC_KEY];
#define MID_RC_GET 4
float RC_GET[4][MID_RC_GET];
float control_scale=1;
float ypr_sb[3];
void NRF_DataAnl(void)
{ 
int16_t RC_GET_TEMP[4];
float RC_GETR[4][MID_RC_GET];	
u8 temp_key[7];
u8 temp;
	u8 i=0,j,sum = 0;
	for( i=0;i<31;i++)
		sum += NRF24L01_RXDATA[i];
	if(!(sum==NRF24L01_RXDATA[31]))	
	{i=0;
		return;
		}	//??sum}
	if(!(NRF24L01_RXDATA[0]==0x8A))		
		{
			i=0;
		return;
			}	//??sum}
	
	if(NRF24L01_RXDATA[1]==0x8A)								//?????,=0x8a,?????
	{ Feed_Rc_Dog(2);//通信看门狗喂狗
		data_rate++;
		RC_GET_TEMP[0]= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];//Rc_Get.THROTTLE
		Rc_Get.THROTTLE= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];//Rc_Get.THROTTLE
		Rc_Get.YAW			= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		Rc_Get.PITCH		= (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		Rc_Get.ROLL 		= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		RC_GET_TEMP[1]= (vs16)((NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6])/3+1000;//RC_Data.YAW	
		RC_GET_TEMP[2]= (vs16)((NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8])/3+1000;//RC_Data.PITCH	
		RC_GET_TEMP[3]= (vs16)((NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10])/3+1000;//RC_Data.ROLL
		RX_CH[AUX1r]=Rc_Get.AUX1			= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];
		RX_CH[AUX4r]=Rc_Get.AUX4			= (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		RX_CH[AUX3r]=Rc_Get.AUX3			= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		RX_CH[AUX2r]=Rc_Get.AUX2			= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];
		Rc_Get.AUX5			= (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];
		//Yaw_sb_rc=(vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24];

		ctrl_angle_offset.x =(float)(Rc_Get.AUX1-500)/1000.*MAX_FIX_ANGLE*2;
		ctrl_angle_offset.y =(float)(Rc_Get.AUX2-500)/1000.*MAX_FIX_ANGLE*2;

		if(fabs(ctrl_angle_offset.x )<0.2)  {ctrl_angle_offset.x =0;}
	 	if(fabs(ctrl_angle_offset.y )<0.2)  {ctrl_angle_offset.y =0;}
		
	  RX_CH[THRr]=	Rc_Get.THROTTLE-RX_CH_FIX[THRr]	;
	  RX_CH[ROLr]=  Rc_Get.ROLL-RX_CH_FIX[ROLr]	;
	  RX_CH[PITr]=  Rc_Get.PITCH-RX_CH_FIX[PITr]	;
		KEY_SEL[0]=(NRF24L01_RXDATA[21])&0x01;
		KEY_SEL[1]=(NRF24L01_RXDATA[21]>>1)&0x01;
		KEY_SEL[2]=(NRF24L01_RXDATA[21]>>2)&0x01;
		KEY_SEL[3]=(NRF24L01_RXDATA[21]>>3)&0x01; 
	  KEY[0]=(NRF24L01_RXDATA[22])&0x01;
		KEY[1]=(NRF24L01_RXDATA[22]>>1)&0x01;
		KEY[2]=(NRF24L01_RXDATA[22]>>2)&0x01;
		KEY[3]=(NRF24L01_RXDATA[22]>>3)&0x01;
		KEY[4]=(NRF24L01_RXDATA[22]>>4)&0x01;
		KEY[5]=(NRF24L01_RXDATA[22]>>5)&0x01;
		KEY[6]=(NRF24L01_RXDATA[22]>>6)&0x01;
		KEY[7]=(NRF24L01_RXDATA[22]>>7)&0x01;
		
		ypr_sb[0]=(int)(((NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24]))/100.;
		ypr_sb[1]=(int)(((NRF24L01_RXDATA[25]<<8)|NRF24L01_RXDATA[26]))/100.;
		ypr_sb[2]=(int)(((NRF24L01_RXDATA[27]<<8)|NRF24L01_RXDATA[28]))/100.;		
		for(j=0;j<3;j++)
		  if(ypr_sb[j]>360)
				ypr_sb[j]-=655.35;
		if(mode.yaw_imu_control)	
		RX_CH[YAWr]=  Rc_Get.YAW-RX_CH_FIX[YAWr]	;
		else{	
		if(fabs( ypr_sb[2])>32&&fabs(ypr_sb[1])<15)	
		RX_CH[YAWr]=  limit_mine(ypr_sb[2]*10,500)	+1500;
		else
		RX_CH[YAWr]=1500;	
	}
		if(mode.dj_lock){
		if(fabs( ypr_sb[1]-17)>40&&fabs(ypr_sb[2])<15)	
		{dj_angle_set+= (ypr_sb[1]-17)*0.008;
		dj_angle_set=LIMIT(dj_angle_set,-12,12);//SCALE_DJ*30,SCALE_DJ*30);
		}
	}
		else
		dj_angle_set=0;	
//-------------------------------------------------------------------------------------------------------------------------		
	 }
	else if(NRF24L01_RXDATA[1]==0x8B)								//?????,=0x8a,?????
	{
			tx_lock=(NRF24L01_RXDATA[3]);
			EN_FIX_GPS=(NRF24L01_RXDATA[4]);
			EN_FIX_LOCKW=(NRF24L01_RXDATA[5]);
			EN_CONTROL_IMU=(NRF24L01_RXDATA[6]);
			EN_FIX_INS=(NRF24L01_RXDATA[7]);
			EN_FIX_HIGH=(NRF24L01_RXDATA[8]);
			EN_TX_GX=(NRF24L01_RXDATA[9]);
			EN_TX_AX=(NRF24L01_RXDATA[10]);
			EN_TX_HM=(NRF24L01_RXDATA[11]);
			EN_TX_YRP=(NRF24L01_RXDATA[12]);
			EN_TX_GPS=(NRF24L01_RXDATA[13]);
			EN_TX_HIGH=(NRF24L01_RXDATA[14]);
			(up_load_set)=(NRF24L01_RXDATA[15]);
			(up_load_pid)=(NRF24L01_RXDATA[16]);
		
	EN_FIX_GPSF=EN_FIX_GPS;
	EN_FIX_LOCKWF=EN_FIX_GPS;
	EN_CONTROL_IMUF=EN_FIX_GPS;
	EN_FIX_INSF=EN_FIX_GPS;
	EN_FIX_HIGHF=EN_FIX_GPS;	
	}
	else 		if(NRF24L01_RXDATA[1]==0x8C)	//??PID1
		{
		  if(mode.en_pid_sb_set){
			SPID.OP = (float)((vs16)(NRF24L01_RXDATA[4]<<8)|NRF24L01_RXDATA[5]);
			SPID.OI = (float)((vs16)(NRF24L01_RXDATA[6]<<8)|NRF24L01_RXDATA[7]);
			SPID.OD = (float)((vs16)(NRF24L01_RXDATA[8]<<8)|NRF24L01_RXDATA[9]);
		  SPID.IP = (float)((vs16)(NRF24L01_RXDATA[10]<<8)|NRF24L01_RXDATA[11]);
			SPID.II = (float)((vs16)(NRF24L01_RXDATA[12]<<8)|NRF24L01_RXDATA[13]);
			SPID.ID = (float)((vs16)(NRF24L01_RXDATA[14]<<8)|NRF24L01_RXDATA[15]);
		  SPID.YP = (float)((vs16)(NRF24L01_RXDATA[16]<<8)|NRF24L01_RXDATA[17]);
			SPID.YI = (float)((vs16)(NRF24L01_RXDATA[18]<<8)|NRF24L01_RXDATA[19]);
			SPID.YD = (float)((vs16)(NRF24L01_RXDATA[20]<<8)|NRF24L01_RXDATA[21]);
				
		
				
	if(mode.att_pid_tune){
//-----------------------------PID--------------------------------

		ctrl_2.PID[PIDPITCH].kp =ctrl_2.PID[PIDROLL].kp  = 0.001*SPID.OP;
		ctrl_2.PID[PIDPITCH].ki =ctrl_2.PID[PIDROLL].ki  = 0.001*SPID.OI;
		ctrl_2.PID[PIDPITCH].kd =ctrl_2.PID[PIDROLL].kd  = 0.001*SPID.OD;
		ctrl_1.PID[PIDPITCH].kp =ctrl_1.PID[PIDROLL].kp  = 0.001*SPID.IP;
		ctrl_1.PID[PIDPITCH].ki =ctrl_1.PID[PIDROLL].ki  = 0.001*SPID.II;
		ctrl_1.PID[PIDPITCH].kd =ctrl_1.PID[PIDROLL].kd  = 0.001*SPID.ID;
		ctrl_2.PID[PIDYAW].kp 	= 0.001*SPID.YP;
		ctrl_2.PID[PIDYAW].ki 	= 0.001*SPID.YI;
		ctrl_2.PID[PIDYAW].kd 	= 0.001*SPID.YD;
		
		
		if(SPID.YP!=0)
		att_tuning_thr_limit=SPID.YP;
		else
		att_tuning_thr_limit=450;//360;	


//----------------------------------ESO-----------------------------		
//		if(SPID.YI!=0){
//		eso_att_outter[PITr].n=eso_att_outter[ROLr].n=eso_att_outter[YAWr].n=SPID.YI;
//		}
//		else{
//		eso_att_outter[PITr].n=eso_att_outter[ROLr].n=eso_att_outter[YAWr].n=1;
//		}
		
//		if(SPID.YI!=0){
//		eso_att_inner[PITr].n=eso_att_inner[ROLr].n=eso_att_inner[YAWr].n=SPID.YI;
//		}
//		else{
//		eso_att_inner[PITr].n=eso_att_inner[ROLr].n=eso_att_inner[YAWr].n=1;
//		}
	
	}

	  //------------7 6 5 4  |  3 2 1 0  KEY
    //height  11
		 else if(KEY[0]==1&&KEY[1]==1){
			 
			// if(!mode.height_safe){
		  ultra_pid.kp =  		0.001*(float)SPID.OP;
			ultra_pid.ki =  		0.001*(float)SPID.OI;
			ultra_pid.kd = 			0.001*(float)SPID.OD;
			wz_speed_pid.kp =   0.001*(float)SPID.IP; 
			wz_speed_pid.ki =   0.001*(float)SPID.II;
			wz_speed_pid.kd =   0.001*(float)SPID.ID;
			 
			eso_att_inner_c[THRr].b0= SPID.YD;
//		 }
//			 else
//			 {
//			ultra_pid_safe.kp =  		 0.001*(float)SPID.OP;
//			ultra_pid_safe.ki =  		 0.001*(float)SPID.OI;
//			ultra_pid_safe.kd = 		 0.001*(float)SPID.OD;
//			wz_speed_pid_safe.kp =   0.001*(float)SPID.IP; 
//			wz_speed_pid_safe.ki =   0.001*(float)SPID.II;
//			wz_speed_pid_safe.kd =   0.001*(float)SPID.ID;
//			 }
		}////nav    01
		else if(KEY[0]==1&&KEY[1]==0){
			pid.nav.in.p = 0.001*(float)SPID.IP;
			pid.nav.in.i = 0.001*(float)SPID.II;
			pid.nav.in.d = 0.001*(float)SPID.ID;
			pid.nav.out.p= 0.001*(float)SPID.OP;
			//if(SPID.OI!=0)
			//pid.nav.out.i=    (float)SPID.OI/100.;//dlf_nav
			//else
			//pid.nav.out.i=8;	
			//pid.avoid.out.p=  (float)SPID.OD/100.;
			
			pid.nav.out.dead= (float)SPID.YP/1000.;//位置死区mm
			pid.nav.in.dead=  (float)SPID.YI/1000.;//速度死区mm/s
			pid.nav.in.dead2= (float)SPID.YD/1000.;
			

		}//circle 10
		else if(KEY[0]==0&&KEY[1]==1){
			pid.circle.in.p = 0.001*(float)SPID.IP;
			pid.circle.in.i = 0.001*(float)SPID.II;
			pid.circle.in.d = 0.001*(float)SPID.ID;
			pid.circle.out.p= 0.001*(float)SPID.OP;	
//			pid.circle.out.dead= (float)SPID.YP/1000.;//位置死区mm
//			pid.circle.in.dead=  (float)SPID.YI/1000.;//速度死区mm/s
//			pid.circle.in.dead2= (float)SPID.YD/1000.;
			

		}
			}
		//	EE_SAVE_PID();
		}
	else 		if(NRF24L01_RXDATA[1]==0x8D)	//??PID2
		{
			HPID.OP = (float)((vs16)(NRF24L01_RXDATA[4]<<8)|NRF24L01_RXDATA[5]);
			HPID.OI = (float)((vs16)(NRF24L01_RXDATA[6]<<8)|NRF24L01_RXDATA[7]);
			HPID.OD = (float)((vs16)(NRF24L01_RXDATA[8]<<8)|NRF24L01_RXDATA[9]);
			
		//	EE_SAVE_PID();
		}


	

}


int cnt_timer2_r=0;
u8 cnt_led_rx=0;
void Nrf_Check_Event(void)
{ 
	u8 rx_len =0;
	static u16 cnt_loss_rc=0;
	u8 sta;

	sta= NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);		//??2401?????
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<RX_DR))	//??ing 
	{ 
		cnt_loss_rc=0;
			
				
				
				rx_len= NRF_Read_Reg(R_RX_PL_WID);
				
				if(rx_len<33)	//??????33?????,???????
				{
					NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
					NRF_DataAnl();	//??2401??????
				
				}
				else 
				{ 
					NRF_Write_Reg(FLUSH_RX,0xff);//?????
				}
				if(cnt_led_rx<2)
				cnt_led_rx++;
				else 
				{
				cnt_led_rx=0;
				}
	}
	else//---------losing_nrf
	 {	
		 if(cnt_loss_rc++>200 )//0.5ms
		 {	force_Thr_low=1;
			 NRF_Write_Reg(FLUSH_RX,0xff);//?????
//		   CH[PIT]=CH[ROL]=CH[YAW]=0;
//			 if(height_ctrl_mode==0)
//			 CH[THR]=-500; 
//			 else
//			 CH[THR]=0;
			 
				
	  	RX_CH[PITr]=RX_CH[ROLr]=RX_CH[YAWr]=1500;
//			if(height_ctrl_mode==0)
//			 RX_CH[THR]=1500; 
//			 else
			 RX_CH[THRr]=1000;
			
		 }
  }
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<TX_DS))	//????,?????
	{
	
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	
	if(sta & (1<<MAX_RT))//??,????
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF_Write_Reg(FLUSH_TX,0xff);
		}
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);
}


void NRF_Send_ARMED(void)//????
{
	uint8_t i,sum;
	u32 _temp;
	u8 cnt=0;

	NRF24L01_TXDATA[cnt++]=0x88;
	NRF24L01_TXDATA[cnt++]=0x01;
	NRF24L01_TXDATA[cnt++]=0x1C;
	_temp=8400;//BAT_FLY;
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);//添加 gps 姿态 状态通信
	_temp = (int)(31.1234*10000000);
  NRF24L01_TXDATA[cnt++]=BYTE3(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE2(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp = 0;
  NRF24L01_TXDATA[cnt++]=BYTE3(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE2(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	if(mode.flow_hold_position){
    if(mode.en_circle_control)
		{
	_temp = (int)(nav_circle[0]*100);//(int)(Pitch*100);
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp = (int)(nav_circle[1]*100);//(int)(Roll*100);// (int)((Target.Yaw+180)*100);//
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
		
		}
		else{
		if(RX_CH[AUX3r] >500){
	_temp = (int)(imu_nav.flow.speed.west*100);//(int)(Pitch*100);
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp = (int)(imu_nav.flow.speed.east*100);//(int)(Roll*100);// (int)((Target.Yaw+180)*100);//
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
		}
		else 
		{
	_temp = (int)(nav[0]*100);//(int)(Pitch*100);
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp = (int)(nav[1]*100);//(int)(Roll*100);// (int)((Target.Yaw+180)*100);//
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
		}	
	 }
	}
//	else if(KEY[0]==1&&KEY[1]==1)
//		{
//    switch (height_ctrl_mode_use)
//		{
//			case 1:

//			_temp = (int)(ALT_POS_BMP*1000);//(int)(Roll*100);// (int)((Target.Yaw+180)*100);//
//			NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
//			NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
//						_temp = (int)(exp_height);//(int)(Pitch*100);
//			NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
//			NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
//			break;
//			case 2:
//			_temp = (int)(ALT_POS_SONAR2*1000);//(int)(Roll*100);// (int)((Target.Yaw+180)*100);//
//			NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
//			NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
//						_temp = (int)(exp_height);//(int)(Pitch*100);
//			NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
//			NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
//			break;
//		
//		}			
//		}
	else{
		
	_temp = (int)(Pitch*100);//(int)(Pitch*100);
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp = (int)(Roll*100);//(int)(Roll*100);// (int)((Target.Yaw+180)*100);//
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	}
  _temp = (int)(Yaw*100);
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp=thr_test;//(plane.get.altitude.sonar);//hight
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp=data_rate;//(plane.get.altitude.sonar);//hight
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp=height_ctrl_mode;//(plane.get.altitude.sonar);//hight
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
//	if(mode.height_upload==0){
//	//	if(height_ctrl_mode==1)
//	  //_temp=bmp_dis_lpf;//(plane.get.altitude.sonar);//hight
//		//else if(height_ctrl_mode==2)
//		_temp=ultra_dis_lpf;}
//	else
//		{
		//if(height_ctrl_mode==1)
	 // _temp=bmp_speed;//(plane.get.altitude.sonar);//hight
		//else if(height_ctrl_mode==2)
		_temp= ALT_POS_SONAR2*1000;//ultra_speed;}

	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp=0;//baroAlt;//(plane.get.altitude.sonar);//hight
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	//gps mode
	NRF24L01_TXDATA[cnt++]=(0);
	//gps no
	NRF24L01_TXDATA[cnt++]=(1);
	//fly mode
	NRF24L01_TXDATA[cnt++]=(2);
	NRF24L01_TXDATA[cnt++]=(sys.flow<<3)|(sys.circle<<2)|(sys.avoid<<1)|(sys.gps);
	NRF24L01_TXDATA[cnt++]=EN_FIX_GPSF;
	NRF24L01_TXDATA[cnt++]=EN_FIX_LOCKWF;
	NRF24L01_TXDATA[cnt++]=EN_CONTROL_IMUF;
	NRF24L01_TXDATA[cnt++]=EN_FIX_INSF;
	NRF24L01_TXDATA[cnt++]=EN_FIX_HIGHF;
	//if(ARMED)	NRF24L01_TXDATA[3]=0xA1;
	//else 			NRF24L01_TXDATA[3]=0xA0;
	sum = 0;
	for(i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[31]=sum;
	
	NRF_TxPacket(NRF24L01_TXDATA,32);
}

void NRF_Send_RC_PID1(void)//????
{	vs16 _temp;	u8 i;	u8 sum = 0;

	NRF24L01_TXDATA[0] = 0x88;	//?????8BSET??
	NRF24L01_TXDATA[1] = 0x02;	//?????8BSET??
	NRF24L01_TXDATA[2] = 0x1C;
	NRF24L01_TXDATA[3] = 0xAD;
	
	_temp =SPID.OP;
	NRF24L01_TXDATA[4]=BYTE1(_temp);
	NRF24L01_TXDATA[5]=BYTE0(_temp);
	_temp =SPID.OI;
	NRF24L01_TXDATA[6]=BYTE1(_temp);
	NRF24L01_TXDATA[7]=BYTE0(_temp);
	_temp = SPID.OD;
	NRF24L01_TXDATA[8]=BYTE1(_temp);
	NRF24L01_TXDATA[9]=BYTE0(_temp);
	_temp =SPID.IP;
	NRF24L01_TXDATA[10]=BYTE1(_temp);
	NRF24L01_TXDATA[11]=BYTE0(_temp);
	_temp =SPID.II;
	NRF24L01_TXDATA[12]=BYTE1(_temp);
	NRF24L01_TXDATA[13]=BYTE0(_temp);
	_temp = SPID.ID;
	NRF24L01_TXDATA[14]=BYTE1(_temp);
	NRF24L01_TXDATA[15]=BYTE0(_temp);
	_temp = SPID.YP;//.YP * 1;
	NRF24L01_TXDATA[16]=BYTE1(_temp);
	NRF24L01_TXDATA[17]=BYTE0(_temp);
	_temp = SPID.YI;//SPID.YI * 1;
	NRF24L01_TXDATA[18]=BYTE1(_temp);
	NRF24L01_TXDATA[19]=BYTE0(_temp);
  _temp = SPID.YD;//.YD * 1;
	NRF24L01_TXDATA[20]=BYTE1(_temp);
	NRF24L01_TXDATA[21]=BYTE0(_temp);
	
	

	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
	NRF_TxPacket(NRF24L01_TXDATA,32);
}

void NRF_Send_RC_PID2(void)//????
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;

	NRF24L01_TXDATA[_cnt++] = 0x88;	//?????8BSET??
	NRF24L01_TXDATA[_cnt++] = 0x03;	//?????8BSET??
	NRF24L01_TXDATA[_cnt++] = 0x1C;
	NRF24L01_TXDATA[_cnt++]=	0xAD;
	_temp = 	  HPID.OP;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		_temp = 	HPID.OI;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		_temp = 	HPID.OD;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		_temp = 	0*100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		_temp = 	0*100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	
	
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}

void NRF_Send_RC_Sensor(void)//????
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;

	NRF24L01_TXDATA[_cnt++] = 0x88;	//?????8BSET??
	NRF24L01_TXDATA[_cnt++] = 0x04;	//?????8BSET??
	NRF24L01_TXDATA[_cnt++] = 0x1C;
	NRF24L01_TXDATA[_cnt++]=	0xAD;

	_temp =  0;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  0;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  0;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  0;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}


void NRF_Send_RC_GPS(void)
{u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	NRF24L01_TXDATA[_cnt++]=0x88;
	NRF24L01_TXDATA[_cnt++]=0x05;
	NRF24L01_TXDATA[_cnt++]=0x1C;//功能字
	NRF24L01_TXDATA[_cnt++]=0xAD;
	_temp32 =  imu_nav.gps.J;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.W;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp32);
	_temp =imu_nav.gps.gps_mode;
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =imu_nav.gps.star_num;
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp32 =  imu_nav.gps.X_O;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.Y_O;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.X_UKF;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.Y_UKF;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp32);


	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}

float DEBUG[10];
void NRF_Send_RC_DEBUG1(void)// out= send/100
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;
 
	NRF24L01_TXDATA[_cnt++] = 0x88;	//?????8BSET??
	NRF24L01_TXDATA[_cnt++] = 30;	//?????8BSET??
	NRF24L01_TXDATA[_cnt++] = 0x1C;
	NRF24L01_TXDATA[_cnt++]=	0xAD;
  
	
	if(mode.att_pid_tune){
	DEBUG[1]=Pitch*100;
	DEBUG[2]=Roll*100;
	DEBUG[3]=Pitch*100;
	
	
	}
	else if(mode.en_circle_control)
	{
	DEBUG[1]=circle.x_flp;
	DEBUG[2]=circle.y_flp;
	DEBUG[3]=nav_circle[0]*100;
	DEBUG[4]=nav_circle[1]*100;	
	}
	else{
	DEBUG[1]=exp_height/10;
	DEBUG[2]=ultra_dis_lpf/10;
	DEBUG[3]=ultra_ctrl_out_use;
	DEBUG[4]=ultra_speed;	
		
		
	}
	for( i=1;i<=9;i++){
	_temp =  DEBUG[i];	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
  }
	
	
	
	
	
	
	
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}


void RC_Send_Task(void)
{
static u16 cnt[4]={0,0,0,0};
static u8 state;

switch(state)
{
	case 0:NRF_Send_ARMED();state=1;
	  break;
	case 1:
		NRF_Send_RC_GPS();state=2;
	 break;
	case 2: NRF_Send_RC_DEBUG1();
		state=3;
	 break;
	case 3:
		if(cnt[0]==0){cnt[0]=1;
		NRF_Send_RC_PID1();}
		else
		{cnt[0]=0;
		NRF_Send_RC_PID2();}
		state=0;
 break;
}
/*
if(cnt[0]++>1)
{//????
NRF_Send_ARMED();
 cnt[0]=0;}
if(cnt[1]++>2)
{	
NRF_Send_RC_GPS();// NRF_Send_RC_Sensor();//????
 cnt[1]=0;
}
if(cnt[2]++>2)
{ NRF_Send_RC_PID1();//????
 cnt[2]=0;}

if(cnt[3]++>2)
{ NRF_Send_RC_PID2();//????
 cnt[3]=0;}
*/

}

void CAL_CHECK(void)
{static u8 state_mpu,state_hml;
static u16 cnt_mpu,cnt_hml;
static u8 check_num_mpu,check_num_hml;
	if(!Mag_CALIBRATED&&!fly_ready)
	switch (state_mpu)
	{
		case 0:
			 if(Rc_Get.YAW-1500>200&&!mode.cal_sel)
				  state_mpu=1;
			 break;
		case 1:
			 if(Rc_Get.YAW-1500<-200&&!mode.cal_sel)
				  state_mpu=2;
			 else if(cnt_mpu++>1000)
			 {cnt_mpu=0;state_mpu=0;check_num_mpu=0;}
			 else if(check_num_mpu>6)
			 {  state_mpu=3;
	LEDRGB_COLOR(YELLOW);delay_ms(100);
	LEDRGB_COLOR(BLACK);delay_ms(100);
	LEDRGB_COLOR(YELLOW);delay_ms(100);
	LEDRGB_COLOR(BLACK);delay_ms(100);
	LEDRGB_COLOR(YELLOW);delay_ms(100);
	LEDRGB_COLOR(BLACK);delay_ms(100);
	LEDRGB_COLOR(YELLOW);delay_ms(500);
				 
				mpu6050.Gyro_CALIBRATE=1;
			  mpu6050.Acc_CALIBRATE=1;}
			  break;
		case 2:
			if(Rc_Get.YAW-1500>200&&!mode.cal_sel)
			{  state_mpu=1;check_num_mpu++;}
			 else if(cnt_mpu++>1000)
			 {cnt_mpu=0;state_mpu=0;check_num_mpu=0;}
			  break;
		case 3:	 
			 if(!mpu6050.Gyro_CALIBRATE)
			 {  
			 cnt_mpu=0;state_mpu=0;check_num_mpu=0;}
			break; 
				
	}
	else
	{cnt_mpu=0;state_mpu=0;check_num_mpu=0;}

if( !mpu6050.Gyro_CALIBRATE&&!fly_ready)
switch (state_hml)
	{
		case 0:
			 if(Rc_Get.YAW-1500>200&&mode.cal_sel)
				  state_hml=1;
			 break;
		case 1:
			 if(Rc_Get.YAW-1500<-200&&mode.cal_sel)
				  state_hml=2;
			 else if(cnt_hml++>1000)
			 {state_hml=0;cnt_hml=0;check_num_hml=0;}
			 else if(check_num_hml>6)
			 {  state_hml=3;
			LEDRGB_COLOR(BLUE);delay_ms(100);
			LEDRGB_COLOR(BLACK);delay_ms(100);
			LEDRGB_COLOR(BLUE);delay_ms(100);
			LEDRGB_COLOR(BLACK);delay_ms(100);
			LEDRGB_COLOR(BLUE);delay_ms(100);
			LEDRGB_COLOR(BLACK);delay_ms(100);
			LEDRGB_COLOR(BLUE);delay_ms(500);Mag_CALIBRATED=1;				 
			 }
			  break;
		case 2:
			if(Rc_Get.YAW-1500>200&&mode.cal_sel)
			{  state_hml=1;check_num_hml++;}
			 else if(cnt_hml++>1000)
			 {state_hml=0;cnt_hml=0;check_num_hml=0;}
			  break;
		case 3:	 
			 if(!Mag_CALIBRATED)
			 {  //flag.calibratingM=0;
			   
			 cnt_hml=0;state_hml=0;check_num_hml=0;}
			break; 
				
	}
	else
	{
	cnt_hml=0;state_hml=0;check_num_hml=0;
	}

}

