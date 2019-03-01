#include "include.h"
#include "imu.h"
#include "nrf.h"
#include "fly_mode.h"
#include "rc.h"
#include "alt_fushion.h"
#include "bat.h"
#include "usart.h"
#include "imu.h"
#include "ctrl.h"
#include "time.h"
#include "eso.h"
#include "rc_mine.h"
#include "ak8975.h"
#include "oldx_mission.h"
_DRONE drone;
#define RX_DR			6	
#define TX_DS			5
#define MAX_RT		4
int RX_CH_FIX_NRF[8]={0};
int PARAM[18][3];
u8 cnt_rst=0,delta_pitch=0,delta_roll=0,delta_yew=0;
u8 send_pid=0,read_pid=0;;
u16 data_rate;
#define MID_RC_KEY 15
#define MID_RC_GET 4
u8 key_rc_reg[7][MID_RC_KEY],acc_3d_step;
float RC_GET[4][MID_RC_GET];
float control_scale=1;
float ypr_sb[3];

void NRF_DataAnl(void)
{ 

u8 temp;
	u8 i=0,j,sum = 0;
	for( i=0;i<31;i++)
		sum += NRF24L01_RXDATA[i];
	if(!(sum==NRF24L01_RXDATA[31]))	
	{i=0;
		return;
		}	 
	if(NRF24L01_RXDATA[0]==0x01)								
	{ 
		#if USE_OLDX_REMOTE
		Feed_Rc_Dog(2);
		#endif
		data_rate++;
		Rc_Get.THROTTLE = (vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2];
		Rc_Get.YAW			= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		Rc_Get.ROLL		  = (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		Rc_Get.PITCH 		= (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		Rc_Get.AUX1			= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		Rc_Get.AUX2			= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];
		Rc_Get.AUX3			= (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		Rc_Get.AUX4			= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		Rc_Get.AUX5			= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];
		
	  KEY[0]=(NRF24L01_RXDATA[19])&0x01;
		KEY[1]=(NRF24L01_RXDATA[19]>>1)&0x01;
		KEY[2]=(NRF24L01_RXDATA[19]>>2)&0x01;
		KEY[3]=(NRF24L01_RXDATA[19]>>3)&0x01;
		KEY[4]=(NRF24L01_RXDATA[19]>>4)&0x01;
    u8 cal_temp[4];
		cal_temp[2]=(NRF24L01_RXDATA[20]>>2)&0x01;
		cal_temp[1]=(NRF24L01_RXDATA[20]>>1)&0x01;
		cal_temp[0]=NRF24L01_RXDATA[20]&0x01;
		if(mpu6050_fc.Acc_CALIBRATE==0&&cal_temp[1]==1)
			 mpu6050_fc.Acc_CALIBRATE=1;
		if(mpu6050_fc.Gyro_CALIBRATE==0&&cal_temp[0]==1)
			 mpu6050_fc.Gyro_CALIBRATE=1;
		if(ak8975_fc.Mag_CALIBRATED==0&&cal_temp[2]==1)
			 ak8975_fc.Mag_CALIBRATED=1;
		acc_3d_step=NRF24L01_RXDATA[22];
		temp=NRF24L01_RXDATA[21];
		if(temp==1)
			read_pid=1;
	 }
	else if(NRF24L01_RXDATA[0]==0x02)//pid1
	{
	  PARAM[0][0] = (vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2];
		PARAM[0][1]	= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		PARAM[0][2]	= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		
		PARAM[1][0] = (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		PARAM[1][1]	= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		PARAM[1][2]	= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];

		PARAM[2][0] = (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		PARAM[2][1]	= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		PARAM[2][2]	= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];

		PARAM[3][0] = (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];
		PARAM[3][1]	= (vs16)(NRF24L01_RXDATA[21]<<8)|NRF24L01_RXDATA[22];
		PARAM[3][2]	= (vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24];
		
		PARAM[4][0] = (vs16)(NRF24L01_RXDATA[25]<<8)|NRF24L01_RXDATA[26];
		PARAM[4][1]	= (vs16)(NRF24L01_RXDATA[27]<<8)|NRF24L01_RXDATA[28];
		PARAM[4][2]	= (vs16)(NRF24L01_RXDATA[29]<<8)|NRF24L01_RXDATA[30];
	
	}
	else if(NRF24L01_RXDATA[0]==0x03)//pid2
	{
	  PARAM[5][0] = (vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2];
		PARAM[5][1]	= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		PARAM[5][2]	= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		
		PARAM[6][0] = (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		PARAM[6][1]	= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		PARAM[6][2]	= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];

		PARAM[7][0] = (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		PARAM[7][1]	= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		PARAM[7][2]	= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];

		PARAM[8][0] = (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];
		PARAM[8][1]	= (vs16)(NRF24L01_RXDATA[21]<<8)|NRF24L01_RXDATA[22];
		PARAM[8][2]	= (vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24];
		
		PARAM[9][0] = (vs16)(NRF24L01_RXDATA[25]<<8)|NRF24L01_RXDATA[26];
		PARAM[9][1]	= (vs16)(NRF24L01_RXDATA[27]<<8)|NRF24L01_RXDATA[28];
		PARAM[9][2]	= (vs16)(NRF24L01_RXDATA[29]<<8)|NRF24L01_RXDATA[30];
	}
	else if(NRF24L01_RXDATA[0]==0x04)//pid3
	{
	  PARAM[10][0] = (vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2];
		PARAM[10][1]	= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		PARAM[10][2]	= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		
		PARAM[11][0] = (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		PARAM[11][1]	= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		PARAM[11][2]	= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];

		PARAM[12][0] = (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		PARAM[12][1]	= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		PARAM[12][2]	= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];

		PARAM[13][0] = (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];
		PARAM[13][1]	= (vs16)(NRF24L01_RXDATA[21]<<8)|NRF24L01_RXDATA[22];
		PARAM[13][2]	= (vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24];
		
		PARAM[14][0] = (vs16)(NRF24L01_RXDATA[25]<<8)|NRF24L01_RXDATA[26];
		PARAM[14][1]	= (vs16)(NRF24L01_RXDATA[27]<<8)|NRF24L01_RXDATA[28];
		PARAM[14][2]	= (vs16)(NRF24L01_RXDATA[29]<<8)|NRF24L01_RXDATA[30];
	}
	else if(NRF24L01_RXDATA[0]==0x05)//pid3
	{
	  PARAM[15][0] = (vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2];
		PARAM[15][1]	= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		PARAM[15][2]	= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		
		PARAM[16][0] = (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		PARAM[16][1]	= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		PARAM[16][2]	= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];

		PARAM[17][0] = (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		PARAM[17][1]	= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		PARAM[17][2]	= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];
		send_pid=1;
	}	
}


int cnt_timer2_r=0;
u8 cnt_led_rx=0;
void Nrf_Check_Event(void)
{ 
	u8 rx_len =0;
	static u16 cnt_loss_rc=0;
	u8 sta;

	sta= NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);		
	if(sta & (1<<RX_DR))	//??ing 
	{ 
		cnt_loss_rc=0;
			
				rx_len= NRF_Read_Reg(R_RX_PL_WID);			
				if(rx_len<33)
				{
					NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
					NRF_DataAnl();	
				}
				else 
				{ 
					NRF_Write_Reg(FLUSH_RX,0xff );
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
		 {	
			 NRF_Write_Reg(FLUSH_RX,0xff);//?????

		 }
  }
	if(sta & (1<<TX_DS))	//????,?????
	{
	
	}

	if(sta & (1<<MAX_RT))//??,????
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF_Write_Reg(FLUSH_TX,0xff);
		}
	}
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);
}

u8 pos_up_sel=0;
u8 pos_up_sel1=1;
void NRF_Send1(void)
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;

	NRF24L01_TXDATA[_cnt++] = 1;	
	
	_temp =  Rol_fc*10;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  Pit_fc*10;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
  _temp =  (Yaw_fc-circle.yaw_off)*10;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
   
	drone.spd[Yr]=VEL_UKF_Y;
	drone.spd[Xr]=VEL_UKF_X;
	drone.spd[Zr]=ALT_VEL_BMP_UKF_OLDX;
	_temp =  drone.spd[Xr]*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  drone.spd[Yr]*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  drone.spd[Zr]*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	if(pos_up_sel)
	{
	drone.pos[Yr]=nav_pos_ctrl[X].exp;
	drone.pos[Xr]=nav_pos_ctrl[Y].exp;
	drone.pos[Zr]=ALT_POS_BMP_UKF_OLDX;
	}	
	else{
	drone.pos[Yr]=POS_UKF_Y;
	drone.pos[Xr]=POS_UKF_X;
	drone.pos[Zr]=ALT_POS_BMP_UKF_OLDX;
	}
	#if defined(POS_FUSION_TEST)
	if(KEY[1])
	_temp =  circle.cx;
	else
	_temp =  drone.pos[Xr]*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
  if(KEY[1])
	_temp =  circle.cy;
	else
	_temp =  drone.pos[Yr]*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	#else
//	if(Rc_Get_PWM.AUX2>1500) 
//		pos_up_sel1=0;
//	else
//		pos_up_sel1=1;
	if(pos_up_sel1==0){
	_temp =  (c2c.gx+2)*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  (c2c.gy+2)*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	}else{
	_temp =  (drone.pos[Xr]+2*0)*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  (drone.pos[Yr]+2*0)*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	}
	#endif
	_temp =  ALT_POS_BMP_UKF_OLDX*100;//drone.pos[Z]*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	_temp =  	m100.GPS_STATUS;
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(fly_ready);
  NRF24L01_TXDATA[_cnt++]=BYTE0(drone.fly_mode);
	NRF24L01_TXDATA[_cnt++]=BYTE0(state_v);
	NRF24L01_TXDATA[_cnt++]=BYTE0(drone.acc_3d_step);  
	drone.bat=bat.percent;
	_temp =  drone.bat*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp=0;
	if(module.flow)
	_temp=1;
	if(module.flow&&circle.connect)
	_temp=2;
	if(m100.STATUS!=99)
  _temp=3;
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);	
	//module
	NRF24L01_TXDATA[_cnt++]=module.gps<<7|c2c.connect<<6|module.flow<<5|module.bmp<<4
	|module.sonar<<3;
	
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}

#include "mavl.h"
#include "oldx_api.h"
void NRF_Send_mission(void)
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;

	NRF24L01_TXDATA[_cnt++] = 7;	
	
	_temp =  smart.att.x*10;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  smart.att.y*10;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
  _temp =  smart.att.z*10;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  smart.spd.x*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  smart.spd.y*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  smart.spd.z*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  smart.pos.x*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  smart.pos.y*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  smart.pos.z*100;//drone.pos[Z]*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);

	NRF24L01_TXDATA[_cnt++]=BYTE0(state_v);
  NRF24L01_TXDATA[_cnt++]=BYTE0(mission_state);
	
	_temp=0;
	if(smart.rc.HEIGHT_MODE==SMART_MODE_SPD)
	  _temp+=1;
	else if(smart.rc.HEIGHT_MODE==SMART_MODE_POS)
		_temp+=2;
	
	if(smart.rc.POS_MODE==SMART_MODE_SPD)
		_temp+=10;
	else if(smart.rc.POS_MODE==SMART_MODE_POS)
		_temp+=20;

	if(smart.rc.ATT_MODE==SMART_MODE_POS)
		_temp+=100;

	//_temp=22;
	NRF24L01_TXDATA[_cnt++]=_temp;
	NRF24L01_TXDATA[_cnt++]=LIMIT(navData.Leg_num-cnt_mission[MISSION_CNT],0,100);
  NRF24L01_TXDATA[_cnt++]=mcuID[2];
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}

void NRF_Send_gps(void)
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;
  vs32 _temp32;
	u8 flag[2];
	
	NRF24L01_TXDATA[_cnt++] = 8;	
	//m100.Lon=134.12345677;
	_temp = (vs16)ABS(lon_human);//latitude;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((ABS(lon_human)-(int)ABS(lon_human))*1000000000);//latitude;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp32);
	//m100.Lat=-34.12345678;
	_temp = (vs16)ABS(lat_human);//latitude;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((ABS(lat_human)-(int)ABS(lat_human))*1000000000);//latitude;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp32);

	if(m100.Lon>0)
	flag[0]=1;
  else
  flag[0]=0;		
	if(m100.Lat>0)
	flag[1]=1;
  else
  flag[1]=0;		
	_temp=flag[0]<<1|flag[1];
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}

void NRF_Send2(void)
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;

	NRF24L01_TXDATA[_cnt++] = 2;	

	for(i=0+1;i<9+1;i++){//9
		_temp =  BLE_DEBUG[i];	
		NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
		NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	}
	for(i=0;i<5;i++){//5
		_temp =  CH[i];	
		NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
		NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	}
		
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}
	

void NRF_Send3(void)
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0,j;

	NRF24L01_TXDATA[_cnt++] = 3;	
	
	NRF24L01_TXDATA[_cnt++]=BYTE0(fly_ready);
  NRF24L01_TXDATA[_cnt++]=BYTE0(drone.fly_mode);
	NRF24L01_TXDATA[_cnt++]=BYTE0(state_v);
	NRF24L01_TXDATA[_cnt++]=BYTE0(drone.acc_3d_step);  
	drone.bat=bat.percent;
	_temp =  drone.bat*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	for(i=0;i<4;i++){//24
		for(j=0;j<3;j++){
			_temp =  PARAM[i][j];	
			NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
			NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		}
	}
	

	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}

void NRF_Send4(void)
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0,j;

	NRF24L01_TXDATA[_cnt++] = 4;	

	for(i=4;i<9;i++){//30
		for(j=0;j<3;j++){
		_temp =  PARAM[i][j];	
		NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
		NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		}
	}
	

	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}


void NRF_Send5(void)
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0,j;

	NRF24L01_TXDATA[_cnt++] = 5;	

	for(i=9;i<13;i++){//30
		for(j=0;j<3;j++){
		_temp =  PARAM[i][j];	
		NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
		NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		}
	}
	

	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}


void NRF_Send6(void)
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0,j;

	NRF24L01_TXDATA[_cnt++] = 6;	

	for(i=13;i<18;i++){//30
		for(j=0;j<3;j++){
		_temp =  PARAM[i][j];	
		NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
		NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		}
	}
	

	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}

void RC_Send_Task(void)
{
static u8 cnt[5]={0};
	if(cnt[0]++>10)
	 cnt[0]=0;
	
if(send_pid||read_pid){
	if(read_pid)
		pid_copy_param();
	if(cnt[1]==0){cnt[1]=1;
	NRF_Send3();}
	else if(cnt[1]==1){cnt[1]=2;
	NRF_Send4();}
	else if(cnt[1]==2){cnt[1]=3;
	NRF_Send5();}
	else{cnt[1]=0;
	NRF_Send6();
	 cnt[2]++;
	}
  if(cnt[2]++>5)
	{
	 cnt[2]=0;
	 if(read_pid==0){
	 send_pid=0;	
	 param_copy_pid();	
	 }else
	 read_pid=0;
	}
}else
{//normal

	if(cnt[3]++>5){
	 cnt[3]=0;
		if(cnt[0]%2==0)
			NRF_Send_mission();
		else
			NRF_Send_gps();
  }else{
  if(cnt[0]%2==0)
	NRF_Send1();
	else
	NRF_Send2();
 }
}

}


void param_copy_pid(void){

	ctrl_1.PID[PIDPITCH].kp= ctrl_1.PID[PIDROLL].kp  = 0.001*PARAM[0][0];
	ctrl_1.PID[PIDPITCH].ki= ctrl_1.PID[PIDROLL].ki  = 0.001*PARAM[0][1];
	ctrl_1.PID[PIDPITCH].kd= ctrl_1.PID[PIDROLL].kd  = 0.001*PARAM[0][2];
	eso_att_inner_c[ROLr].b0=eso_att_inner_c[PITr].b0= 			0.1*PARAM[1][0];
	eso_att_inner_c[YAWr].b0  											 = 			0.1*PARAM[1][1];
	//eso_att_inner_c[ROLr].eso_dead=eso_att_inner_c[PITr].eso_dead=   PARAM[1][2]*0.001;
	k_pitch=   PARAM[1][2]*0.01;
	ctrl_1.PID[PIDYAW].kp 	= 0.001*PARAM[2][0];
	ctrl_1.PID[PIDYAW].ki 	= 0.001*PARAM[2][1];
	ctrl_1.PID[PIDYAW].kd 	= 0.001*PARAM[2][2];
  //------
	ctrl_2.PID[PIDPITCH].kp =ctrl_2.PID[PIDROLL].kp  = 0.001*PARAM[3][0];
	ctrl_2.PID[PIDPITCH].ki =ctrl_2.PID[PIDROLL].ki  = 0.001*PARAM[3][1];
	ctrl_2.PID[PIDPITCH].kd =ctrl_2.PID[PIDROLL].kd  = 0.001*PARAM[3][2];
//	ctrl_2.PID[PIDYAW].kp 	= 0.001*PARAM[4][0];
//	ctrl_2.PID[PIDYAW].ki 	= 0.001*PARAM[4][1];
//	ctrl_2.PID[PIDYAW].kd 	= 0.001*PARAM[4][2];
	ctrl_2.PID[PIDYAW].kp 	= 0.001*PARAM[5][0];
	ctrl_2.PID[PIDYAW].ki 	= 0.001*PARAM[5][1];
	ctrl_2.PID[PIDYAW].kd 	= 0.001*PARAM[5][2];
  //
	wz_speed_pid.kp  = 0.001*PARAM[6][0];
	wz_speed_pid.ki  = 0.001*PARAM[6][1];
	wz_speed_pid.kd  = 0.001*PARAM[6][2];

	ultra_pid.kp = 0.001*PARAM[7][0];
	ultra_pid.ki = 0.001*PARAM[7][1];
	ultra_pid.kd = 0.001*PARAM[7][2];

	nav_spd_pid.kp	= 	0.001*PARAM[8][0];
	nav_spd_pid.ki  =   0.001*PARAM[8][1];
	nav_spd_pid.kd  =   0.001*PARAM[8][2];
  //
	nav_pos_pid.kp  = 0.001*PARAM[9][0];
	nav_pos_pid.ki  = 0.001*PARAM[9][1];
	nav_pos_pid.kd  = 0.001*PARAM[9][2];

	eso_pos[Y].b0=eso_pos[X].b0 = PARAM[10][0];
	eso_pos[Zr].b0=               PARAM[10][1];
	eso_pos[Y].eso_dead=eso_pos[X].eso_dead = PARAM[10][2];

	eso_pos_spd[Zr].b0 	= 				PARAM[11][0];
	nav_spd_pid.flt_nav 	= 0.001*PARAM[11][1];
	eso_pos_spd[Zr].eso_dead	=   PARAM[11][2];
  //
	eso_pos_spd[Y].b0=eso_pos_spd[X].b0  = PARAM[12][0];
	//	nav_spd_pid.flt_nav_kd = 0.001*PARAM[12][1];
	//	nav_spd_pid.dead =             PARAM[12][2];

	nav_spd_pid.f_kp = 			 0.001*PARAM[13][0];
	nav_spd_pid.flt_nav_kd = 0.001*PARAM[13][1];
	nav_spd_pid.dead =             PARAM[13][2];

	nav_acc_pid.f_kp 	= 			0.001*	PARAM[14][0];
	nav_acc_pid.kp 	=         0.001*  PARAM[14][1];
	nav_acc_pid.dead	=               PARAM[14][2];
  //
	track_pid[0].kp = 0.0001*PARAM[15][0];
	track_pid[0].kd = 0.0001*PARAM[15][1];
	//track_pid[0].kp = 0.0001*PARAM[15][2];


	k_sensitivity[0] = 0.01*PARAM[16][0];
	k_sensitivity[1] = 0.01*PARAM[16][1];
	k_sensitivity[2] = 0.01*PARAM[16][2];

	mission_sel					 =  PARAM[17][0];
	LENGTH_OF_DRONE=    						 PARAM[17][1];
	UART_UP_LOAD_SEL_FORCE=          PARAM[17][2];

}



void pid_copy_param(void){

	PARAM[0][0]=ctrl_1.PID[PIDPITCH].kp*1000;
	PARAM[0][1]=ctrl_1.PID[PIDPITCH].ki*1000;
	PARAM[0][2]=ctrl_1.PID[PIDPITCH].kd*1000;

	PARAM[1][0]=eso_att_inner_c[PITr].b0*10;
	PARAM[1][1]=eso_att_inner_c[YAWr].b0*10;
	//PARAM[1][2]=eso_att_inner_c[ROLr].eso_dead*1000;
	PARAM[1][2]=k_pitch*100;
	PARAM[2][0]=ctrl_1.PID[PIDYAW].kp *1000;
	PARAM[2][1]=ctrl_1.PID[PIDYAW].ki *1000;
	PARAM[2][2]=ctrl_1.PID[PIDYAW].kd *1000;
  //
	PARAM[3][0]=ctrl_2.PID[PIDPITCH].kp *1000;
	PARAM[3][1]=ctrl_2.PID[PIDPITCH].ki *1000;
	PARAM[3][2]=ctrl_2.PID[PIDPITCH].kd *1000;
	//0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
	// 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
	// 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
	PARAM[5][0]=ctrl_2.PID[PIDYAW].kp*1000;
	PARAM[5][1]=ctrl_2.PID[PIDYAW].ki*1000;
	PARAM[5][2]=ctrl_2.PID[PIDYAW].kd*1000;
  //
	PARAM[6][0]=wz_speed_pid.kp*1000 ;
	PARAM[6][1]=wz_speed_pid.ki*1000 ;
	PARAM[6][2]=wz_speed_pid.kd*1000 ;

	PARAM[7][0]=ultra_pid.kp *1000;
	PARAM[7][1]=ultra_pid.ki *1000;
	PARAM[7][2]=ultra_pid.kd *1000;

	PARAM[8][0]=nav_spd_pid.kp	*1000;
	PARAM[8][1]=nav_spd_pid.ki  *1000;
	PARAM[8][2]=nav_spd_pid.kd  *1000;
  //
	PARAM[9][0]=nav_pos_pid.kp *1000;
	PARAM[9][1]=nav_pos_pid.ki *1000;
	PARAM[9][2]=nav_pos_pid.kd *1000;

	PARAM[10][0]=eso_pos[Y].b0;
	PARAM[10][1]=eso_pos[Zr].b0;
	PARAM[10][2]=eso_pos[Y].eso_dead;

	PARAM[11][0]=eso_pos_spd[Zr].b0;
	PARAM[11][1]=nav_spd_pid.flt_nav *1000;
	PARAM[11][2]=eso_pos_spd[Zr].eso_dead	;
  //
	PARAM[12][0]=eso_pos_spd[Y].b0;
	//nav_spd_pid.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
	//nav_spd_pid.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );

	PARAM[13][0]=nav_spd_pid.f_kp *1000;
	PARAM[13][1]=nav_spd_pid.flt_nav_kd *1000;
	PARAM[13][2]=nav_spd_pid.dead ;

	PARAM[14][0]=nav_acc_pid.f_kp *1000;
	PARAM[14][1]=nav_acc_pid.kp 	*1000;
	PARAM[14][2]=nav_acc_pid.dead	;
  //
	PARAM[15][0]=track_pid[0].kp*10000;		
  PARAM[15][1]=track_pid[0].kd*10000;
	PARAM[15][2]=0;

	PARAM[16][0]=k_sensitivity[0] *100;
	PARAM[16][1]=k_sensitivity[1] *100;
	PARAM[16][2]=k_sensitivity[2] *100;

	PARAM[17][0]=mission_sel;
	PARAM[17][1]=LENGTH_OF_DRONE;
	PARAM[17][2]=UART_UP_LOAD_SEL_FORCE;

}
