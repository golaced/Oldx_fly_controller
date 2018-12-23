#include "LIS3MDL.h"
#include "include.h"
#include "usart_fc.h"
#include "ultrasonic.h"
#include "hml5833l.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "rc.h"
#include "att.h"
#include "height_ctrl.h"
#include "alt_kf.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "circle.h"
#include "error.h"
#include "flow.h"
#include "gps.h"
#include "ublox.h"
#include "flow.h"
#include "nav_ukf.h"
#include "Ano_OF.h"
#include "wifi_ctrl.h"

FLOW_P5A flow_5a;
u8 navigaition_mode;
void Data_Receive_Anl11(u8 *data_buf,u8 num);
void Usart2_Init(u32 br_num)//--GOL-link
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

	

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART2, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART2, ENABLE); 
}

struct _QR qr;
RESULT color;
float dt_flow_rx;
u16 data_rate_gol_link;
PID_STA HPID,SPID,FIX_PID,NAV_PID;
struct _PID_SET pid;
RC_GETDATA Rc_Get;
struct _plane plane;
struct _slam slam;
struct _IMU_NAV imu_nav;
struct _MODE mode;
u8 LOCK, KEY[8],KEY_SEL[4];
struct _FLOW_DEBUG flow_debug;
u8 NAV_BOARD_CONNECT=0;
u8 force_fly_ready;
int par[12];
float k_flow_opencv=0.88;
float sonar_fc,baroAlt_fc;
float fc_rx_time;
float flow_module_set_yaw;
u8 acc_3d_step,imu_feed_dog;
u8 FC_CONNECT;
u16 fc_loss_cnt=0;
u8 fly_ready_force_zero=1;
void Data_Receive_Anl(u8 *data_buf,u8 num)
{ static u8 flag;
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
   if(*(data_buf+2)==0x81)//IMU_FRAME  QR
  { FC_CONNECT=1;
		fc_loss_cnt=0;
		module.fc=1;
		fc_rx_time = Get_Cycle_T(GET_T_FC);			
    fly_ready=(*(data_buf+10)||force_fly_ready||en_ble_debug)&&fly_ready_force_zero;	
		fly_ready_rx=*(data_buf+10);
		qr.x=((vs16)(*(data_buf+11)<<8)|*(data_buf+12));
		qr.y=((vs16)(*(data_buf+13)<<8)|*(data_buf+14));
		qr.z=((vs16)(*(data_buf+15)<<8)|*(data_buf+16));
		qr.pit=((vs16)(*(data_buf+17)<<8)|*(data_buf+16));
		qr.rol=((vs16)(*(data_buf+19)<<8)|*(data_buf+20));
		qr.yaw=((vs16)(*(data_buf+21)<<8)|*(data_buf+22));
		qr.spdx=-((vs16)(*(data_buf+23)<<8)|*(data_buf+24))/10*k_flow_opencv;
		qr.spdy=-((vs16)(*(data_buf+25)<<8)|*(data_buf+26))/10*k_flow_opencv;
		qr.yaw_off=(float)((int16_t)(*(data_buf+27)<<8)|*(data_buf+28))/10.;
		qr.connect=*(data_buf+29);
		qr.check=*(data_buf+30);
		qr.use_spd=*(data_buf+31);
		sonar_fc=(float)((int16_t)(*(data_buf+32)<<8)|*(data_buf+33))/1000.;
    baroAlt_fc=(float)((int16_t)(*(data_buf+34)<<8)|*(data_buf+35));
		acc_3d_step=*(data_buf+36);
		three_wheel_car.car_mode=*(data_buf+37);
    three_wheel_car.gain[0]=(float)((int16_t)(*(data_buf+38)<<8)|*(data_buf+39))/100.;
		three_wheel_car.gain[1]=(float)((int16_t)(*(data_buf+40)<<8)|*(data_buf+41))/100.;
		three_wheel_car.gain[2]=(float)((int16_t)(*(data_buf+42)<<8)|*(data_buf+43))/100.;
	  three_wheel_car.car_cmd_spd[0]=(float)((int16_t)(*(data_buf+44)<<8)|*(data_buf+45))/100.;
		three_wheel_car.car_cmd_spd[1]=(float)((int16_t)(*(data_buf+46)<<8)|*(data_buf+47))/100.;
		three_wheel_car.car_cmd_spd[2]=(float)((int16_t)(*(data_buf+48)<<8)|*(data_buf+49))/100.;
	}
		else if(*(data_buf+2)==0x14)//IMU_FRAME
  {
	
		if(!mpu6050.Acc_CALIBRATE&&*(data_buf+22)==1)
		mpu6050.Acc_CALIBRATE=1;
		if(!mpu6050.Gyro_CALIBRATE&&*(data_buf+23)==1)
		mpu6050.Gyro_CALIBRATE=1;
		if(!ak8975.Mag_CALIBRATED&&*(data_buf+24)==1)
		ak8975.Mag_CALIBRATED=1;
 
	}
		else if(*(data_buf+2)==0x82)//
  {
	  par[0]=(int16_t)(*(data_buf+4)<<8)|*(data_buf+5);//op
    par[1]=(int16_t)(*(data_buf+6)<<8)|*(data_buf+7);
		par[2]=(int16_t)(*(data_buf+8)<<8)|*(data_buf+9);
		par[3]=(int16_t)(*(data_buf+10)<<8)|*(data_buf+11);//ip
		par[4]=(int16_t)(*(data_buf+12)<<8)|*(data_buf+13);
		par[5]=(int16_t)(*(data_buf+14)<<8)|*(data_buf+15);
		par[6]=(int16_t)(*(data_buf+16)<<8)|*(data_buf+17);//yp
		par[7]=(int16_t)(*(data_buf+18)<<8)|*(data_buf+19);
		par[8]=(int16_t)(*(data_buf+20)<<8)|*(data_buf+21);
		par[9]=((int16_t)(*(data_buf+22)<<8)|*(data_buf+23));//hp
		par[10]=((int16_t)(*(data_buf+24)<<8)|*(data_buf+25));
		par[11]=((int16_t)(*(data_buf+26)<<8)|*(data_buf+27));
		//kf_data_sel=*(data_buf+28);
		if(!lis3mdl.Acc_CALIBRATE&&*(data_buf+29)==1)
		lis3mdl.Acc_CALIBRATE=1;
		if(!lis3mdl.Gyro_CALIBRATE&&*(data_buf+30)==1)
		lis3mdl.Gyro_CALIBRATE=1;
		static u8 mag_flag;
		if(!lis3mdl.Mag_CALIBRATED&&*(data_buf+31)==1&&mag_flag==0)
		mag_flag=lis3mdl.Mag_CALIBRATED=1;
	  
		if(!lis3mdl.Mag_CALIBRATED&&*(data_buf+31)==0)
			mag_flag=0;
	}	
		else if(*(data_buf+2)==0x83)//
  {
	 k_flow_devide=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/1000.;
   flow_module_offset_x=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/1000.;
	 flow_module_offset_y=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/1000.;
	 flow_module_set_yaw=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/10.;
	 en_px4_mapper=*(data_buf+12);
   imu_feed_dog=*(data_buf+13);		
	
	}
  else  if(*(data_buf+2)==0x04)//
  { m100.control_loss=0;
		m100.control_connect=1;
		imu_feed_dog=FC_CONNECT=1;
		fc_loss_cnt=0;
		module.fc=1;
		m100.px4_tar_mode=*(data_buf+4);
	  m100.control_spd[0]=(float)((int16_t)(*(data_buf+5)<<8)|*(data_buf+6))/1000.;
		m100.control_spd[1]=(float)((int16_t)(*(data_buf+7)<<8)|*(data_buf+8))/1000.;
		m100.control_spd[2]=(float)((int16_t)(*(data_buf+9)<<8)|*(data_buf+10))/1000.;
		m100.control_yaw=(float)((int16_t)(*(data_buf+11)<<8)|*(data_buf+12))/100.;
	}			
}
 

u8 RxBuffer[60];
u8 RxState = 0;
u8 RxBufferNum = 0;
u8 RxBufferCnt = 0;
u8 RxLen = 0;
u8 com_data ;
static u8 _data_len = 0,_data_cnt = 0;
void USART2_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART2->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志

		com_data = USART2->DR;
				if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
		{
			RxState=2;
			RxBuffer[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer[2]=com_data;
		}
		else if(RxState==3&&com_data<60)
		{
			RxState = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer[4+_data_cnt]=com_data;
			Data_Receive_Anl(RxBuffer,_data_cnt+5);
			Data_Receive_Anl11(RxBuffer,_data_cnt+5);//px4 from fc
		}
		else
			RxState = 0;
	
	}
	

   OSIntExit(); 

}

void UsartSend_GOL_LINK(uint8_t ch)
{


	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
USART_SendData(USART2, ch); 
}

static void Send_Data_GOL_LINK(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK(dataToSend[i]);
}


//------------------------------------------------------GOL_LINK----------------------------------------------------
#include "ekf_ins.h"
#include "ukf_task.h"
#include "ukf_baro.h"
void Send_UKF1(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x12;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = (vs16) (flow.rate);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( imu_nav.flow.speed.x*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  (vs16)(imu_nav.flow.speed.y*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (flow_ground_temp[0]*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (flow_ground_temp[1]*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp = (vs16)( FLOW_POS_X*100);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (FLOW_POS_Y*100);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	//-------------------
	_temp = (vs16)(ALT_VEL_SONAR*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(ALT_POS_SONAR2*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ALT_VEL_BMP*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32) (ALT_POS_BMP*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
  _temp = (vs16)( UKF_POSD*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32) (UKF_VELD*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	//-------------oldx_ukf flow qr fushion
	_temp = (vs16)( X_ukf_Pos[0]*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (X_ukf_Pos[1]*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	//spd
	_temp = (vs16)( X_ukf[1]*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (X_ukf[4]*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16) (X_ukf_baro[3]*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (X_ukf_baro[4]*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

  _temp = pi_flow.sensor.x;//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = pi_flow.sensor.y;//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = pi_flow.sensor.z;//navUkfata.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}

void Send_CAL(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x02;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = (vs16)(mpu6050.Acc_Offset.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(mpu6050.Acc_Offset.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(mpu6050.Acc_Offset.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( mpu6050.Gyro_Offset.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( mpu6050.Gyro_Offset.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( mpu6050.Gyro_Offset.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( ak8975.Mag_Offset.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ak8975.Mag_Offset.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ak8975.Mag_Offset.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp=  (vs16)( ak8975.Mag_Gain.x*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ak8975.Mag_Gain.y*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ak8975.Mag_Gain.z*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( 0);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  

void Send_MEMS(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x10;//功能字
	data_to_send[_cnt++]=0;//数据量

	_temp = (vs16)(imu_fushion.Acc.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(imu_fushion.Acc.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(imu_fushion.Acc.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( imu_fushion.Gyro.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( imu_fushion.Gyro.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( imu_fushion.Gyro.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( imu_fushion.Mag_Val.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( imu_fushion.Mag_Val.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( imu_fushion.Mag_Val.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)(baroAlt);//ultra_distance;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);

	
	_temp=  (vs16)( flow_matlab_data[0]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=  (vs16)( flow_matlab_data[1]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=  (vs16)( flow_matlab_data[2]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=  (vs16)( flow_matlab_data[3]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp=  (vs16)( baro_matlab_data[1]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  

void Send_ATT(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x11;//功能字
	data_to_send[_cnt++]=0;//数据量

	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( ref_q[0]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ref_q[1]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q[2]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q[3]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(Pitch_mid_down*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(Roll_mid_down*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw_mid_down*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( ref_q_imd_down[0]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ref_q_imd_down[1]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q_imd_down[2]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q_imd_down[3]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  

int16_t DEBUG_BUF[10];
void Send_BLE_DEBUG(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
  
	if(debug_pi_flow[0])
	_temp =  1;	
	else	
	_temp =  en_ble_debug;
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[1];	
	else	
	DEBUG_BUF[0]=_temp =  ax;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[2];	
	else	
	DEBUG_BUF[1]=_temp =  ay;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[3];	
	else	
	DEBUG_BUF[2]=_temp =  az;//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[4];	
	else	
	DEBUG_BUF[3]=_temp =  gx;//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[5];	
	else	
	DEBUG_BUF[4]=_temp =  gy;//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[6];	
	else	
	DEBUG_BUF[5]=_temp =  gz;//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[7];	
	else	
	DEBUG_BUF[6]=_temp =  hx;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[8];	
	else	
	DEBUG_BUF[7]=_temp =  hy;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[9];	
	else	
	DEBUG_BUF[8]=_temp =  hz;//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp=  ( pi_flow.sensor.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=  ( pi_flow.sensor.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=  ( pi_flow.sensor.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=module.sonar;
	data_to_send[_cnt++]=module.gps;
	data_to_send[_cnt++]=module.flow||module.flow_iic;
	data_to_send[_cnt++]=module.laser;
	data_to_send[_cnt++]=module.pi_flow;
	data_to_send[_cnt++]=module.acc;
	data_to_send[_cnt++]=module.gyro;
	data_to_send[_cnt++]=module.hml;
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	#if !EN_DMA_UART2
	Send_Data_GOL_LINK(data_to_send, _cnt);
	#endif
}




void Send_GPS(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp1;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x14;//功能字
	data_to_send[_cnt++]=0;//数据量
  //origin
	_temp1 = (vs32)( gpsx.longitude*10000000);//ultra_distance;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
	_temp1 = (vs32)( gpsx.latitude*10000000);//ultra_distance;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
	_temp = (vs16)( gpsx.spd*100);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( gpsx.angle*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(gpsx.gpssta);// gpsx.gpssta);//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( gpsx.posslnum);//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( gpsx.svnum);//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	
	//KF
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  




void Send_PX4(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp1;
	vs32 _temp32;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x04;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = (vs16)(m100.Pit*10);//Pitch;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rol*10);//Roll;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Yaw*10);//Yaw;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(m100.H*1000);//altitude;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(ultra.relative_height*10);//height velocity;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs32)(m100.Lat);//latitude;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((m100.Lat-(int)(m100.Lat))*1000000000);//latitude;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	
	_temp = (vs32)(m100.Lon);//latitude;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((m100.Lon-(int)(m100.Lon))*1000000000);//latitude;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);

	_temp = (vs16)(m100.Bat);//Bat;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = (vs16)(m100.Rc_pit);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_rol);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_yaw);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_thr);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_mode);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_gear);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(m100.STATUS&&m100.m100_data_refresh&&m100.connect);//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.GPS_STATUS);//GPS_S;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.spd[0]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.spd[1]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.spd[2]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	//KF
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  

void GOL_LINK_TASK(void)//5ms
{
static u8 cnt[10];
static u8 flag[10];
Send_PX4();	
}


int sd_save[20];
u8 sd_save_sel=0;
void sd_save_publish(void)
{
u8 i=0;	
switch(sd_save_sel)
{
	case 0:
sd_save[i++]=RollR*100;
sd_save[i++]=PitchR*100;
sd_save[i++]=YawR*100;
sd_save[i++]=YawRm*100;
sd_save[i++]=0;
	
sd_save[i++]=Global_GPS_Sensor.NED_Acc[Xr]*100;
sd_save[i++]=Global_GPS_Sensor.NED_Acc[Yr]*100;
sd_save[i++]=X_ukf_global[1]*100;
sd_save[i++]=X_ukf_global[4]*100;
sd_save[i++]= X_ukf_Pos[1]*100;

sd_save[i++]= X_ukf_Pos[0]*100;
sd_save[i++]=0;
sd_save[i++]=pi_flow.check&&pi_flow.connect;
sd_save[i++]= Global_GPS_Sensor.NED_Posf_reg[Yr]*100;
sd_save[i++]= Global_GPS_Sensor.NED_Posf_reg[Xr]*100;

sd_save[i++]=0;
sd_save[i++]=0;
sd_save[i++]=0;
sd_save[i++]=0;
sd_save[i++]=0;
  break;
}
}





u16 nrf_uart_cnt;
void data_per_uart4(u8 sel)
{
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  vs32 _temp32;
  u8 flag[2];
switch(sel){
	case SEND_IMU_MEMS:
	cnt_reg=nrf_uart_cnt;
  SendBuff2[nrf_uart_cnt++]=0xAA;
	SendBuff2[nrf_uart_cnt++]=0xAF;
	SendBuff2[nrf_uart_cnt++]=0x10;//功能字
	SendBuff2[nrf_uart_cnt++]=0;//数据量

	_temp = (vs16)(imu_fushion.Acc.x);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(imu_fushion.Acc.y);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(imu_fushion.Acc.z);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( imu_fushion.Gyro.x);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( imu_fushion.Gyro.y);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( imu_fushion.Gyro.z);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( imu_fushion.Mag_Val.x);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( imu_fushion.Mag_Val.y);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( imu_fushion.Mag_Val.z);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	#if USE_ANO_FLOW
	_temp = ano_flow.h*1000;
	#else
	_temp = ultra_distance;
	#endif
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	#if USE_M100_IMU
	_temp32 = (vs32)(m100.H*1000);//ultra_distance;
	#else
	_temp32 = (vs32)(baroAlt);//ultra_distance;
	#endif
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);

	
	_temp=  (vs16)( flow_matlab_data[0]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=  (vs16)( flow_matlab_data[1]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=  (vs16)( flow_matlab_data[2]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=  (vs16)( flow_matlab_data[3]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);

	_temp=  (vs16)( baro_matlab_data[1]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
  _temp=  m100.connect&&m100.m100_data_refresh&&(m100.GPS_STATUS>=3);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);

	SendBuff2[cnt_reg+3] = nrf_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< nrf_uart_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[nrf_uart_cnt++] = sum;
	break;//------------
	
	case SEND_IMU_ATT:
	cnt_reg=nrf_uart_cnt;	
	SendBuff2[nrf_uart_cnt++]=0xAA;
	SendBuff2[nrf_uart_cnt++]=0xAF;
	SendBuff2[nrf_uart_cnt++]=0x11;//功能字
	SendBuff2[nrf_uart_cnt++]=0;//数据量
  #if YAW_USE_MAD_BUT_ATT_USE_EKF
	_temp = (vs16)(PitchRm*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(RollRm*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(YawRm*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	#else
	_temp = (vs16)(Pitch*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(Roll*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	#endif
	_temp = (vs16)( ref_q[0]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ref_q[1]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q[2]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q[3]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(Pitch_mid_down*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(Roll_mid_down*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw_mid_down*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( ref_q_imd_down[0]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ref_q_imd_down[1]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q_imd_down[2]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q_imd_down[3]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);

	
	SendBuff2[cnt_reg+3] = nrf_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< nrf_uart_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[nrf_uart_cnt++] = sum;
	break;//---------
	
	case SEND_IMU_FLOW:
	cnt_reg=nrf_uart_cnt;
  SendBuff2[nrf_uart_cnt++]=0xAA;
	SendBuff2[nrf_uart_cnt++]=0xAF;
	SendBuff2[nrf_uart_cnt++]=0x12;//功能字
	SendBuff2[nrf_uart_cnt++]=0;//数据量
	
	_temp = (vs16) (flow.rate);//ukf autoquad
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( imu_nav.flow.speed.x*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp =  (vs16)(imu_nav.flow.speed.y*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16) (flow_ground_temp[0]*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16) (flow_ground_temp[1]*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( FLOW_POS_X*100);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16) (FLOW_POS_Y*100);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	//-------------------
	if(pi_flow.insert&&module.sonar==0)
	_temp = (vs16)(	pi_flow.spdz*1000);
	else
	_temp = (vs16)(ALT_VEL_SONAR*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	#if SONAR_USE_FLOW
	sys.sonar=1;
	#endif
	#if SENSOR_FORM_PI_FLOW_SONAR_NOT
	if(sys.sonar)
	_temp = (vs16)(ALT_POS_SONAR3*1000);//navUkfData.posE[0]*1000;
	else
	_temp= 9000;
	#else
	if(pi_flow.insert&&module.sonar==0)
	_temp = (vs16)(pi_flow.z_o*1000);	
	else if(sys.sonar)
	_temp = (vs16)(ALT_POS_SONAR3*1000);//navUkfData.posE[0]*1000;
	else
	_temp= 9000;
	#endif
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ALT_VEL_BMP_EKF*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 = (vs32) (ALT_POS_BMP_EKF*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
  _temp =  (vs16)  (UKF_VELD_F*1000);//  (ALT_VEL_BMP_UNION*1000);//(ALT_VEL_BMP_UNION*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 = (vs32) (UKF_POSD*1000);//(ALT_POS_BMP_EKF*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	//-------------oldx_ukf flow qr fushion
	if(pi_flow.insert&&gpsx.pvt.PVT_fixtype==0&&module.flow==0)
	_temp = (vs16)(	pi_flow.x*1000);
	else
	_temp = (vs16)( X_ukf_Pos[0]*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	if(pi_flow.insert&&gpsx.pvt.PVT_fixtype==0&&module.flow==0)
	_temp = (vs16)(	pi_flow.y*1000);
	else
	_temp = (vs16) (X_ukf_Pos[1]*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	//spd-----------------------------------------
	if(pi_flow.insert&&gpsx.pvt.PVT_fixtype==0&&module.flow==0)
	_temp = (vs16)(	pi_flow.spdx*1000);
	else
	_temp = (vs16)( X_ukf[1]*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	if(pi_flow.insert&&gpsx.pvt.PVT_fixtype==0&&module.flow==0)
	_temp = (vs16)(	pi_flow.spdy*1000);
	else
	_temp = (vs16) (X_ukf[4]*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp = (vs16) (X_ukf_baro[3]*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16) (X_ukf_baro[4]*1000);//navUkfData.posE[0]*1000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
 	SendBuff2[cnt_reg+3] = nrf_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< nrf_uart_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[nrf_uart_cnt++] = sum;
	break;//---------

	
	case SEND_IMU_GPS:
	cnt_reg=nrf_uart_cnt;
	SendBuff2[nrf_uart_cnt++]=0xAA;
	SendBuff2[nrf_uart_cnt++]=0xAF;
	SendBuff2[nrf_uart_cnt++]=0x14;//功能字
	SendBuff2[nrf_uart_cnt++]=0;//数据量
 
	_temp = (vs32)(local_Lon);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((local_Lon-(int)(local_Lon))*1000000000);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	
	_temp = (vs32)(local_Lat);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((local_Lat-(int)(local_Lat))*1000000000);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);

  _temp32 = (vs32)(r1);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	_temp32 = (vs32)(r2);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
  
	 //origin
	_temp = (gpsx.pvt.PVT_fixtype);// gpsx.gpssta);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (gpsx.pvt.PVT_numsv);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)( gpsx.pvt.PVT_longitude*10000000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	_temp32 = (vs32)( gpsx.pvt.PVT_latitude*10000000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	_temp = (vs16)( gpsx.pvt.PVT_speed*100);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)( gpsx.angle*10);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);

	
	SendBuff2[cnt_reg+3] = nrf_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< nrf_uart_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[nrf_uart_cnt++] = sum;
	break;//---------
	
	case SEND_ALL://<-----------------------------------------------USING-------------------------------------------
	cnt_reg=nrf_uart_cnt;
	SendBuff2[nrf_uart_cnt++]=0xAA;
	SendBuff2[nrf_uart_cnt++]=0xAF;
	SendBuff2[nrf_uart_cnt++]=0x88;//功能字
	SendBuff2[nrf_uart_cnt++]=0;//数据量
  #if YAW_USE_MAD_BUT_ATT_USE_EKF
		_temp = (vs16)(PitchRm*10);
		SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
		SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
		_temp = (vs16)(RollRm*10);
		SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
		SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
		_temp = (vs16)(YawRm*10);
		SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
		SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	#else
		_temp = (vs16)(Pitch*10);
		SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
		SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
		_temp = (vs16)(Roll*10);
		SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
		SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
		_temp = (vs16)(Yaw*10);
		SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
		SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	#endif
	//-------------------
	_temp = (vs16)(Gps_information.local_spd_flt[Zr]*1000);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	#if SONAR_USE_FLOW
	sys.sonar=1;
	#endif
  if(sys.sonar)
	_temp = (vs16)(ALT_POS_SONAR2*1000);
	else
	_temp= 9000;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	//------------z fusion
  _temp =  (vs16)  (X_kf_baro[1]*1000);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 =(vs32) (X_kf_baro[0]*1000);
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	//-------------x y fushion
//	X_ukf_Pos[0]=12345.12345;
//	X_ukf_Pos[1]=-12345.12345;
	_temp32 = (vs32)( X_ukf_Pos[0]*1000);
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
  _temp32 = (vs32)( X_ukf_Pos[1]*1000);
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	//spd-----------------------------------------
	_temp = (vs16)( X_ukf[1]*1000);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16) (X_ukf[4]*1000);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);

	//--------------------	
	_temp = (vs16)ABS(local_Lon);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((ABS(local_Lon)-(int)ABS(local_Lon))*1000000000);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	
	_temp = (vs16)ABS(local_Lat);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((ABS(local_Lat)-(int)ABS(local_Lat))*1000000000);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);

  _temp32 = (vs32)(r1);
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	_temp32 = (vs32)(r2);
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
  
	 //origin
	if(gpsx.pvt.PVT_numsv>4&&Gps_information.connect&&Gps_information.kf_init)
	_temp = (gpsx.pvt.PVT_fixtype);
	else
	_temp = 99;	
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	if(gpsx.pvt.PVT_numsv>4&&Gps_information.connect)
	_temp = (gpsx.pvt.PVT_numsv);
	else
	_temp = (amf.quality);	
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
  //Car
  _temp = (vs16)(three_wheel_car.body_spd[0]*100);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(three_wheel_car.body_spd[1]*100);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(three_wheel_car.body_spd[2]*100);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(three_wheel_car.pos[0]*100);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(three_wheel_car.pos[1]*100);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
  //UWB Drone
	_temp = (vs16)(pos_f_uwb[0]*100*i_uwb_lps_tag.init);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(pos_f_uwb[1]*100*i_uwb_lps_tag.init);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw_uwb*100*i_uwb_lps_tag.init);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = navigaition_mode;
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	if(local_Lon>0)
	flag[0]=1;
  else
  flag[0]=0;		
	if(local_Lat>0)
	flag[1]=1;
  else
  flag[1]=0;		
	_temp=flag[0]<<1|flag[1];
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	SendBuff2[cnt_reg+3] = nrf_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< nrf_uart_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[nrf_uart_cnt++] = sum;
  break;	
	
	case SEND_SD:
	cnt_reg=nrf_uart_cnt;
	SendBuff2[nrf_uart_cnt++]=0xAA;
	SendBuff2[nrf_uart_cnt++]=0xAF;
	SendBuff2[nrf_uart_cnt++]=0x99;//功能字
	SendBuff2[nrf_uart_cnt++]=0;//数据量
  for(i=0;i<20;i++){
	_temp = (vs16)(sd_save[i]);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
  }

	
	SendBuff2[cnt_reg+3] = nrf_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< nrf_uart_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[nrf_uart_cnt++] = sum;
  break;	
	
	case SEND_PIX:
	cnt_reg=nrf_uart_cnt;
	SendBuff2[nrf_uart_cnt++]=0xAA;
	SendBuff2[nrf_uart_cnt++]=0xAF;
	SendBuff2[nrf_uart_cnt++]=0x04;//功能字
	SendBuff2[nrf_uart_cnt++]=0;//数据量
	
	_temp = (vs16)(m100.Pit*10);//Pitch;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rol*10);//Roll;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Yaw*10);//Yaw;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(m100.H*1000);//altitude;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(ultra.relative_height*10);//height velocity;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	float temp_pos;
	#if USE_M100_IMU
	temp_pos=GPS_W_F;
	#else
	temp_pos=m100.Lat;
	#endif
	_temp = (vs32)(temp_pos);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((temp_pos-(int)(temp_pos))*1000000000);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);
	#if USE_M100_IMU
	temp_pos=GPS_J_F;
	#else
	temp_pos=m100.Lon;
	#endif
	_temp = (vs32)(temp_pos);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((temp_pos-(int)(temp_pos))*1000000000);//latitude;
	SendBuff2[nrf_uart_cnt++]=BYTE3(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE2(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp32);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp32);

	_temp = (vs16)(m100.Bat);//Bat;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);

	_temp = (vs16)(m100.Rc_pit);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_rol);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_yaw);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_thr);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_mode);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Rc_gear);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(m100.STATUS&&m100.m100_data_refresh&&m100.connect);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.GPS_STATUS);//GPS_S;
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.spd[0]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.spd[1]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.spd[2]*1000);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(three_wheel_car.body_spd[0]);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(three_wheel_car.body_spd[1]);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(three_wheel_car.body_spd[2]);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	
	SendBuff2[cnt_reg+3] = nrf_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< nrf_uart_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[nrf_uart_cnt++] = sum;
	break;
	case SEND_QR:
	cnt_reg=nrf_uart_cnt;
	SendBuff2[nrf_uart_cnt++]=0xAA;
	SendBuff2[nrf_uart_cnt++]=0xAF;
	SendBuff2[nrf_uart_cnt++]=0x05;//功能字
	SendBuff2[nrf_uart_cnt++]=0;//数据量
  if(qr.connect==0)
	SendBuff2[nrf_uart_cnt++]=66;	
	else
	SendBuff2[nrf_uart_cnt++]=qr.check;
	_temp = (vs16)(qr.x);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(qr.y);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(qr.z);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(qr.pix_x);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(qr.pix_y);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);		
	_temp = (vs16)(qr.center_x);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(qr.center_y);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);	
	_temp = (vs16)(qr.yaw);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	SendBuff2[cnt_reg+3] = nrf_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< nrf_uart_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[nrf_uart_cnt++] = sum;
	break;
	case SEND_WIFI:
	cnt_reg=nrf_uart_cnt;
	SendBuff2[nrf_uart_cnt++]=0xAA;
	SendBuff2[nrf_uart_cnt++]=0xAF;
	SendBuff2[nrf_uart_cnt++]=0x98;//功能字
	SendBuff2[nrf_uart_cnt++]=0;//数据量
	SendBuff2[nrf_uart_cnt++]=wifiCtrl.connect;
  _temp = (vs16)(wifiCtrl.pitch);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(wifiCtrl.roll);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(wifiCtrl.thrust);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (vs16)(wifiCtrl.yaw);
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	SendBuff2[cnt_reg+3] = nrf_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< nrf_uart_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[nrf_uart_cnt++] = sum;
	break;
	case SEND_IMU_DEBUG:
	cnt_reg=nrf_uart_cnt;
	SendBuff2[nrf_uart_cnt++]=0xAA;
	SendBuff2[nrf_uart_cnt++]=0xAF;
	SendBuff2[nrf_uart_cnt++]=0x01;//功能字
	SendBuff2[nrf_uart_cnt++]=0;//数据量
  
	if(debug_pi_flow[0])
	_temp =  1;	
	else	
	_temp =  en_ble_debug;
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[1];	
	else	
	_temp =  DEBUG_BUF[0];
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[2];	
	else	
	_temp =  DEBUG_BUF[1];
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[3];	
	else	
	_temp =  DEBUG_BUF[2];
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[4];	
	else	
	_temp =  DEBUG_BUF[3];
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[5];	
	else	
	_temp =  DEBUG_BUF[4];
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[6];	
	else	
	_temp =  DEBUG_BUF[5];
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[7];	
	else	
	_temp =  DEBUG_BUF[6];
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[8];	
	else	
	_temp =  DEBUG_BUF[7];
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	if(debug_pi_flow[0])
	_temp =  debug_pi_flow[9];	
	else	
	_temp =  DEBUG_BUF[8];//
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=  ( pi_flow.sensor.x);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=  ( pi_flow.sensor.y);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=  ( pi_flow.sensor.z);//ultra_distance;
	SendBuff2[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff2[nrf_uart_cnt++]=BYTE0(_temp);
	
	SendBuff2[nrf_uart_cnt++]=module.sonar;
	SendBuff2[nrf_uart_cnt++]=module.gps;
	SendBuff2[nrf_uart_cnt++]=module.flow||module.flow_iic;
	SendBuff2[nrf_uart_cnt++]=module.laser;
	SendBuff2[nrf_uart_cnt++]=module.pi_flow;
	SendBuff2[nrf_uart_cnt++]=module.acc;
	SendBuff2[nrf_uart_cnt++]=module.gyro;
	if(lis3mdl.Mag_CALIBRATED)
	SendBuff2[nrf_uart_cnt++]=99;	
	else
	SendBuff2[nrf_uart_cnt++]=module.hml;
	SendBuff2[3] = nrf_uart_cnt-4;

	for( i=0;i<nrf_uart_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[nrf_uart_cnt++] = sum;
	break;
	default:break;
}
}

void GOL_LINK_TASK_DMA(void)//5ms
{
static u8 cnt[10];
static u8 flag[10]={0};
if(m100.connect)
flag[0]=1;	

if(cnt[4]++>2){cnt[4]=0;
 data_per_uart4(SEND_IMU_DEBUG);
}

if(flag[0]){
	data_per_uart4(SEND_PIX);	
	if(cnt[1]++>9){cnt[1]=0;
	data_per_uart4(SEND_QR);
	}		
}	
else{
	data_per_uart4(SEND_ALL);	
	if(cnt[1]++>9){cnt[1]=0;
	sd_save_publish();	
	data_per_uart4(SEND_SD);
	}	
}

#if defined(USE_WIFI_CONTROL)
if(cnt[2]++>3){cnt[3]=0;
data_per_uart4(SEND_WIFI);
}	
#endif

}

void clear_nrf_uart(void)
{u16 i;
nrf_uart_cnt=0;
for(i=0;i<SEND_BUF_SIZE2;i++)
SendBuff2[i]=0;

}

void Uart5_Init(u32 br_num)//-----video sonar Flow
{
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	//配置PC12作为UART5　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD2作为UART5　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置UART5
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(UART5, &USART_InitStructure);
	


	//使能UART5接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//使能USART5
	USART_Cmd(UART5, ENABLE); 
}


_AMF amf;
//数据解析
void Data_Receive_Anl_FLOW(u8 *data_buf,u8 num)
{
	u8 i;
	u8 sum = 0;
	for(i=0;i<(num-1);i++)
	{
		sum += *(data_buf+i);
	}
	
	if(!(sum==*(data_buf+num-1)))		
	{	
		return;		//验证sum
	}
	
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		
	{	
		return;		//验证帧头
	}
	
	//数据
	if(*(data_buf+2)==0X01)
	{ module.flow_iic=1;
		//flow_rad.frame_count = (*(data_buf+4)<<8)|*(data_buf+5);
		flow_rad.integrated_x = (int16_t)((*(data_buf+6)<<8)|*(data_buf+7));
		flow_rad.integrated_y = (int16_t)((*(data_buf+8)<<8)|*(data_buf+9));
		flow_rad.integrated_xgyro = (int16_t)((*(data_buf+10)<<8)|*(data_buf+11));
		flow_rad.integrated_ygyro = (int16_t)((*(data_buf+12)<<8)|*(data_buf+13));
		flow_rad.integrated_zgyro = (int16_t)((*(data_buf+14)<<8)|*(data_buf+15));
		flow_rad.integration_time_us = (*(data_buf+16)<<24)|(*(data_buf+17)<<16)|(*(data_buf+18)<<8)|*(data_buf+19);
		//flow_rad. = (*(data_buf+20)<<24)|(*(data_buf+21)<<16)|(*(data_buf+22)<<8)|*(data_buf+23);
		flow_rad.distance = (*(data_buf+24)<<8)|*(data_buf+25);
		flow_rad.temperature = (*(data_buf+26)<<8)|*(data_buf+27);
		flow_rad.quality = *(data_buf+28);
	}
	if(*(data_buf+2)==0X69)
	{ amf.connect=1;
		amf.loss_cnt=0;
		amf.yaw_off_set=90;//模块安装角度偏差
		flow_update=1;
		module.flow=1;
		module.sonar=sys.sonar=ultra_ok = 1;
		amf.att[0] =(float)(int16_t)((*(data_buf+4)<<8)|*(data_buf+5))/10.;
		amf.att[1] =(float)(int16_t)((*(data_buf+6)<<8)|*(data_buf+7))/10.;
		amf.att[2] =(float)(int16_t)((*(data_buf+8)<<8)|*(data_buf+9))/10.;
		amf.att[2] = To_180_degrees(amf.att[2]-amf.yaw_off_set);
		amf.spd[0] =(float)(int16_t)((*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		amf.spd[1] =(float)(int16_t)((*(data_buf+12)<<8)|*(data_buf+13))/1000.;
		amf.spd[2] =(float)(int16_t)((*(data_buf+14)<<8)|*(data_buf+15))/1000.;
		
		amf.pos[0] =(float)(int16_t)((*(data_buf+16)<<8)|*(data_buf+17))/100.;
		amf.pos[1] =(float)(int16_t)((*(data_buf+18)<<8)|*(data_buf+19))/100.;
		amf.pos[2] =(float)(int16_t)((*(data_buf+20)<<8)|*(data_buf+21))/100.;
		
		amf.spd_o[0] =(float)(int16_t)((*(data_buf+22)<<8)|*(data_buf+23))/1000.;
		amf.spd_o[1] =(float)(int16_t)((*(data_buf+24)<<8)|*(data_buf+25))/1000.;
		amf.pos_o[2] =(float)(int16_t)((*(data_buf+26)<<8)|*(data_buf+27))/100.;
		amf.pos_o[3] =(float)(int16_t)((*(data_buf+28)<<8)|*(data_buf+29))/100.;
		
		amf.maker[0] =(float)(int16_t)((*(data_buf+30)<<8)|*(data_buf+31))/1.;//x
		amf.maker[1] =(float)(int16_t)((*(data_buf+32)<<8)|*(data_buf+33))/1.;//y
		amf.maker[2] =(float)(int16_t)((*(data_buf+34)<<8)|*(data_buf+35))/1.;//z
		amf.maker[3] =(float)(int16_t)((*(data_buf+36)<<8)|*(data_buf+37))/1.;//yaw
		amf.mems_state=*(data_buf+38);
		amf.amf_reset=*(data_buf+39);
		amf.quality=*(data_buf+40);
		amf.strong=*(data_buf+41);
		amf.spd_body[0]=(amf.spd[0]*cos(amf.yaw_off_set*0.0173)+amf.spd[1]*sin(amf.yaw_off_set*0.0173));//x
	  amf.spd_body[1]=(amf.spd[1]*cos(amf.yaw_off_set*0.0173)-amf.spd[0]*sin(amf.yaw_off_set*0.0173));//y
    amf.spd_body[2]=amf.spd[2];//z
		
	}
}

void RX_FLOW_OPENMV(u8 in)
{
 static u8 state;
 static u8 cnt;
 static u8 buf[30];
 switch(state)
 {
	 case 0:
		if(in==0xBA)
		state=1;
		break;	  
	case 1:
		if(in==0xAF)
		{state=2;cnt=0;}
		else
		state=0;
	 break;	  
  case 2:
		 if(in==0x0D&&cnt==6)
		   {
				flow_5a.flow_x_integral=(int16_t)((*(buf+0)<<8)|*(buf+1))+flow_5a.offx;
				flow_5a.flow_y_integral=(int16_t)((*(buf+2)<<8)|*(buf+3))+flow_5a.offy;
				flow_5a.quality=(int16_t)((*(buf+4)<<8)|*(buf+5));
				flow_5a.integration_timespan=0;
				flow_5a.ground_distance=0;
				flow_5a.version=0;
				flow_update=1;
				state=0;	
			 }
		 else if(cnt>7)
			 state=0;
		 else
			 buf[cnt++]=in;
   break;
 }	 
}	


void UART5_IRQHandler(void)
{
static u8 RxBuffer[50];
static u8 _data_len = 0,_data_cnt = 0;
static u8 state = 0;
	
	u8 com_data;
 OSIntEnter();    
  //接收中断
		if(UART5->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART5->DR;
	}
	
	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//清除中断标志

		com_data = UART5->DR;
		#if FLOW_USE_OPENMV
		RX_FLOW_OPENMV(com_data);
		#endif
		#if USE_ANO_FLOW
		AnoOF_GetOneByte(com_data);
		#endif
		if(state==0&&com_data==0xAA)
		{
			state=1;
			RxBuffer[0]=com_data;
		}
		else if(state==1&&com_data==0xAF)
		{
			state=2;
			RxBuffer[1]=com_data;
		}
		else if(state==2&&com_data<0XF1)
		{
			state=3;
			RxBuffer[2]=com_data;
		}
		else if(state==3&&com_data<50)
		{
			state = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(state==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				state = 5;
		}
		else if(state==5)
		{
			state = 0;
			RxBuffer[4+_data_cnt]=com_data;
			Data_Receive_Anl_FLOW(RxBuffer,_data_cnt+5);
		}
		else
			state = 0;
	
		flow_uart_rx_oldx(com_data,&flow,&flow_rad);//Ultra_Get(com_data);
		
	}

     OSIntExit();    
}

void Uart5_Send(unsigned char *DataToSend ,u8 data_num)
{

while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, data_num); ;//USART1, ch); 

}



void Usart1_Init(u32 br_num)//-------GPS
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART1, ENABLE); 
   delay_ms(100);
   Ublox_PVT_Mode();

}

float dlf_odroid=0.5;
 void Data_Receive_Anl_odroid(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==0x01)//FLOW_MINE_frame
  {
	 imu_nav.flow.speed.x_f= dlf_odroid*(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))+(1-dlf_odroid)*imu_nav.flow.speed.x_f;///10.;//UKF
	 imu_nav.flow.speed.y_f= dlf_odroid*(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))+(1-dlf_odroid)*imu_nav.flow.speed.y_f;///10.; 
	}
}

u8 RxBuffer_u1[50];
u8 RxState_u1 = 0;
u8 RxBufferNum_u1 = 0;
u8 RxBufferCnt_u1 = 0;
u8 RxLen_u1 = 0;
static u8 _data_len_u1 = 0,_data_cnt_u1 = 0;
//GPS
#define TEST_GPS 0//<----------------------------------------------
u16 USART3_RX_STA=0;
//串口发送缓存区 	
__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//发送缓冲,最大USART3_MAX_SEND_LEN字节
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//接收缓冲,最大USART3_MAX_RECV_LEN个字节.
u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//串口1,发送缓存区

nmea_msg gpsx; 		
u8  buf_GPRMC[100]={'G','P','R','M','C',','};//GPS信息
u8  buf_GPRMCt[100]={"GPRMC,144326.00,A,5107.0017737,N,11402.3291611,W,0.080,323.3,210307,0.0,E,A*20"};//GPS信息
u8  buf_GPGGA[100]={'G','P','G','G','A',','};//GPS信息
u8  buf_GPGGAt[100]={"GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60"};//GPS信息
u8  buf_imu_dj[20];
float angle_imu_dj[3];
u8 gps_good=0;
u16 gps_loss_cnt=0;
void IMU_DJ_ANGLE(u8 data)
{
//2.1.2 请求GPRMC
//说明：请求输出时间、日期、定位及地面航向和地面速度
	static u8 state0,cnt_buf;
switch(state0){
			case 0:if(data=='R')
			state0=1;
			break;
			case 1:if(data=='M')
			{state0=2;cnt_buf=0;}
			else
			state0=0;
			break;
			case 2:if(data=='C')
			{state0=3;cnt_buf=0;}
			else
			state0=0;
			break;
			case 3:if(data==',')
			{state0=4;cnt_buf=0;}
			else
			state0=0;
			break;
			case 4:if(data=='*')
			{buf_GPRMC[6+cnt_buf++]=data;state0=5;}
			else if(cnt_buf>90)
			{cnt_buf=0;state0=0;}	
			else
			buf_GPRMC[6+cnt_buf++]=data;
			break;
			case 5:
				#if TEST_GPS
			NMEA_GPRMC_Analysis(&gpsx,buf_GPRMCt);	
			#else
			NMEA_GPRMC_Analysis(&gpsx,buf_GPRMC);	//GPRMC解析	
			#endif
			cnt_buf=0;state0=0;
			break;
		}
}

//GPS PVT
void Ublox_PVT_Mode(void)
{	 u8 TX_BUF[11],i;
			TX_BUF[0]=0xB5;
			TX_BUF[1]=0x62;
			TX_BUF[2]=0x06;
			TX_BUF[3]=0x01;
			TX_BUF[4]=0x03;
			TX_BUF[5]=0x00;
			TX_BUF[6]=0x01;
			TX_BUF[7]=0x07;
			TX_BUF[8]=0x01;
			TX_BUF[9]=0x13;
			TX_BUF[10]=0x51;
for(i=0;i<11;i++){	
while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, TX_BUF[i]); 
}
}

//GPS DOP
void Ublox_DOP_Mode(void)
{	 u8 TX_BUF[11],i;
			TX_BUF[0]=0xB5;
			TX_BUF[1]=0x62;
			TX_BUF[2]=0x06;
			TX_BUF[3]=0x01;
			TX_BUF[4]=0x03;
			TX_BUF[5]=0x00;
			TX_BUF[6]=0x01;
			TX_BUF[7]=0x04;//0x07;
			TX_BUF[8]=0x01;
			TX_BUF[9]=0x13;
			TX_BUF[10]=0x51;
for(i=0;i<11;i++){	
while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, TX_BUF[i]); 
}
}
float k_gps_dow_spd=2;
uint32_t gpsData_lastPosUpdate,gpsData_lastVelUpdate,gps_update[2];
u8 Gps_data_get_PVT(u8 in)
{
	static u8 buf[100],state;	
	static u16 cnt;
	u16 j;
	u8 i=0,sum_a=0,sum_b=0;
	switch(state)
	{
		case 0:
			if(in==0xB5)
			{
			 for(j=0;j<100;j++)
			 buf[j]=0;
			cnt=0;buf[cnt++]=in;state++;}
		break;
	  case 1:
			if(in==0x62)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
	  case 2:
			if(in==0x01)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
		case 3:
			if(in==0x07)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
		case 4:
			if(in==0x5c)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;	
		case 5:
			if(in==0x00)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;	
		case 6:
     if(cnt<100)
			 buf[cnt++]=in;
		 else 
			 {  
					for(i=2;i<98;i++)
					{
					sum_a+=buf[i];
					sum_b+=sum_a;
					}
			   if(sum_a==buf[98]&&sum_b==buf[99])//0.1~0.2delay
					{
					 module.gps=1;
					 gpsx.pvt.rx_cnt++;	
					 gps_update[0]=gps_update[1]=1;
					 gpsData_lastVelUpdate=gpsData_lastPosUpdate = micros() - GPS_LATENCY;	
					 gpsx.pvt.rx_dt=Get_Cycle_T(GET_T_PVT); 	
					 gpsx.pvt.PVT_fixtype=buf[20+6];
					 gpsx.pvt.PVT_numsv=buf[23+6];
					 gpsx.pvt.PVT_longitude=(double)(long)((((u32)buf[27+6])<<24)|((u32)buf[26+6])<<16|((u32)buf[25+6])<<8|((u32)buf[24+6]))/10000000.;
					 gpsx.pvt.PVT_latitude=(double)(long)((((u32)buf[31+6])<<24)|((u32)buf[30+6])<<16|((u32)buf[29+6])<<8|((u32)buf[28+6]))/10000000.;
					 static float off;
					 static u8 init_flag;
           float temp;					 
					 temp=(float)(int)((((u32)buf[39+6])<<24)|((u32)buf[38+6])<<16|((u32)buf[37+6])<<8|((u32)buf[36+6]))/1000.;						 
					 if(temp>0&&init_flag==0&&gpsx.pvt.PVT_fixtype>=1&&gpsx.pvt.PVT_numsv>=4)
					 {off=temp;init_flag=1;}	 
					 gpsx.pvt.PVT_height=temp-off;
					
					 gpsx.pvt.PVT_Hacc=((((u32)buf[43+6])<<24)|((u32)buf[42+6])<<16|((u32)buf[41+6])<<8|((u32)buf[40+6]));		
					 gpsx.pvt.PVT_Vacc=((((u32)buf[47+6])<<24)|((u32)buf[46+6])<<16|((u32)buf[45+6])<<8|((u32)buf[44+6]));		 
					 gpsx.pvt.PVT_Sacc=((((u32)buf[71+6])<<24)|((u32)buf[70+6])<<16|((u32)buf[69+6])<<8|((u32)buf[68+6]));		
					 gpsx.pvt.PVT_Headacc=((((u32)buf[75+6])<<24)|((u32)buf[74+6])<<16|((u32)buf[73+6])<<8|((u32)buf[72+6]))*1e-5;								 
					 
					 gpsx.pvt.PVT_North_speed= Moving_Median(28,3,(float)(int)((((u32)buf[51+6])<<24)|((u32)buf[50+6])<<16|((u32)buf[49+6])<<8|((u32)buf[48+6]))/1000.);
					 gpsx.pvt.PVT_East_speed= Moving_Median(29,3,(float)(int)((((u32)buf[55+6])<<24)|((u32)buf[54+6])<<16|((u32)buf[53+6])<<8|((u32)buf[52+6]))/1000.);
					 gpsx.pvt.PVT_Down_speed= (float)(int)((((u32)buf[59+6])<<24)|((u32)buf[58+6])<<16|((u32)buf[57+6])<<8|((u32)buf[56+6]))/1000.;
					 gpsx.pvt.PVT_speed=(float)(int)((((u32)buf[63+6])<<24)|((u32)buf[62+6])<<16|((u32)buf[61+6])<<8|((u32)buf[60+6]))/1000.;	

					 gpsx.pvt.headMot=(int)((((u32)buf[67+6])<<24)|((u32)buf[66+6])<<16|((u32)buf[65+6])<<8|((u32)buf[64+6]))*1e-5;
				   gpsx.pvt.headVeh=(int)((((u32)buf[87+6])<<24)|((u32)buf[86+6])<<16|((u32)buf[85+6])<<8|((u32)buf[84+6]))*1e-5;								 
					}
					state=0;
			 }
    break;		
	}
}


u8 Gps_data_get_DOP(u8 in)
{
	static u8 buf[18+8],state;	
	static u16 cnt;
	u16 j;
	u8 i=0,sum_a=0,sum_b=0;
	switch(state)
	{
		case 0:
			if(in==0xB5)
			{
			 for(j=0;j<18+8;j++)
			 buf[j]=0;
			cnt=0;buf[cnt++]=in;state++;}
		break;
	  case 1:
			if(in==0x62)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
	  case 2:
			if(in==0x01)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
		case 3:
			if(in==0x04)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
		case 4:
			if(in==18)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;	
		case 5:
			if(in==0x00)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;	
		case 6:
     if(cnt<18+8)
			 buf[cnt++]=in;
		 else 
			 {  
					for(i=2;i<18+8-2;i++)
					{
					sum_a+=buf[i];
					sum_b+=sum_a;
					}
			   if(sum_a==buf[18+8-2]&&sum_b==buf[18+8-1])//0.1~0.2delay
					{
					gpsx.pvt.gDOP=(((u32)buf[5+6])<<8|((u32)buf[4+6]));		
					gpsx.pvt.pDOP=(((u32)buf[7+6])<<8|((u32)buf[6+6]));	
					gpsx.pvt.tDOP=(((u32)buf[9+6])<<8|((u32)buf[8+6]));	
					gpsx.pvt.vDOP=(((u32)buf[11+6])<<8|((u32)buf[10+6]));	
					gpsx.pvt.hDOP=(((u32)buf[13+6])<<8|((u32)buf[12+6]));	
					gpsx.pvt.nDOP=(((u32)buf[15+6])<<8|((u32)buf[14+6]));	
					gpsx.pvt.eDOP=(((u32)buf[17+6])<<8|((u32)buf[16+6]));							
				  }
					state=0;
			 }
    break;		
	}
}


u8 Gps_data_get_POSLLH(u8 in)
{
	static u8 buf[28+8],state;	
	static u16 cnt;
	u16 j;
	u8 i=0,sum_a=0,sum_b=0;
	switch(state)
	{
		case 0:
			if(in==0xB5)
			{
			 for(j=0;j<28+8;j++)
			 buf[j]=0;
			cnt=0;buf[cnt++]=in;state++;}
		break;
	  case 1:
			if(in==0x62)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
	  case 2:
			if(in==0x01)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
		case 3:
			if(in==0x02)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
		case 4:
			if(in==28)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;	
		case 5:
			if(in==0x00)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;	
		case 6:
     if(cnt<28+8)
			 buf[cnt++]=in;
		 else 
			 {  
					for(i=2;i<28+8-2;i++)
					{
					sum_a+=buf[i];
					sum_b+=sum_a;
					}
			   if(sum_a==buf[28+8-2]&&sum_b==buf[28+8-1])//0.1~0.2delay
					{
					gpsx.ubm.gpsPosFlag=1;	
					gpsx.ubm.iTOW = ((((u32)buf[3+6])<<24)|((u32)buf[2+6])<<16|((u32)buf[1+6])<<8|((u32)buf[0+6]));	
					gpsx.ubm.lon = (double)(long)((((u32)buf[7+6])<<24)|((u32)buf[6+6])<<16|((u32)buf[5+6])<<8|((u32)buf[4+6]))/10000000.;
					gpsx.ubm.lat = (double)(long)((((u32)buf[11+6])<<24)|((u32)buf[10+6])<<16|((u32)buf[9+6])<<8|((u32)buf[8+6]))/10000000.;
					gpsx.ubm.height = (double)(long)((((u32)buf[19+6])<<24)|((u32)buf[18+6])<<16|((u32)buf[17+6])<<8|((u32)buf[16+6]))/1000.;    // mm => m
					gpsx.ubm.hAcc = (double)((((u32)buf[23+6])<<24)|((u32)buf[22+6])<<16|((u32)buf[21+6])<<8|((u32)buf[20+6]))/1000.;      // mm => m
					gpsx.ubm.vAcc = (double)((((u32)buf[27+6])<<24)|((u32)buf[26+6])<<16|((u32)buf[25+6])<<8|((u32)buf[24+6]))/1000.;      // mm => m				
          gpsx.ubm.lastPosUpdate = micros() - GPS_LATENCY; 						
				  }
					state=0;
			 }
    break;		
	}
}


u8 Gps_data_get_VALNED(u8 in)
{
	static u8 buf[36+8],state;	
	static u16 cnt;
	u16 j;
	u8 i=0,sum_a=0,sum_b=0;
	switch(state)
	{
		case 0:
			if(in==0xB5)
			{
			 for(j=0;j<36+8;j++)
			 buf[j]=0;
			cnt=0;buf[cnt++]=in;state++;}
		break;
	  case 1:
			if(in==0x62)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
	  case 2:
			if(in==0x01)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
		case 3:
			if(in==0x012)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;
		case 4:
			if(in==36)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;	
		case 5:
			if(in==0x00)
			{buf[cnt++]=in;state++;}
			else
			 state=0;	
		break;	
		case 6:
     if(cnt<36+8)
			 buf[cnt++]=in;
		 else 
			 {  
					for(i=2;i<36+8-2;i++)
					{
					sum_a+=buf[i];
					sum_b+=sum_a;
					}
			   if(sum_a==buf[36+8-2]&&sum_b==buf[36+8-1])//0.1~0.2delay
					{
					gpsx.ubm.gpsVelFlag=1;	
					gpsx.ubm.iTOW = ((((u32)buf[3+6])<<24)|((u32)buf[2+6])<<16|((u32)buf[1+6])<<8|((u32)buf[0+6]));	
					gpsx.ubm.velN = Moving_Median(30,3,(double)(long)((((u32)buf[7+6])<<24)|((u32)buf[6+6])<<16|((u32)buf[5+6])<<8|((u32)buf[4+6])) * 0.01f);           // cm => m
					gpsx.ubm.velE = Moving_Median(31,3,(double)(long)((((u32)buf[11+6])<<24)|((u32)buf[10+6])<<16|((u32)buf[9+6])<<8|((u32)buf[8+6])) * 0.01f);           // cm => m
					gpsx.ubm.velD = Moving_Median(32,3,(double)(long)((((u32)buf[15+6])<<24)|((u32)buf[14+6])<<16|((u32)buf[13+6])<<8|((u32)buf[12+6])) * 0.01f);           // cm => m
					gpsx.ubm.speed = (double)((((u32)buf[19+6])<<24)|((u32)buf[18+6])<<16|((u32)buf[17+6])<<8|((u32)buf[16+6])) * 0.01f;        // cm/s => m/s
					gpsx.ubm.heading = (double)(long)((((u32)buf[27+6])<<24)|((u32)buf[26+6])<<16|((u32)buf[25+6])<<8|((u32)buf[24+6])) * 1e-5f;
					gpsx.ubm.sAcc = (double)((((u32)buf[31+6])<<24)|((u32)buf[30+6])<<16|((u32)buf[29+6])<<8|((u32)buf[28+6])) * 0.01f;           // cm/s => m/s
					gpsx.ubm.cAcc = (double)((((u32)buf[35+6])<<24)|((u32)buf[34+6])<<16|((u32)buf[33+6])<<8|((u32)buf[32+6])) * 1e-5f;
					gpsx.ubm.lastVelUpdate = micros() - GPS_LATENCY; 						
				  }
					state=0;
			 }
    break;		
	}
}
///--
 void Data_Receive_Anl11(u8 *data_buf,u8 num)//rx px4
{ static u8 flag;
	double zen,xiao;
	static float m100_h_off;
	static float m100_hr,m100_attr[3];
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		
	{i=0;
		return;}		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))				//判断帧头
 {i=0;
		return;}
	if(*(data_buf+2)==0x01)//
  { 
		imu_feed_dog=FC_CONNECT=1;
		px4.loss_cnt=0;
		px4.connect=1;
	 	px4.rx_dt=Get_Cycle_T(GET_T_M100); 	
		flag=!flag;
	  px4.Pit=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10.;
		px4.Rol=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10.;
		px4.Yaw=To_180_degrees(1*(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.);
		
		px4.H=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		
		if(m100.H!=m100_hr||m100_attr[0]!=px4.Pit||m100_attr[1]!=px4.Rol||m100_attr[2]!=px4.Yaw)
		{px4.cnt_m100_data_refresh=0;
		 px4.m100_data_refresh=1;
		}
		m100_hr=px4.H;
		m100_attr[0]=px4.Pit;
		m100_attr[1]=px4.Rol;
		m100_attr[2]=px4.Yaw;
		
		px4.H_Spd=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/1000.;
		zen=(*(data_buf+14)<<8)|*(data_buf+15);
		xiao=(double)((u32)(*(data_buf+16)<<24)|(*(data_buf+17)<<16)|(*(data_buf+18)<<8)|*(data_buf+19))/1000000000.;
		px4.Lat=zen+xiao;
		zen=(*(data_buf+20)<<8)|*(data_buf+21);
		xiao=(double)((u32)(*(data_buf+22)<<24)|(*(data_buf+23)<<16)|(*(data_buf+24)<<8)|*(data_buf+25))/1000000000.;
		px4.Lon=zen+xiao;
		
		px4.Bat=(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27));
		px4.Rc_rol=(float)((int16_t)(*(data_buf+28)<<8)|*(data_buf+29));//rol
		px4.Rc_yaw=(float)((int16_t)(*(data_buf+30)<<8)|*(data_buf+31));//yaw
		px4.Rc_gear=(float)((int16_t)(*(data_buf+32)<<8)|*(data_buf+33));//gear
		px4.Rc_mode=(float)((int16_t)(*(data_buf+34)<<8)|*(data_buf+35));//mode
		px4.Rc_thr=(float)((int16_t)(*(data_buf+36)<<8)|*(data_buf+37));//thr
		px4.Rc_pit=(float)((int16_t)(*(data_buf+38)<<8)|*(data_buf+39));//pit
		px4.STATUS=*(data_buf+40);		
		if(px4.Lat!=0&&px4.Lon!=0)
		px4.GPS_STATUS=3;
    else		
		px4.GPS_STATUS=*(data_buf+41);
		px4.spd[0]=(float)((int16_t)(*(data_buf+42)<<8)|*(data_buf+43))/1000.;
		px4.spd[1]=(float)((int16_t)(*(data_buf+44)<<8)|*(data_buf+45))/1000.;
		px4.spd[2]=(float)((int16_t)(*(data_buf+46)<<8)|*(data_buf+47))/1000.;
	}
	else if(*(data_buf+2)==0x21)//Qr land
  {
	//m100.rx_dt=Get_Cycle_T(GET_T_M100); 	
	qr.connect=1;
	qr.loss_cnt=0;
	qr.check=(*(data_buf+4));///10.;
	qr.x=(int16_t)(*(data_buf+5)<<8)|*(data_buf+6);
	qr.y=(int16_t)(*(data_buf+7)<<8)|*(data_buf+8);
	qr.z=(int16_t)(*(data_buf+9)<<8)|*(data_buf+10);
	qr.pit=(int16_t)(*(data_buf+11)<<8)|*(data_buf+12);
	qr.rol=(int16_t)(*(data_buf+13)<<8)|*(data_buf+14);
	qr.yaw=(int16_t)(*(data_buf+15)<<8)|*(data_buf+16);
	
	if(qr.check){
		int temp=((int16_t)(*(data_buf+17)<<8)|*(data_buf+18));
		if(ABS(temp)<1000)
		qr.pix_x=temp;
		temp=((int16_t)(*(data_buf+19)<<8)|*(data_buf+20));
		if(ABS(temp)<1000)
		qr.pix_y=temp;	
	}
//	/*1  0
//	
//	  2  3*/
//	avoid_color[0]=*(data_buf+21);	
//	avoid_color[1]=*(data_buf+22);
//	avoid_color[2]=*(data_buf+23);
//	avoid_color[3]=*(data_buf+24);
	qr.center_x=-((int16_t)(*(data_buf+25)<<8)|*(data_buf+26));	
	qr.center_y=-((int16_t)(*(data_buf+27)<<8)|*(data_buf+28));
	}		
	else if(*(data_buf+2)==0x67)//UWB
  {
		dis_uwb[0]=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
		dis_uwb[1]=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
		dis_uwb[2]=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
		dis_uwb[3]=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
		
	}
  
}


 _CAR three_wheel_car;
float laser_nav[2];
 void Data_Receive_Anl_Outer_Linker(u8 *data_buf,u8 num)//rx px4
{ static u8 flag;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		
	{i=0;
		return;}		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))				//判断帧头
 {i=0;
		return;}
	if(*(data_buf+2)==0x78)//
  {
		three_wheel_car.encoder[0]=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
		three_wheel_car.encoder[1]=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
		three_wheel_car.encoder[2]=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
		three_wheel_car.body_spd[0]=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/100;
		three_wheel_car.body_spd[1]=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/100;
		three_wheel_car.body_spd[2]=(float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/100;
		three_wheel_car.pos[0]=(float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/100;
		three_wheel_car.pos[1]=(float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/100;
		three_wheel_car.pos[2]=(float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/100;
		
	}else if(*(data_buf+2)==0x06)//
  {
		laser_nav[0]=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/1000.;
		laser_nav[1]=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/1000.;
	}
  
}


void Send_TO_CAR(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x78;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = three_wheel_car.car_mode;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)((three_wheel_car.gain[0])*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)((three_wheel_car.gain[1])*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)((three_wheel_car.gain[2])*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)((three_wheel_car.car_cmd_spd[0])*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)((three_wheel_car.car_cmd_spd[1])*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)((three_wheel_car.car_cmd_spd[2])*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	

	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
  for(i=0;i<_cnt;i++)
    {
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, data_to_send[i]); 
		}
}


u8 RxBuffer11[50];
u8 RxState11 = 0;
u8 RxBufferNum11 = 0;
u8 RxBufferCnt11 = 0;
u8 RxLen11 = 0;
u16 _data_cnt11=0;
u16 _data_len11=0;
u8  buf_GNVTG[65];//GPS信息
void USART1_IRQHandler(void)//-------------------GPS
{ OSIntEnter(); 
	u8 com_data;
	static u16 cnt;
	static u8 state0,cnt_buf;
	if(USART1->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART1->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志

		com_data = USART1->DR;
		#if !USE_M100_IMU
		IMU_DJ_ANGLE(com_data);
		Gps_data_get_PVT(com_data);
		Gps_data_get_DOP(com_data);
		Gps_data_get_POSLLH(com_data);
		Gps_data_get_VALNED(com_data);
		#endif
		Uart1_GPS_IRQ(com_data);
		if(cnt++>50)
			cnt=0;
		else
		USART3_RX_BUF[cnt]=com_data;
		
		if(RxState11==0&&com_data==0xAA)
		{
			RxState11=1;
			RxBuffer11[0]=com_data;
		}
		else if(RxState11==1&&com_data==0xAF)
		{
			RxState11=2;
			RxBuffer11[1]=com_data;
		}
		else if(RxState11==2&&com_data>0&&com_data<0XF1)
		{
			RxState11=3;
			RxBuffer11[2]=com_data;
		}
		else if(RxState11==3&&com_data<50)
		{
			RxState11= 4;
			RxBuffer11[3]=com_data;
			_data_len11 = com_data;
			_data_cnt11= 0;
		}
		else if(RxState11==4&&_data_len11>0)
		{
			_data_len11--;
			RxBuffer11[4+_data_cnt11++]=com_data;
			if(_data_len11==0)
				RxState11 = 5;
		}
		else if(RxState11==5)
		{
			RxState11 = 0;
			RxBuffer11[4+_data_cnt11]=com_data;
			Data_Receive_Anl_Outer_Linker(RxBuffer11,_data_cnt11+5);
		}
		else
			RxState11 = 0;
		
	}

   OSIntExit(); 

}

void USART6_IRQHandler(void)//-------------------GPS
{ OSIntEnter(); 
	u8 com_data;
	static u16 cnt;
	static u8 state0,cnt_buf;
	if(USART6->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART6->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART6,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART6,USART_IT_RXNE);//清除中断标志
		com_data = USART6->DR;
		Uart6_GPS_IRQ(com_data);
	}
   OSIntExit(); 
}


void Usart4_Init(u32 br_num)//-------SD_board
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

	

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(UART4, ENABLE);  //使能串口1 
	
	USART_ClearFlag(UART4, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(UART4, ENABLE); 
}


u16 laser_buf[360];
void Laser_Get(u8 com_data)
{
	static u8 ultra_tmp,state;
	float temp1,temp2,temp;
	float dt;
  u16 angle;
	u16 dis;
  static u8 cnt,buf[9];
	u8 check,i;
  u16 Strength;
	
	switch(state)
	{
		case 0:
			if(com_data==0x59)
			{state=1;cnt=0;}
		break;
		case 1:
			if(com_data==0x59)
			{state=2;cnt=0;}
			else 
			 state=0;
		break	;
		case 2:
			 if(cnt<=9)
			  buf[cnt++]=com_data;
			 else
			 {state=0;
				
				 check=0x59+0x59+buf[0]+buf[1]+buf[2]+buf[3]+buf[4]+buf[5]+buf[6]+buf[7];
				 if(check==buf[8])
				 {
				
				 dis=(buf[1]<<8|buf[0]); 
				 Strength=buf[3]<<8|buf[2];
				 angle= buf[5]<<8|buf[4];
					 if(Strength>10)
					 { 
						 
						 if(angle%10==0)
						 {
						 laser_buf[angle/10]=dis;//cm					 
						 }
						
					 }
					 else  if(angle%10==0)
					 {
					 laser_buf[angle/10]=8888;//cm					 
					 }	 
				 } 
			 }
			 break;
	}
}

M100 m100,px4;
int debug_pi_flow[20];
struct _FLOW_PI pi_flow;
void Data_Receive_Anl4(u8 *data_buf,u8 num)
{ static u8 flag;
	double zen,xiao;
	float temp;
	static float m100_hr,m100_attr[3];
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==0x66)//PI_FLOW FUSION OUT
  { 
		module.pi_flow=1;
		pi_flow.loss_cnt=0;
		pi_flow.insert=1;
	
		pi_flow.x=(float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100.;
		pi_flow.y=(float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100.;
		pi_flow.z=(float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/1000.;
		
		temp=(float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		if(fabs(temp)<8)
		pi_flow.spdx=temp;
		temp=(float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000.;
		if(fabs(temp)<8)
		pi_flow.spdy=temp;
		temp=(float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/1000.;
		if(fabs(temp)<8)
		pi_flow.spdz=temp;
		
		pi_flow.pit=(float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100.;
		pi_flow.rol=(float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100.;
		pi_flow.yaw=(float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100.;

    pi_flow.connect=*(data_buf+22);
		pi_flow.check=*(data_buf+23);
		
		pi_flow.z_o=(float)((vs16)(*(data_buf+24)<<8)|*(data_buf+25))/1000.;
	}
	else  if(*(data_buf+2)==0x77)//PI_FLOW FUSION OUT
  { 
		pi_flow.sensor.update=1;
		//pi_flow.sensor.dt=Get_Cycle_T(TEST);	
		pi_flow.sensor.last_update = micros() - FLOW_PI_LATENCY;	
		pi_flow.sensor.spdx=(float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000.;
		pi_flow.sensor.spdy=(float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000.;
		pi_flow.sensor.qual=*(data_buf+8);
	
	}	
		else  if(*(data_buf+2)==0x88)//PI_FLOW FUSION OUT
  { 
		pi_flow.sensor.loss_cnt=0;
		pi_flow.sensor.connect=*(data_buf+4);
		pi_flow.sensor.check=*(data_buf+5);
		pi_flow.sensor.x=((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
		pi_flow.sensor.y=((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
		pi_flow.sensor.z=((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
	}	
	else  if(*(data_buf+2)==0x11)//PI_FLOW FUSION OUT
  { 
		debug_pi_flow[0]=((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
		debug_pi_flow[1]=((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
		debug_pi_flow[2]=((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
		debug_pi_flow[3]=((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		debug_pi_flow[4]=((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
		debug_pi_flow[5]=((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
		debug_pi_flow[6]=((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
		debug_pi_flow[7]=((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
		debug_pi_flow[8]=((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
		debug_pi_flow[9]=((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
		debug_pi_flow[10]=((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
		debug_pi_flow[11]=((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
		debug_pi_flow[12]=((vs16)(*(data_buf+28)<<8)|*(data_buf+29));
		debug_pi_flow[13]=((vs16)(*(data_buf+30)<<8)|*(data_buf+31));
		debug_pi_flow[14]=((vs16)(*(data_buf+32)<<8)|*(data_buf+33));
		debug_pi_flow[15]=((vs16)(*(data_buf+34)<<8)|*(data_buf+35));
		debug_pi_flow[16]=((vs16)(*(data_buf+36)<<8)|*(data_buf+37));
		debug_pi_flow[17]=((vs16)(*(data_buf+38)<<8)|*(data_buf+39));
		debug_pi_flow[18]=((vs16)(*(data_buf+40)<<8)|*(data_buf+41));
		debug_pi_flow[19]=((vs16)(*(data_buf+42)<<8)|*(data_buf+43));
	}
 	
}
 
u8 uwb_buf[126];
float dis_uwb[4];

int numtrans(int num)
{
      int dist;
      for (size_t i = num; i < num+8; i++)
            {
                if (uwb_buf[i]>'9')
                {
                    dist = dist*16+ (unsigned int)(uwb_buf[i]-'a'+10);
                }
                else
                {
                    dist= dist *16+ (unsigned int)(uwb_buf[i]-'0');
                }
            }
    return dist;
}    

i_uwb_lps_tag_struct i_uwb_lps_tag;

void Byte_to_Float(float *f,unsigned char byte[])  
{  
    FloatLongType fl;  
    fl.ldata=0;  
    fl.ldata=byte[3];  
    fl.ldata=(fl.ldata<<8)|byte[2];  
    fl.ldata=(fl.ldata<<8)|byte[1];  
    fl.ldata=(fl.ldata<<8)|byte[0];  
    *f=fl.fdata;  
}  

uint8 I_UWB_LPS_Tag_DateFrame0_Unpack(uint8 *data_frame)
{
	uint8 data_frame_sum = 0;//校验位
	uint8 i = 0;
	unsigned char buf[4];
	
	if((data_frame[0] == 0x55) && (data_frame[1] == 0x01))//帧头包头检测
	{
		for(i = 0;i < (128 - 1);i++)//最后字节为校验和
		{
			data_frame_sum += data_frame[i];
		}

		if(data_frame_sum == data_frame[128-1])//检验和检查
		{
			i_uwb_lps_tag.id = data_frame[2];
			
			i_uwb_lps_tag.position.x = (float)Byte32(sint32 ,data_frame[6],data_frame[5],data_frame[4], 0) / 256000.0f;
			i_uwb_lps_tag.position.y = (float)Byte32(sint32 ,data_frame[9],data_frame[8],data_frame[7], 0) / 256000.0f;
			i_uwb_lps_tag.position.z = (float)Byte32(sint32 ,data_frame[12],data_frame[11],data_frame[10], 0) / 256000.0f;
			
			i_uwb_lps_tag.velocity.x = (float)Byte32(sint32 ,data_frame[15],data_frame[14],data_frame[13], 0) / 2560000.0f;
			i_uwb_lps_tag.velocity.y = (float)Byte32(sint32 ,data_frame[18],data_frame[17],data_frame[16], 0) / 2560000.0f;
			i_uwb_lps_tag.velocity.z = (float)Byte32(sint32 ,data_frame[21],data_frame[20],data_frame[19], 0) / 2560000.0f;
			
		if((i_uwb_lps_tag.velocity_r.x!=i_uwb_lps_tag.velocity.x||i_uwb_lps_tag.velocity_r.y!=i_uwb_lps_tag.velocity.y)
				&&(i_uwb_lps_tag.position.x!=0||i_uwb_lps_tag.position.y!=0))
			i_uwb_lps_tag.state=1;
			else
			i_uwb_lps_tag.state=0;
			i_uwb_lps_tag.velocity_r.x=i_uwb_lps_tag.velocity.x;
			i_uwb_lps_tag.velocity_r.y=i_uwb_lps_tag.velocity.y;
			i_uwb_lps_tag.velocity_r.z=i_uwb_lps_tag.velocity.z;
			i_uwb_lps_tag.dis_buf[0] = (float)Byte32(sint32 ,data_frame[24],data_frame[23],data_frame[22], 0) / 256000.0f;
			i_uwb_lps_tag.dis_buf[1] = (float)Byte32(sint32 ,data_frame[27],data_frame[26],data_frame[25], 0) / 256000.0f;
			i_uwb_lps_tag.dis_buf[2] = (float)Byte32(sint32 ,data_frame[30],data_frame[29],data_frame[28], 0) / 256000.0f;
			i_uwb_lps_tag.dis_buf[3] = (float)Byte32(sint32 ,data_frame[33],data_frame[32],data_frame[31], 0) / 256000.0f;
			i_uwb_lps_tag.dis_buf[4] = (float)Byte32(sint32 ,data_frame[36],data_frame[35],data_frame[34], 0) / 256000.0f;
			i_uwb_lps_tag.dis_buf[5] = (float)Byte32(sint32 ,data_frame[39],data_frame[38],data_frame[37], 0) / 256000.0f;
			i_uwb_lps_tag.dis_buf[6] = (float)Byte32(sint32 ,data_frame[42],data_frame[41],data_frame[40], 0) / 256000.0f;
			i_uwb_lps_tag.dis_buf[7] = (float)Byte32(sint32 ,data_frame[45],data_frame[44],data_frame[43], 0) / 256000.0f;
			
			buf[0] = data_frame[46];buf[1] = data_frame[47];buf[2] = data_frame[48];buf[3] = data_frame[49];
			Byte_to_Float(&i_uwb_lps_tag.gyro.x,buf);
			
			buf[0] = data_frame[50];buf[1] = data_frame[51];buf[2] = data_frame[52];buf[3] = data_frame[53];
			Byte_to_Float(&i_uwb_lps_tag.gyro.y,buf);
			
			buf[0] = data_frame[54];buf[1] = data_frame[55];buf[2] = data_frame[56];buf[3] = data_frame[57];
			Byte_to_Float(&i_uwb_lps_tag.gyro.z,buf);
			
			buf[0] = data_frame[58];buf[1] = data_frame[59];buf[2] = data_frame[60];buf[3] = data_frame[61];
			Byte_to_Float(&i_uwb_lps_tag.acc.x,buf);
			
			buf[0] = data_frame[62];buf[1] = data_frame[63];buf[2] = data_frame[64];buf[3] = data_frame[65];
			Byte_to_Float(&i_uwb_lps_tag.acc.y,buf);
			
			buf[0] = data_frame[66];buf[1] = data_frame[67];buf[2] = data_frame[68];buf[3] = data_frame[69];
			Byte_to_Float(&i_uwb_lps_tag.acc.z,buf);
		
			i_uwb_lps_tag.angle.x = (float) (Byte16(sint16, data_frame[83],  data_frame[82])) / 100.0f;
			i_uwb_lps_tag.angle.y = (float) (Byte16(sint16, data_frame[85],  data_frame[84])) / 100.0f;
			i_uwb_lps_tag.angle.z = (float) (Byte16(sint16, data_frame[87],  data_frame[86])) / 100.0f;
			
			
			buf[0] = data_frame[88];buf[1] = data_frame[89];buf[2] = data_frame[90];buf[3] = data_frame[91];
			Byte_to_Float(&i_uwb_lps_tag.Q.q0,buf);
			
			buf[0] = data_frame[92];buf[1] = data_frame[93];buf[2] = data_frame[94];buf[3] = data_frame[95];
			Byte_to_Float(&i_uwb_lps_tag.Q.q1,buf);
			
			buf[0] = data_frame[96];buf[1] = data_frame[97];buf[2] = data_frame[98];buf[3] = data_frame[99];
			Byte_to_Float(&i_uwb_lps_tag.Q.q2,buf);
			
			buf[0] = data_frame[100];buf[1] = data_frame[101];buf[2] = data_frame[102];buf[3] = data_frame[103];
			Byte_to_Float(&i_uwb_lps_tag.Q.q3,buf);
			
			i_uwb_lps_tag.system_time = Byte32(uint32,data_frame[115], data_frame[114], data_frame[113],  data_frame[112]);
			
			i_uwb_lps_tag.sensor_status = data_frame[116];
			
			return 1;
		}
	}
	
	return 0;
}

void UWB_GET_INF(u8 data)
{
 static u8 state,cnt,data_frame[130];
 switch(state)
 {
	 case 0:
		   if(data==0x55){
	       state=1;
			   data_frame[0]=data;}
	 break;
	 case 1:
		   if(data==0x01)
			 {state=2;cnt=2;
				 data_frame[1]=data;}
			 else
				 state=0;
	 break;	 
	 case 2:
      data_frame[cnt++]=data;
      if(cnt>128-1)
			{
       I_UWB_LPS_Tag_DateFrame0_Unpack(data_frame);
			 state=0;
			}	
   break;	 
 }
}



u8 RxBuffer4_test[25],RxBuffer4_test_cnt;
u8 RxBuffer4[50];
u8 RxState4 = 0;
u8 RxBufferNum4 = 0;
u8 RxBufferCnt4 = 0;
u8 RxLen4 = 0;
static u8 _data_len4 = 0,_data_cnt4 = 0;
void UART4_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(UART4->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART4->DR;
	}

  //接收中断
	if( USART_GetITStatus(UART4,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);//清除中断标志

		com_data = UART4->DR;
		#if USE_IMU_BACK_IO_AS_SONAR
		Ultra_Get(com_data);
		#endif
		#if USE_UWB_AS_POS
		UWB_GET_INF(com_data);
		#endif
		#if USE_GPS_U4
		Uart1_GPS_IRQ(com_data);
		#endif
		wifiLinkTask(com_data);
		RxBuffer4_test[RxBuffer4_test_cnt++]=com_data;
		if(RxBuffer4_test_cnt>24)
			RxBuffer4_test_cnt=0;
		 Laser_Get(com_data);
		if(RxState4==0&&com_data==0xAA)
		{
			RxState4=1;
			RxBuffer4[0]=com_data;
		}
		else if(RxState4==1&&com_data==0xAF)
		{
			RxState4=2;
			RxBuffer4[1]=com_data;
		}
		else if(RxState4==2&&com_data>0&&com_data<0XF1)
		{
			RxState4=3;
			RxBuffer4[2]=com_data;
		}
		else if(RxState4==3&&com_data<50)
		{
			RxState4 = 4;
			RxBuffer4[3]=com_data;
			_data_len4 = com_data;
			_data_cnt4 = 0;
		}
		else if(RxState4==4&&_data_len4>0)
		{
			_data_len4--;
			RxBuffer4[4+_data_cnt4++]=com_data;
			if(_data_len4==0)
				RxState4 = 5;
		}
		else if(RxState4==5)
		{
			RxState4 = 0;
			RxBuffer4[4+_data_cnt4]=com_data;
			Data_Receive_Anl4(RxBuffer4,_data_cnt4+5);
			Data_Receive_Anl11(RxBuffer4,_data_cnt4+5);
		}
		else
			RxState4 = 0;
	
	}
	
   OSIntExit(); 
}

void UsartSend_M100(uint8_t ch)
{

while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
USART_SendData(UART4, ch);  
}


void UsartSend_SD(uint8_t ch)
{

while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
USART_SendData(UART4, ch); ;//USART1, ch); 
}

static void Send_Data_SD(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_SD(dataToSend[i]);
}


void Laser_start(void)
{

UsartSend_SD( 0x42); //USART1, ch); 
UsartSend_SD( 0x57); //USART1, ch); 
UsartSend_SD( 0x02); //USART1, ch); 
UsartSend_SD( 0x00); //USART1, ch); 
UsartSend_SD( 0x00); //USART1, ch); UsartSend_SD(UART4, 0x42); ;//USART1, ch); 
UsartSend_SD( 0x00); //USART1, ch); 
UsartSend_SD( 0x02); //USART1, ch); 
UsartSend_SD( 0x02); //USART1, ch); 
}

void Laser_stop(void)
{

UsartSend_SD( 0x42); //USART1, ch); 
UsartSend_SD( 0x57); //USART1, ch); 
UsartSend_SD( 0x02); //USART1, ch); 
UsartSend_SD( 0x00); //USART1, ch); 
UsartSend_SD( 0x00); //USART1, ch); UsartSend_SD(UART4, 0x42); ;//USART1, ch); 
UsartSend_SD( 0x00); //USART1, ch); 
UsartSend_SD( 0x01); //USART1, ch); 
UsartSend_SD( 0x02); //USART1, ch); 
}



void Send_TO_FLOW_PI(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x66;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = (int)(k_flow_devide*1000.);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(flow_module_offset_x*1000.);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(flow_module_offset_y*1000.);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(flow_module_set_yaw*10.);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	if(sys.sonar&&ALT_POS_SONAR3>0)
	_temp = (int)(ALT_POS_SONAR3*1000);	
	else
	_temp = (int)(0*1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int)((Yaw)*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

u8 en_px4_mapper;
void Send_TO_FLOW_NAV_GPS(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x99;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = (vs16)((Pitch)*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)((Roll)*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)((Yaw)*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs32)(GPS_J_F);//latitude;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((GPS_J_F-(int)(GPS_J_F))*1000000000);//latitude;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	
	_temp = (vs32)(GPS_W_F);//latitude;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)((GPS_W_F-(int)(GPS_W_F))*1000000000);//latitude;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	
  _temp = (vs16) ((X_kf_baro[0])*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  //_temp = en_px4_mapper;
  _temp = en_px4_mapper&&module.gps&&gpsx.pvt.PVT_numsv>=1&&gpsx.pvt.PVT_fixtype>=3;
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

void UART2_Put_Char(unsigned char DataToSend)
{
while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, DataToSend); 
}

//------------------------------------------------------GOL_LINK_SD----------------------------------------------------
void Send_IMU_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x05;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = Roll*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pitch*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Yaw*10	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[1]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[2]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[3]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
data_to_send[3] = _cnt-4;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Usart3_Init(u32 br_num)//-------SONAR
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

	
   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART3, ENABLE); 
}
#include "sonar_avoid.h"
float rate_gps_board;
 void Data_Receive_Anl3(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	if(*(data_buf+2)==0x01)//SONAR
  {
//	//rate_gps_board=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));///10.;
//	sonar_avoid[0]=((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
//	sonar_avoid[1]=((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
//	sonar_avoid[2]=((int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
//	sonar_avoid[3]=((int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
//	sonar_avoid[4]=((int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
//	sonar_avoid[5]=((int16_t)(*(data_buf+14)<<8)|*(data_buf+15));
//	sonar_avoid[6]=((int16_t)(*(data_buf+16)<<8)|*(data_buf+17));
//	sonar_avoid[7]=((int16_t)(*(data_buf+18)<<8)|*(data_buf+19));

	sys.avoid=1;
	}	
	else if(*(data_buf+2)==0x02)//SLAM_frame
  {
  imu_nav.gps.Y_UKF=((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
	imu_nav.gps.X_UKF=((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
	imu_nav.gps.Y_O=  ((int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
	imu_nav.gps.X_O=  ((int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
	imu_nav.gps.J=  ((int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
	imu_nav.gps.W=  ((int16_t)(*(data_buf+14)<<8)|*(data_buf+15));	
	sys.gps=1;	
	}			
}
 

u8 RxBuffer3[50];
u8 RxState3 = 0;
u8 RxBufferNum3 = 0;
u8 RxBufferCnt3 = 0;
u8 RxLen3 = 0;
static u8 _data_len3 = 0,_data_cnt3 = 0;
void USART3_IRQHandler(void)
{  OSIntEnter();  
	u8 com_data;
	
	if(USART3->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART3->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志

		com_data = USART3->DR;
		Ultra_Get( com_data);

				if(RxState3==0&&com_data==0xAA)
		{
			RxState3=1;
			RxBuffer3[0]=com_data;
		}
		else if(RxState3==1&&com_data==0xAF)
		{
			RxState3=2;
			RxBuffer3[1]=com_data;
		}
		else if(RxState3==2&&com_data>0&&com_data<0XF1)
		{
			RxState3=3;
			RxBuffer3[2]=com_data;
		}
		else if(RxState3==3&&com_data<50)
		{
			RxState3 = 4;
			RxBuffer3[3]=com_data;
			_data_len3 = com_data;
			_data_cnt3 = 0;
		}
		else if(RxState3==4&&_data_len3>0)
		{
			_data_len3--;
			RxBuffer3[4+_data_cnt3++]=com_data;
			if(_data_len3==0)
				RxState3 = 5;
		}
		else if(RxState3==5)
		{
			RxState3 = 0;
			RxBuffer3[4+_data_cnt3]=com_data;
			Data_Receive_Anl3(RxBuffer3,_data_cnt3+5);
		}
		else
			RxState3 = 0;
	
	}

 OSIntExit();        
}

void UsartSend_GPS(uint8_t ch)
{

while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); ;//USART1, ch); 
}

static void Send_Data_GPS(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GPS(dataToSend[i]);
}

void UsartSend_GOL_LINK_NAV(uint8_t ch)
{
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); 
}

static void Send_Data_GOL_LINK_NAV(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK_NAV(dataToSend[i]);
}

void Send_IMU_NAV(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	
	_temp = q_nav[0]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK_NAV(data_to_send, _cnt);
}


u8 SendBuff1[SEND_BUF_SIZE1];	//发送数据缓冲区
void data_per_uart1(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
u16 i=0; 	
unsigned int temp=0xaF+9;

char ctemp;	
	
SendBuff1[i++]=0xa5;
SendBuff1[i++]=0x5a;
SendBuff1[i++]=14+8;
SendBuff1[i++]=0xA2;

if(ax<0)ax=32768-ax;
ctemp=ax>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=ax;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(ay<0)ay=32768-ay;
ctemp=ay>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=ay;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(az<0)az=32768-az;
ctemp=az>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=az;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(gx<0)gx=32768-gx;
ctemp=gx>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gx;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(gy<0)gy=32768-gy;
ctemp=gy>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gy;
SendBuff1[i++]=ctemp;
temp+=ctemp;
//-------------------------
if(gz<0)gz=32768-gz;
ctemp=gz>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gz;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hx<0)hx=32768-hx;
ctemp=hx>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hx;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hy<0)hy=32768-hy;
ctemp=hy>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hy;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hz<0)hz=32768-hz;
ctemp=hz>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hz;
SendBuff1[i++]=ctemp;
temp+=ctemp;

SendBuff1[i++]=temp%256;
SendBuff1[i++]=(0xaa);
//
 temp=0xaF+2+2;
SendBuff1[i++]=(0xa5);
SendBuff1[i++]=(0x5a);
SendBuff1[i++]=(14+4);
SendBuff1[i++]=(0xA1);


if(yaw<0)yaw=32768-yaw;
ctemp=yaw>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=yaw;							
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(pitch<0)pitch=32768-pitch;
ctemp=pitch>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=pitch;
SendBuff1[i++]=ctemp;
temp+=ctemp;
							 
if(roll<0)roll=32768-roll;
ctemp=roll>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=roll;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(alt<0)alt=32768-alt;
ctemp=alt>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;				
ctemp=alt;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(tempr<0)tempr=32768-tempr;
ctemp=tempr>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=tempr;
SendBuff1[i++]=ctemp;	   
temp+=ctemp;

if(press<0)press=32768-press;
ctemp=press>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=press;
SendBuff1[i++]=ctemp;
temp+=ctemp;

ctemp=IMUpersec>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=IMUpersec;
SendBuff1[i++]=ctemp;
temp+=ctemp;

SendBuff1[i++]=(temp%256);
SendBuff1[i++]=(0xaa);
}


u8 SendBuff2[SEND_BUF_SIZE2];	//发送数据缓冲区
void data_per_uart2(void)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
  SendBuff2[_cnt++]=0xAA;
	SendBuff2[_cnt++]=0xAF;
	SendBuff2[_cnt++]=0x01;//功能字
	SendBuff2[_cnt++]=0;//数据量
	
	_temp = ultra_distance;//
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);

	
	
	
	SendBuff2[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[_cnt++] = sum;
}

u8 en_ble_debug=0;
void GOL_LINK_BLE_DEBUG(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	SendBuff2[_cnt++]=0xAA;
	SendBuff2[_cnt++]=0xAF;
	SendBuff2[_cnt++]=0x12;//功能字
	SendBuff2[_cnt++]=0;//数据量

	_temp =  en_ble_debug;
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  ax;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  ay;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  az;//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  gx;//navUkfData.posE[0]*1000;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  gy;//navUkfData.posE[0]*1000;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  gz;//navUkfData.posE[0]*1000;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  hx;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  hy;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  hz;//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	
	SendBuff2[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[_cnt++] = sum;
}

u8 SendBuff3[SEND_BUF_SIZE3];	//发送数据缓冲区
void data_per_uart3(void)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
  SendBuff3[_cnt++]=0xAA;
	SendBuff3[_cnt++]=0xAF;
	SendBuff3[_cnt++]=0x01;//功能字
	SendBuff3[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);

	_temp = q_nav[0]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	
	SendBuff3[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff3[i];
	SendBuff3[_cnt++] = sum;
}


u8 SendBuff4[SEND_BUF_SIZE4];	//发送数据缓冲区

 FLOW flow;//flow 数据
 FLOW_RAD flow_rad;//陀螺仪数据


void UsartSend_M100_px4(uint8_t ch)
{

while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, ch);  
}

void m100_contrl_px4(float x,float y,float z,float yaw,u8 mode)
{
	vs16 _temp;
	UsartSend_M100_px4(0xFA);
	UsartSend_M100_px4(0xFB);
	UsartSend_M100_px4(0x04);
	UsartSend_M100_px4(0x01);

	UsartSend_M100_px4(mode);	
	_temp=x*1000;
	UsartSend_M100_px4(BYTE1(_temp));
	UsartSend_M100_px4(BYTE0(_temp));
	_temp=y*1000;
	UsartSend_M100_px4(BYTE1(_temp));
	UsartSend_M100_px4(BYTE0(_temp));
	_temp=z*1000;
	UsartSend_M100_px4(BYTE1(_temp));
	UsartSend_M100_px4(BYTE0(_temp));
	_temp=yaw*100;
	UsartSend_M100_px4(BYTE1(_temp));
	UsartSend_M100_px4(BYTE0(_temp));

	UsartSend_M100_px4(0xFE);	
}

