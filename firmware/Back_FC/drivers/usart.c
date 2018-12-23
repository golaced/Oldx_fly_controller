#include "alt_fushion.h"
#include "usart.h"
#include "data_transfer.h"
#include "ultrasonic.h"
#include "imu.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "rc.h"
#include "pos_ctrl.h"
#include "ctrl.h"
#include "height_ctrl.h"
#include "mymath.h"
#include "led.h"
#include "sbus.h"
#include "filter.h"
#include "mavl.h"
 _CAR three_wheel_car;
VSLAM vslam;

float Bytes2Float(unsigned char *bytes,int num)
{
    unsigned char cByte[24];
    int i;
    for (i=0;i<num;i++)
    {
	cByte[num-1-i] = bytes[i];
    }   
    float pfValue=*(float*)&cByte;
    return  pfValue;
}

void Float2Bytes(float pfValue,unsigned char* bytes)
{
  char* pchar=(char*)&pfValue;
  for(int i=0;i<sizeof(float);i++)
  {
    *bytes=*pchar;
     pchar++;
     bytes++;  
  }
}



///TX
void Uart5_Send(unsigned char *DataToSend ,u8 data_num)
{

while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
USART_SendData(UART5, data_num); ;//USART1, ch); 

}

void Uart3_Send(u8 data_num)
{
while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, data_num); ;//USART1, ch); 
}

void UsartSend_UP_LINK(uint8_t ch)
{

	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, ch); 
}

static void Send_Data_UP_LINK(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_UP_LINK(dataToSend[i]);
}


void UsartSend_APP(uint8_t ch)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, ch); 
}

static void Send_Data_APP(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_APP(dataToSend[i]);
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

void UART2_Put_Char(unsigned char DataToSend)
{

while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, DataToSend); 

}





//  INIT 
int16_t BLE_DEBUG[16];
M100 m100,px4;
void Usart2_Init(u32 br_num)//--GOL-link
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	#if USE_MINI_BOARD
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
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
	#else
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
  #endif
	

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART2, ENABLE); 
}

float dt_flow_rx;
u16 data_rate_gol_link;
PID_STA HPID,SPID,FIX_PID,NAV_PID;
struct SMART smart,smart_in;
RC_GETDATA Rc_Get;
RC_GETDATA Rc_Get_PWM,Rc_Get_SBUS;
struct _IMU_NAV imu_nav;

u8 LOCK, KEY[8],KEY_SEL[4];
struct _FLOW_DEBUG flow_debug;
u8 NAV_BOARD_CONNECT=0;
u32 imu_loss_cnt;
float flow_matlab_data[4],baro_matlab_data[2];
float POS_UKF_X,POS_UKF_Y,VEL_UKF_X,VEL_UKF_Y;
int k_scale_pix;
u8 pos_kf_state[3];
u8 m100_connect;
struct _FLOW_PI pi_flow;
int debug_pi_flow[20];
float car_pos[3];
float car_spd[3];
//float k_size_gps=2;//1.68;
float k_size_gps=1.68;
void Data_Receive_Anl(u8 *data_buf,u8 num)
{ double zen,xiao;
	vs16 rc_value_temp;
	float temp_pos[2];
	u8 sum = 0,flag;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	if(*(data_buf+2)==0x88)//ALL
  {
		imu_loss_cnt=0;
    NAV_BOARD_CONNECT=1;
		
		m100.Pit=Pitch=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10.;
    m100.Rol=Roll=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10.;
		m100.Yaw=Yaw=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.;
		
		ALT_VEL_BMP=(float)(int16_t)((*(data_buf+10)<<8)|*(data_buf+11))/1000.*k_size_gps;//m
		float temp=(float)(int16_t)((*(data_buf+12)<<8)|*(data_buf+13))/1000.;//m
		#if !SONAR_USE_FC&&!SONAR_USE_FC1
			if(temp<3){ultra.measure_ok=1;
			m100.H_G=ALT_POS_SONAR2 = temp;}
		#endif
		ALT_VEL_BMP_EKF=Moving_Median(0,0,(float)(int16_t)((*(data_buf+14)<<8)|*(data_buf+15))/1000.);//m
		ALT_POS_BMP_EKF=(float)(int32_t)((*(data_buf+16)<<24)|(*(data_buf+17)<<16)|(*(data_buf+18)<<8)|*(data_buf+19))/1000.;//m

		m100.X_Pos_local=POS_UKF_X=(float)(int32_t)((*(data_buf+20)<<24)|(*(data_buf+21)<<16)|(*(data_buf+22)<<8)|*(data_buf+23))/1000.;//m
		m100.Y_Pos_local=POS_UKF_Y=(float)(int32_t)((*(data_buf+24)<<24)|(*(data_buf+25)<<16)|(*(data_buf+26)<<8)|*(data_buf+27))/1000.;//m
		
		m100.X_Spd_b=VEL_UKF_X=Moving_Median(1,0,(float)(int16_t)((*(data_buf+28)<<8)|*(data_buf+29))/1000.);//m
		m100.Y_Spd_b=VEL_UKF_Y=Moving_Median(2,0,(float)(int16_t)((*(data_buf+30)<<8)|*(data_buf+31))/1000.);//m			
		#if USE_ANO_FLOW
		if(ALT_POS_SONAR2<2)
		{
		m100.Y_Spd_b=VEL_UKF_Y=ano_flow.spdy;
		m100.X_Spd_b=VEL_UKF_X=ano_flow.spdx;
		}
		#endif
    now_position[LON]=POS_UKF_X;//m  lon->0 X
		now_position[LAT]=POS_UKF_Y;//m  lat->1 Y			
		
		zen=(int16_t)((*(data_buf+32)<<8)|*(data_buf+33));
		xiao=(double)((u32)(*(data_buf+34)<<24)|(*(data_buf+35)<<16)|(*(data_buf+36)<<8)|*(data_buf+37))/1000000000.;
		m100.Init_Lon=zen+xiao;
		zen=(int16_t)((*(data_buf+38)<<8)|*(data_buf+39));
		xiao=(double)((u32)(*(data_buf+40)<<24)|(*(data_buf+41)<<16)|(*(data_buf+42)<<8)|*(data_buf+43))/1000000000.;
		m100.Init_Lat=zen+xiao;
		
		m100.r1=r1=((u32)(*(data_buf+44)<<24)|(*(data_buf+45)<<16)|(*(data_buf+46)<<8)|*(data_buf+47));
		m100.r2=r2=((u32)(*(data_buf+48)<<24)|(*(data_buf+49)<<16)|(*(data_buf+50)<<8)|*(data_buf+51));

		m100.STATUS=*(data_buf+52);
		m100.GPS_STATUS=*(data_buf+53);
	  three_wheel_car.body_spd[0]=car_spd[0]=(float)(int16_t)((*(data_buf+54)<<8)|*(data_buf+55))/100.;
		three_wheel_car.body_spd[1]=car_spd[1]=(float)(int16_t)((*(data_buf+56)<<8)|*(data_buf+57))/100.;
		three_wheel_car.body_spd[2]=car_spd[2]=(float)(int16_t)((*(data_buf+58)<<8)|*(data_buf+59))/100.;//m
		three_wheel_car.pos[0]=car_pos[0]=(float)(int16_t)((*(data_buf+60)<<8)|*(data_buf+61))/100.;
		three_wheel_car.pos[1]=car_pos[1]=(float)(int16_t)((*(data_buf+62)<<8)|*(data_buf+63))/100.;//m
		m100.uwb_f[0]=(float)(int16_t)((*(data_buf+64)<<8)|*(data_buf+65))/100.;//x
		m100.uwb_f[1]=(float)(int16_t)((*(data_buf+66)<<8)|*(data_buf+67))/100.;//y
		m100.uwb_f[3]=(float)(int16_t)((*(data_buf+68)<<8)|*(data_buf+69))/100.;//yaw
		m100.navigation_mode=*(data_buf+70);
		flag=*(data_buf+71);
//		if((flag>>1&0x00000001)==0)
//			m100.Init_Lon*=-1;
//		if((flag&0x00000001)==0)
//			m100.Init_Lat*=-1;
	}
  else if(*(data_buf+2)==0x01)//debug
  {
	flow_debug.en_ble_debug= *(data_buf+4);	
	flow_debug.ax=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
	flow_debug.ay=((int16_t)(*(data_buf+7)<<8)|*(data_buf+8));
	flow_debug.az=((int16_t)(*(data_buf+9)<<8)|*(data_buf+10));
	flow_debug.gx=((int16_t)(*(data_buf+11)<<8)|*(data_buf+12));
	flow_debug.gy=((int16_t)(*(data_buf+13)<<8)|*(data_buf+14));
	flow_debug.gz=((int16_t)(*(data_buf+15)<<8)|*(data_buf+16));
	flow_debug.hx=((int16_t)(*(data_buf+17)<<8)|*(data_buf+18));
	flow_debug.hy=((int16_t)(*(data_buf+19)<<8)|*(data_buf+20));
  flow_debug.hz=((int16_t)(*(data_buf+21)<<8)|*(data_buf+22));
	  #if USE_FLOW_PI
		circle.x=(int16_t)((*(data_buf+23)<<8)|*(data_buf+24));//m
		circle.y=(int16_t)((*(data_buf+25)<<8)|*(data_buf+26));//m
		circle.z=(int16_t)((*(data_buf+27)<<8)|*(data_buf+28));//m
		#endif
	#if defined(M_DRONE_330)
  module.sonar=1;
  #else		
	module.sonar=	*(data_buf+29);
  #endif
	module.gps=	*(data_buf+30);
	module.flow=	*(data_buf+31);
	module.laser=	*(data_buf+32);
  module.pi_flow=	*(data_buf+33);
	module.acc_imu=	*(data_buf+34);
	module.gyro_imu=	*(data_buf+35);
	module.hml_imu=	*(data_buf+36);
	}		
}
 
#define MAX_BUF_NUM_LINK 80
u8 RxBuffer[MAX_BUF_NUM_LINK];
u8 RxState = 0;
u8 RxBufferNum = 0;
u8 RxBufferCnt = 0;
u8 RxLen = 0;
u8 com_data ;
static u8 _data_len = 0,_data_cnt = 0;
void USART2_IRQHandler(void)
{ //OSIntEnter(); 
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
		else if(RxState==3&&com_data<MAX_BUF_NUM_LINK)//MAX_send num==50
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
		}
		else
			RxState = 0;

	}
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

void Send_IMU_TO_FLOW(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x81;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pit_fc*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Rol_fc*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw_fc1*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	#if USE_RECIVER_MINE
	_temp =  NS==0||Rc_Get_PWM.THROTTLE>1250||((Rc_Get.THROTTLE>1050))||fly_ready||ble_imu_force;
	#else
	_temp =  NS==0||Rc_Get_PWM.THROTTLE>1250||(mode_oldx.use_dji&&(Rc_Get.THROTTLE>1050))||fly_ready||ble_imu_force;
	#endif//
	data_to_send[_cnt++]=BYTE0(_temp);	
	
 	_temp = (vs16)(circle.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(circle.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(circle.z);//q_nav[0]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
 	_temp = (vs16)(circle.pit);//q_nav[1]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(circle.rol);// q_nav[2]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(circle.yaw);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
 	_temp = (vs16)(circle.spdx);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(circle.spdy);//mode_oldx.save_video;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(yaw_qr_off*10);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
  _temp =(vs16)( circle.connect);//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =(vs16)( circle.check);//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =(vs16)( circle.use_spd);//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	
	#if SONAR_USE_FC||SONAR_USE_FC1
	_temp =(vs16)(ALT_POS_SONAR2*1000);
	#else
	_temp=0;
	#endif
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp =(vs16)(baro.relative_height);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=acc_3d_step;
	data_to_send[_cnt++]=BYTE0(_temp);

  //car_cmd 
	_temp=three_wheel_car.car_mode;
	data_to_send[_cnt++]=BYTE0(_temp);
   _temp = (vs16)(three_wheel_car.gain[0]*100);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp =(vs16)(three_wheel_car.gain[1]*100);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp =(vs16)(three_wheel_car.gain[2]*100);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =(vs16)(three_wheel_car.car_cmd_spd[0]*100);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =(vs16)(three_wheel_car.car_cmd_spd[1]*100);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =(vs16)(three_wheel_car.car_cmd_spd[2]*100);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}


void Send_PID(void)
{ u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x82;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = SPID.OP;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = SPID.OI;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = SPID.OD;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = SPID.IP;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = SPID.II;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = SPID.ID;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = SPID.YP;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = SPID.YI;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = SPID.YD;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = HPID.OP;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = HPID.OI;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = HPID.OD;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp=nav_pos_ctrl[X].mode;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  mpu6050_fc.Acc_CALIBRATE;
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp =  mpu6050_fc.Gyro_CALIBRATE;
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp =  ak8975_fc.Mag_CALIBRATED;
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  

u8 feed_imu_dog=1;
struct IMU_PAR imu_board;
void Send_IMU_PARM(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x83;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = imu_board.k_flow_sel*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu_board.flow_module_offset_x*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu_board.flow_module_offset_y*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = imu_board.flow_set_yaw*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = mode_oldx.px4_map;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = feed_imu_dog;
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  

//send to imu board
void GOL_LINK_TASK(void)
{
static u8 cnt[4];

if(cnt[1]++>0||1)
{cnt[1]=0;
Send_IMU_TO_FLOW();
}
if(cnt[2]++>20)
{cnt[2]=0;
if(cnt[3]){cnt[3]=0;
Send_IMU_PARM();}
else{cnt[3]=1;
Send_PID();}
}
}

void Uart5_Init(u32 br_num)//-----odroid
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
	#if USE_MINI_FC_FLOW_BOARD
	USART_InitStructure.USART_BaudRate = br_num;//²¨ÌØÂÊÉèÖÃ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//×Ö³¤Îª8Î»Êý¾Ý¸ñÊ½
	USART_InitStructure.USART_StopBits = USART_StopBits_2;//Ò»¸öÍ£Ö¹Î»
	USART_InitStructure.USART_Parity = USART_Parity_Even;//ÎÞÆæÅ¼Ð£ÑéÎ»
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ÎÞÓ²¼þÊý¾ÝÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//ÊÕ·¢Ä£Ê½	
	#else
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	#endif
	USART_Init(UART5, &USART_InitStructure);
	
	//使能UART5接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//使能USART5
	USART_Cmd(UART5, ENABLE); 
}

struct ROBOT robot;
float mark_map[10][5];//x y z yaw id
u16 PWM_DJI[4]={0};
void Data_Receive_Anl5(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))
	{i=0;
		return;	}	//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		
		{i=0;
		return;	}			//判断帧头
  if(*(data_buf+2)==0x21)//QR
  {
	circle.connect=1;
	circle.lose_cnt=0;
	track.check=circle.check=(*(data_buf+4));///10.;
	circle.x=(int16_t)((*(data_buf+5)<<8)|*(data_buf+6));
	circle.y=(int16_t)((*(data_buf+7)<<8)|*(data_buf+8));
	circle.z=(int16_t)((*(data_buf+9)<<8)|*(data_buf+10));
	circle.pit=(int16_t)((*(data_buf+11)<<8)|*(data_buf+12));
	circle.rol=(int16_t)((*(data_buf+13)<<8)|*(data_buf+14));
	circle.yaw_off=yaw_off_qr;
	circle.yaw=To_180_degrees((int16_t)((*(data_buf+15)<<8)|*(data_buf+16))+circle.yaw_off);	
	circle.spdx=(int16_t)((*(data_buf+17)<<8)|*(data_buf+18));
	circle.spdy=(int16_t)((*(data_buf+19)<<8)|*(data_buf+20));
	//map	
	mark_map[0][0]=(int16_t)((*(data_buf+21)<<8)|*(data_buf+22));
	mark_map[0][1]=(int16_t)((*(data_buf+23)<<8)|*(data_buf+24));
	mark_map[0][2]=(int16_t)((*(data_buf+25)<<8)|*(data_buf+26));
	mark_map[0][3]=(int16_t)((*(data_buf+27)<<8)|*(data_buf+28));
	mark_map[0][4]=*(data_buf+29);
	mark_map[1][0]=(int16_t)((*(data_buf+30)<<8)|*(data_buf+31));
	mark_map[1][1]=(int16_t)((*(data_buf+32)<<8)|*(data_buf+33));
	mark_map[1][2]=(int16_t)((*(data_buf+34)<<8)|*(data_buf+35));
	mark_map[1][3]=(int16_t)((*(data_buf+36)<<8)|*(data_buf+37));
	mark_map[1][4]=*(data_buf+38);
	}	
	else if(*(data_buf+2)==0x02)//CIRCLE
  {
	circle.connect=1;
	circle.lose_cnt=0;
	track.check=circle.check=(*(data_buf+4));///10.;
	circle.x=(int16_t)((*(data_buf+5)<<8)|*(data_buf+6));
	circle.y=(int16_t)((*(data_buf+7)<<8)|*(data_buf+8));
	circle.r=(int16_t)((*(data_buf+9)<<8)|*(data_buf+10));
	}
	else if(*(data_buf+2)==0x22)//QR MAP2
	{
	mark_map[2][0]=(int16_t)((*(data_buf+4)<<8)|*(data_buf+5));
	mark_map[2][1]=(int16_t)((*(data_buf+6)<<8)|*(data_buf+7));
	mark_map[2][2]=(int16_t)((*(data_buf+8)<<8)|*(data_buf+9));
	mark_map[2][3]=(int16_t)((*(data_buf+10)<<8)|*(data_buf+11));
	mark_map[2][4]=*(data_buf+12);
	mark_map[3][0]=(int16_t)((*(data_buf+13)<<8)|*(data_buf+14));
	mark_map[3][1]=(int16_t)((*(data_buf+15)<<8)|*(data_buf+16));
	mark_map[3][2]=(int16_t)((*(data_buf+17)<<8)|*(data_buf+18));
	mark_map[3][3]=(int16_t)((*(data_buf+19)<<8)|*(data_buf+20));
	mark_map[3][4]=*(data_buf+21);
	mark_map[4][0]=(int16_t)((*(data_buf+22)<<8)|*(data_buf+23));
	mark_map[4][1]=(int16_t)((*(data_buf+24)<<8)|*(data_buf+25));
	mark_map[4][2]=(int16_t)((*(data_buf+26)<<8)|*(data_buf+27));
	mark_map[4][3]=(int16_t)((*(data_buf+28)<<8)|*(data_buf+29));
	mark_map[4][4]=*(data_buf+30);	
	mark_map[5][0]=(int16_t)((*(data_buf+31)<<8)|*(data_buf+32));
	mark_map[5][1]=(int16_t)((*(data_buf+33)<<8)|*(data_buf+34));
	mark_map[5][2]=(int16_t)((*(data_buf+35)<<8)|*(data_buf+36));
	mark_map[5][3]=(int16_t)((*(data_buf+37)<<8)|*(data_buf+38));
	mark_map[5][4]=*(data_buf+39);
	}
	else if(*(data_buf+2)==0x61)//move robot
	{
	robot.connect=1;
  robot.loss_cnt=0;		
	robot.track_x=(int16_t)((*(data_buf+4)<<8)|*(data_buf+5));
	robot.track_y=(int16_t)((*(data_buf+6)<<8)|*(data_buf+7));
	robot.track_r=(int16_t)((*(data_buf+8)<<8)|*(data_buf+9));
	robot.mark_check=(int16_t)((*(data_buf+10)<<8)|*(data_buf+11));
	robot.camera_x=(int16_t)((*(data_buf+12)<<8)|*(data_buf+13));
	robot.camera_y=(int16_t)((*(data_buf+14)<<8)|*(data_buf+15));
	robot.camera_z=(int16_t)((*(data_buf+16)<<8)|*(data_buf+17));
	robot.pit=(int16_t)((*(data_buf+18)<<8)|*(data_buf+19));
  robot.rol=(int16_t)((*(data_buf+20)<<8)|*(data_buf+21));
  robot.yaw=(int16_t)((*(data_buf+22)<<8)|*(data_buf+23));
  robot.mark_x=(int16_t)((*(data_buf+24)<<8)|*(data_buf+25));
  robot.mark_y=(int16_t)((*(data_buf+26)<<8)|*(data_buf+27));		
	robot.mark_r=(int16_t)((*(data_buf+28)<<8)|*(data_buf+29));		
	}	
	else if(*(data_buf+2)==0x62)//move robot map
	{
	robot.mark_map[0][0]=(int16_t)((*(data_buf+4)<<8)|*(data_buf+5));
	robot.mark_map[0][1]=(int16_t)((*(data_buf+6)<<8)|*(data_buf+7));
	robot.mark_map[0][2]=(int16_t)((*(data_buf+8)<<8)|*(data_buf+9));
	robot.mark_map[0][3]=(int16_t)((*(data_buf+10)<<8)|*(data_buf+11));
	robot.mark_map[0][4]=*(data_buf+12);
	robot.mark_map[1][0]=(int16_t)((*(data_buf+13)<<8)|*(data_buf+14));
	robot.mark_map[1][1]=(int16_t)((*(data_buf+15)<<8)|*(data_buf+16));
	robot.mark_map[1][2]=(int16_t)((*(data_buf+17)<<8)|*(data_buf+18));
	robot.mark_map[1][3]=(int16_t)((*(data_buf+19)<<8)|*(data_buf+20));
	robot.mark_map[1][4]=*(data_buf+21);
	robot.mark_map[2][0]=(int16_t)((*(data_buf+22)<<8)|*(data_buf+23));
	robot.mark_map[2][1]=(int16_t)((*(data_buf+24)<<8)|*(data_buf+25));
	robot.mark_map[2][2]=(int16_t)((*(data_buf+26)<<8)|*(data_buf+27));
	robot.mark_map[2][3]=(int16_t)((*(data_buf+28)<<8)|*(data_buf+29));
	robot.mark_map[2][4]=*(data_buf+30);	
	robot.mark_map[3][0]=(int16_t)((*(data_buf+31)<<8)|*(data_buf+32));
	robot.mark_map[3][1]=(int16_t)((*(data_buf+33)<<8)|*(data_buf+34));
	robot.mark_map[3][2]=(int16_t)((*(data_buf+35)<<8)|*(data_buf+36));
	robot.mark_map[3][3]=(int16_t)((*(data_buf+37)<<8)|*(data_buf+38));
	robot.mark_map[3][4]=*(data_buf+39);
	}	
}

u8 RxBuffer5[50];
u8 RxState5 = 0;
u8 RxBufferNum5 = 0;
u8 RxBufferCnt5 = 0;
u8 RxLen5 = 0;
static u8 _data_len5 = 0,_data_cnt5 = 0;
u8 sbus_data_i_check[50];
void UART5_IRQHandler(void)
{ //OSIntEnter(); 
	u8 com_data,i;
	static u8 state,state1;
	static uint8_t byteCNT = 0,byteCNT1;

	static uint32_t lastTime = 0;
	uint32_t curTime;
	uint32_t interval = 0;
	if(UART5->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART5->DR;
	}

  //接收中断
	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//清除中断标志

		com_data = UART5->DR;
	
		#if USE_MINI_FC_FLOW_BOARD||USE_VER_3	
		
		Rc_Get_SBUS.lose_cnt=0;
		Rc_Get_SBUS.connect=1;
		oldx_sbus_rx(com_data);
		//for check input
		switch(state1)
		{
			case 0:
			if(com_data==0x0f)	
			{
			for(i=0;i<50;i++)
				sbus_data_i_check[i++]=0;
			  byteCNT1=0;sbus_data_i_check[byteCNT1++]=com_data;state1=1;}
				break;
		  case 1:
				if(byteCNT1>49||com_data==0x0f)
				{
				state1=0;
				byteCNT1=0;
				}else{		
				sbus_data_i_check[byteCNT1++]=com_data;   
		  	}
			  break;
		}
		
    if(channels[16]==500||channels[16]==503){Feed_Rc_Dog(2);
		Rc_Get_SBUS.update=1;Rc_Get_SBUS.lose_cnt_rx=0; }
		
		#else
		#if SONAR_USE_FC1
		Ultra_Get(com_data);
		#endif
				if(RxState5==0&&com_data==0xAA)
		{
			RxState5=1;
			RxBuffer5[0]=com_data;
		}
		else if(RxState5==1&&com_data==0xAF)
		{
			RxState5=2;
			RxBuffer5[1]=com_data;
		}
		else if(RxState5==2&&com_data>0&&com_data<0XF1)
		{
			RxState5=3;
			RxBuffer5[2]=com_data;
		}
		else if(RxState5==3&&com_data<50)
		{
			RxState5 = 4;
			RxBuffer5[3]=com_data;
			_data_len5 = com_data;
			_data_cnt5 = 0;
		}
		else if(RxState5==4&&_data_len5>0)
		{
			_data_len5--;
			RxBuffer5[4+_data_cnt5++]=com_data;
			if(_data_len5==0)
				RxState5 = 5;
		}
		else if(RxState5==5)
		{
			RxState5 = 0;
			RxBuffer5[4+_data_cnt5]=com_data;
			Data_Receive_Anl5(RxBuffer5,_data_cnt5+5);
		}
		else
			RxState5 = 0;
		#endif
	}
	
}


void Usart1_Init(u32 br_num)//-------UPload_board1  蓝牙上位机通讯
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
//	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
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
}


u8 RxBuffer_app[50];
u8 RxState_app = 0;
u8 RxBufferNum_app = 0;
u8 RxBufferCnt_app = 0;
u8 RxLen_app = 0;
static u8 _data_len_app = 0,_data_cnt_app = 0;

void USART1_IRQHandler(void)
{			
  uint8_t c,t;
	if(USART1->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART1->DR;
	}
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志
		c = USART1->DR;		
		fifo_write_ch(&uart_rx_fifo, c);
		
    //USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
  }

  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {   		
		if(fifo_read_ch(&uart_tx_fifo, &t)) USART_SendData(USART1, t);
		else USART_SendData(USART1, 0x55);
				
    if (fifo_used(&uart_tx_fifo) == 0)              // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
    {
      // Disable the EVAL_COM1 Transmit interrupt 
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }
  }		
}



u8 RxBuffer4[50];
u8 RxState4 = 0;
u8 RxBufferNum4 = 0;
u8 RxBufferCnt4 = 0;
u8 RxLen4 = 0;
static u8 _data_len4 = 0,_data_cnt4 = 0;

void UART4_IRQHandler(void)
{
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
			RxLen4 = com_data;
			RxBufferCnt4 = 0;
		}
		else if(RxState4==4&&RxLen4>0)
		{
			RxLen4--;
			RxBuffer4[4+RxBufferCnt4++]=com_data;
			if(RxLen4==0)
				RxState4 = 5;
		}
		else if(RxState4==5)
		{
			RxState4 = 0;
			RxBuffer4[4+RxBufferCnt4]=com_data;
		}
		else
			RxState4 = 0;
	}
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
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
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
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//GPIO_PuPd_NOPULL ;
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

void Usart3_Init(u32 br_num)//-------sonar
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	#if USE_MINI_BOARD
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
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
	
	#else
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
 #endif
	
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


 void Data_Receive_AnlPx4(u8 *data_buf,u8 num)
{
	double zen,xiao;
	static float m100_hr,m100_attr[3];
	unsigned char  buf_float[4],_cnt;
	vs16 rc_value_temp,rc_value_temp1;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	u8 sum_in=*(data_buf+num-1);
	///if(!(sum==*(data_buf+num-1)))		
	//	return;		//ÅÐ¶Ïsum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//ÅÐ¶ÏÖ¡Í·
	if(*(data_buf+2)==0x01)//
  {
    Feed_Rc_Dog(2);
		px4.loss_cnt=0;
		px4.connect=1;
	  px4.Pit=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10.;
		px4.Rol=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10.;
		px4.Yaw=To_180_degrees(1*(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.);
		
		px4.H=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		
		if(px4.H!=m100_hr||m100_attr[0]!=px4.Pit||m100_attr[1]!=px4.Rol||m100_attr[2]!=px4.Yaw)
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
		px4.Rc_pit=(float)((int16_t)(*(data_buf+36)<<8)|*(data_buf+37));//thr
		px4.Rc_thr=(float)((int16_t)(*(data_buf+38)<<8)|*(data_buf+39));//pit
		px4.STATUS=*(data_buf+40);		
		if(px4.Lat!=0&&px4.Lon!=0)
		px4.GPS_STATUS=3;
    else		
		px4.GPS_STATUS=*(data_buf+41);
		px4.spd[0]=(float)((int16_t)(*(data_buf+42)<<8)|*(data_buf+43))/1000.;
		px4.spd[1]=(float)((int16_t)(*(data_buf+44)<<8)|*(data_buf+45))/1000.;
		px4.spd[2]=(float)((int16_t)(*(data_buf+46)<<8)|*(data_buf+47))/1000.;
		px4.uwb_o[0]=(float)((int16_t)(*(data_buf+48)<<8)|*(data_buf+49))/1000.;
		px4.uwb_o[1]=(float)((int16_t)(*(data_buf+50)<<8)|*(data_buf+51))/1000.;
		px4.uwb_o[2]=(float)((int16_t)(*(data_buf+52)<<8)|*(data_buf+53))/1000.;	
	}
	else if(*(data_buf+2)==0x21)//Qr land
  {
		c2c.connect=circle.connect=1;
		c2c.lose_cnt=0;
		c2c.check=*(data_buf+4);	
		c2c.x=(int16_t)(*(data_buf+5)<<8)|*(data_buf+6);
		c2c.y=(int16_t)(*(data_buf+7)<<8)|*(data_buf+8);
		c2c.z=(int16_t)(*(data_buf+9)<<8)|*(data_buf+10);
		c2c.pit=(int16_t)(*(data_buf+11)<<8)|*(data_buf+12);
		c2c.rol=(int16_t)(*(data_buf+13)<<8)|*(data_buf+14);
		c2c.yaw=(int16_t)(*(data_buf+15)<<8)|*(data_buf+16);
	
	if(c2c.check){
		int temp=((int16_t)(*(data_buf+17)<<8)|*(data_buf+18));
		if(ABS(temp)<1000)
		c2c.pix_x=temp;
		temp=((int16_t)(*(data_buf+19)<<8)|*(data_buf+20));
		if(ABS(temp)<1000)
		c2c.pix_y=temp;	
	}
	c2c.center_x=-((int16_t)(*(data_buf+25)<<8)|*(data_buf+26));	
	c2c.center_y=-((int16_t)(*(data_buf+27)<<8)|*(data_buf+28));
	#if CAMERA_INVERT
	c2c.x*=-1;
	c2c.y*=-1;
	c2c.pix_x*=-1;
	c2c.pix_y*=-1;
	#endif
	
	}
	 else if(*(data_buf+2)==0x30)//Laser SLAM
  {
		vslam.connect=1;
		vslam.lose_cnt=0;
		_cnt=4;
		buf_float[3]=*(data_buf+(_cnt++));buf_float[2]=*(data_buf+(_cnt++));
		buf_float[1]=*(data_buf+(_cnt++));buf_float[0]=*(data_buf+(_cnt++));
		vslam.y=Bytes2Float(buf_float,4);
		buf_float[3]=*(data_buf+(_cnt++));buf_float[2]=*(data_buf+(_cnt++));
		buf_float[1]=*(data_buf+(_cnt++));buf_float[0]=*(data_buf+(_cnt++));
		vslam.x=-Bytes2Float(buf_float,4);
		buf_float[3]=*(data_buf+(_cnt++));buf_float[2]=*(data_buf+(_cnt++));
		buf_float[1]=*(data_buf+(_cnt++));buf_float[0]=*(data_buf+(_cnt++));
		vslam.yaw=-To_180_degrees(Bytes2Float(buf_float,4)*57.3);

	}
}

u8 RxBuffer3[60];
u8 RxState3 = 0;
u8 RxBufferNum3 = 0;
u8 RxBufferCnt3 = 0;
u8 RxLen3 = 0;
static u8 _data_len3 = 0,_data_cnt3 = 0;
void USART3_IRQHandler(void)
{ // OSIntEnter();  
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
		#if PX4_SDK
		UsartSend_GOL_LINK(com_data);
		#endif
    Ultra_Get(com_data);
		
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
		else if(RxState3==3&&com_data<60)
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
		
				Data_Receive_AnlPx4(RxBuffer3,_data_cnt3+5);
			
				#if USE_VER_6
				Data_Receive_Anl5(RxBuffer3,_data_cnt3+5);
				#endif
		}
		else
			RxState3 = 0;
	}
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


//-------------------------NAV_BOARD_LINK

//------------------------------------------------------GOL_LINK----------------------------------------------------

void UsartSend_GOL_LINK_NAV(uint8_t ch)
{
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); 
}

void Send_Data_GOL_LINK_NAV(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK_NAV(dataToSend[i]);
}


u8 SendBuff1[SEND_BUF_SIZE1];	//发送数据缓冲区
u8 SendBuff1_cnt;
void Usart1_Send_DMA(u8 *dataToSend , u8 length)
{u8 i;
for	(i=0;i<length;i++)
SendBuff1[SendBuff1_cnt++]=dataToSend[i];
}


void data_per_uart1(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
u16 i=0; 	
unsigned int temp=0xaF+9;
char ctemp;	
BLE_DEBUG[1]=ax;
BLE_DEBUG[2]=ay;
BLE_DEBUG[3]=az;	
BLE_DEBUG[4]=gx;
BLE_DEBUG[5]=gy;
BLE_DEBUG[6]=gz;
BLE_DEBUG[7]=hx;
BLE_DEBUG[8]=hy;
BLE_DEBUG[9]=hz;
BLE_DEBUG[10]=yaw;
BLE_DEBUG[11]=pitch;
BLE_DEBUG[12]=roll;
BLE_DEBUG[13]=alt;
BLE_DEBUG[13]=tempr;
BLE_DEBUG[14]=press;
BLE_DEBUG[15]=IMUpersec;
}


u8 SendBuff2[SEND_BUF_SIZE2];	//发送数据缓冲区
void data_per_uart2(void)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
  SendBuff2[_cnt++]=0xAA;
	SendBuff2[_cnt++]=0xAF;
	SendBuff2[_cnt++]=0x61;//功能字
	SendBuff2[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);

	_temp = q_nav[0]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	
	
	_temp =  0;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  0;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  ALT_POS_SONAR2*1000;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	
	
	SendBuff2[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[_cnt++] = sum;
}



u8 SendBuff4[SEND_BUF_SIZE4];	//发送数据缓冲区
u16 nrf_uart_cnt;
int sd_save[25*3];
void data_per_uart4(u8 sel)
{
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  

switch(sel){
	case SEND_M100:
	cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x51;//功能字
	SendBuff4[nrf_uart_cnt++]=0;//数据量
	_temp=m100.Pit*10;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
  _temp=m100.Rol*10;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Yaw*10;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=m100.H*1000;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.H_Spd*1000;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=m100.Lat;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=(m100.Lat-(int)m100.Lat)*1000000000;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE3(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE2(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Lon;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=(m100.Lon-(int)m100.Lon)*1000000000;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE3(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE2(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Bat*100;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Rc_pit;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Rc_rol;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Rc_yaw;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Rc_thr;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Rc_mode;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.Rc_gear;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.m100_connect;
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.GPS_STATUS;
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mode_oldx.en_sd_save;
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);

	
	SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;
	break;
	case SEND_ALT:
		cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x03;//功能字
	SendBuff4[nrf_uart_cnt++]=0;//数据量
//	_temp = exp_height;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
//	_temp = ultra_dis_lpf;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	//_temp = ultra_ctrl_out;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	//_temp = wz_speed;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
 //	_temp = thr_value	;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = ALT_POS_BMP*1000	;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = ALT_POS_SONAR2*1000	;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
  _temp = mode_oldx.en_sd_save;
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = ALT_POS_BMP_EKF*1000	;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = ALT_VEL_BMP_EKF*1000	;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = ultra_distance	;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	
	SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;
	break;
	case SEND_IMU:
		cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x05;//功能字
	SendBuff4[nrf_uart_cnt++]=0;//数据量
	_temp = Roll*10;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = Pitch*10;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = Yaw*10	;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=(mpu6050.Gyro.x);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=(mpu6050.Gyro.y);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=(mpu6050.Gyro.z);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc.x;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc.y;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc.z;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.q[0]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.q[1]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.q[2]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=m100.q[3]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);

	_temp=ak8975.Mag_Adc.x;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=ak8975.Mag_Adc.y;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=ak8975.Mag_Adc.z;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=ak8975.Mag_Val.x;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
		_temp=ak8975.Mag_Val.y;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
		_temp=ak8975.Mag_Val.z;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
		_temp=mode_oldx.en_sd_save;
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;
	break;
	case SEND_FLOW:
		cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x06;//功能字
	SendBuff4[nrf_uart_cnt++]=0;//数据量
	_temp=imu_nav.flow.speed.x_f*1000;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=imu_nav.flow.speed.y_f*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=imu_nav.flow.speed.x*1000;//origin
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=imu_nav.flow.speed.y*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=target_position[LAT]*100;//
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=target_position[LON]*100;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=now_position[LAT]*100;//
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=now_position[LON]*100;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=flow_matlab_data[0]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=flow_matlab_data[1]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=flow_matlab_data[2]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=flow_matlab_data[3]*1000;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);

	//_temp=baroAlt_fc;//baro_matlab_data[0];
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	//_temp=acc_bmp*1000;//baro_matlab_data[1];
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=circle.check&&circle.connect;///10.;
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=circle.x;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=circle.y;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=circle.z;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=circle.pit;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=circle.rol;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=circle.yaw;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=k_scale_pix;
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;
	break;
  case SEND_PID:
		cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x07;//功能字
	SendBuff4[nrf_uart_cnt++]=0;//数据量
	_temp=SPID.OP;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.OI;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.OD;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=SPID.IP;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.II;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.ID;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=SPID.YP;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.YI;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.YD;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=HPID.OP;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=HPID.OI;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=HPID.OD;//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mode_oldx.en_sd_save;
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;
	break;
	case SEND_DEBUG:
		cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x08;//功能字
	SendBuff4[nrf_uart_cnt++]=0;//数据量
	for(i=0;i<16;i++){
	_temp=BLE_DEBUG[i];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	}
	
	SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;
	break;
	
	case SEND_MARKER:
	cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x09;//功能字
	SendBuff4[nrf_uart_cnt++]=0;//数据量
	for(i=0;i<4;i++){
	_temp=mark_map[i][0];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mark_map[i][1];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mark_map[i][2];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	}
  _temp=mark_map[0][3];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mark_map[1][3];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mark_map[2][3];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=mark_map[3][3];//filter
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
  SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;
	break;
	case SEND_SD_SAVE1://<--------------------------------sd general save 
	cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x81;//功能字
	SendBuff4[nrf_uart_cnt++]=0;//数据量
	for(i=0;i<20;i++){
	_temp=sd_save[i];//id
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	}

  SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;	
	break;
  case SEND_SD_SAVE2:
	cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x82;//功能字
	SendBuff4[nrf_uart_cnt++]=0;//数据量
	for(i=20;i<20*2;i++){
	_temp=sd_save[i];//id
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	}

  SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;	
	break;
	case SEND_SD_SAVE3:
	cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x83;//功能字
	SendBuff4[nrf_uart_cnt++]=0;//数据量

	for(i=20*2;i<20*3;i++){
	_temp=sd_save[i];//id
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	}
	SendBuff4[nrf_uart_cnt++]=mode_oldx.en_sd_save;
	SendBuff4[nrf_uart_cnt++]=mode_oldx.cal_rc;
	SendBuff4[nrf_uart_cnt++]=mode_oldx.mems_state;
	if(m100.STATUS>0)
	SendBuff4[nrf_uart_cnt++]=m100.GPS_STATUS;
	else
	SendBuff4[nrf_uart_cnt++]=0;
	SendBuff4[nrf_uart_cnt++]=module.acc_imu==2&&module.gyro_imu==2&&module.hml_imu==2;
	
	
	
	
  SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;	
	break;
	case SEND_QR:
	cnt_reg=nrf_uart_cnt;
	SendBuff4[nrf_uart_cnt++]=0xAA;
	SendBuff4[nrf_uart_cnt++]=0xAF;
	SendBuff4[nrf_uart_cnt++]=0x61;//功能字
	SendBuff4[nrf_uart_cnt++]=0;//数据量

  static int x,y,z;
	if(circle.x!=0)
		x=circle.x*10;
	if(circle.y!=0)
		y=-circle.y*10;
	if(circle.z!=0)
		z=circle.z*10;
	//qr
	_temp = (int)(x);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (int)(y);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (int)(z);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	//exp
	_temp = (int)(nav_pos_ctrl[0].exp*1000);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (int)(nav_pos_ctrl[1].exp*1000);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (int)(exp_height);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	//kf
	_temp = (int)(POS_UKF_X*1000);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (int)(POS_UKF_Y*1000);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	_temp = (int)(ALT_POS_SONAR2*1000);
	SendBuff4[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff4[nrf_uart_cnt++]=BYTE0(_temp);
	
	
  SendBuff4[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
	for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[nrf_uart_cnt++] = sum;	
	break;
	
	
	default:break;
}
}

#include "rc.h"
#define ARUCO_MAP 0
#define QR_LAND 1
u8 sd_pub_sel=ARUCO_MAP;
void sd_publish(void)
{u8 cnt=0;
	switch(sd_pub_sel){
	case ARUCO_MAP:	//mark slam
	//sd 1	
	sd_save[cnt++]=mark_map[0][0];
	sd_save[cnt++]=mark_map[0][1];
	sd_save[cnt++]=mark_map[0][2];	
	sd_save[cnt++]=mark_map[0][3];
	sd_save[cnt++]=mark_map[0][4];

	sd_save[cnt++]=mark_map[1][0];
	sd_save[cnt++]=mark_map[1][1];
	sd_save[cnt++]=mark_map[1][2];	
	sd_save[cnt++]=mark_map[1][3];
	sd_save[cnt++]=mark_map[1][4];

	sd_save[cnt++]=mark_map[2][0];
	sd_save[cnt++]=mark_map[2][1];
	sd_save[cnt++]=mark_map[2][2];	
	sd_save[cnt++]=mark_map[2][3];
	sd_save[cnt++]=mark_map[2][4];

	sd_save[cnt++]=mark_map[3][0];
	sd_save[cnt++]=mark_map[3][1];
	sd_save[cnt++]=mark_map[3][2];	
	sd_save[cnt++]=mark_map[3][3];
	sd_save[cnt++]=mark_map[3][4];

	//sd 2  20~39
	sd_save[cnt++]=mark_map[4][0];
	sd_save[cnt++]=mark_map[4][1];
	sd_save[cnt++]=mark_map[4][2];	
	sd_save[cnt++]=mark_map[4][3];
	sd_save[cnt++]=mark_map[4][4];

	sd_save[cnt++]=mark_map[5][0];
	sd_save[cnt++]=mark_map[5][1];
	sd_save[cnt++]=mark_map[5][2];	
	sd_save[cnt++]=mark_map[5][3];
	sd_save[cnt++]=mark_map[5][4];

	sd_save[cnt++]=circle.check&&circle.connect;
	sd_save[cnt++]=circle.x;
	sd_save[cnt++]=circle.y;
	sd_save[cnt++]=circle.z;
	sd_save[cnt++]=circle.pit;

	sd_save[cnt++]=circle.rol;
	sd_save[cnt++]=circle.yaw;
	sd_save[cnt++]=flow_matlab_data[0]*1000;	
	sd_save[cnt++]=flow_matlab_data[1]*1000;
	sd_save[cnt++]=ALT_POS_SONAR2*1000;
	//sd 3 40~59

	sd_save[cnt++]=circle.spdy;//flow_matlab_data[2]*1000;	
	sd_save[cnt++]=circle.spdx;//flow_matlab_data[3]*1000;
	sd_save[cnt++]=0;
	sd_save[cnt++]=mpu6050_fc.Acc.x;
	sd_save[cnt++]=mpu6050_fc.Acc.y;

	sd_save[cnt++]=mpu6050_fc.Acc.z;
	sd_save[cnt++]=mpu6050_fc.Gyro.x;
	sd_save[cnt++]=mpu6050_fc.Gyro.y;
	sd_save[cnt++]=mpu6050_fc.Gyro.z;
	sd_save[cnt++]=ak8975_fc.Mag_Val.x;

	sd_save[cnt++]=ak8975_fc.Mag_Val.y;
	sd_save[cnt++]=ak8975_fc.Mag_Val.z;
	sd_save[cnt++]=Pit_fc*100;
	sd_save[cnt++]=Rol_fc*100;
	sd_save[cnt++]=Yaw_fc*100;

	sd_save[cnt++]=VEL_UKF_X*100;//flow_matlab_data[2]*1000;
	sd_save[cnt++]=VEL_UKF_Y*100;//flow_matlab_data[3]*1000;
	sd_save[cnt++]=POS_UKF_X*100;
	sd_save[cnt++]=POS_UKF_Y*100;
	sd_save[cnt++]=0;
	break;
	///
	case QR_LAND:	//track 
	//sd 1	
	sd_save[cnt++]=Pit_fc*100;
	sd_save[cnt++]=Rol_fc*100;
	sd_save[cnt++]=Yaw_fc*100;	
	sd_save[cnt++]=0;
	sd_save[cnt++]=0;

	sd_save[cnt++]=ALT_POS_SONAR2*1000;
	sd_save[cnt++]=POS_UKF_Y*1000;
	sd_save[cnt++]=POS_UKF_X*1000;	
	sd_save[cnt++]=VEL_UKF_Y*1000;
	sd_save[cnt++]=VEL_UKF_X*1000;

	sd_save[cnt++]=acc_body[Y];
	sd_save[cnt++]=acc_body[X];
	sd_save[cnt++]=0;	
	sd_save[cnt++]=0;
	sd_save[cnt++]=0;

	sd_save[cnt++]=robot.connect&&robot.mark_check;
	sd_save[cnt++]=robot.camera_x;
	sd_save[cnt++]=robot.camera_y;	
	sd_save[cnt++]=robot.camera_z;
	sd_save[cnt++]=robot.yaw;

	//sd 2  20~39
	sd_save[cnt++]=robot.rol;
	sd_save[cnt++]=robot.pit;
	sd_save[cnt++]=robot.track_x;	
	sd_save[cnt++]=robot.track_y;
	sd_save[cnt++]=robot.track_r;

	sd_save[cnt++]=robot.mark_x;
	sd_save[cnt++]=robot.mark_y;
	sd_save[cnt++]=robot.mark_r;	
	sd_save[cnt++]=aux.ero[Xr];
	sd_save[cnt++]=aux.ero[Yr];

	sd_save[cnt++]=aux.att[0]*10;
	sd_save[cnt++]=aux.att[1]*10;
	sd_save[cnt++]=0;//spd obo
	sd_save[cnt++]=0;
	sd_save[cnt++]=0;

	sd_save[cnt++]=robot.mark_map[0][0];
	sd_save[cnt++]=robot.mark_map[0][1];
	sd_save[cnt++]=robot.mark_map[0][2];	
	sd_save[cnt++]=robot.mark_map[0][3];
	sd_save[cnt++]=robot.mark_map[0][4];	
	//sd 3 40~59

	sd_save[cnt++]=robot.mark_map[1][0];
	sd_save[cnt++]=robot.mark_map[1][1];
	sd_save[cnt++]=robot.mark_map[1][2];	
	sd_save[cnt++]=robot.mark_map[1][3];
	sd_save[cnt++]=robot.mark_map[1][4];	

	sd_save[cnt++]=robot.mark_map[2][0];
	sd_save[cnt++]=robot.mark_map[2][1];
	sd_save[cnt++]=robot.mark_map[2][2];	
	sd_save[cnt++]=robot.mark_map[2][3];
	sd_save[cnt++]=robot.mark_map[2][4];		

	sd_save[cnt++]=robot.mark_map[3][0];
	sd_save[cnt++]=robot.mark_map[3][1];
	sd_save[cnt++]=robot.mark_map[3][2];	
	sd_save[cnt++]=robot.mark_map[3][3];
	sd_save[cnt++]=robot.mark_map[3][4];	

	sd_save[cnt++]=ultra_ctrl.exp;
	sd_save[cnt++]=ultra_ctrl.now;
	sd_save[cnt++]=wz_speed_pid_v.exp;	
	sd_save[cnt++]=wz_speed_pid_v.now;
	sd_save[cnt++]=0;	
	break;
	}
}	


void clear_nrf_uart(void)
{u16 i;
for(i=0;i<SEND_BUF_SIZE4;i++)
SendBuff4[i]=0;
}
