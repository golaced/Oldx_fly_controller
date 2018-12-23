#include "flow.h"
#include "include.h"
#define LIMIT_FLOW( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
u8 flow_update;
static unsigned char FLOW_STATE[4];
static unsigned char flow_buf[27];
static unsigned char flow_buf_rad[45];
float ByteToFloat(unsigned char* byteArry)
{
  return *((float*)byteArry);
}

void flow_uart_rx_oldx(unsigned char data,FLOW *flow,FLOW_RAD *flow_rad)
{
static unsigned char s_flow=0,data_cnt=0;
float sonar_new;
static float  temp,sonar_lp;
unsigned char cnt_offset=0;	
unsigned char get_one_fame=0;
unsigned char floattobyte[4];
		switch(s_flow)
	 {
    case 0: if(data==0xFE)
			s_flow=1;
			break;
		case 1: if(data==0x1A||data==0x2C)
				{ s_flow=2;}
			else
			s_flow=0;
			break;
	  case 2:
			if(data_cnt<4)
			{s_flow=2; FLOW_STATE[data_cnt++]=data;}
		  else
			{data_cnt=0;s_flow=3;flow_buf[data_cnt++]=data;}
		 break;
		case 3:
		 if(FLOW_STATE[3]==100){
			if(data_cnt<26)
			{s_flow=3; flow_buf[data_cnt++]=data;}
		  else
			{data_cnt=0;s_flow=4;}
		}
		else if(FLOW_STATE[3]==106){
			if(data_cnt<44)
			{s_flow=3; flow_buf_rad[data_cnt++]=data;}
		  else
			{data_cnt=0;s_flow=4;}
		}
		else
			{data_cnt=0;s_flow=0;}
			 break;
		case 4:get_one_fame=1;s_flow=0;data_cnt=0;break;
		default:s_flow=0;data_cnt=0;break;
	 }//--end of s_uart
		

	 if(get_one_fame)
	 { module.flow=1;
		 flow->flow_cnt++;
		 if(FLOW_STATE[3]==100){
		 flow->time_sec=(flow_buf[7]<<64)|(flow_buf[6]<<56)|(flow_buf[5]<<48)|(flow_buf[4]<<40)
		 |(flow_buf[3]<<32)|(flow_buf[2]<<16)|(flow_buf[1]<<8)|(flow_buf[0]);
  	 floattobyte[0]=flow_buf[8];
		 floattobyte[1]=flow_buf[9];
		 floattobyte[2]=flow_buf[10];
		 floattobyte[3]=flow_buf[11];
		 flow->flow_comp_x.originf =ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf[12];
		 floattobyte[1]=flow_buf[13];
		 floattobyte[2]=flow_buf[14];
		 floattobyte[3]=flow_buf[15];
		 flow->flow_comp_y.originf =ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf[16];
		 floattobyte[1]=flow_buf[17];
		 floattobyte[2]=flow_buf[18];
		 floattobyte[3]=flow_buf[19];
		 flow->hight.originf=ByteToFloat(floattobyte);//sonar_new;//0.05f * sonar_new + 0.95f * sonar_lp;
		 sonar_lp = sonar_new;
		 flow->flow_x.origin=(int16_t)((flow_buf[20])|(flow_buf[21]<<8));
	   flow->flow_y.origin=(int16_t)((flow_buf[22])|(flow_buf[23]<<8));
		 flow->id=flow_buf[24];
	   flow->quality=flow_buf[25]; //Optical flow quality / confidence. 0: bad, 255: maximum quality
     flow->new_data_flag	=1;
		 }
	 else if(FLOW_STATE[3]==106)
	 {  flow_update=1;
	 	  flow_rad->time_usec=(flow_buf_rad[7]<<64)|(flow_buf_rad[6]<<56)|(flow_buf_rad[5]<<48)|(flow_buf_rad[4]<<40)
		 |(flow_buf_rad[3]<<32)|(flow_buf_rad[2]<<16)|(flow_buf_rad[1]<<8)|(flow_buf_rad[0]);
  	 flow_rad->integration_time_us=(flow_buf_rad[11]<<32)|(flow_buf_rad[10]<<16)|(flow_buf_rad[9]<<8)|(flow_buf_rad[8]);
		 floattobyte[0]=flow_buf_rad[12];
		 floattobyte[1]=flow_buf_rad[13];
		 floattobyte[2]=flow_buf_rad[14];
		 floattobyte[3]=flow_buf_rad[15];
		 flow_rad->integrated_x=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[16];
		 floattobyte[1]=flow_buf_rad[17];
		 floattobyte[2]=flow_buf_rad[18];
		 floattobyte[3]=flow_buf_rad[19];
		 flow_rad->integrated_y=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[20];
		 floattobyte[1]=flow_buf_rad[21];
		 floattobyte[2]=flow_buf_rad[22];
		 floattobyte[3]=flow_buf_rad[23];
		 flow_rad->integrated_xgyro=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[24];
		 floattobyte[1]=flow_buf_rad[25];
		 floattobyte[2]=flow_buf_rad[26];
		 floattobyte[3]=flow_buf_rad[27];
		 flow_rad->integrated_ygyro=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[28];
		 floattobyte[1]=flow_buf_rad[29];
		 floattobyte[2]=flow_buf_rad[30];
		 floattobyte[3]=flow_buf_rad[31];
		 flow_rad->integrated_zgyro=ByteToFloat(floattobyte);
		 flow_rad->time_delta_distance_us=(flow_buf_rad[35]<<32)|(flow_buf_rad[34]<<16)|(flow_buf_rad[33]<<8)|(flow_buf_rad[32]);
		 floattobyte[0]=flow_buf_rad[36];
		 floattobyte[1]=flow_buf_rad[37];
		 floattobyte[2]=flow_buf_rad[38];
		 floattobyte[3]=flow_buf_rad[39];
		 flow_rad->distance=ByteToFloat(floattobyte);
		 flow_rad->temperature=(flow_buf_rad[41]<<8)|(flow_buf_rad[40]);
		 flow_rad->sensor_id=(flow_buf_rad[42]);
		 flow_rad->quality=(flow_buf_rad[43]);		 
	 }	
	 }
}

float flow_module_offset_y=0,flow_module_offset_x=-0.05;//光流安装位移 单位米
#define USE_FLOW_FLY_ROBOT 1
#if USE_FLOW_FLY_ROBOT//使用飞行实验室的光流模块
#define K_PIX 1.8*0.7
static float k_scale_pix=K_PIX;
static float scale_pix=0.0055*K_PIX;//0.002;//.003;//0.005;
#else
#define K_PIX 1
static float k_scale_pix=K_PIX;
static float scale_pix=0.0055*K_PIX;//0.002;//.003;//0.005;
#endif
float k_flow_devide=1;
float rate_threshold = 0.3615f; 

static float flow_k[2]={0.15,0.15};
static float k_gro_off=1;
static float scale_px4_flow=1300;
static float flow_rad_fix_k[2]={0.8,1},flow_rad_fix_k2=0.0;
static float dead_rad_fix[2]={40,50};

static float flow_rad_fix[4];
static float x_flow_orign_temp,y_flow_orign_temp;
static float yaw_comp[2];
static float flow_filter[2];
float flow_per_out[4];
void flow_pertreatment_oldx( FLOW_RAD *flow_in ,float flow_height){
float flow_gyrospeed[3];	
	  flow_gyrospeed[0] = (float)flow_in->integrated_xgyro / (float)flow_in->integration_time_us * 1000000.0f;  
		flow_gyrospeed[1] = (float)flow_in->integrated_ygyro / (float)flow_in->integration_time_us * 1000000.0f;  
		flow_gyrospeed[2] = (float)flow_in->integrated_zgyro / (float)flow_in->integration_time_us * 1000000.0f;  
	  static  unsigned char n_flow;
	  static float gyro_offset_filtered[3],att_gyrospeed_filtered[3],flow_gyrospeed_filtered[3];
		float flow_ang[2];

		if (fabs(flow_gyrospeed[0]) < rate_threshold) {  
		flow_ang[0] = (flow_in->integrated_x / (float)flow_in->integration_time_us * 1000000.0f) * flow_k[0];//for now the flow has to be scaled (to small)  
		}  
		else {  
		//calculate flow [rad/s] and compensate for rotations (and offset of flow-gyro)  
		flow_ang[0] = ((flow_in->integrated_x - 
		LIMIT_FLOW(flow_in->integrated_xgyro*k_gro_off,-fabs(flow_in->integrated_x),fabs(flow_in->integrated_x))) 
		/ (float)flow_in->integration_time_us * 1000000.0f  
		+ gyro_offset_filtered[0]*0) * flow_k[0];//for now the flow has to be scaled (to small)  
		}  
		
		if (fabs(flow_gyrospeed[1]) < rate_threshold) {  
		flow_ang[1] = (flow_in->integrated_y / (float)flow_in->integration_time_us * 1000000.0f) * flow_k[1];//for now the flow has to be scaled (to small)  
		}  
		else {  
		//calculate flow [rad/s] and compensate for rotations (and offset of flow-gyro)  
		flow_ang[1] = ((flow_in->integrated_y - 
		LIMIT_FLOW(flow_in->integrated_ygyro*k_gro_off,-fabs(flow_in->integrated_y),fabs(flow_in->integrated_y))) / (float)flow_in->integration_time_us * 1000000.0f  
		+ gyro_offset_filtered[1]*0) * flow_k[1];//for now the flow has to be scaled (to small)  
		}  
	
		yaw_comp[0] = - flow_module_offset_y * (flow_gyrospeed[2] - gyro_offset_filtered[2]);  
		yaw_comp[1] = flow_module_offset_x * (flow_gyrospeed[2] - gyro_offset_filtered[2]);  
		/* flow measurements vector */  

		x_flow_orign_temp=flow_ang[1]*scale_px4_flow;
		y_flow_orign_temp=flow_ang[0]*scale_px4_flow;		
	
		flow_filter[0]=x_flow_orign_temp;
		flow_filter[1]=y_flow_orign_temp;

		flow_per_out[0]=flow_filter[0]*flow_height*scale_pix;
		flow_per_out[1]=flow_filter[1]*flow_height*scale_pix;
		if (fabs(flow_gyrospeed[2]) < rate_threshold) {
    flow_per_out[2]=(x_flow_orign_temp)*flow_height*scale_pix;
		flow_per_out[3]=(y_flow_orign_temp)*flow_height*scale_pix;
		} else {
		flow_per_out[2]=(x_flow_orign_temp)*flow_height*scale_pix;// - yaw_comp[1] * flow_k[1];//1
		flow_per_out[3]=(y_flow_orign_temp)*flow_height*scale_pix;// - yaw_comp[0] * flow_k[0];//0
		}
	
		flow_per_out[0]*=k_flow_devide;
		flow_per_out[1]*=k_flow_devide;
		flow_per_out[2]*=k_flow_devide;
		flow_per_out[3]*=k_flow_devide;	
}