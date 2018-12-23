#ifndef __FLOW_H
#define __FLOW_H	 
#include "stm32f4xx.h" //f103请修改头文件
#include <math.h>
/*
Copyright(C) 2017 DianYin Innovations Corporation. All rights reserved.

OLDX飞控使用 ADRC controller 自抗扰控制器 designed by Golaced from 云逸创新(滇鹰科技)

qq群:
567423074
淘宝店铺：
https://shop124436104.taobao.com/?spm=2013.1.1000126.2.iNj4nZ
手机店铺地址：
http://shop124436104.m.taobao.com

flow.lib 封装了串口读取光流数据的函数和光流数据预处理函数，输出摄像头实际物理速度，
注意：本程序不对使用后造成的炸鸡负责
版本V1.0 
*/

typedef struct
{
	float average;//Flow in m in x-sensor direction, angular-speed compensated
	float averager;//Flow in m in x-sensor direction, angular-speed compensated
	float originf;
	int16_t origin;
}FLOW_DATA;

 typedef struct
{
	uint64_t  time_sec;
	u32 flow_cnt;
	float rate;
	u8   id;
	FLOW_DATA flow_x;
	FLOW_DATA flow_y;
	FLOW_DATA flow_comp_x;//Flow in m in x-sensor direction, angular-speed compensated
	FLOW_DATA flow_comp_y;
	u8 quality; //Optical flow quality / confidence. 0: bad, 255: maximum quality
	FLOW_DATA hight;//ground_distance	float	Ground distance in m. Positive value: distance known. Negative value: Unknown distance    
  u8 new_data_flag;	
}FLOW;

typedef struct
{
 uint64_t time_usec; ///< Timestamp (microseconds, synced to UNIX time or since system boot)
 uint32_t integration_time_us; ///< Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
 float integrated_x; ///< Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
 float integrated_y; ///< Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
 float integrated_xgyro; ///< RH rotation around X axis (rad)
 float integrated_ygyro; ///< RH rotation around Y axis (rad)
 float integrated_zgyro; ///< RH rotation around Z axis (rad)
 uint32_t time_delta_distance_us; ///< Time in microseconds since the distance was sampled.
 float distance; ///< Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
 int16_t temperature; ///< Temperature * 100 in centi-degrees Celsius
 uint8_t sensor_id; ///< Sensor ID
 uint8_t quality; ///< Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
}FLOW_RAD;


//--------------------------------------------------------------------------------------------------
void flow_uart_rx_oldx(unsigned char data,FLOW *flow,FLOW_RAD *flow_rad);//串口解码程序  放到中断里
void flow_pertreatment_oldx( FLOW_RAD *flow_in ,float flow_height);//光流数据预处理

extern float flow_per_out[4];//光流最终输出为数组中 [2] [3]
//----------参数---------
extern float k_flow_devide;//光流输出增益  按实际波形和运动情况进行调节
extern float flow_module_offset_y,flow_module_offset_x;//光流安装偏移/m
extern u8 flow_update;
void Read_Px4flow(void);

//----------Example---------
/*
float flow_spd_x,flow_spd_y;
FLOW_RAD flow_rad_use;  
flow_rad_use.integration_time_us=integration_timespan;
flow_rad_use.integrated_xgyro=accumulated_gyro_x;
flow_rad_use.integrated_ygyro=accumulated_gyro_y;
flow_rad_use.integrated_zgyro=accumulated_gyro_z;		 
flow_rad_use.integrated_x=accumulated_flow_x;
flow_rad_use.integrated_y=accumulated_flow_y;
flow_pertreatment_oldx( &flow_rad_use , flow_height_fliter);

flow_spd_x=flow_per_out[2];
flow_spd_y=flow_per_out[3];
*/
#endif











