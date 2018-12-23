#ifndef __GPS_H
#define __GPS_H	 
#include "include.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//ATK-NEO-6M GPS模块驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/10/26
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved							  
////////////////////////////////////////////////////////////////////////////////// 	   


//GPS NMEA-0183协议重要参数结构体定义 
//卫星信息
__packed typedef struct  
{										    
 	u8 num;		//卫星编号
	u8 eledeg;	//卫星仰角
	u16 azideg;	//卫星方位角
	u8 sn;		//信噪比		   
}nmea_slmsg;  
//UTC时间信息
__packed typedef struct  
{										    
 	u16 year;	//年份
	u8 month;	//月份
	u8 date;	//日期
	u8 hour; 	//小时
	u8 min; 	//分钟
	u8 sec; 	//秒钟
}nmea_utc_time;   	   
//NMEA 0183 协议解析后数据存放结构体


__packed typedef struct  
{
	u32 rx_cnt;
	float rx_dt;	
	u8 PVT_fixtype;//2-2D  3-3D
	u8 PVT_numsv;
	double PVT_longitude;
	double PVT_latitude;
	float PVT_height;//m
	float PVT_North_speed,PVT_East_speed,PVT_Down_speed,PVT_speed;//m/s		
	float headMot,headVeh;//degree
	float PVT_Headacc;	//degree

	u16 PVT_Hacc,PVT_Vacc,PVT_Sacc;//mm
	u16 gDOP ;//- Geometric DOP
	u16 pDOP ;//- Position DOP
	u16 tDOP ;// Time DOP
	u16 vDOP ;// Vertical DOP
	u16 hDOP ;// Horizontal DOP
	u16 nDOP ;// Northing DOP
	u16 eDOP ;// Easting DOP
}PVT;

__packed typedef struct{  
u8 gpsPosFlag;
u8 gpsVelFlag;	
unsigned long iTOW;
    double lat;
    double lon;
    float height;   // above mean sea level (m)
    float hAcc;     // horizontal accuracy est (m)
    float vAcc;     // vertical accuracy est (m)
    float velN;     // north velocity (m/s)
    float velE;     // east velocity (m/s)
    float velD;     // down velocity (m/s)
    float speed;    // ground speed (m/s)
    float heading;  // deg
    float sAcc;     // speed accuracy est (m/s)
    float cAcc;     // course accuracy est (deg)
    float pDOP;     // position Dilution of Precision
    float hDOP;
    float vDOP;
    float tDOP;
    float nDOP;
    float eDOP;
    float gDOP;

    unsigned long TPtowMS;    // timepulse time of week (ms)
    unsigned long lastReceivedTPtowMS;

    unsigned long lastTimepulse;
    unsigned long lastPosUpdate;
    unsigned long lastVelUpdate;
    unsigned long lastMessage;
}UBM;

__packed typedef struct  
{	
  u8 rmc_mode;	
 	u8 svnum;					//可见卫星数
	nmea_slmsg slmsg[12];		//最多12颗卫星
	nmea_utc_time utc;			//UTC时间
	double latitude;				//纬度 分扩大100000倍,实际要除以100000
	u8 nshemi;					//北纬/南纬,N:北纬;S:南纬				  
	double longitude;			    //经度 分扩大100000倍,实际要除以100000
	u8 ewhemi;					//东经/西经,E:东经;W:西经
	u8 gpssta;					//GPS状态:0,未定位;1,非差分定位;2,差分定位;6,正在估算.				  
 	u8 posslnum;				//用于定位的卫星数,0~12.
 	u8 possl[12];				//用于定位的卫星编号
	u8 fixmode;					//定位类型:1,没有定位;2,2D定位;3,3D定位
	u16 pdop;					//位置精度因子 0~500,对应实际值0~50.0
	u16 hdop;					//水平精度因子 0~500,对应实际值0~50.0
	u16 vdop;					//垂直精度因子 0~500,对应实际值0~50.0 
  
	int altitude;			 	//海拔高度,放大了10倍,实际除以10.单位:0.1m	 
	u16 speed;					//地面速率,放大了1000倍,实际除以10.单位:0.001公里/小时	 
	uint16_t course_earth;                    //?????????????(0-359?)
  uint16_t course_mag;                      //?????????????(0-359?)
	float spd,angle;
	float angle_off;
	u8 ewhemi_angle_off;
  PVT pvt; 
	UBM ubm;
}nmea_msg; 
//////////////////////////////////////////////////////////////////////////////////////////////////// 	
//UBLOX NEO-6M 配置(清除,保存,加载等)结构体
__packed typedef struct  
{										    
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG CFG ID:0X0906 (小端模式)
	u16 dlength;				//数据长度 12/13
	u32 clearmask;				//子区域清除掩码(1有效)
	u32 savemask;				//子区域保存掩码
	u32 loadmask;				//子区域加载掩码
	u8  devicemask; 		  	//目标器件选择掩码	b0:BK RAM;b1:FLASH;b2,EEPROM;b4,SPI FLASH
	u8  cka;		 			//校验CK_A 							 	 
	u8  ckb;			 		//校验CK_B							 	 
}_ublox_cfg_cfg; 

//UBLOX NEO-6M 消息设置结构体
__packed typedef struct  
{										    
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG MSG ID:0X0106 (小端模式)
	u16 dlength;				//数据长度 8
	u8  msgclass;				//消息类型(F0 代表NMEA消息格式)
	u8  msgid;					//消息 ID 
								//00,GPGGA;01,GPGLL;02,GPGSA;
								//03,GPGSV;04,GPRMC;05,GPVTG;
								//06,GPGRS;07,GPGST;08,GPZDA;
								//09,GPGBS;0A,GPDTM;0D,GPGNS;
	u8  iicset;					//IIC消输出设置    0,关闭;1,使能.
	u8  uart1set;				//UART1输出设置	   0,关闭;1,使能.
	u8  uart2set;				//UART2输出设置	   0,关闭;1,使能.
	u8  usbset;					//USB输出设置	   0,关闭;1,使能.
	u8  spiset;					//SPI输出设置	   0,关闭;1,使能.
	u8  ncset;					//未知输出设置	   默认为1即可.
 	u8  cka;			 		//校验CK_A 							 	 
	u8  ckb;			    	//校验CK_B							 	 
}_ublox_cfg_msg; 

//UBLOX NEO-6M UART端口设置结构体
__packed typedef struct  
{										    
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG PRT ID:0X0006 (小端模式)
	u16 dlength;				//数据长度 20
	u8  portid;					//端口号,0=IIC;1=UART1;2=UART2;3=USB;4=SPI;
	u8  reserved;				//保留,设置为0
	u16 txready;				//TX Ready引脚设置,默认为0
	u32 mode;					//串口工作模式设置,奇偶校验,停止位,字节长度等的设置.
 	u32 baudrate;				//波特率设置
 	u16 inprotomask;		 	//输入协议激活屏蔽位  默认设置为0X07 0X00即可.
 	u16 outprotomask;		 	//输出协议激活屏蔽位  默认设置为0X07 0X00即可.
 	u16 reserved4; 				//保留,设置为0
 	u16 reserved5; 				//保留,设置为0 
 	u8  cka;			 		//校验CK_A 							 	 
	u8  ckb;			    	//校验CK_B							 	 
}_ublox_cfg_prt; 

//UBLOX NEO-6M 时钟脉冲配置结构体
__packed typedef struct  
{										    
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG TP ID:0X0706 (小端模式)
	u16 dlength;				//数据长度
	u32 interval;				//时钟脉冲间隔,单位为us
	u32 length;				 	//脉冲宽度,单位为us
	signed char status;			//时钟脉冲配置:1,高电平有效;0,关闭;-1,低电平有效.			  
	u8 timeref;			   		//参考时间:0,UTC时间;1,GPS时间;2,当地时间.
	u8 flags;					//时间脉冲设置标志
	u8 reserved;				//保留			  
 	signed short antdelay;	 	//天线延时
 	signed short rfdelay;		//RF延时
	signed int userdelay; 	 	//用户延时	
	u8 cka;						//校验CK_A 							 	 
	u8 ckb;						//校验CK_B							 	 
}_ublox_cfg_tp; 

//UBLOX NEO-6M 刷新速率配置结构体
__packed typedef struct  
{										    
 	u16 header;					//cfg header,固定为0X62B5(小端模式)
	u16 id;						//CFG RATE ID:0X0806 (小端模式)
	u16 dlength;				//数据长度
	u16 measrate;				//测量时间间隔，单位为ms，最少不能小于200ms（5Hz）
	u16 navrate;				//导航速率（周期），固定为1
	u16 timeref;				//参考时间：0=UTC Time；1=GPS Time；
 	u8  cka;					//校验CK_A 							 	 
	u8  ckb;					//校验CK_B							 	 
}_ublox_cfg_rate; 
				 
int NMEA_Str2num(u8 *buf,u8*dx);
void GPS_Analysis(nmea_msg *gpsx,u8 *buf);
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,u8 *buf);
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,u8 *buf);
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,u8 *buf);
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,u8 *buf);
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,u8 *buf);
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,u8 *buf);
u8 Ublox_Cfg_Cfg_Save(void);
u8 Ublox_Cfg_Msg(u8 msgid,u8 uart1set);
u8 Ublox_Cfg_Prt(u32 baudrate);
u8 Ublox_Cfg_Tp(u32 interval,u32 length,signed char status);
u8 Ublox_Cfg_Rate(u16 measrate,u8 reftime);
void Ublox_Send_Date(u8* dbuf,u16 len);

extern nmea_msg gpsx; 											//GPS信息

typedef struct{
	u8 connect;
	u16 lose_cnt;
	unsigned char satellite_num,fix_type;
	unsigned char Location_stu;			//定位状态
	double latitude,latitudef;						//纬度		
	double longitude,longitudef;						//经度		
	int N_vel;							//南北向速度
	int E_vel;							//东西向速度
	int D_vel;
	float last_N_vel;							//上次南北向速度
	float last_E_vel;							//上次东西向速度
	float last_D_vel;
	float real_N_vel;
	float real_E_vel;
	float real_D_vel;
	float local_pos[3];
	float local_spd_flt[3];
	unsigned char run_heart,update[5],kf_init;			//运行心跳
	double start_latitude;					//起始纬度		
	double start_longitude;					//起始经度		
	int latitude_offset;
	int longitude_offset;
	float hope_latitude;
	float hope_longitude;
	float hope_latitude_err;
	float hope_longitude_err;
	unsigned char new_pos_get;
	unsigned char Back_home_f;
	float home_latitude;
	float home_longitude;
	float PVT_height,Speed_norm;//m
	float headMot,headVeh,off_earth;//degree
	float PVT_Headacc;	//degree
   
	u16 PVT_Hacc,PVT_Vacc,PVT_Sacc;//mm
}GPS_INF;


extern GPS_INF Gps_information,Gps_information6;
void Drv_GpsPin_Init(void);
void Uart1_GPS_IRQ(u8 RX_dat );
void Uart6_GPS_IRQ(u8 RX_dat );
void GPS_Data_Processing_Task(u8 dT_ms);
void GPS_Data_Processing_Task6(u8 dT_ms);
void Drv_GpsPin_Init6(void);
float get_declination(float lat, float lon);
#endif  

 



