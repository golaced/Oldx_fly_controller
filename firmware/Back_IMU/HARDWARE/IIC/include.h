#ifndef _INCLUDE_H_
#define _INCLUDE_H_

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stm32f4xx.h>	
#include "ekf_ins.h"
#include "time.h"
#include "mpu6050.h"
#include "parameter.h"
#include "delay.h" 
#include "malloc.h"   
#include "usart_fc.h"   
#include "spi_nrf.h"							
#include "rc_mine.h"						
#include "nrf.h"							
#include "ultrasonic.h"
#include "rc.h"
#include "att.h"
#include "height_ctrl.h"
#include "flash.h"
#include "dma.h"
#define DRONE_330X6
//#define DRONE_330

#if defined(DRONE_330X6)
#define USE_OUTER_LINK 0 //使用串口1作为外部2号数传
#define USE_GPS_U4 0     //使用串口IMU作为外GPS
#define USE_UWB_AS_POS 0//使用 UWB作为全局位置测量
//#define USE_WIFI_CONTROL  //使用正点原子wifi模块
#define NO_HML_CORRECT 1
#endif

#if defined(DRONE_330)
#define USE_OUTER_LINK 1 //使用串口1作为外部2号数传
#define USE_GPS_U4 1     //使用串口IMU作为外GPS
#define USE_UWB_AS_POS 1//使用 UWB作为全局位置测量
//#define USE_WIFI_CONTROL  //使用正点原子wifi模块
#endif

//================SYSTEM==================
#define NVIC_GROUP NVIC_PriorityGroup_2		//中断分组选择
#define EN_DMA_UART1 0  //UPLOAD
#define EN_DMA_UART2 1  //GOL_LINK
#define EN_DMA_UART3 0  //GPS
#define EN_DMA_UART4 0 //SD
/***********************************************/
//================SENSORS===================
#define IMU_UPDATE 1  //使用LSD IMU
#define USE_MINI_BOARD 1 //使用LSD IMU
#define NEW_IMU 1  //使用LSD IMU

#define USE_VER_5 0//Finalv1硬件 IIC mems
#define USE_VER_4 1//过渡版 SPI mems  FinalV2
#if USE_VER_4
#define USE_VER_3 1//SPI mems
#else
#define USE_VER_3 0//第一版 IIC mems
#endif

//------------FUSION PARAM----------
#define PX4_USE_FLOW 1 //px4连接下用光流
#define USE_UKF_FROM_AUTOQUAD 0
//姿态位置融合使用UKF
#if USE_UKF_FROM_AUTOQUAD
#define UKF_IN_ONE_THREAD 0
//单独一个线程跑完所以融合
#else
#define UKF_IN_ONE_THREAD 0 
#endif

#define USE_M100_IMU 0  //使用DJI SDK数据
#define USE_LASER_AVOID 0

#define USE_CYCLE_HML_CAL  1//1->使用椭球拟合
#define GET_TIME_NUM 	(30)		//设置获取时间的数组数量
//-------------SONAR PARAM-----------
//#define USE_LIDAR             //使用北xing雷达
#define USE_US100           //使用us100型号超声波 
//#define URM07
//#define USE_KS103					//使用ks103型号超声波
//#define SONAR_SAMPLE1					//0-5m 32ms  no fix
//#define SONAR_SAMPLE2					//0-5m 100ms T fixf
#define SONAR_SAMPLE3					//0-11m 68ms no fix
//#define SONAR_USE_SCL  
//#define SONAR_USE_TIG
#define SONAR_USE_UART 

#define SONAR_HEIGHT 80
#if !defined(USE_WIFI_CONTROL)
#define USE_IMU_BACK_IO_AS_SONAR 0 //SONAR口故障下超声波采集
#endif
//---------------------FLOW PARAM---------------------------------
#define FLOW_SET_ANGLE 0 //光流安装角度ANO
#define FLOW_SET_ANGLE1 180*0.0173 //全局光流旋转角度
#define SONAR_USE_FLOW 0 //使用PixFLow光流的超声波
#define SENSOR_FORM_PI_FLOW_SONAR_NOT 1 //使用OLDX-AMF但是高度使用超声波

#define SENSOR_FORM_PI_FLOW 0  //使用OLDX-AMF 二维码SLAM模块
#define USE_ANO_FLOW 0 //使用匿名光流
#define FLOW_USE_IIC 0  //Px4 IIc 光流
#define FLOW_USE_OPENMV 0//Openmv Oldx 光流
#define USE_FLOW_FLY_ROBOT 1//使用飞行实验室的光流模块


//===================PARAM==================
#define PI                     3.14159265359f //圆周率
#define RAD_DEG            		 57.2957795056f //弧度转化成角度的比例因子
#define DEG_RAD                0.01745329252f //角度转化成弧度的比例因子
#define GRAVITY_MSS                  9.80665f //地球重力加速度
#define earthRate                0.000072921f //地球自转角速度
#define earthRadius                6378145.0f //地球半径
#define earthRadiusInv          1.5678540e-7f //地球半径的倒数

#define OFFSET_AV_NUM 	50					//校准偏移量时的平均次数。
#define FILTER_NUM 			10					//滑动平均滤波数值个数

#define TO_ANGLE 				0.06103f 		//0.061036 //   4000/65536  +-2000   ???

#define FIX_GYRO_Y 			1.02f				//陀螺仪Y轴固有补偿
#define FIX_GYRO_X 			1.02f				//陀螺仪X轴固有补偿

#define TO_M_S2 				0.23926f   	//   980cm/s2    +-8g   980/4096
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	角度转弧度

#define MAX_ACC  4096.0f						//+-8G		加速度计量程
#define TO_DEG_S 500.0f      				//T = 2ms  默认为2ms ，数值等于1/T

#define RtA 		57.324841				
#define AtR    	0.0174533				
#define Acc_G 	0.0011963				
#define Gyro_G 	0.03051756				
#define Gyro_Gr	0.0005426

#define A_X 0
#define A_Y 1
#define A_Z 2
#define G_Y 3
#define G_X 4
#define G_Z 5
#define TEM 6
#define ITEMS 7

#define ROLr 0
#define PITr 1
#define THRr 2
#define YAWr 3
#define AUX1r 4 
#define AUX2r 5
#define AUX3r 6
#define AUX4r 7

enum
{
	SETBIT0 = 0x0001,  SETBIT1 = 0x0002,	SETBIT2 = 0x0004,	 SETBIT3 = 0x0008,
	SETBIT4 = 0x0010,	 SETBIT5 = 0x0020,	SETBIT6 = 0x0040,	 SETBIT7 = 0x0080,
	SETBIT8 = 0x0100,	 SETBIT9 = 0x0200,	SETBIT10 = 0x0400, SETBIT11 = 0x0800,
	SETBIT12 = 0x1000, SETBIT13 = 0x2000,	SETBIT14 = 0x4000, SETBIT15 = 0x8000		
};
//CLR BIT.    Example: a &= CLRBIT0
enum
{
	CLRBIT0 = 0xFFFE,  CLRBIT1 = 0xFFFD,	CLRBIT2 = 0xFFFB,	 CLRBIT3 = 0xFFF7,	
	CLRBIT4 = 0xFFEF,	 CLRBIT5 = 0xFFDF,	CLRBIT6 = 0xFFBF,	 CLRBIT7 = 0xFF7F,
	CLRBIT8 = 0xFEFF,	 CLRBIT9 = 0xFDFF,	CLRBIT10 = 0xFBFF, CLRBIT11 = 0xF7FF,
	CLRBIT12 = 0xEFFF, CLRBIT13 = 0xDFFF,	CLRBIT14 = 0xBFFF, CLRBIT15 = 0x7FFF
};
//CHOSE BIT.  Example: a = b&CHSBIT0
enum
{
	CHSBIT0 = 0x0001,  CHSBIT1 = 0x0002,	CHSBIT2 = 0x0004,	 CHSBIT3 = 0x0008,
	CHSBIT4 = 0x0010,	 CHSBIT5 = 0x0020,	CHSBIT6 = 0x0040,	 CHSBIT7 = 0x0080,
	CHSBIT8 = 0x0100,	 CHSBIT9 = 0x0200,	CHSBIT10 = 0x0400, CHSBIT11 = 0x0800,
	CHSBIT12 = 0x1000, CHSBIT13 = 0x2000,	CHSBIT14 = 0x4000, CHSBIT15 = 0x8000		
};

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).M4同M3类似,只是寄存器地址变了.
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //输出 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入

typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef unsigned char  uint8;                   /* defined for unsigned 8-bits integer variable 	无符号8位整型变量  */
typedef signed   char  int8;                    /* defined for signed 8-bits integer variable		有符号8位整型变量  */
typedef unsigned short uint16;                  /* defined for unsigned 16-bits integer variable 	无符号16位整型变量 */
typedef signed   short int16;                   /* defined for signed 16-bits integer variable 		有符号16位整型变量 */
typedef unsigned int   uint32;                  /* defined for unsigned 32-bits integer variable 	无符号32位整型变量 */
typedef signed   int   int32;                   /* defined for signed 32-bits integer variable 		有符号32位整型变量 */
typedef float          fp32;                    /* single precision floating point variable (32bits) 单精度浮点数（32位长度） */
typedef double         fp64;                    /* double precision floating point variable (64bits) 双精度浮点数（64位长度） */


typedef struct 
{ u8 pi_flow;
	u8 fc;
	u8 acc;
	u8 gyro;
	u8 hml,outer_hml;
	u8 gps;
	u8 sonar;
	u8 flow;
	u8 flow_iic;
	u8 laser;
	u8 dji;
}SYSTEM;
extern u8 fly_ready,force_Thr_low;
extern SYSTEM module;
#endif

