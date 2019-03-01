#ifndef _INCLUDE_H_
#define _INCLUDE_H_
#include "flash.h"
#include "stm32f4xx.h"
#include "scheduler.h"
#include "time.h"
#include "init.h"
#include "parameter.h"
#include "pwm_in.h"
#include "usart.h"
#include "usbd_user_hid.h"
#include "data_transfer.h"
#include "pos_ctrl.h"
#include "dma.h"
#include "stm32f4xx_dma.h"
#include "eso.h"
#include "fly_mode.h"
#include "ano_of.h"
extern u8 ble_imu_force;
extern u8 mcuID[3];
extern int id_chip;
#define TUNNING_DRONE_CHIP_ID 0x2F

//#define M_DRONE_380X4S //IMAV MAP 1
#define IMAV1 121

//#define M_DRONE_380X4  //IMAV SEARCH 2
#define IMAV2 154

//#define M_DRONE_380X4S2  //IMAV MAP 3  MINE
#define IMAV3 96

#define HANX6_BIGX6
//#define HANX6_BIG
//#define HANX6//MINI
//#define M_DRONE250X4
//#define M_DRONE_330X6
//#define M_DRONE_330 
//#define M_DRONE_PX4
//#define M_CAR

//-----------------------------------------------------------------------
#if defined(HANX6_BIG)
#define PX4_SDK 0  //PX4 外部控制
#define USE_MINI_FC_FLOW_BOARD 0 //使用MINI——OLDX——FLOW板子
#define USE_VER_8_PWM 1  //Car and new boardVer F2
#define USE_VER_8 0  //Car and new boardVer F2
#define USE_VER_7 1
#define USE_OLDX_REMOTE 0

//#define BLDC_PAN
#define W2C_MARK
#define USE_CIRCLE 0
//#define USE_LED 
//#define AUTO_MISSION    //使用自动任务API
//#define USE_SAFE_SWITCH

//#define USE_KF	
#define USE_UWB
#define MAXMOTORS 		(12)		//电机数量
#define FLASH_USE_STM32 0 
#define YAW_INVER 	 0														//YAW反向控制
#define YAW_FC 0
#define MAX_CTRL_ANGLE			12.0f										//遥控能达到的最大角度
#define MAX_CTRL_YAW_SPEED_RC 120										//遥控器Yaw速度限制
#define MAX_THR       80 			///%	油门通道最大占比80%，留20%给控制量

#define PIT_GAIN_INIT 1 

#define ESO_AI 2.2
#define ESO_YI 0
//
#define TRAJ1_1  2//2 circle 1 line  3 way points   4 jerk  5 car
#define HOLD_THR_PWM  LIMIT(480,0,500)
#endif

//-----------------------------------------------------------------------
#if defined(HANX6_BIGX6)
#define PX4_SDK 0  //PX4 外部控制
#define USE_MINI_FC_FLOW_BOARD 0 //使用MINI——OLDX——FLOW板子
#define USE_VER_8_PWM 1  //Car and new boardVer F2
#define USE_VER_8 0  //Car and new boardVer F2
#define USE_VER_7 1
#define USE_OLDX_REMOTE 0

//#define BLDC_PAN
#define W2C_MARK
#define USE_CIRCLE 0
//#define USE_LED 
#define AUTO_MISSION    //使用自动任务API

//#define USE_KF	
#define USE_UWB
#define MAXMOTORS 		(6)		//电机数量
#define FLASH_USE_STM32 0 
#define YAW_INVER 	 0														//YAW反向控制
#define YAW_FC 1
#define MAX_CTRL_ANGLE			15.0f										//遥控能达到的最大角度
#define MAX_CTRL_YAW_SPEED_RC 200										//遥控器Yaw速度限制
#define CAMERA_INVERT 0
#define MAX_THR       80 			///%	油门通道最大占比80%，留20%给控制量

#define ESO_AI 0
#define ESO_YI 0
#define PIT_GAIN_INIT 1
//
#define TRAJ1_1  2//2 circle 1 line  3 way points   4 jerk  5 car
#define HOLD_THR_PWM  LIMIT(400,0,500)
#endif


//-----------------------------------------------------------------------
#if defined(HANX6)
#define PX4_SDK 0  //PX4 外部控制
#define USE_MINI_FC_FLOW_BOARD 0 //使用MINI——OLDX——FLOW板子
#define USE_VER_8_PWM 1  //Car and new boardVer F2
#define USE_VER_8 0  //Car and new boardVer F2
#define USE_VER_7 1
#define USE_OLDX_REMOTE 0

//#define BLDC_PAN
#define W2C_MARK
#define USE_CIRCLE 0
//#define USE_LED 
#define AUTO_MISSION    //使用自动任务API

//#define USE_KF	
#define USE_UWB
#define MAXMOTORS 		(6)		//电机数量
#define FLASH_USE_STM32 0 
#define YAW_INVER 	 0														//YAW反向控制
#define YAW_FC 1
#define MAX_CTRL_ANGLE			15.0f										//遥控能达到的最大角度
#define MAX_CTRL_YAW_SPEED_RC 200										//遥控器Yaw速度限制
#define CAMERA_INVERT 0
#define MAX_THR       80 			///%	油门通道最大占比80%，留20%给控制量

#define ESO_AI 3.5
#define ESO_YI 0
#define PIT_GAIN_INIT 1.5
//
#define TRAJ1_1  2//2 circle 1 line  3 way points   4 jerk  5 car
#define HOLD_THR_PWM  LIMIT(400,0,500)
#endif

//---------------------------------1-------------------------------------
#if defined(M_DRONE_380X4S)//IMAV MAP
#define PX4_SDK 0  //PX4 外部控制
//#define PX4_LINK
#define USE_MINI_FC_FLOW_BOARD 0 //使用MINI——OLDX——FLOW板子
#define USE_VER_8_PWM 1  //Car and new boardVer F2
#define USE_VER_8 0  //Car and new boardVer F2
#define USE_VER_7 1
#define USE_OLDX_REMOTE 0

#define BLDC_PAN
#define W2C_MARK
#define USE_CIRCLE 0
//#define USE_LED 
//#define USE_CARGO 
#define AUTO_MISSION    //使用自动任务API

#define USE_KF	
#define USE_UWB
#define MAXMOTORS 		(4)		//电机数量
#define FLASH_USE_STM32 1 
#define YAW_INVER 	  0														//YAW反向控制
#define YAW_FC 0
#define MAX_CTRL_ANGLE			30.0f										//遥控能达到的最大角度

#define ESO_AI 22
#define ESO_YI 22
//
#define TRAJ1_1  2//2 circle 1 line  3 way points   4 jerk  5 car
#define HOLD_THR_PWM  LIMIT(500,0,500)
#endif
//--------------------------------2--------------------------------------
#if defined(M_DRONE_380X4)//IMAV SERACH
#define PX4_SDK 0  //PX4 外部控制
//#define PX4_LINK
#define USE_MINI_FC_FLOW_BOARD 0 //使用MINI——OLDX——FLOW板子
#define USE_VER_8_PWM 1  //Car and new boardVer F2
#define USE_VER_8 0  //Car and new boardVer F2
#define USE_VER_7 1
#define USE_OLDX_REMOTE 0

//#define BLDC_PAN
#define W2C_MARK
#define USE_CIRCLE 0
//#define USE_LED 
#define USE_CARGO 
#define AUTO_MISSION    //使用自动任务API

#define USE_KF	
#define USE_UWB
#define MAXMOTORS 		(4)		//电机数量
#define FLASH_USE_STM32 0 
#define YAW_INVER 	  0														//YAW反向控制
#define YAW_FC 0
#define MAX_CTRL_ANGLE			30.0f										//遥控能达到的最大角度

#define ESO_AI 22
#define ESO_YI 22
//
#define TRAJ1_1  2//2 circle 1 line  3 way points   4 jerk  5 car
#define HOLD_THR_PWM  LIMIT(500,0,500)
#endif
//-----------------------------3----------------------------------------
#if defined(M_DRONE_380X4S2)//IMAV MAP2
#define PX4_SDK 0  //PX4 外部控制
//#define PX4_LINK
#define USE_MINI_FC_FLOW_BOARD 0 //使用MINI——OLDX——FLOW板子
#define USE_VER_8_PWM 1  //Car and new boardVer F2
#define USE_VER_8 0  //Car and new boardVer F2
#define USE_VER_7 1
#define USE_OLDX_REMOTE 0

#define BLDC_PAN
#define W2C_MARK
#define USE_CIRCLE 0
//#define USE_LED 
//#define USE_CARGO 
#define AUTO_MISSION    //使用自动任务API

//#define USE_KF	
#define USE_UWB
#define MAXMOTORS 		(4)		//电机数量
#define FLASH_USE_STM32 0 
#define YAW_INVER 	  0														//YAW反向控制
#define YAW_FC 0
#define MAX_CTRL_ANGLE			30.0f										//遥控能达到的最大角度
#define MAX_CTRL_YAW_SPEED_RC 300										//遥控器Yaw速度限制
#define CAMERA_INVERT 1
#define MAX_THR       80 			///%	油门通道最大占比80%，留20%给控制量

#define PIT_GAIN_INIT 1
#define ESO_AI 30
#define ESO_YI 22
//
#define TRAJ1_1  2//2 circle 1 line  3 way points   4 jerk  5 car
#define HOLD_THR_PWM  LIMIT(450,0,550)
#endif
//-----------------------------------------------------------------------
#if defined(M_DRONE_330X6)
#define PX4_SDK 0  //PX4 外部控制
#define PX4_LINK
#define USE_MINI_FC_FLOW_BOARD 0 //使用MINI——OLDX——FLOW板子
#define USE_VER_8_PWM 1  //Car and new boardVer F2
#define USE_VER_8 0  //Car and new boardVer F2
#define USE_VER_7 1
#define USE_OLDX_REMOTE 0

//#define BLDC_PAN
#define W2C_MARK
#define USE_CIRCLE 0
#define USE_LED 
#define AUTO_MISSION    //使用自动任务API

#define USE_UWB
#define MAXMOTORS 		(6)		//电机数量
#define FLASH_USE_STM32 0 
#define YAW_INVER 	  0														//YAW反向控制
//
#define TRAJ1_1  2//2 circle 1 line  3 way points   4 jerk  5 car
#define HOLD_THR_PWM  LIMIT(450,0,500)
#endif
//-----------------------------------------------------------------------
#if defined(M_DRONE_330)
#define PX4_SDK 0  //PX4 外部控制
#define PX4_LINK
#define USE_MINI_FC_FLOW_BOARD 0 //使用MINI——OLDX——FLOW板子
#define USE_VER_8_PWM 1  //Car and new boardVer F2
#define USE_VER_8 0  //Car and new boardVer F2
#define USE_VER_7 1
#define USE_OLDX_REMOTE 0

#define BLDC_PAN
#define W2C_MARK
#define USE_CIRCLE 0

#define USE_UWB
#define MAXMOTORS 		(4)		//电机数量
#define FLASH_USE_STM32 1 
#define YAW_INVER 	  0																//YAW反向控制
//
#define TRAJ1_1  2//2 circle 1 line  3 way points   4 jerk  5 car
#define HOLD_THR_PWM  LIMIT(450,0,500)
#endif
//-----------------------------------------------------------------------
#if defined(M_DRONE_PX4)
#define PX4_SDK 1  //PX4 外部控制
#define USE_MINI_FC_FLOW_BOARD 0 //使用MINI——OLDX——FLOW板子
#define USE_VER_8 0  //Car and new boardVer F2
#define USE_VER_7 1
#define MAXMOTORS 		(4)		//电机数量
#endif
//-----------------------------------------------------------------------
#if defined(M_CAR)
#define PX4_SDK 1  //PX4 外部控制
#define USE_MINI_FC_FLOW_BOARD 0 //使用MINI——OLDX——FLOW板子
#define USE_VER_8 1  //Car and new boardVer F2
#define USE_VER_7 1
#define MAXMOTORS 		(4)		//电机数量
#endif

//================测试程序==============
//--------------任选1个------------------
//#define  HEIGHT_TEST  // 高度自动控制
//#define  POS_TEST     // 位置自动控制
//#define  POS_SPD_TEST   // 速度自动控制
//#define  AUTO_DOWN    // 自动降落
//#define  AUTO_MAPPER   // 自动测绘
//#define  PID_TUNNING   // 姿态调参模式
//#define  AUTO_HOME     // 回航测试
//#define  ROBOT_LAND_TEST    // 移动平台降落测试
//#define  INDOOR_FPV_SERCH
//#define  WAY_POINT_TEST



//#define  SD_SAVER        // 使能AUX2通道作为SD卡存储使能
//#define  DEBUG_MODE    //测试模式 PWM不输出 可与其他模式互选
//#define  DEBUG_INROOM
//#define FAN_IS_3AXIS 0
//================系统===================

#define USE_VER_6 0//iic mems && fc with sbus
#define USE_VER_5 0
#if USE_VER_7
#define USE_VER_4 1 
#else
#define USE_VER_4 0 //spi baro
#endif
#if USE_VER_4||USE_VER_6
#define USE_VER_3 1
#else
#define USE_VER_3 0 //
#endif
#define USE_MINI_FC_FLOW_BOARD_BUT_USB_SBUS 1
#define USE_RECIVER_MINE 0  
#define EN_ATT_CAL_FC 1  //姿态使用FC解算
#define USE_ANO_GROUND 1  //实验匿名地面站 否则使用  安卓APP 和 OLDX-PC上位机
#define USE_HT_GROUND  0  //与匿名互选 则使用HTlink协议
#define USE_MINI_BOARD  1  //使用新OLD-X 飞控板
#define USE_BLE_FOR_APP 1  //使用蓝牙
#define USE_FAN_AS_BLDC 0  //使用单旋翼四轴
#define USE_ANO_FLOW 0
#define USE_ZIN_BMP 0  
#define USE_M100_IMU 0  
#define USE_FLOW_PI 0  //使用OLDX光流模块

#define TUNING_ONE_AXIX 0
#define TUNING_X 0   //0->y 1->x   
#define TUNING_Z 0   //higher than tuning x or y
#define SONAR_USE_FC 0  //FC采集超声波USE IDLE 串口
#define SONAR_USE_FC1 0  //FC采集超声波USE ODROID 串口
#define USE_US100           //使用us100型号超声波 
 


#define GET_TIME_NUM 	(10)		//设置获取时间的数组数量
#define CH_NUM 				(8) 	//接收机通道数量
#define USE_TOE_IN_UNLOCK 1 // 0：默认解锁方式，1：外八解锁方式
/***************中断优先级******************/
#define NVIC_GROUP NVIC_PriorityGroup_3		//中断分组选择
#define NVIC_PWMIN_P			1		//接收机采集
#define NVIC_PWMIN_S			1
#define NVIC_TIME_P       2		//暂未使用
#define NVIC_TIME_S       0
#define NVIC_UART_P				5		//暂未使用
#define NVIC_UART_S				1
#define NVIC_UART2_P			3		//串口2中断
#define NVIC_UART2_S			1
//================传感器===================
#define IMU_HML_ADD_500 0          
#define ACC_ADJ_EN 									//是否允许校准加速度计,(定义则允许)

#define OFFSET_AV_NUM 	50					//校准偏移量时的平均次数。
#define FILTER_NUM 			10					//滑动平均滤波数值个数

#define TO_ANGLE 				0.06103f 		//0.061036 //   4000/65536  +-2000   ???

#define FIX_GYRO_Y 			1.02f				//陀螺仪Y轴固有补偿
#define FIX_GYRO_X 			1.02f				//陀螺仪X轴固有补偿

#define TO_M_S2 				0.23926f   	//   980cm/s2    +-8g   980/4096
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	角度转弧度

#define MAX_ACC  4096.0f						//+-8G		加速度计量程
#define TO_DEG_S 500.0f      				//T = 2ms  默认为2ms ，数值等于1/T

#define LON 0
#define LAT 1

#define ROLr 0
#define PITr 1
#define THRr 2
#define YAWr 3
#define AUX1r 4 
#define AUX2r 5
#define AUX3r 6
#define AUX4r 7

#define X 0
#define Y 1

#define Xr 0
#define Yr 1
#define Zr 2

#define East 0
#define North 1

#define RC_PITCH 0
#define RC_ROLL  1
#define RC_THR   2
#define RC_YAW   3

#define RC_GEAR   4
#define RC_MODE   6

//============== DMA使能=========================
#define EN_DMA_UART1 1  //UPLOAD
#define EN_DMA_UART2 0  //FLOW
#define EN_DMA_UART3 0  //GPS
#define EN_DMA_UART4 1 //SD
//================控制=====================
#define MAX_VERTICAL_SPEED_UP  5000										//最大上升速度mm/s
#define MAX_VERTICAL_SPEED_DW  3000										//最大下降速度mm/s

#define USE_RC_INVERT 0  														//反向控制
#define ANGLE_TO_MAX_AS 		30.0f										//角度误差N时，期望角速度达到最大（可以通过调整CTRL_2的P值调整）
#define CTRL_2_INT_LIMIT 		0.5f *MAX_CTRL_ANGLE		//外环积分幅度

#define MAX_CTRL_ASPEED 	 	300.0f									//ROL,PIT允许的最大控制角速度
#define MAX_CTRL_YAW_SPEED 	225//225.0f									//YAW允许的最大控制角速度
#define CTRL_1_INT_LIMIT 		0.5f *MAX_CTRL_ASPEED		//内环积分幅度

#define MAX_PWM				100			///%	最大PWM输出为100%油门
#define READY_SPEED   20			///%	解锁后电机转速20%油门
//=========================================
extern float att_test[2];
enum
{
 A_X = 0,
 A_Y ,
 A_Z ,
 G_Y ,
 G_X ,
 G_Z ,
 TEM ,
 ITEMS ,
};

// CH_filter[],0横滚，1俯仰，2油门，3航向		
enum
{
 ROL= 0,
 PIT ,
 THR ,
 YAW ,
 AUX1 ,
 AUX2 ,
 AUX3 ,
 AUX4 ,
};


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


#endif

