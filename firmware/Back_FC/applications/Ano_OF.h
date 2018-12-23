#ifndef __ANO_OF_H_
#define __ANO_OF_H_

#include "stm32f4xx.h"
//以下为全局变量，在其他文件中，引用本h文件，即可在其他文件中访问到以下变量
//光流信息质量：QUA
//光照强度：LIGHT
extern uint8_t 	OF_QUA,OF_LIGHT;
//原始光流信息，具体意义见光流模块手册
extern int8_t	OF_DX,OF_DY;
//融合后的光流信息，具体意义见光流模块手册
extern int16_t	OF_DX2,OF_DY2,OF_DX2FIX,OF_DY2FIX;
//原始高度信息和融合后高度信息
extern uint16_t	OF_ALT,OF_ALT2;
//原始陀螺仪数据
extern int16_t	OF_GYR_X,OF_GYR_Y,OF_GYR_Z;
//滤波后陀螺仪数据
extern int16_t	OF_GYR_X2,OF_GYR_Y2,OF_GYR_Z2;
//原始加速度数据
extern int16_t	OF_ACC_X,OF_ACC_Y,OF_ACC_Z;
//滤波后加速度数据
extern int16_t	OF_ACC_X2,OF_ACC_Y2,OF_ACC_Z2;
//欧拉角格式的姿态数据
extern float	OF_ATT_ROL,OF_ATT_PIT,OF_ATT_YAW;
//四元数格式的姿态数据
extern float	OF_ATT_S1,OF_ATT_S2,OF_ATT_S3,OF_ATT_S4;


 typedef struct
{
 float spdx,spdy;
 float o_h,h;
 char  qual;

 float k_f;	
}ANOF;
extern ANOF ano_flow;

//本函数是唯一一个需要外部调用的函数，因为光流模块是串口输出数据
//所以本函数需要在串口接收中断中调用，每接收一字节数据，调用本函数一次
void AnoOF_GetOneByte(uint8_t data);

#endif
