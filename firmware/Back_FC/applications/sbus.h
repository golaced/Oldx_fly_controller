//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，不对使用该程序造成飞行器失控负责
//OLDX-AutoPilot
//SBUS解码程序
//创建日期:2017/4/15
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 北理云逸创新团队 2016-2024
//All rights reserved
//********************************************************************************


////////////////////////////////////////////////////////////////////////////////// 	 

#include "stm32f4xx.h"
extern int16_t channels[18];//SBUS通道输出
extern uint8_t failsafe_status;//遥控器信号状态
//------------------------------------------------//
extern uint8_t sbus_data_i[26];
extern uint8_t sbus_data[26];
extern uint8_t sbus_passthrough;
#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
//------------------------------------------------//
void oldx_sbus_rx(u8 com_data);//将此函数放到SBUS串口中断