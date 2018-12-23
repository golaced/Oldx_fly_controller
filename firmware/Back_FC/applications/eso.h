#ifndef _ESO_H_
#define	_ESO_H_
#include "math.h"
#include "stm32f4xx.h"
/*
Copyright(C) 2017 DianYin Innovations Corporation. All rights reserved.

OLDX飞控使用 ADRC controller 自抗扰控制器 designed by Golaced from 云逸创新(滇鹰科技)

qq群:
567423074
淘宝店铺：
https://shop124436104.taobao.com/?spm=2013.1.1000126.2.iNj4nZ
手机店铺地址：
http://shop124436104.m.taobao.com

ADRC.lib 封装了姿态控制和位置控制库函数，具体实现可阅读韩老的自抗扰原著
函数v是控制系统的输入，y是控制系统的输出，反馈给ESO，u是ADRC的输出控制量
注意：本控制器不对使用后造成的炸鸡负责
*/

#define ESO_PARA_USE_REAL_TIME 1
typedef struct
{ //control parameter
	float KP,KD,KI;
	float b0,b01;
	float err_limit;
	float eso_dead;
	//mode
	u8 init;
	u8 level;
	u8 not_use_px4;
	u8 auto_b0;
	u8 out_mode,use_td;
	u8 eso_for_z;
	//adrc
	u8 n; 
	float h0;
	float z[3],e;
	float disturb,disturb_u,disturb_u_reg;
	float beta0,beta1,beta2,beta3;
	float alfa1,alfa2,alfa0,tao;	
	float h,integer;
	float v1,v2,r0,h1,r1,c,u;
	//safe check
	u8 Thr_Low;
	float Thr_Weight;
}ESO;

extern ESO eso_pos[3],eso_pos_spd[3];
extern ESO eso_att_outter_c[4],eso_att_inner_c[4];

void OLDX_SMOOTH_IN_ESO(ESO *eso_in,float in);
float OLDX_AUTO_B0(ESO *eso_in,float v,float y,float u,float T,float MAX);
float OLDX_ATT_CONTRL_OUTER_ESO(ESO *eso_in,float v,float y,float u,float T,float MAX,float ero_px4,float kp_in,u16 thr);
float OLDX_ATT_CONTRL_INNER_ESO(ESO *eso_in,float v,float y,float u,float T,float MAX,float kp_in,u16 thr);
float OLDX_POS_CONTROL_ESO(ESO *eso_in,float v,float y,float u,float T,float MAX,float kp_in,u16 thr);
#endif

