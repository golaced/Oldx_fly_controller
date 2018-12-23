#include "include.h"
#include "imu.h"
#include "error.h"
#include "alt_kf.h"
ERO ero;
SYS_STATE sys;
#define ERO_ANGLE 12
#define CHECK_ERO_NUM_ATT 8
#define CHECK_TIME_ATT 3
int signe(float in)
{
if(in>0)
	return 1;
else
	return -1;
}

void att_ero_check(void){
	u8 i;
	static u8 state[3];
	static float angle_reg[3];
	static u8 cnt_1s[3];
	static int flag[3];
	static int cnt_over[3];
	
if(ero.ero_rst_att)	{
	ero.ero_rst_att=0;
	for(i=0;i<3;i++)	
		cnt_over[i]=0;
	
for(i=0;i<3;i++)	
		ero.ero_att=0;
}



	switch(state[0]){
		case 0:
			if(fabs(Pitch)>ERO_ANGLE){
			angle_reg[0]=Pitch;
			flag[0]=signe(Pitch);
		  state[0]=1;cnt_1s[0]=0;}
			break;
		case 1:
			if(cnt_1s[0]++>20*CHECK_TIME_ATT)
			{cnt_over[0]-=2;state[0]=0;}
			else {
				if((flag[0]>0&&Pitch<-ERO_ANGLE)||(flag[0]<0&&Pitch>ERO_ANGLE))
				{cnt_over[0]++;state[0]=0;}
			}
		 break;
	}	
	
		switch(state[1]){
		case 0:
			if(fabs(Roll)>ERO_ANGLE){
			angle_reg[1]=Roll;
			flag[1]=signe(Roll);
		  state[1]=1;cnt_1s[1]=0;}
			break;
		case 1:
			if(cnt_1s[1]++>20*2)
			{cnt_over[1]-=2;state[1]=0;}
			else {
				if((flag[1]>0&&Roll<-ERO_ANGLE)||(flag[1]<0&&Roll>ERO_ANGLE))
				{cnt_over[1]++;state[1]=0;}
			}
		 break;
	}	
		
//-------------------	
for(i=0;i<3;i++)	{
	if(cnt_over[i]<0)
		cnt_over[i]=0;
	else if(cnt_over[i]>CHECK_ERO_NUM_ATT)
		cnt_over[i]=CHECK_ERO_NUM_ATT;}
	
for(i=0;i<3;i++)	{
	if(cnt_over[i]>=CHECK_ERO_NUM_ATT)
		ero.ero_att=1;	}
}

#define ERO_HIGHT_SPEED 550/1000
#define CHECK_ERO_NUM_HIGHT 5
#define CHECK_TIME_HIGHT 3
void hight_ero_check(void){
		u8 i;
	static u8 state[3];
	static int angle_reg[3];
	static u8 cnt_1s[3];
	static int cnt_over[3];
	static int flag[3];
if(ero.ero_rst_h)	{
	ero.ero_rst_h=0;
	for(i=0;i<3;i++)	
		cnt_over[i]=0;
	
		ero.ero_rst_h=0;
}



	switch(state[0]){
		case 0:
			if(fabs(-ALT_VEL_BMP)>ERO_HIGHT_SPEED){
			angle_reg[0]=-ALT_VEL_BMP;
			flag[0]=signe(-ALT_VEL_BMP);
		  state[0]=1;cnt_1s[0]=0;}
			break;
		case 1:
			if(cnt_1s[0]++>20*CHECK_TIME_HIGHT)
			{cnt_over[0]-=2;state[0]=0;}
			else {
				if((flag[0]>0&&-ALT_VEL_BMP<-ERO_HIGHT_SPEED)||(flag[0]<0&&-ALT_VEL_BMP>ERO_HIGHT_SPEED))
				{cnt_over[0]++;state[0]=0;}
			}
		 break;
	}	
	
//-------------------	
for(i=0;i<3;i++)	
	if(cnt_over[i]<0)
		cnt_over[i]=0;
	else if(cnt_over[i]>CHECK_ERO_NUM_HIGHT)
		cnt_over[i]=CHECK_ERO_NUM_HIGHT;
	
for(i=0;i<3;i++)	
	if(cnt_over[i]>=CHECK_ERO_NUM_HIGHT)
		ero.ero_hight=1;	
	
	
}