#include "usart_fc.h"
#include "include.h"
void BubbleSort(u16 *R,u16 n)
   { 
     int i,j;
     u8 exchange;
     for(i=1;i<n;i++){
       exchange=0; 
       for(j=n-1;j>=i;j--) 
        if(R[j+1]<R[j]){
          R[0]=R[j+1]; 
          R[j+1]=R[j];
          R[j]=R[0];
          exchange=1;
         }
       if(!exchange) 
             return;
     } 
    } 
#define DIV_NUM 8 
u16 Laser_avoid[60];
float flt_laser=0.8;
float rate_use[2]={0.16,0.66};
void Laser_cal(void)
{
u32 temp;
u16 i;
u8 j;	
u16 k;
u8 cnt;	
Laser_avoid[0]=DIV_NUM;
u16 laser_buf_temp[360];
u16 bufs[360/DIV_NUM];
for (i=0;i<360;i++)
	{
		laser_buf_temp[i]=laser_buf[(i-360/DIV_NUM/2)%360];
	}	
	
for(j=0;j<DIV_NUM;j++){
			temp=0;
	    cnt=0;
	for(i=j*360/DIV_NUM;i<j*360/DIV_NUM+360/DIV_NUM;i++)
      {  
				if(laser_buf_temp[i]!=0)
				bufs[cnt++]=laser_buf_temp[i];
			}
			BubbleSort(bufs,cnt);
			cnt=0;
			for(i=360/DIV_NUM*rate_use[0];i<360/DIV_NUM*(1-rate_use[1]);i++)
      {  		
				 temp+=bufs[i];
				 cnt++;
			}	
				Laser_avoid[j+1]=flt_laser*temp/(cnt+1)+(1-flt_laser)*Laser_avoid[j+1];

}
}