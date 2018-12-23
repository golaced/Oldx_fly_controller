

#include "include.h"
#include "rc.h"
#include "my_math.h"
#include "mpu6050.h"
#include "height_ctrl.h"
#include "ultrasonic.h"
s8 CH_in_Mapping[CH_NUM] = {0,1,2,3,4,5,6,7};    //通道映射
u16 RX_CH[CH_NUM];
int RX_CH_FIX[4];

void CH_Mapping_Fun(u16 *in,u16 *Mapped_CH)
{
	u8 i;
	for( i = 0 ; i < CH_NUM ; i++ )
	{
		*( Mapped_CH + i ) = *( in + CH_in_Mapping[i] );
	}
}

s16 CH[CH_NUM];

float CH_Old[CH_NUM];
float CH_filter[CH_NUM];
float CH_filter_Old[CH_NUM];
float CH_filter_D[CH_NUM];
u8 NS,CH_Error[CH_NUM];
u16 NS_cnt,CLR_CH_Error[CH_NUM];
 
s16 MAX_CH[CH_NUM]  = {1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 };	//摇杆最大
s16 MIN_CH[CH_NUM]  = {1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 };	//摇杆最小
char CH_DIR[CH_NUM] = {0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    };  //摇杆方向
#define CH_OFFSET 500


float filter_A;

void RC_Duty( float T , u16 tmp16_CH[CH_NUM] )
{
	u8 i;
	s16 CH_TMP[CH_NUM];
	static u16 Mapped_CH[CH_NUM];

	if( NS == 1 )
	{
		CH_Mapping_Fun(tmp16_CH,Mapped_CH);
	}
	else if( NS == 2 )
	{
		CH_Mapping_Fun(RX_CH,Mapped_CH);
	}
	
	for( i = 0;i < CH_NUM ; i++ )
	{
		if( (u16)Mapped_CH[i] > 2500 || (u16)Mapped_CH[i] < 500 )
		{
			CH_Error[i]=1;
			CLR_CH_Error[i] = 0;
		}
		else
		{
			CLR_CH_Error[i]++;
			if( CLR_CH_Error[i] > 200 )
			{
				CLR_CH_Error[i] = 2000;
				CH_Error[i] = 0;
			}
		}

		if( NS == 1 || NS == 2 )
		{
			if( CH_Error[i] ) //单通道数据错误
			{
				
			}
			else
			{
				//CH_Max_Min_Record();
				CH_TMP[i] = ( Mapped_CH[i] ); //映射拷贝数据，大约 1000~2000
				
				if( MAX_CH[i] > MIN_CH[i] )
				{
					if( !CH_DIR[i] )
					{
						CH[i] =   LIMIT ( (s16)( ( CH_TMP[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - CH_OFFSET ), -500, 500); //归一化，输出+-500
					}
					else
					{
						CH[i] = - LIMIT ( (s16)( ( CH_TMP[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - CH_OFFSET ), -500, 500); //归一化，输出+-500
					}
				}	
				else
				{
					fly_ready = 0;
				}
			}
		}	
		else //未接接收机或无信号（遥控关闭或丢失信号）
		{

		}
//=================== filter ===================================
//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
//=================== filter =================================== 		
			
			filter_A = 3.14f *20 *T;
			
			if( ABS(CH_TMP[i] - CH_filter[i]) <100 )
			{
				CH_filter[i] += filter_A *(CH[i] - CH_filter[i]) ;
			}
			else
			{
				CH_filter[i] += 0.5f *filter_A *( CH[i] - CH_filter[i]) ;
			}
// 					CH_filter[i] = Fli_Tmp;
			CH_filter_D[i] 	= ( CH_filter[i] - CH_filter_Old[i] );
			CH_filter_Old[i] = CH_filter[i];
			CH_Old[i] 		= CH[i];
	}
	//======================================================================
//	Fly_Ready(T);		//解锁判断
	//======================================================================
	if(++NS_cnt>200)  // 400ms  未插信号线。
	{
		NS_cnt = 0;
		NS = 0;
	}
}

u8 fly_ready = 0;
s16 ready_cnt=0;

void Fly_Ready(float T)
{
	if( CH_filter[2] < -400 )  							//油门小于10%
	{
		if( fly_ready && ready_cnt != -1 ) //解锁完成，且已退出解锁上锁过程
		{
			//ready_cnt += 1000 *T;
		}
#if(USE_TOE_IN_UNLOCK)		
		if( CH_filter[3] < -400 )							
		{
			if( CH_filter[1] > 400 )
			{
				if( CH_filter[0] > 400 )
				{
					if( ready_cnt != -1 )				   //外八满足且退出解锁上锁过程
					{
						ready_cnt += 3 *1000 *T;
					}
				}

			}

		}
#else
		if( CH_filter[3] < -400 )					      //左下满足		
		{
			if( ready_cnt != -1 && fly_ready )	//判断已经退出解锁上锁过程且已经解锁
			{
				ready_cnt += 1000 *T;
			}
		}
		else if( CH_filter[3] > 400 )      			//右下满足
		{
			if( ready_cnt != -1 && !fly_ready )	//判断已经退出解锁上锁过程且已经上锁
			{
				ready_cnt += 1000 *T;
			}
		}
#endif		
		else if( ready_cnt == -1 )						//4通道(CH[3])回位
		{
			ready_cnt=0;
		}
	}
	else
	{
		ready_cnt=0;
	}

	
	if( ready_cnt > 1000 ) // 1000ms 
	{
		ready_cnt = -1;
		//fly_ready = ( fly_ready==1 ) ? 0 : 1 ;
		if( !fly_ready )
		{
			fly_ready = 1;
			mpu6050.Gyro_CALIBRATE = 2;
		}
		else
		{
			fly_ready = 0;
		}
	}

}

void Feed_Rc_Dog(u8 ch_mode) //400ms内必须调用一次
{
	NS = ch_mode;
	NS_cnt = 0;
}

//=================== filter ===================================
//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
//=================== filter =================================== 	
u8 height_ctrl_mode = 0,height_ctrl_mode_use;
extern u8 ultra_ok;
void Mode_FC(void)
{static u8 last_height_ctrl_mode;
	if(1){
	if( !fly_ready)// || CH_filter[THR]<-400 ) //只在上锁时 以及 油门 低于10% 的时候，允许切换模式，否则只能向模式0切换。
	{
		if( RX_CH[AUX4r] < 100 )
		{
			height_ctrl_mode = 0;
		}
		else if(RX_CH[AUX4r] >800  )
		{
			height_ctrl_mode = 1;//气压计
		}
		else
		{
			if(ultra_ok == 1)
			{
				height_ctrl_mode = 2;//超声波
			}
			else
			{
				height_ctrl_mode = 1;
			}
		}
	}
	else
	{
		
		
		if( RX_CH[AUX4r] < 100 )
		{
			height_ctrl_mode = 0;
		}
		else if(RX_CH[AUX4r] >800  )
		{
			height_ctrl_mode = 1;//气压计
		}
		else
		{
			if(ultra_ok == 1)
			{
				height_ctrl_mode = 2;//超声波
			}
			else
			{
				height_ctrl_mode = 1;
			}
		}
	}
}
	else
	{
	
	if( RX_CH[AUX4r] < 100 )
		{
			height_ctrl_mode = 0;
		}
		else if(RX_CH[AUX4r] >800  )
		{
			height_ctrl_mode = 1;//气压计
		}
	}
	
	
//	last_height_ctrl_mode=height_ctrl_mode;
}


