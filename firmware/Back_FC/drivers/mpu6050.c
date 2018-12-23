#include "include.h"
#include "mpu6050.h"
#include "mymath.h"
#include "i2c_soft.h"
#include "imu.h"
#include "mpu9250.h"
#include "icm20602.h"
MPU6050_STRUCT mpu6050,mpu6050_fc;
int16andUint8_t rawMag[3];

int16andUint8_t rawMPU6050Temperature;
u8 mpu6050_buffer[14];
u8 mpu6050_ok;

//		L   H    C
//a   y-  x-
//g   x-  y+  z-

void MPU6050_Read(void)
{
    //I2C_FastMode = 1;
	  #if USE_VER_3&&!USE_VER_6
	  #if USE_VER_7
	  Icm20602_Read();
	  #else
    MPU9250_ReadValue();
	  #endif
	  #else
    IIC_Read_nByte(MPU6050_ADDR,MPU6050_RA_ACCEL_XOUT_H,14,mpu6050_buffer);
	  #endif
}


s32 sum_temp[7]= {0,0,0,0,0,0,0};
float sum_temp_att[2]={0};
s32 sum_temp_3d[7]= {0,0,0,0,0,0,0};
u16 acc_sum_cnt = 0,acc_sum_cnt_3d=0,acc_smple_cnt_3d=0,gyro_sum_cnt = 0;
#define OFFSET_AV_NUM_ACC 50
void MPU6050_Data_Offset()
{
#ifdef ACC_ADJ_EN

    if(mpu6050_fc.Acc_CALIBRATE == 1||need_init_mems==1)
    {
        if(my_sqrt(my_pow(mpu6050_fc.Acc_I16.x)+my_pow(mpu6050_fc.Acc_I16.y)+my_pow(mpu6050_fc.Acc_I16.z)) < 2500)
        {
            sensor_setup.Offset.mpu_flag = 1;
        }
        else if(my_sqrt(my_pow(mpu6050_fc.Acc_I16.x)+my_pow(mpu6050_fc.Acc_I16.y)+my_pow(mpu6050_fc.Acc_I16.z)) > 2600)
        {
            sensor_setup.Offset.mpu_flag = 0;
        }

        acc_sum_cnt++;
				if(mpu6050_fc.Cali_3d&&1){
//			  sum_temp[A_X] += (mpu6050_fc.Acc_I16.x - mpu6050_fc.Off_3d.x)*mpu6050_fc.Gain_3d.x ;
//        sum_temp[A_Y] += (mpu6050_fc.Acc_I16.y - mpu6050_fc.Off_3d.y)*mpu6050_fc.Gain_3d.y ;
//        sum_temp[A_Z] += (mpu6050_fc.Acc_I16.z - mpu6050_fc.Off_3d.z)*mpu6050_fc.Gain_3d.z - 65536/16;
				  sum_temp_att[0]+=Pit_fc1;
					sum_temp_att[1]+=Rol_fc1;
				}
				
				{
        sum_temp[A_X] += mpu6050_fc.Acc_I16.x;
        sum_temp[A_Y] += mpu6050_fc.Acc_I16.y;
        sum_temp[A_Z] += mpu6050_fc.Acc_I16.z - 65536/16;   // +-8G
				}
        sum_temp[TEM] += mpu6050_fc.Tempreature;

        if( acc_sum_cnt >= OFFSET_AV_NUM )
        {   
            mpu6050_fc.Acc_Offset.x = sum_temp[A_X]/OFFSET_AV_NUM;
            mpu6050_fc.Acc_Offset.y = sum_temp[A_Y]/OFFSET_AV_NUM;
            mpu6050_fc.Acc_Offset.z = sum_temp[A_Z]/OFFSET_AV_NUM;
					  mpu6050_fc.att_off[0]=(float)sum_temp_att[0]/OFFSET_AV_NUM;
					  mpu6050_fc.att_off[1]=(float)sum_temp_att[1]/OFFSET_AV_NUM;
            mpu6050_fc.Acc_Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
            acc_sum_cnt =0;
            mpu6050_fc.Acc_CALIBRATE = 0;
            WRITE_PARM();
					  need_init_mems=2;
            sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;
				  	sum_temp_att[1]=sum_temp_att[0]=0;
        }
    }
#endif

    if(mpu6050_fc.Gyro_CALIBRATE||need_init_mems==2)
    {
        gyro_sum_cnt++;
        sum_temp[G_X] += mpu6050_fc.Gyro_I16.x;
        sum_temp[G_Y] += mpu6050_fc.Gyro_I16.y;
        sum_temp[G_Z] += mpu6050_fc.Gyro_I16.z;
        sum_temp[TEM] += mpu6050_fc.Tempreature;

        if( gyro_sum_cnt >= OFFSET_AV_NUM )
        {
            mpu6050_fc.Gyro_Offset.x = (float)sum_temp[G_X]/OFFSET_AV_NUM;
            mpu6050_fc.Gyro_Offset.y = (float)sum_temp[G_Y]/OFFSET_AV_NUM;
            mpu6050_fc.Gyro_Offset.z = (float)sum_temp[G_Z]/OFFSET_AV_NUM;
            mpu6050_fc.Gyro_Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
            gyro_sum_cnt =0;need_init_mems=0;
            if(mpu6050_fc.Gyro_CALIBRATE == 1)
			{
               WRITE_PARM();
			}  
            mpu6050_fc.Gyro_CALIBRATE = 0;
            sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
        }
    }
}

void Transform(float itx,float ity,float itz,float *it_x,float *it_y,float *it_z)
{
    *it_x = itx;
    *it_y = ity;
    *it_z = itz;

}

s16 FILT_BUF[ITEMS][(FILTER_NUM + 1)];
uint8_t filter_cnt = 0,filter_cnt_old = 0;

float mpu6050_tmp[ITEMS];
float mpu_fil_tmp[ITEMS];
float test_ang =0,test_ang_old=0,test_ang_d,test_fli_a,test_i;
float ACC_MAX[3][2];
void MPU6050_Data_Prepare(float T)
{
    u8 i;
    s32 FILT_TMP[ITEMS] = {0,0,0,0,0,0,0};
//	float auto_offset_temp[3];
    float Gyro_tmp[3];


    MPU6050_Data_Offset(); //校准函数

    /*读取buffer原始数据*/
		#if USE_VER_3&&!USE_VER_6
		#if USE_VER_7
			mpu6050_fc.Acc_I16.x =(s16)((((u16)mpu_buffer[0]) << 8) | mpu_buffer[1]);//>>1;// + 2 *sensor.Tempreature_C;// + 5 *sensor.Tempreature_C;
			mpu6050_fc.Acc_I16.y =(s16)((((u16)mpu_buffer[2]) << 8) | mpu_buffer[3]);//>>1;// + 2 *sensor.Tempreature_C;// + 5 *sensor.Tempreature_C;
			mpu6050_fc.Acc_I16.z =(s16)((((u16)mpu_buffer[4]) << 8) | mpu_buffer[5]);//>>1;// + 4 *sensor.Tempreature_C;// + 7 *sensor.Tempreature_C;

			mpu6050_fc.Gyro_I16.x=(s16)((((u16)mpu_buffer[ 8]) << 8) | mpu_buffer[ 9]) ;
			mpu6050_fc.Gyro_I16.y=(s16)((((u16)mpu_buffer[10]) << 8) | mpu_buffer[11]) ;
			mpu6050_fc.Gyro_I16.z=(s16)((((u16)mpu_buffer[12]) << 8) | mpu_buffer[13]) ;
		#else
			mpu6050_fc.Acc_I16.x=rawAccel[1].value;
			mpu6050_fc.Acc_I16.y=rawAccel[0].value;
			mpu6050_fc.Acc_I16.z=rawAccel[2].value;
			mpu6050_fc.Gyro_I16.x=rawGyro[1].value;
			mpu6050_fc.Gyro_I16.y=rawGyro[0].value;
			mpu6050_fc.Gyro_I16.z=rawGyro[2].value;	
		#endif
		#else
    mpu6050_fc.Acc_I16.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) ;
    mpu6050_fc.Acc_I16.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) ;
    mpu6050_fc.Acc_I16.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) ;

    mpu6050_fc.Gyro_I16.x = ((((int16_t)mpu6050_buffer[ 8]) << 8) | mpu6050_buffer[ 9]) ;
    mpu6050_fc.Gyro_I16.y = ((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) ;
    mpu6050_fc.Gyro_I16.z = ((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) ;
    #endif
		#if USE_VER_6
		mpu6050_fc.Acc_I16.y = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) ;
    mpu6050_fc.Acc_I16.x = -((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) ;
    mpu6050_fc.Acc_I16.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) ;

    mpu6050_fc.Gyro_I16.y = ((((int16_t)mpu6050_buffer[ 8]) << 8) | mpu6050_buffer[ 9]) ;
    mpu6050_fc.Gyro_I16.x = -((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) ;
    mpu6050_fc.Gyro_I16.z = ((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) ;
		#endif
    Gyro_tmp[0] = mpu6050_fc.Gyro_I16.x ;//
    Gyro_tmp[1] = mpu6050_fc.Gyro_I16.y ;//
    Gyro_tmp[2] = mpu6050_fc.Gyro_I16.z ;//
    #if USE_VER_3&&!USE_VER_6
    mpu6050_fc.Tempreature=rawMPU6050Temperature.value;
		#else
    mpu6050_fc.Tempreature = ((((int16_t)mpu6050_buffer[6]) << 8) | mpu6050_buffer[7]); //tempreature
		#endif
    mpu6050_fc.TEM_LPF += 2 *3.14f *T *(mpu6050_fc.Tempreature - mpu6050_fc.TEM_LPF);
    mpu6050_fc.Ftempreature = mpu6050_fc.TEM_LPF/340.0f + 36.5f;

//======================================================================
    if( ++filter_cnt > FILTER_NUM )
    {
        filter_cnt = 0;
        filter_cnt_old = 1;
    }
    else
    {
        filter_cnt_old = (filter_cnt == FILTER_NUM)? 0 : (filter_cnt + 1);
    }
//10 170 4056
//		if(fabs(mpu6050_fc.Off_3d.x)>10||fabs(mpu6050_fc.Off_3d.y)>10||fabs(mpu6050_fc.Off_3d.z)>10)
//			mpu6050_fc.Cali_3d=1;
		int en_off_3d_off=0;
	
	 static u8 init;
		if(!init){init=1;
	  switch(id_chip)
		{
			case IMAV1://1
				ACC_MAX[Xr][0]=4084;//+
				ACC_MAX[Xr][1]=-4108;//-
				ACC_MAX[Yr][0]=4070;//+
				ACC_MAX[Yr][1]=-4132;//-
				ACC_MAX[Zr][0]=4159;//+
				ACC_MAX[Zr][1]=-4100;//-
				mpu6050_fc.Off_3d.x=-24;//ACC_MAX[Xr][0]+ACC_MAX[Xr][1];
				mpu6050_fc.Off_3d.y=-30;//ACC_MAX[Yr][0]+ACC_MAX[Yr][1];
				mpu6050_fc.Off_3d.z=35;//ACC_MAX[Zr][0]+ACC_MAX[Zr][1];
				mpu6050_fc.Gain_3d.x=(4096*2)/(ACC_MAX[Xr][0]-ACC_MAX[Xr][1]);
				mpu6050_fc.Gain_3d.y=(4096*2)/(ACC_MAX[Yr][0]-ACC_MAX[Yr][1]);
				mpu6050_fc.Gain_3d.z=(4096*2)/(ACC_MAX[Zr][0]-ACC_MAX[Zr][1]);
				mpu6050_fc.Cali_3d=1;
		 break;
			case IMAV2://2
				ACC_MAX[Xr][0]=4068;//+
				ACC_MAX[Xr][1]=-4120;//-
				ACC_MAX[Yr][0]=4025;//+
				ACC_MAX[Yr][1]=-4160;//-
				ACC_MAX[Zr][0]=4129;//+
				ACC_MAX[Zr][1]=-4060;//-
				mpu6050_fc.Off_3d.x=-18;//ACC_MAX[Xr][0]+ACC_MAX[Xr][1];
				mpu6050_fc.Off_3d.y=-85;//ACC_MAX[Yr][0]+ACC_MAX[Yr][1];
				mpu6050_fc.Off_3d.z=65;//ACC_MAX[Zr][0]+ACC_MAX[Zr][1];
				mpu6050_fc.Gain_3d.x=(4096*2)/(ACC_MAX[Xr][0]-ACC_MAX[Xr][1]);
				mpu6050_fc.Gain_3d.y=(4096*2)/(ACC_MAX[Yr][0]-ACC_MAX[Yr][1]);
				mpu6050_fc.Gain_3d.z=(4096*2)/(ACC_MAX[Zr][0]-ACC_MAX[Zr][1]);
				mpu6050_fc.Cali_3d=1;
		 break;
			case IMAV3://3
				ACC_MAX[Xr][0]=4004;//+
				ACC_MAX[Xr][1]=-4192;//-
				ACC_MAX[Yr][0]=4100;//+
				ACC_MAX[Yr][1]=-4105;//-
				ACC_MAX[Zr][0]=4060;//+
				ACC_MAX[Zr][1]=-4140;//-
				mpu6050_fc.Off_3d.x=-68;//ACC_MAX[Xr][0]+ACC_MAX[Xr][1];
				mpu6050_fc.Off_3d.y=-10;//ACC_MAX[Yr][0]+ACC_MAX[Yr][1];
				mpu6050_fc.Off_3d.z=50;//ACC_MAX[Zr][0]+ACC_MAX[Zr][1];
				mpu6050_fc.Off_3d.x=ACC_MAX[Xr][0]+ACC_MAX[Xr][1];
			  mpu6050_fc.Off_3d.y=ACC_MAX[Yr][0]+ACC_MAX[Yr][1];
				mpu6050_fc.Off_3d.z=ACC_MAX[Zr][0]+ACC_MAX[Zr][1];
				mpu6050_fc.Gain_3d.x=(4096*2)/(ACC_MAX[Xr][0]-ACC_MAX[Xr][1]);
				mpu6050_fc.Gain_3d.y=(4096*2)/(ACC_MAX[Yr][0]-ACC_MAX[Yr][1]);
				mpu6050_fc.Gain_3d.z=(4096*2)/(ACC_MAX[Zr][0]-ACC_MAX[Zr][1]);
				mpu6050_fc.Cali_3d=1;
		 break;
		 default:
			  mpu6050_fc.Cali_3d=0;
		 break;
		}	
	 }
		
    /* 得出校准后的数据 */
	 if(mpu6050_fc.Cali_3d&&1){
			  mpu6050_tmp[A_X] = (mpu6050_fc.Acc_I16.x - mpu6050_fc.Off_3d.x)*mpu6050_fc.Gain_3d.x ;
        mpu6050_tmp[A_Y] = (mpu6050_fc.Acc_I16.y - mpu6050_fc.Off_3d.y)*mpu6050_fc.Gain_3d.y ;
        mpu6050_tmp[A_Z] = (mpu6050_fc.Acc_I16.z - mpu6050_fc.Off_3d.z)*mpu6050_fc.Gain_3d.z ;
	 }
   else{	 
    if(sensor_setup.Offset.mpu_flag == 0)
    {
        mpu6050_tmp[A_X] = (mpu6050_fc.Acc_I16.x - mpu6050_fc.Acc_Offset.x) ;
        mpu6050_tmp[A_Y] = (mpu6050_fc.Acc_I16.y - mpu6050_fc.Acc_Offset.y) ;
        mpu6050_tmp[A_Z] = (mpu6050_fc.Acc_I16.z - mpu6050_fc.Acc_Offset.z) ;
    }
    else
    {
        mpu6050_tmp[A_X] = 2*(mpu6050_fc.Acc_I16.x - mpu6050_fc.Acc_Offset.x) ;
        mpu6050_tmp[A_Y] = 2*(mpu6050_fc.Acc_I16.y - mpu6050_fc.Acc_Offset.y) ;
        mpu6050_tmp[A_Z] = 2*(mpu6050_fc.Acc_I16.z - mpu6050_fc.Acc_Offset.z - 2048) ;
    }
  }
    mpu6050_tmp[G_X] = Gyro_tmp[0] - mpu6050_fc.Gyro_Offset.x ;//
    mpu6050_tmp[G_Y] = Gyro_tmp[1] - mpu6050_fc.Gyro_Offset.y ;//
    mpu6050_tmp[G_Z] = Gyro_tmp[2] - mpu6050_fc.Gyro_Offset.z ;//


    /* 更新滤波滑动窗口数组 */
    FILT_BUF[A_X][filter_cnt] = mpu6050_tmp[A_X];
    FILT_BUF[A_Y][filter_cnt] = mpu6050_tmp[A_Y];
    FILT_BUF[A_Z][filter_cnt] = mpu6050_tmp[A_Z];
    FILT_BUF[G_X][filter_cnt] = mpu6050_tmp[G_X];
    FILT_BUF[G_Y][filter_cnt] = mpu6050_tmp[G_Y];
    FILT_BUF[G_Z][filter_cnt] = mpu6050_tmp[G_Z];

    for(i=0; i<FILTER_NUM; i++)
    {
        FILT_TMP[A_X] += FILT_BUF[A_X][i];
        FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
        FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
        FILT_TMP[G_X] += FILT_BUF[G_X][i];
        FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
        FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
    }


    mpu_fil_tmp[A_X] = (float)( FILT_TMP[A_X] )/(float)FILTER_NUM;
    mpu_fil_tmp[A_Y] = (float)( FILT_TMP[A_Y] )/(float)FILTER_NUM;
    mpu_fil_tmp[A_Z] = (float)( FILT_TMP[A_Z] )/(float)FILTER_NUM;


    mpu_fil_tmp[G_X] = (float)( FILT_TMP[G_X] )/(float)FILTER_NUM;
    mpu_fil_tmp[G_Y] = (float)( FILT_TMP[G_Y] )/(float)FILTER_NUM;
    mpu_fil_tmp[G_Z] = (float)( FILT_TMP[G_Z] )/(float)FILTER_NUM;


    /*坐标转换*/
    Transform(mpu_fil_tmp[A_X],mpu_fil_tmp[A_Y],mpu_fil_tmp[A_Z],&mpu6050_fc.Acc.x,&mpu6050_fc.Acc.y,&mpu6050_fc.Acc.z);
    Transform(mpu_fil_tmp[G_X],mpu_fil_tmp[G_Y],mpu_fil_tmp[G_Z],&mpu6050_fc.Gyro.x,&mpu6050_fc.Gyro.y,&mpu6050_fc.Gyro.z);

    mpu6050_fc.Gyro_deg.x = mpu6050_fc.Gyro.x *TO_ANGLE;
    mpu6050_fc.Gyro_deg.y = mpu6050_fc.Gyro.y *TO_ANGLE;
    mpu6050_fc.Gyro_deg.z = mpu6050_fc.Gyro.z *TO_ANGLE;

}
