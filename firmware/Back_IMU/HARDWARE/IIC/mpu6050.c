

#include "mpu6050.h"
#include "iic_soft.h"
#include "filter.h"
#include "LIS3MDL.h"
MPU6050_STRUCT mpu6050,mpu6050_r,imu_fushion;


void MPU6050_Data_Prepare(float T)
{	
	u8 i;

	imu_fushion.Acc.x=lis3mdl.Acc_t.x;
	imu_fushion.Acc.y=lis3mdl.Acc_t.y;
	imu_fushion.Acc.z=lis3mdl.Acc_t.z;
	imu_fushion.Gyro.x=lis3mdl.Gyro_t.x;
	imu_fushion.Gyro.y=lis3mdl.Gyro_t.y;
	imu_fushion.Gyro.z=lis3mdl.Gyro_t.z;
	imu_fushion.Gyro_deg.x=lis3mdl.Gyro_deg_t.x;
	imu_fushion.Gyro_deg.y=lis3mdl.Gyro_deg_t.y;
	imu_fushion.Gyro_deg.z=lis3mdl.Gyro_deg_t.z;
	imu_fushion.Mag_Val.x=lis3mdl.Mag_Val_t.x;
	imu_fushion.Mag_Val.y=lis3mdl.Mag_Val_t.y;
	imu_fushion.Mag_Val.z=lis3mdl.Mag_Val_t.z;
	#if USE_M100_IMU
	imu_fushion.Alt = (m100.H);
	#else
	imu_fushion.Alt=lis3mdl.Alt;
	#endif

}
