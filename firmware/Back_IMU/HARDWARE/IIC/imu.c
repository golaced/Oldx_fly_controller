
#include "imu.h"
#include "include.h"
#include "hml5833l.h"
#include "my_math.h"
#include "filter.h"
#include "gps.h"
float Kp =0.1;//625f;//2.25f;//0.6f   ;             	// proportional gain governs rate of convergence to accelerometer/magnetometer
float Kp_yaw=0.625*2;
float Ki =0.001f    ;            	// 0.001  integral gain governs rate of convergence of gyroscope biases

#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1				//(Hz)

float q_nav[4];
xyz_f_t reference_v;
ref_t 	ref;
float reference_vr[3];
//xyz_f_t Gravity_Vec;  				//解算的重力向量
	
float Roll,Pitch,Yaw;    				//姿态角
float Roll_mid_down,Pitch_mid_down,Yaw_mid_down;    				//姿态角
float ref_q[4] = {1,0,0,0};
float norm_acc,norm_q;
float norm_acc_lpf;
xyz_f_t mag_sim_3d;
extern u8 fly_ready;

int test_flag[3]={1,-1,1};
xyz_f_t mag_sim_3d,acc_3d_hg,acc_ng,acc_ng_offset;
//use
float mag_norm ,mag_norm_xyz, yaw_mag_view[5];
//------------------KF  parameter------------------
float gh_yaw=0.15;
float ga_yaw=0.1;//<---use
float gw_yaw=0.1;
float yaw_kf;
double P_kf_yaw[4]={1,0,0,1}; 
double X_kf_yaw[2]={0,0};
float k_kf_z=1.1;//1.428;
u8 yaw_cross;
float yaw_qr_off;
u8 dis_angle_lock;
u8 yaw_cal_by_qr;
float yaw_qr_off_local;
float yaw_off_earth=0;//90;
u8 init_ahrs;
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw) 
{		
	float ref_err_lpf_hz;
	static float yaw_correct;
	float mag_norm_tmp;
	static xyz_f_t mag_tmp;
	static float yaw_mag;
	static u16 cnt;
	static u8 init;
	if(cnt++>256&&init==0)
	{
	init=1;
	ref.err.x=ref.err.y=ref.err.z=0;
	ref.err_Int.x=ref.err_Int.y=ref.err_Int.z=0;
	ref.err_lpf.x=ref.err_lpf.y=ref.err_lpf.z=0;
	ref.err_tmp.x=ref.err_tmp.y=ref.err_tmp.z=0;
	ref.g.x=ref.g.y=ref.g.z=0;
	}else{
	X_kf_yaw[0]=yaw_mag_view[4];
	X_kf_yaw[1]=0;
	}
	
	mag_norm_tmp = 20 *(6.28f *half_T);	
	
	mag_norm_xyz = my_sqrt(imu_fushion.Mag_Val.x * imu_fushion.Mag_Val.x + imu_fushion.Mag_Val.y * imu_fushion.Mag_Val.y + imu_fushion.Mag_Val.z * imu_fushion.Mag_Val.z);
	if(mag_norm_xyz==0)mag_norm_xyz=0.0001;
		if( mag_norm_xyz != 0)
	{
		mag_tmp.x += mag_norm_tmp *( (float)imu_fushion.Mag_Val.x /( mag_norm_xyz ) - mag_tmp.x);
		mag_tmp.y += mag_norm_tmp *( (float)imu_fushion.Mag_Val.y /( mag_norm_xyz ) - mag_tmp.y);	
		mag_tmp.z += mag_norm_tmp *( (float)imu_fushion.Mag_Val.z /( mag_norm_xyz ) - mag_tmp.z);	
	}

	simple_3d_trans(&reference_v,&mag_tmp,&mag_sim_3d);
	
	mag_norm = my_sqrt(mag_sim_3d.x * mag_sim_3d.x + mag_sim_3d.y *mag_sim_3d.y);
	if(mag_norm==0)mag_norm=0.0001;
	if( mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 && mag_norm != 0)
	{
		yaw_mag_view[1] = fast_atan2( ( mag_sim_3d.y/mag_norm ) , ( mag_sim_3d.x/mag_norm) ) *57.3f;
		
	}
		float calMagY,calMagX,magTmp2[3],euler[3];
		magTmp2[0]=test_flag[0]*imu_fushion.Mag_Val.x;
		magTmp2[1]=test_flag[1]*imu_fushion.Mag_Val.y;
		magTmp2[2]=test_flag[2]*imu_fushion.Mag_Val.z;
		euler[1]=Pitch/RAD_DEG  ;
		euler[0]=Roll/RAD_DEG   ;
	
		calMagY = magTmp2[2] * sin(euler[0]) - magTmp2[1] * cos(euler[0]); //倾斜补偿磁力计的Y轴分量
		calMagX = magTmp2[0] * cos(euler[1]) + magTmp2[1] * sin(euler[1]) * sin(euler[0]) + magTmp2[2] * sin(euler[1]) * cos(euler[0]); //倾斜补偿磁力计的X轴分量

		yaw_mag_view[0] = fast_atan2(calMagY, calMagX) * RAD_DEG; //计算Yaw (-PI < Roll < PI) 并将弧度转化成角度
	  yaw_mag_view[3]=yaw_mag_view[0]/2+yaw_mag_view[1]/2;
  	//yaw_mag=yaw_mag_view[1] ;
	
		magTmp2[0]=imu_fushion.Mag_Val.x;
		magTmp2[1]=imu_fushion.Mag_Val.y;
		magTmp2[2]=imu_fushion.Mag_Val.z;
		euler[0]=Pitch_mid_down/RAD_DEG  ;
		euler[1]=Roll_mid_down/RAD_DEG  ;
    calMagY = magTmp2[0] * cos(euler[1]) + magTmp2[1] * sin(euler[1])* sin(euler[0])+magTmp2[2] * sin(euler[1]) * cos(euler[0]); 
    calMagX = magTmp2[1] * cos(euler[0]) + magTmp2[2] * sin(euler[0]);
		float tempy;
		
		
		if( dis_angle_lock||(fabs(Roll_mid_down)<30 && fabs(Pitch_mid_down)<30))
		tempy=To_180_degrees(fast_atan2(calMagX,calMagY)* RAD_DEG );

		yaw_mag_view[4]=Moving_Median(14,5,tempy+yaw_off_earth);	
		
		double Z_yaw[2]={ yaw_mag_view[4] , 0 };
		if(yaw_mag_view[4]*X_kf_yaw[0]<0&&!(fabs(yaw_mag_view[4])<90))
		{Z_yaw[0]=X_kf_yaw[0];yaw_cross=1;}
		else
		yaw_cross=0;

	//=============================================================================
	// 计算等效重力向量//十分重要
	if(mode.en_imu_ekf==0){
	reference_vr[0]=reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_vr[1]=reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_vr[2]=reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);}//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]

	//这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
	//根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
	//所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。       
	//=============================================================================
	  acc_ng.x = 10 *TO_M_S2 *(ax - 4096*reference_v.x) - acc_ng_offset.x;
		acc_ng.y = 10 *TO_M_S2 *(ay - 4096*reference_v.y) - acc_ng_offset.y;
		acc_ng.z = 10 *TO_M_S2 *(az - 4096*reference_v.z) - acc_ng_offset.z;
		
		acc_3d_hg.z = acc_ng.x *reference_v.x + acc_ng.y *reference_v.y + acc_ng.z *reference_v.z;

	// 计算加速度向量的模
	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);   
	norm_acc_lpf +=  NORM_ACC_LPF_HZ *(6.28f *half_T) *(norm_acc - norm_acc_lpf);  //10hz *3.14 * 2*0.001
	if(amf.connect && 0)
	yaw_mag=To_180_degrees(amf.att[2]);
	else
  yaw_mag=yaw_mag_view[1]+Gps_information.off_earth;
	
	#if NO_HML_CORRECT
	  yaw_mag=YawR;
	#endif
	static u8 hml_cal_reg;
	if(ak8975.Mag_CALIBRATED==1&&hml_cal_reg==0)
	YawR=yaw_mag;
	hml_cal_reg=ak8975.Mag_CALIBRATED;
	
	
  	if(norm_acc==0)norm_acc=0.0001;
	if(ABS(ax)<4400 && ABS(ay)<4400 && ABS(az)<4400 )
	{	
		//把加计的三维向量转成单位向量。
		ax = ax / norm_acc;//4096.0f;
		ay = ay / norm_acc;//4096.0f;
		az = az / norm_acc;//4096.0f; 
		
		if( 3800 < norm_acc && norm_acc < 4400 )
		{
			/* 叉乘得到误差 */
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
	    //ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;
			
			/* 误差低通 */
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *( ref.err_tmp.x  - ref.err_lpf.x );
			ref.err_lpf.y += ref_err_lpf_hz *( ref.err_tmp.y  - ref.err_lpf.y );
	//			 ref.err_lpf.z += ref_err_lpf_hz *( ref.err_tmp.z  - ref.err_lpf.z );
			
			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//
//				ref.err.z = ref.err_lpf.z ;
		}
	}
	else
	{
		ref.err.x = 0; 
		ref.err.y = 0  ;
//		ref.err.z = 0 ;
	}
	/* 误差积分 */
	ref.err_Int.x += ref.err.x *Ki *2 *half_T ;
	ref.err_Int.y += ref.err.y *Ki *2 *half_T ;
	ref.err_Int.z += ref.err.z *Ki *2 *half_T ;
	
	/* 积分限幅 */
	ref.err_Int.x = LIMIT(ref.err_Int.x, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.y = LIMIT(ref.err_Int.y, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.z = LIMIT(ref.err_Int.z, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	static u16 init_mag_cnt;
	if( reference_v.z > 0.0f )
	{
		
		if(init_mag_cnt++<400 &&init_ahrs==0)
		{ 
			yaw_correct = Kp_yaw *100 *LIMIT( To_180_degrees(yaw_mag - YawR),-45,45 );
		}
		else if(( fly_ready||(fabs(Pitch)>20)||(fabs(Roll)>20)))
		{ init_ahrs=1;
	//	yaw_correct = Kp *0.2f *To_180_degrees(yaw_mag - YAW_R);
			yaw_correct = Kp_yaw *0.35 *LIMIT( To_180_degrees(yaw_mag - YawR),-45,45 );
			//已经解锁，只需要低速纠正。
		}
		else
		{
			yaw_correct = Kp_yaw *1.5f *To_180_degrees(yaw_mag - YawR);
			//没有解锁，视作开机时刻，快速纠正
		}
		
		if(init_ahrs&&module.outer_hml)
			Kp=0.3;
	}

	ref.g.x = (gx - reference_v.x *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	ref.g.y = (gy - reference_v.y *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.y + ref.err_Int.y) ) ;		  //IN RADIAN
	ref.g.z = (gz - reference_v.z *yaw_correct) *ANGLE_TO_RADIAN;
	
	/* 用叉积误差来做PI修正陀螺零偏 */
	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] +(-ref_q[1]*ref.g.x - ref_q[2]*ref.g.y - ref_q[3]*ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0]*ref.g.x + ref_q[2]*ref.g.z - ref_q[3]*ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0]*ref.g.y - ref_q[1]*ref.g.z + ref_q[3]*ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0]*ref.g.z + ref_q[1]*ref.g.y - ref_q[2]*ref.g.x)*half_T;  

	/* 四元数规一化 normalise quaternion */
	norm_q = my_sqrt(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]);
	if(norm_q==0)norm_q=1;
	q_nav[0]=ref_q[0] = ref_q[0] / norm_q;
	q_nav[1]=ref_q[1] = ref_q[1] / norm_q;
	q_nav[2]=ref_q[2] = ref_q[2] / norm_q;
	q_nav[3]=ref_q[3] = ref_q[3] / norm_q;
	
	*rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f  ;// 
	//*yaw =X_kf_yaw[0];// yaw_mag;
}

float q_so3[4];
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; /** bias estimation */
static float q0q0, q0q1, q0q2, q0q3;
static float q1q1, q1q2, q1q3;
static float q2q2, q2q3;
static float q3q3;
static bool bFilterInit = false;
//! Using accelerometer, sense the gravity vector.
//! Using magnetometer, sense yaw.
void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

float invSqrt1(float number) 
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}


void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt,float *rol,float *pit,float *yaw) 
{
	float recipNorm;
	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

	// Make filter converge to initial solution faster
	// This function assumes you are in static position.
	// WARNING : in case air reboot, this can cause problem. But this is very unlikely happen.
	if(bFilterInit == false) {
		NonlinearSO3AHRSinit(ax,ay,az,mx,my,mz);
		bFilterInit = true;
	}
        	
	//! If magnetometer measurement is available, use it.
	if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
		float hx, hy, hz, bx, bz;
		float halfwx, halfwy, halfwz;
	
		// Normalise magnetometer measurement
		// Will sqrt work better? PX4 system is powerful enough?
    	recipNorm = invSqrt1(mx * mx + my * my + mz * mz);
    	mx *= recipNorm;
    	my *= recipNorm;
    	mz *= recipNorm;
    
    	// Reference direction of Earth's magnetic field
    	hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    	hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		  hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
    	bx = sqrt(hx * hx + hy * hy);
    	bz = hz;
    
    	// Estimated direction of magnetic field
    	halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    	halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    	halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
    
    	// Error is sum of cross product between estimated direction and measured direction of field vectors
    	halfex += (my * halfwz - mz * halfwy);
    	halfey += (mz * halfwx - mx * halfwz);
    	halfez += (mx * halfwy - my * halfwx);
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		float halfvx, halfvy, halfvz;
	
		// Normalise accelerometer measurement
		recipNorm = invSqrt1(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += ay * halfvz - az * halfvy;
		halfey += az * halfvx - ax * halfvz;
		halfez += ax * halfvy - ay * halfvx;
	}

	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			gyro_bias[0] += twoKi * halfex * dt;	// integral error scaled by Ki
			gyro_bias[1] += twoKi * halfey * dt;
			gyro_bias[2] += twoKi * halfez * dt;
			
			// apply integral feedback
			gx += gyro_bias[0];
			gy += gyro_bias[1];
			gz += gyro_bias[2];
		}
		else {
			gyro_bias[0] = 0.0f;	// prevent integral windup
			gyro_bias[1] = 0.0f;
			gyro_bias[2] = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	//! Integrate rate of change of quaternion
#if 0
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
#endif 

	// Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
	//! q_k = q_{k-1} + dt*\dot{q}
	//! \dot{q} = 0.5*q \otimes P(\omega)
	dq0 = 0.5f*(-q1 * gx - q2 * gy - q3 * gz);
	dq1 = 0.5f*(q0 * gx + q2 * gz - q3 * gy);
	dq2 = 0.5f*(q0 * gy - q1 * gz + q3 * gx);
	dq3 = 0.5f*(q0 * gz + q1 * gy - q2 * gx); 

	q0 += dt*dq0;
	q1 += dt*dq1;
	q2 += dt*dq2;
	q3 += dt*dq3;
	
	// Normalise quaternion
	recipNorm = invSqrt1(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
   	q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;   
		float Rot_matrix[9];
		// Convert q->R, This R converts inertial frame to body frame.
		Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
		Rot_matrix[1] = 2.f * (q1*q2 + q0*q3);	// 12
		Rot_matrix[2] = 2.f * (q1*q3 - q0*q2);	// 13
		Rot_matrix[3] = 2.f * (q1*q2 - q0*q3);	// 21
		Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
		Rot_matrix[5] = 2.f * (q2*q3 + q0*q1);	// 23
		Rot_matrix[6] = 2.f * (q1*q3 + q0*q2);	// 31
		Rot_matrix[7] = 2.f * (q2*q3 - q0*q1);	// 32
		Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33

		//1-2-3 Representation.
		//Equation (290) 
		//Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, James Diebel.
		// Existing PX4 EKF code was generated by MATLAB which uses coloum major order matrix.
		*rol = atan2f(Rot_matrix[5], Rot_matrix[8])*57.3;	//! Roll
		*pit=  asinf(Rot_matrix[2])*57.3;	//! Pitch
		*yaw = atan2f(Rot_matrix[1], Rot_matrix[0])*57.3;		//! Yaw
		q_so3[0]=q0;
		q_so3[1]=q1;
		q_so3[2]=q2;
		q_so3[3]=q3;
					
}

