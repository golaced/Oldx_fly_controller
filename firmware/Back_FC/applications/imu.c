#include "imu.h"
#include "include.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "mymath.h"
#include "filter.h"
#include "quar.h"
//--------------------------------------匿名
#define Kp 0.3f                	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Kp_Yaw 0.3f
#define Ki 0.005f                	// 0.001  integral gain governs rate of convergence of gyroscope biases
float Kp_use;
#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1				//(Hz)

xyz_f_t reference_v;
ref_t 	ref;
float reference_vr_imd_down[3],reference_vr[3];
//xyz_f_t Gravity_Vec;  				//解算的重力向量
	
float Roll,Pitch,Yaw,Roll_mid_down,Pitch_mid_down,Yaw_mid_down,Rol_fc,Pit_fc,Yaw_fc;    				//姿态角

float ref_q[4] = {1,0,0,0},q_nav[4],ref_q_imd_down[4];
float norm_acc,norm_q;
float norm_acc_lpf;

float mag_norm ,mag_norm_xyz ;

xyz_f_t mag_sim_3d,acc_3d_hg,acc_ng,acc_ng_offset;

u8 acc_ng_cali;
extern u8 fly_ready;
float yaw_mag;

float accConfidenceDecay 			  =	5.2f;
float accConfidence      = 1.0f; 
#define HardFilter(O,N)  ((O)*0.9f+(N)*0.1f)
#define accelOneG 10
void calculateAccConfidence(float accMag_in)
{
	// G.K. Egan (C) computes confidence in accelerometers when
	// aircraft is being accelerated over and above that due to gravity

	static float accMagP = 1.0f;

	float accMag =accMag_in/ accelOneG;  // HJI Added to convert MPS^2 to G's

	accMagP  = HardFilter(accMagP, accMag );


	accConfidence=((accConfidenceDecay * sqrt(fabs(accMagP - 1.0f))));
  if(accConfidence>1)
		accConfidence=1;
	if(accConfidence<0)
		accConfidence=0;
}

void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw) 
{	static u8 init;
	float ref_err_lpf_hz;
	static float yaw_correct;
	float mag_norm_tmp;
	static xyz_f_t mag_tmp;
	
	if(!init)
	{
	init=1;
	ref.err_tmp.x=ref.err_tmp.y=ref.err_tmp.z=0;
	ref.err.x=ref.err.y=ref.err.z=0;
	ref.err_lpf.x=ref.err_lpf.y=ref.err_lpf.z=0;
	ref.err_Int.x=ref.err_Int.y=ref.err_Int.z=0;
	ref.g.x=ref.g.y=ref.g.z=0;
	}
	mag_norm_tmp = 20 *(6.28f *half_T);	
	
	mag_norm_xyz = my_sqrt(ak8975_fc.Mag_Val.x * ak8975_fc.Mag_Val.x + ak8975_fc.Mag_Val.y * ak8975_fc.Mag_Val.y + ak8975_fc.Mag_Val.z * ak8975_fc.Mag_Val.z);
	if(mag_norm_xyz==0)
		mag_norm_xyz=0.0001;
		if( mag_norm_xyz != 0)
	{
		mag_tmp.x += mag_norm_tmp *( (float)ak8975_fc.Mag_Val.x /( mag_norm_xyz ) - mag_tmp.x);
		mag_tmp.y += mag_norm_tmp *( (float)ak8975_fc.Mag_Val.y /( mag_norm_xyz ) - mag_tmp.y);	
		mag_tmp.z += mag_norm_tmp *( (float)ak8975_fc.Mag_Val.z /( mag_norm_xyz ) - mag_tmp.z);	
	}

	/*
	void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out)
	
	罗盘数据是机体坐标下的，且磁场方向不是平行于地面，如果飞机倾斜，投影计算的角度会存在误差。
	此函数可在一定范围内做近似转换，让结果逼近实际角度，减小飞机倾斜的影响。
	注意：该函数内的计算并不是正确也不是准确的，正确的计算相对复杂，这里不给出，在未来的版本中会再更新。
	*/
	simple_3d_trans(&reference_v,&mag_tmp,&mag_sim_3d); 
	
	mag_norm = my_sqrt(mag_sim_3d.x * mag_sim_3d.x + mag_sim_3d.y *mag_sim_3d.y);
	if(mag_norm==0)
		mag_norm=0.0001;
	if( mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 && mag_norm != 0)
	{
		yaw_mag = fast_atan2( ( mag_sim_3d.y/mag_norm ) , ( mag_sim_3d.x/mag_norm) ) *57.3f;
		
	}
	#if USE_MINI_FC_FLOW_BOARD
	  yaw_mag=Yaw_fc1;
	#else
	static float yaw_imu_off;
	if(NAV_BOARD_CONNECT)
	yaw_imu_off=Yaw_fc-yaw_mag;	
	
	yaw_mag=Yaw_fc1+yaw_imu_off;
	#endif
	  yaw_mag=Yaw_fc1;//no hml correct
//---------acc norm---------
	float norm;
	norm = sqrt(ax*(ax) + ay*(ay) + az*az)/4096.*9.8;
	calculateAccConfidence(norm);
	
	Kp_use =	Kp* accConfidence ;
	//=============================================================================
	// 计算等效重力向量
	reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];

	
	//这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
	//根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
	//所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。       
	//=============================================================================

	if(acc_ng_cali)
	{
		if(acc_ng_cali==2)
		{
			acc_ng_offset.x = 0;
			acc_ng_offset.y = 0;
			acc_ng_offset.z = 0;
		}
			
		acc_ng_offset.x += 10 *TO_M_S2 *(ax - 4096*reference_v.x) *0.0125f ;
		acc_ng_offset.y += 10 *TO_M_S2 *(ay - 4096*reference_v.y) *0.0125f ;
		acc_ng_offset.z += 10 *TO_M_S2 *(az - 4096*reference_v.z) *0.0125f ;	
		
		acc_ng_cali ++;
		if(acc_ng_cali>=82) //start on 2
		{
			acc_ng_cali = 0;
		}
	}
	
	acc_ng.x = 10 *TO_M_S2 *(ax - 4096*reference_v.x) - acc_ng_offset.x;
	acc_ng.y = 10 *TO_M_S2 *(ay - 4096*reference_v.y) - acc_ng_offset.y;
	acc_ng.z = 10 *TO_M_S2 *(az - 4096*reference_v.z) - acc_ng_offset.z;
	
	acc_3d_hg.z = acc_ng.x *reference_v.x + acc_ng.y *reference_v.y + acc_ng.z *reference_v.z;
	

	// 计算加速度向量的模
	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);   
  if(norm_acc==0)
		norm_acc=1;
  
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
	
	if( reference_v.z > 0.0f )
	{
		if( fly_ready  )
		{
			yaw_correct = Kp_Yaw *0.2f *To_180_degrees(yaw_mag - Yaw_fc1);
			//已经解锁，只需要低速纠正。
		}
		else
		{
			yaw_correct = Kp_Yaw *1.5f *To_180_degrees(yaw_mag - Yaw_fc1);
			//没有解锁，视作开机时刻，快速纠正
		}
// 		if( yaw_correct>360 || yaw_correct < -360  )
// 		{
// 			yaw_correct = 0;
// 			//限制纠正范围+-360，配合+-180度取值函数
// 		}
	}
	else
	{
		yaw_correct = 0; //角度过大，停止修正，修正的目标值可能不正确
	}

	
	ref.g.x = (gx - reference_v.x *yaw_correct) *ANGLE_TO_RADIAN + ( Kp_use*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	ref.g.y = (gy - reference_v.y *yaw_correct) *ANGLE_TO_RADIAN + ( Kp_use*(ref.err.y + ref.err_Int.y) ) ;		  //IN RADIAN
	ref.g.z = (gz - reference_v.z *yaw_correct) *ANGLE_TO_RADIAN;
	
	/* 用叉积误差来做PI修正陀螺零偏 */

	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] +(-ref_q[1]*ref.g.x - ref_q[2]*ref.g.y - ref_q[3]*ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0]*ref.g.x + ref_q[2]*ref.g.z - ref_q[3]*ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0]*ref.g.y - ref_q[1]*ref.g.z + ref_q[3]*ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0]*ref.g.z + ref_q[1]*ref.g.y - ref_q[2]*ref.g.x)*half_T;  

	/* 四元数规一化 normalise quaternion */
	norm_q = my_sqrt(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]);
	if(norm_q==0)
		norm_q=1;
	ref_q_imd_down_fc[0]=ref_q[0] = ref_q[0] / norm_q;
	ref_q_imd_down_fc[1]=ref_q[1] = ref_q[1] / norm_q;
	ref_q_imd_down_fc[2]=ref_q[2] = ref_q[2] / norm_q;
	ref_q_imd_down_fc[3]=ref_q[3] = ref_q[3] / norm_q;
	
  reference_vr_imd_down_fc[0] = 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[3] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[2]);
	reference_vr_imd_down_fc[1] = 2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[3]);
	reference_vr_imd_down_fc[2] = 1 - 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[2]);
	*rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;

	*yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f  ;// 
	//*yaw = yaw_mag;

}


//------------------------------------梯度下降-----------------------------------------------
// Definitions

#define sampleFreq	400.0f		// sample frequency in Hz

float Yaw_fc_q;
//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = 0.01f;								// 2 * proportional gain (Kp)
volatile float q0_m_fc = 1.0f, q1_m_fc = 0.0f, q2_m_fc = 0.0f, q3_m_fc = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
volatile float q0_fc = 1.0f, q1_fc = 0.0f, q2_fc = 0.0f, q3_fc = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
	float ref_q_imd_down_fc[4] = {1,0,0,0};
	float reference_vr_imd_down_fc[3];
	u8 init_q=1;
//void MadgwickAHRSupdate(float dt,float gx, float gy, float gz, float ax, float ay, float az, 
//	float mx, float my, float mz,float *rol,float *pit,float *yaw){
//  float T;
//	float recipNorm;
//	float s0, s1, s2, s3;
//	float qDot1, qDot2, qDot3, qDot4;
//	float hx, hy;
//	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
//  static u16 init_cnt;
//		if(init_cnt++>100){init_cnt=100+1;T=dt;
//		}
//		else 
//		{		
//	  if(init_q){
//    float Pit,Rol;
//    Pit=-atan(mpu6050_fc.Acc.x/mpu6050_fc.Acc.z)*57.3;
//		Rol=atan(mpu6050_fc.Acc.y/mpu6050_fc.Acc.z)*57.3;
//		float euler[2]; 	
//		euler[1]=Pit*0.0173  ;
//		euler[0]=Rol*0.0173  ;	
//		#if IMU_HML_ADD_500
//		float magTmp2 [3];	
//		magTmp2[0]=ak8975_fc.Mag_Val.x;
//		magTmp2[1]=ak8975_fc.Mag_Val.y;
//		magTmp2[2]=ak8975_fc.Mag_Val.z;
//		float calMagY = magTmp2[0] * cos(euler[1]) + magTmp2[1] * sin(euler[1])* sin(euler[0])+magTmp2[2] * sin(euler[1]) * cos(euler[0]); 
//		float calMagX = magTmp2[1] * cos(euler[0]) + magTmp2[2] * sin(euler[0]);
//		float yaw_mag=To_180_degrees(fast_atan2(calMagX,calMagY)* 57.3 +180);
//		#endif	
//		float angle_cal[3];
//		angle_cal[0]=euler[0];
//		angle_cal[1]=euler[1];
//		angle_cal[2]=yaw_mag;
//		euler_to_q(angle_cal,ref_q_imd_down_fc);
//	  q0_fc=ref_q_imd_down_fc[0];
//	  q1_fc=ref_q_imd_down_fc[1];
//	  q2_fc=ref_q_imd_down_fc[2];
//	  q3_fc=ref_q_imd_down_fc[3];	
//	  }T=2*dt;
//		
//	}
//	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
//	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {

//	MadgwickAHRSupdateIMU(T,gx, gy, gz, ax, ay, az);
//	ref_q_imd_down_fc[0]=q0_fc;
//	ref_q_imd_down_fc[1]=q1_fc;
//	ref_q_imd_down_fc[2]=q2_fc;
//	ref_q_imd_down_fc[3]=q3_fc;
//	reference_vr_imd_down_fc[0] = 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[3] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[2]);
//	reference_vr_imd_down_fc[1] = 2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[3]);
//	reference_vr_imd_down_fc[2] = 1 - 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[2]);
//	*rol = atan2(2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[3]),1 - 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[2])) *57.3f;
//	*pit = asin(2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[3] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[2])) *57.3f;
//	*yaw = fast_atan2(2*(-ref_q_imd_down_fc[1]*ref_q_imd_down_fc[2] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[3]), 2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[0] + ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1]) - 1) *57.3f  ;// 

//		return;
//	}
//	// Rate of change of quaternion from gyroscope
//	qDot1 = 0.5f * (-q1_fc * gx - q2_fc * gy - q3_fc * gz);
//	qDot2 = 0.5f * (q0_fc * gx + q2_fc * gz - q3_fc * gy);
//	qDot3 = 0.5f * (q0_fc * gy - q1_fc * gz + q3_fc * gx);
//	qDot4 = 0.5f * (q0_fc * gz + q1_fc * gy - q2_fc * gx);

//	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

//		// Normalise accelerometer measurement
//		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
//		ax *= recipNorm;
//		ay *= recipNorm;
//		az *= recipNorm;   

//		// Normalise magnetometer measurement
//		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
//		mx *= recipNorm;
//		my *= recipNorm;
//		mz *= recipNorm;

//		// Auxiliary variables to avoid repeated arithmetic
//		_2q0mx = 2.0f * q0_fc * mx;
//		_2q0my = 2.0f * q0_fc * my;
//		_2q0mz = 2.0f * q0_fc * mz;
//		_2q1mx = 2.0f * q1_fc * mx;
//		_2q0 = 2.0f * q0_fc;
//		_2q1 = 2.0f * q1_fc;
//		_2q2 = 2.0f * q2_fc;
//		_2q3 = 2.0f * q3_fc;
//		_2q0q2 = 2.0f * q0_fc * q2_fc;
//		_2q2q3 = 2.0f * q2_fc * q3_fc;
//		q0q0 = q0_fc * q0_fc;
//		q0q1 = q0_fc * q1_fc;
//		q0q2 = q0_fc * q2_fc;
//		q0q3 = q0_fc * q3_fc;
//		q1q1 = q1_fc * q1_fc;
//		q1q2 = q1_fc * q2_fc;
//		q1q3 = q1_fc * q3_fc;
//		q2q2 = q2_fc * q2_fc;
//		q2q3 = q2_fc * q3_fc;
//		q3q3 = q3_fc * q3_fc;

//		// Reference direction of Earth's magnetic field
//		hx = mx * q0q0 - _2q0my * q3_fc + _2q0mz * q2_fc + mx * q1q1 + _2q1 * my * q2_fc + _2q1 * mz * q3_fc - mx * q2q2 - mx * q3q3;
//		hy = _2q0mx * q3_fc + my * q0q0 - _2q0mz * q1_fc + _2q1mx * q2_fc - my * q1q1 + my * q2q2 + _2q2 * mz * q3_fc - my * q3q3;
//		_2bx = my_sqrt(hx * hx + hy * hy);
//		_2bz = -_2q0mx * q2_fc + _2q0my * q1_fc + mz * q0q0 + _2q1mx * q3_fc - mz * q1q1 + _2q2 * my * q3_fc - mz * q2q2 + mz * q3q3;
//		_4bx = 2.0f * _2bx;
//		_4bz = 2.0f * _2bz;

//		// Gradient decent algorithm corrective step
//		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2_fc * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3_fc + _2bz * q1_fc) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) 
//		+ _2bx * q2_fc * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
//		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1_fc * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3_fc * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) 
//		+ (_2bx * q2_fc + _2bz * q0_fc) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3_fc - _4bz * q1_fc) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
//		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2_fc * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
//		+ (-_4bx * q2_fc - _2bz * q0_fc) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1_fc + _2bz * q3_fc) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0_fc - _4bz * q2_fc) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
//		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3_fc + _2bz * q1_fc) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + 
//		(-_2bx * q0_fc + _2bz * q2_fc) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1_fc * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
//		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
//		s0 *= recipNorm;
//		s1 *= recipNorm;
//		s2 *= recipNorm;
//		s3 *= recipNorm;

//		// Apply feedback step
//		qDot1 -= beta * s0;
//		qDot2 -= beta * s1;
//		qDot3 -= beta * s2;
//		qDot4 -= beta * s3;
//	}

//	// Integrate rate of change of quaternion to yield quaternion
//	q0_fc += qDot1 * (1.0f / sampleFreq);
//	q1_fc += qDot2 * (1.0f / sampleFreq);
//	q2_fc += qDot3 * (1.0f / sampleFreq);
//	q3_fc += qDot4 * (1.0f / sampleFreq);

//	// Normalise quaternion
//	recipNorm = invSqrt(q0_fc * q0_fc + q1_fc * q1_fc + q2_fc * q2_fc + q3_fc * q3_fc);
//	q0_fc *= recipNorm;
//	q1_fc *= recipNorm;
//	q2_fc *= recipNorm;
//	q3_fc *= recipNorm;

//	ref_q_imd_down[0]=q0_fc;
//	ref_q_imd_down[1]=q1_fc;
//	ref_q_imd_down[2]=q2_fc;
//	ref_q_imd_down[3]=q3_fc;
//	reference_vr_imd_down_fc[0] = 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[3] - ref_q_imd_down_fc[0]*ref_q_imd_down_fc[2]);
//	reference_vr_imd_down_fc[1] = 2*(ref_q_imd_down_fc[0]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[3]);
//	reference_vr_imd_down_fc[2] = 1 - 2*(ref_q_imd_down_fc[1]*ref_q_imd_down_fc[1] + ref_q_imd_down_fc[2]*ref_q_imd_down_fc[2]);
//	*rol = fast_atan2(2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]),1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2])) *57.3f;
//	*pit = asin(2*(ref_q[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2])) *57.3f;
//	*yaw = fast_atan2(2*(-ref_q_imd_down[1]*ref_q_imd_down[2] - ref_q_imd_down[0]*ref_q_imd_down[3]), 2*(ref_q_imd_down[0]*ref_q_imd_down[0] + ref_q_imd_down[1]*ref_q_imd_down[1]) - 1) *57.3f  ;// 

//}

////---------------------------------------------------------------------------------------------------
//// IMU algorithm update

//void MadgwickAHRSupdateIMU(float dt,float gx, float gy, float gz, float ax, float ay, float az) {
//	float recipNorm;
//	float s0, s1, s2, s3;
//	float qDot1, qDot2, qDot3, qDot4;
//	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

//	// Rate of change of quaternion from gyroscope
//	qDot1 = 0.5f * (-q1_fc * gx - q2_fc * gy - q3_fc * gz);
//	qDot2 = 0.5f * (q0_fc * gx + q2_fc * gz - q3_fc * gy);
//	qDot3 = 0.5f * (q0_fc * gy - q1_fc * gz + q3_fc * gx);
//	qDot4 = 0.5f * (q0_fc * gz + q1_fc * gy - q2_fc * gx);

//	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

//		// Normalise accelerometer measurement
//		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
//		ax *= recipNorm;
//		ay *= recipNorm;
//		az *= recipNorm;   

//		// Auxiliary variables to avoid repeated arithmetic
//		_2q0 = 2.0f * q0_fc;
//		_2q1 = 2.0f * q1_fc;
//		_2q2 = 2.0f * q2_fc;
//		_2q3 = 2.0f * q3_fc;
//		_4q0 = 4.0f * q0_fc;
//		_4q1 = 4.0f * q1_fc;
//		_4q2 = 4.0f * q2_fc;
//		_8q1 = 8.0f * q1_fc;
//		_8q2 = 8.0f * q2_fc;
//		q0q0 = q0_fc * q0_fc;
//		q1q1 = q1_fc * q1_fc;
//		q2q2 = q2_fc * q2_fc;
//		q3q3 = q3_fc * q3_fc;

//		// Gradient decent algorithm corrective step
//		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
//		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_fc - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
//		s2 = 4.0f * q0q0 * q2_fc + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
//		s3 = 4.0f * q1q1 * q3_fc - _2q1 * ax + 4.0f * q2q2 * q3_fc - _2q2 * ay;
//		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
//		s0 *= recipNorm;
//		s1 *= recipNorm;
//		s2 *= recipNorm;
//		s3 *= recipNorm;

//		// Apply feedback step
//		qDot1 -= beta * s0;
//		qDot2 -= beta * s1;
//		qDot3 -= beta * s2;
//		qDot4 -= beta * s3;
//	}

//	// Integrate rate of change of quaternion to yield quaternion
//	q0_fc += qDot1 * dt;
//	q1_fc += qDot2 * dt;
//	q2_fc += qDot3 * dt;
//	q3_fc += qDot4 * dt;

//	// Normalise quaternion
//	recipNorm = invSqrt(q0_fc * q0_fc + q1_fc * q1_fc + q2_fc * q2_fc + q3_fc * q3_fc);
//	q0_fc *= recipNorm;
//	q1_fc *= recipNorm;
//	q2_fc *= recipNorm;
//	q3_fc *= recipNorm;
//	
//	

//}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}



