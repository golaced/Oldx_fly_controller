//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files
#include "imu.h"
#include "include.h"
#include "hml5833l.h"
#include "my_math.h"
#include "filter.h"
#include "MadgwickAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	400.0f		// sample frequency in Hz


//---------------------------------------------------------------------------------------------------
// Variable definitions
// AHRS algorithm update
float ref_q_imd_down[4] = {1,0,0,0};
float reference_vr_imd_down[3];
volatile float beta = 0.01f;								// 2 * proportional gain (Kp)
volatile float q0_m = 1.0f, q1_m = 0.0f, q2_m = 0.0f, q3_m = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
//---------------------------------------------------------------------------------------------------
volatile float q0_fc = 1.0f, q1_fc = 0.0f, q2_fc = 0.0f, q3_fc = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
// Function declarations

float invSqrt(float x);
//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
//	float ref_q_imd_down[4] = {1,0,0,0};
float reference_vr_imd_down_fc[3];
u8 init_q=1;

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


//5.0f, 0.01f, 0.01f
//5.0f, 0.25f, 0.01f
volatile float beta_start=200;
volatile float beta_end=0.1666;
volatile float beta_step=0.01;

volatile float beta;
//accConfidenceDecay = 1.0f / sqrt(eepromConfig.accelCutoff);
float accConfidenceDecay = 5.2f;
float hmlConfidenceDecay = 2.8f;
float accConfidence      = 1.0f; 
float hmlConfidence 			= 1.0f; 
float norm_hml_view;
#define HardFilter(O,N)  ((O)*0.9f+(N)*0.1f)
#define accelOneG 9.8
void calculateAccConfidence(float accMag_in)
{
	// G.K. Egan (C) computes confidence in accelerometers when
	// aircraft is being accelerated over and above that due to gravity

	static float accMagP = 1.0f;
  float accMag=accMag_in;
	accMag /= accelOneG;  // HJI Added to convert MPS^2 to G's

	accMagP  = HardFilter(accMagP, accMag );

	accConfidence=((accConfidenceDecay * sqrt(fabs(accMagP - 1.0f))));
  if(accConfidence>1)
		accConfidence=1;
	if(accConfidence<0)
		accConfidence=0;
}


float hmlOneMAG= 855.0;
void calculateHmlConfidence(float Mag_in)
{
	// G.K. Egan (C) computes confidence in accelerometers when
	// aircraft is being accelerated over and above that due to gravity

	static float MagP = 1.0f;
  float Mag;
  Mag  = Mag_in/hmlOneMAG;  // HJI Added to convert MPS^2 to G's

	MagP  = HardFilter(MagP, Mag );

	hmlConfidence=((hmlConfidenceDecay*sqrt(fabs(MagP - 1.0f))));
  if(hmlConfidence>1)
		hmlConfidence=1;
	if(hmlConfidence<0)
		hmlConfidence=0;
}
// *  smpl_frq    sampling frequency of AHRS data
// *  b_start     algorithm gain starting value
// *  b_end       algorithm gain end value
// *  b_step      algorithm gain decrement step size
u8 init_hml_norm;
u16 init_hml_norm_cnt;
int madgwick_update_new(float ax,float ay,float az, float wx,float wy,float wz, float mx,float my ,float mz,float *rol,float *pit,float *yaw,float T)								
{
    static u8 init;
    float recip_norm,norm;
    float s0, s1, s2, s3;
    float dq1, dq2, dq3, dq4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

		if(!init){init=1;
				beta = beta_start;
		}
		
    /* Check for valid magnetometer data. */
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        return 0;
    }else if(!init_hml_norm&&init_hml_norm_cnt++>200){init_hml_norm=1;			
			hmlOneMAG = sqrt(mx*(mx) + my*(my) + mz*mz)*1.1;
			if(hmlOneMAG>440)
				module.hml=2;
			else 
				module.hml=1;
		}
    /* Check if beta has reached its specified end value or if it has to be
     * decremented. */
    if (beta > beta_end)
    {
        /* Decrement beta only if it does not fall below the specified end
         * value. */
        if ((beta - beta_step) > beta_end)
        {
            beta -= beta_step;
        }
        else
        {
            beta = beta_end;
        }
    }else beta=beta_end;

    /* Calculate quaternion rate of change of from angular velocity. */
    dq1 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
    dq2 = 0.5f * (q0 * wx + q2 * wz - q3 * wy);
    dq3 = 0.5f * (q0 * wy - q1 * wz + q3 * wx);
    dq4 = 0.5f * (q0 * wz + q1 * wy - q2 * wx);

    /* Calculate feedback only if accelerometer measurement is valid. This
     * prevents NaNs in acceleration normalization. */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {   
			  norm = sqrt(ax*(ax) + ay*(ay) + az*az)/4096.*9.8;
				calculateAccConfidence(norm);
		   	norm_hml_view = sqrt(mx*(mx) + my*(my) + mz*mz);
			  calculateHmlConfidence(norm_hml_view);
				beta *= accConfidence * hmlConfidence;
			
        /* Normalize accelerometer measurement. */
        recip_norm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        /* Normalize magnetometer measurement. */
        recip_norm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recip_norm;
        my *= recip_norm;
        mz *= recip_norm;

        /* Auxiliary variables to avoid repeated arithmetic and therefore
         * improve performance. */
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
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

        /* Reference direction of earth magnetic field. */
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        /* Gradient decent algorithm corrective step. */
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        /* Normalize step magnitude. */
        recip_norm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        /* Apply feedback step. */
        dq1 -= beta * s0;
        dq2 -= beta * s1;
        dq3 -= beta * s2;
        dq4 -= beta * s3;
    }

    /* Integrate quaternion rate of change to get quaternion describing the
     * current orientation. */
    q0 += dq1 * T;
    q1 += dq2 * T;
    q2 += dq3 * T;
    q3 += dq4 * T;

    /* Normalize quaternion. */
    recip_norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recip_norm;
    q1 *= recip_norm;
    q2 *= recip_norm;
    q3 *= recip_norm;
  #if YAW_USE_MAD_BUT_ATT_USE_EKF
	*rol = fast_atan2(2*(q0*q1 + q2*q3),1 - 2*(q1*q1 + q2*q2)) *57.3f;
	*pit = asin(2*(q1*q3 - q0*q2)) *57.3f;
	*yaw = fast_atan2(2*(-q1*q2 - q0*q3), 2*(q0*q0 + q1*q1) - 1) *57.3f  ;// 
	#else	
	ref_q_imd_down[0]=q0;
	ref_q_imd_down[1]=q1;
	ref_q_imd_down[2]=q2;
	ref_q_imd_down[3]=q3;
	reference_vr_imd_down[0] = 2*(ref_q_imd_down[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2]);
	reference_vr_imd_down[1] = 2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]);
	reference_vr_imd_down[2] = 1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2]);
	*rol = fast_atan2(2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]),1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q_imd_down[1]*ref_q_imd_down[2] - ref_q_imd_down[0]*ref_q_imd_down[3]), 2*(ref_q_imd_down[0]*ref_q_imd_down[0] + ref_q_imd_down[1]*ref_q_imd_down[1]) - 1) *57.3f  ;// 
  #endif
    return 1;
}
//====================================================================================================
// END OF CODE
//====================================================================================================
