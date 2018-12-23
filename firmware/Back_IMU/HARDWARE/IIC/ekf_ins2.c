#include "ekf_ins.h"
#include "mpu6050.h"
#include "hml_sample.h"
#include "hml5833l.h"
#include "ms5611_2.h"
#include "ekf_math.h"

NavStruct Nav;       //导航结构体
GPS_Sensor_Struct           Global_GPS_Sensor;
#define DEG_RAD                0.01745329252f //角度转化成弧度的比例因子
float  Global_Now_Euler[3];
float Global_accle_m_s2[3], Global_gyro_rad_s[3], Global_Q[4], Global_Init_Q[4];


#define Xn 7
#define Qn 7
#define Zn 4
#define Rn 4

static float EKF_X[Xn];
static float EKF_Z[Zn];
static float EKF_P[Xn*Xn];
static float EKF_Q[Qn];
static float EKF_R[Rn];
static float EKF_K[Xn*Zn];
static float EKF_F[Xn*Xn]; 
static float EKF_G[Xn*Qn]; 
static float EKF_H[Zn*Xn];

	
static float qmag;
#if defined USE_MAG
		static float euler[3];
#else
		static float q[4];
		static float u[3];
#endif	

static void EKF_Clear(void)
{
		int i;
		for(i = 0; i < Xn; i++)
		{
				EKF_X[i] = 0;
		}
		for(i = 0; i < Zn; i++)
		{
				EKF_Z[i] = 0;
		}
		for(i = 0; i < Qn; i++)
		{
				EKF_Q[i] = 0;
		}
		for(i = 0; i < Rn; i++)
		{
				EKF_R[i] = 0;
		}
		for(i = 0; i < Xn * Xn; i++)
		{
				EKF_P[i] = 0;
				EKF_F[i] = 0;
		}
		for(i = 0; i < Xn*Zn; i++)
		{
				EKF_K[i] = 0;
		}
		for(i = 0; i < Xn*Qn; i++)
		{
				EKF_G[i] = 0;
		}
		for(i = 0; i < Zn*Xn; i++)
		{
				EKF_H[i] = 0;
		}
}

static void EKF_AttitudeInit(void)
{	
		EKF_Clear();
	
		EKF_P[0] = EKF_P[8] = EKF_P[16] = EKF_P[24] = 1e-6f;
		EKF_P[32] = EKF_P[40] = EKF_P[48] = 1e-12f;
	
		EKF_X[0] = 1.0f;
		EKF_X[1] = EKF_X[2] = EKF_X[3] = 0.0f;
	
		EKF_X[4] = EKF_X[5] = EKF_X[6] = 0.0f;
	
		EKF_Q[0]  = EKF_Q[1] = EKF_Q[2] = EKF_Q[3] = 1e-6f;// Quaternion process noise
		EKF_Q[4]  = EKF_Q[5] = EKF_Q[6] = 1e-6f;      // Gyro bias process noise, rad/s
	
		EKF_R[0]  = EKF_R[1] = EKF_R[2] = EKF_R[3] = 4e-3f;
#if 1	
		EKF_X[0] = Global_Init_Q[0];
		EKF_X[1] = Global_Init_Q[1];
		EKF_X[2] = Global_Init_Q[2];
		EKF_X[3] = Global_Init_Q[3];
#endif
}

static void EKF_Xdot_Cal(float x[Xn], float u[3], float xdot[Xn])
{
		float wx, wy, wz, q0, q1, q2, q3;
		q0 = x[0];
		q1 = x[1];
		q2 = x[2];
		q3 = x[3];
		wx = u[0] - x[4];
		wy = u[1] - x[5];
		wz = u[2] - x[6];
	
		// qdot = Q*w
    xdot[0] = (-q1 * wx - q2 * wy - q3 * wz) / 2.0f;
    xdot[1] = (q0 * wx - q3 * wy + q2 * wz) / 2.0f;
    xdot[2] = (q3 * wx + q0 * wy - q1 * wz) / 2.0f;
    xdot[3] = (-q2 * wx + q1 * wy + q0 * wz) / 2.0f;
	
		// best guess is that bias stays constant
		xdot[4] = 0.0f;
		xdot[5] = 0.0f;
		xdot[6] = 0.0f;
}

static void EKF_RungeKutta(float X[Xn], float U[3], float dT)
{
    float dT2 = dT / 2.0f, K1[Xn], K2[Xn], K3[Xn], K4[Xn], Xlast[Xn];
    uint8_t i;

    for (i = 0; i < Xn; i++) 
		{
        Xlast[i] = X[i]; // make a working copy
    }
    EKF_Xdot_Cal(X, U, K1); // k1 = f(x,u)
    for (i = 0; i < Xn; i++) 
		{
        X[i] = Xlast[i] + dT2 * K1[i];
    }
    EKF_Xdot_Cal(X, U, K2); // k2 = f(x+0.5*dT*k1,u)
    for (i = 0; i < Xn; i++) 
		{
        X[i] = Xlast[i] + dT2 * K2[i];
    }
    EKF_Xdot_Cal(X, U, K3); // k3 = f(x+0.5*dT*k2,u)
    for (i = 0; i < Xn; i++) 
		{
        X[i] = Xlast[i] + dT * K3[i];
    }
    EKF_Xdot_Cal(X, U, K4); // k4 = f(x+dT*k3,u)

    // Xnew  = X + dT*(k1+2*k2+2*k3+k4)/6
    for (i = 0; i < Xn; i++) 
		{
				X[i] = Xlast[i] + dT * (K1[i] + 2.0f * K2[i] + 2.0f * K3[i] + K4[i]) / 6.0f;
    }
}

static void EKF_LinearizeFG(float x[Xn], float u[3], float f[Xn*Xn], float g[Xn*Qn])
{
		float wx, wy, wz, q0, q1, q2, q3;
	
		q0 = x[0];
		q1 = x[1];
		q2 = x[2];
		q3 = x[3];
		wx = u[0] - x[4];
		wy = u[1] - x[5];
		wz = u[2] - x[6];
	
		// dqdot/dq
    f[0*Xn+0]  = 0;
    f[0*Xn+1]  = -wx / 2.0f;
    f[0*Xn+2]  = -wy / 2.0f;
    f[0*Xn+3]  = -wz / 2.0f;
	
    f[1*Xn+0]  = wx / 2.0f;
    f[1*Xn+1]  = 0;
    f[1*Xn+2]  = wz / 2.0f;
    f[1*Xn+3]  = -wy / 2.0f;
	
    f[2*Xn+0]  = wy / 2.0f;
    f[2*Xn+1]  = -wz / 2.0f;
    f[2*Xn+2]  = 0;
    f[2*Xn+3]  = wx / 2.0f;
		
    f[3*Xn+0]  = wz / 2.0f;
    f[3*Xn+1]  = wy / 2.0f;
    f[3*Xn+2]  = -wx / 2.0f;
    f[3*Xn+3]  = 0;
		
		// dqdot/dwbias
    f[0*Xn+4] = q1 / 2.0f;
    f[0*Xn+5] = q2 / 2.0f;
    f[0*Xn+6] = q3 / 2.0f;
    f[1*Xn+4] = -q0 / 2.0f;
    f[1*Xn+5] = q3 / 2.0f;
    f[1*Xn+6] = -q2 / 2.0f;
    f[2*Xn+4] = -q3 / 2.0f;
    f[2*Xn+5] = -q0 / 2.0f;
    f[2*Xn+6] = q1 / 2.0f;
    f[2*Xn+4] = q2 / 2.0f;
    f[2*Xn+5] = -q1 / 2.0f;
    f[2*Xn+6] = -q0 / 2.0f;

		g[0*Qn+0] = 1.0f;
		g[1*Qn+1] = 1.0f;
		g[2*Qn+2] = 1.0f;
		g[3*Qn+3] = 1.0f;
		g[4*Qn+4] = 1.0f;
		g[5*Qn+5] = 1.0f;
		g[6*Qn+6] = 1.0f;
}

//gyro_data unit rad/s  dT unit s
static void EKF_StatePrediction(float gyro_data[3], float dT)
{
		static float U[3];
		static float q[4], euler[3];
		U[0] = gyro_data[0];
    U[1] = gyro_data[1];
    U[2] = gyro_data[2];
		EKF_LinearizeFG(EKF_X, U, EKF_F, EKF_G);
		EKF_RungeKutta(EKF_X, U, dT);
#if 1 // 防止偏航角在+/-180°是发生突变
		q[0] = EKF_X[0];
		q[1] = EKF_X[1];
		q[2] = EKF_X[2];
		q[3] = EKF_X[3];
		Quaternion2Euler(q, euler);	
		if(euler[2] > 180)
		{
				euler[2] = -180;
		}
		else if(euler[2] < -180)
		{
				euler[2] = 180;
		}
		Euler2Quaternion(euler, q);
		EKF_X[0] = q[0];
		EKF_X[1] = q[1];
		EKF_X[2] = q[2];
		EKF_X[3] = q[3];
#endif
}

static void EKF_CovariancePrediction(float F[Xn*Xn], float G[Xn*Qn], float Q[Qn], float dT, float P[Xn*Xn])
{	
#if 1
		int i;
		float fdt[Xn * Xn];
		float AT[Xn * Xn];
		float Ap[Xn * Xn];
		float A[Xn * Xn];
		float ApAT[Xn * Xn];
		for(i = 0; i < Xn * Xn; i++)
		{
				//A[i] = 0;
				A[i] = fdt[i] = F[i] * dT;
		}
		A[0*Xn+0] = fdt[0*Xn+0] + 1.0f;
		A[1*Xn+1] = fdt[1*Xn+1] + 1.0f;
		A[2*Xn+2] = fdt[2*Xn+2] + 1.0f;
		A[3*Xn+3] = fdt[3*Xn+3] + 1.0f;
		A[4*Xn+4] = fdt[4*Xn+4] + 1.0f;
		A[5*Xn+5] = fdt[5*Xn+5] + 1.0f;
		A[6*Xn+6] = fdt[6*Xn+6] + 1.0f;
		ML_R_X_ML_R(A, P, Ap, Xn, Xn, Xn, Xn);
		Matrix_Tran(A, AT, Xn, Xn);		
		ML_R_X_ML_R(Ap, AT, ApAT, Xn, Xn, Xn, Xn);
	#if 1
		for(i = 0; i < Xn*Xn; i++)
		{
				P[i] = ApAT[i];
		}
	#endif
		P[0*Xn+0] = P[0*Xn+0] + Q[0];	
		P[1*Xn+1] = P[1*Xn+1] + Q[1];
		P[2*Xn+2] = P[2*Xn+2] + Q[2];
		P[3*Xn+3] = P[3*Xn+3] + Q[3];
		P[4*Xn+4] = P[4*Xn+4] + Q[4];
		P[5*Xn+5] = P[5*Xn+5] + Q[5];
		P[6*Xn+6] = P[6*Xn+6] + Q[6];
#else
		float Dummy[Xn*Xn], dTsq;
    uint8_t i, j, k;
    // Pnew = (I+F*T)*P*(I+F*T)' + T^2*G*Q*G' = T^2[(P/T + F*P)*(I/T + F') + G*Q*G')]
    dTsq = dT * dT;
    for (i = 0; i < Xn; i++) 
		{ // Calculate Dummy = (P/T +F*P)
        for (j = 0; j < Xn; j++) 
				{
            Dummy[i*Xn+j] = P[i*Xn+j] / dT;
            for (k = 0; k < Xn; k++) 
						{
                Dummy[i*Xn+j] += F[i*Xn+k] * P[k*Xn+j];
            }
        }
    }
    for (i = 0; i < Xn; i++) 
		{ // Calculate Pnew = Dummy/T + Dummy*F' + G*Qw*G'
        for (j = i; j < Xn; j++) 
				{ // Use symmetry, ie only find upper triangular
            P[i*Xn+j] = Dummy[i*Xn+j] / dT;
            for (k = 0; k < Xn; k++) 
						{
                P[i*Xn+j] += Dummy[i*Xn+k] * F[j*Xn+k]; // P = Dummy/T + Dummy*F'
            }
            for (k = 0; k < Qn; k++) 
						{
                P[i*Xn+j] += Q[k] * G[i*Qn+k] * G[j*Qn+k]; // P = Dummy/T + Dummy*F' + G*Q*G'
            }
            P[j*Xn+i] = P[i*Xn+j] = P[i*Xn+j] * dTsq; // Pnew = T^2*P and fill in lower triangular;
        }
    }
#endif
}

static void EKF_INSCovariancePrediction(float dT)
{
    EKF_CovariancePrediction(EKF_F, EKF_G, EKF_Q, dT, EKF_P);
}

static void EKF_LinearizeH(float h[Zn*Xn])
{
		h[0*Xn+0] = 1.0f;
    h[1*Xn+1] = 1.0f;
    h[2*Xn+2] = 1.0f;
		h[3*Xn+3] = 1.0f;
}

static void EKF_Z_Cal(float accel[3], float mag_data[3], float z[Zn])
{
#if defined USE_MAG	
	
		#if defined USE_MAG_HMC5883L
			euler[2] = atan2(mag_data[1], mag_data[0]) * RAD_DEG;		
		#elif defined	USE_MAG_LSM303D
			Get_Ref_Euler(accel, mag_data, &Mag_LSM303D_Calibration, euler);
		#endif
	
		Global_Show_Val[5] = euler[0];
		Global_Show_Val[6] = euler[1];
		Global_Show_Val[7] = euler[2];
	
		//euler[2] -= Global_Heading_Ref;
#else		
		u[0] = accel[0];
    u[1] = accel[1];
    u[2] = accel[2];
		
		q[0] = EKF_X[0];
		q[1] = EKF_X[1];
		q[2] = EKF_X[2];
		q[3] = EKF_X[3];
		
		Quaternion2Euler(q, euler);
		
		//Global_Show_Val[5] = euler[0] = atan2(u[1], u[2]) * RAD_DEG;
		//Global_Show_Val[6] = euler[1] = atan2(-1 * u[0], u[2]) * RAD_DEG;
#endif	
		Euler2Quaternion(euler, z);
		qmag = sqrt(z[0]*z[0] + z[1]*z[1] + z[2]*z[2] + z[3]*z[3]); 
		z[0] /= qmag;
		z[1] /= qmag;
		z[2] /= qmag;
		z[3] /= qmag;		
}

static void EKF_K_Cal(float P[Xn*Xn], float h[Zn*Xn], float r[Zn], float k[Xn*Zn])
{
		float hT[Xn*Zn];
		float hp[Zn*Xn];
		float hphT[Zn*Zn];
		float inv_hphT[Zn*Zn];
		float phT[Xn*Zn];
			
		Matrix_Tran(h, hT, Zn, Xn);

		ML_R_X_ML_R(h, P, hp, Zn, Xn, Xn, Xn);
		ML_R_X_ML_R(hp, hT, hphT, Zn, Xn, Xn, Zn);
	
		hphT[0*Zn+0] += r[0];
		hphT[1*Zn+1] += r[1];
		hphT[2*Zn+2] += r[2];
		hphT[3*Zn+3] += r[3];

		Matrix_4X4_Inv(hphT, inv_hphT);

		ML_R_X_ML_R(P, hT, phT, Xn, Xn, Xn, Zn);
		ML_R_X_ML_R(phT, inv_hphT, k, Xn, Zn, Zn, Zn);				
}

static void EKF_Correction_X(float z[Zn], float h[Zn*Xn], float k[Xn*Zn], float x[Xn])
{
		float hx[Zn];
		float error[Zn];
		float cx[Xn];
		float qmag = 1;
		int i;
	
		ML_R_X_ML_R(h, x, hx, Zn, Xn, Xn, 1);

		error[0] = z[0] - hx[0];
		error[1] = z[1] - hx[1];
		error[2] = z[2] - hx[2];
		error[3] = z[3] - hx[3];
		
		ML_R_X_ML_R(k, error, cx, Xn, Zn, Zn, 1);

		for(i = 0; i < Xn; i++)
		{
				x[i] += cx[i];
		}
		qmag = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]);
		x[0] /= qmag;
		x[1] /= qmag;
		x[2] /= qmag;
		x[3] /= qmag;
}

static void EKF_Correction_P(float k[Xn*Zn], float h[Zn*Xn], float P[Xn*Xn])
{
		float kh[Xn*Xn];
		float khp[Xn*Xn];
		int i;

		ML_R_X_ML_R(k, h, kh, Xn, Zn, Zn, Xn);
		ML_R_X_ML_R(kh, P, khp, Xn, Xn, Xn, Xn);

		for(i = 0; i < Xn*Xn; i++)		
		{
				P[i] -= khp[i];
		}
}



void EKF_INS_GPS_Run(void)
{
		float gyro[3], accel[3], mag_data[3];
		float euler[3];
		float dT = 10 / 1000.0f;  //15ms
		static uint8_t isInit = false;
	
		if(!isInit)
		{
				EKF_AttitudeInit();
				isInit = true;
		}
		else
		{
		  	gyro[0] = mpu6050.Gyro_deg.x*DEG_RAD;
				gyro[1] = mpu6050.Gyro_deg.y*DEG_RAD*-1;
				gyro[2] = mpu6050.Gyro_deg.z*DEG_RAD*-1;
			
				accel[0] = mpu6050.Acc.x/4096* 1*9.8;
				accel[1] = mpu6050.Acc.y/4096* 1*9.8;
				accel[2] = mpu6050.Acc.z/4096* 1*9.8;
			
				mag_data[0] = ak8975.Mag_Val.x;///float)Global_Mag_Val[0];
				mag_data[1] = ak8975.Mag_Val.y;//(float)Global_Mag_Val[1];
				mag_data[2] = -ak8975.Mag_Val.z;//(float)Global_Mag_Val[2];
			
				EKF_StatePrediction(gyro, dT);
			
				EKF_INSCovariancePrediction(dT);

				EKF_Z_Cal(accel, mag_data, EKF_Z);
				EKF_LinearizeH(EKF_H);
#if 1		
				EKF_K_Cal(EKF_P, EKF_H, EKF_R, EKF_K);					
				EKF_Correction_X(EKF_Z, EKF_H, EKF_K, EKF_X);
				EKF_Correction_P(EKF_K, EKF_H, EKF_P);
#endif
				Nav.q[0] = EKF_X[0];
				Nav.q[1] = EKF_X[1];
				Nav.q[2] = EKF_X[2];
				Nav.q[3] = EKF_X[3];
			
				Quaternion2Euler(Nav.q, euler);	
				
				Global_Now_Euler[0] = euler[0];
				Global_Now_Euler[1] = euler[1];
				Global_Now_Euler[2] = euler[2];
		}
}







