/******************** +++++++++++++++++++++++++++ ********************************
  * 作者   ：未知
 * 文件名  ：mymath.c
 * 描述    ：快速计算

**********************************************************************************/

#include "my_math.h"
#include "FastMath.h"
int sign_flow(float in,float dead)
{
if (my_abs(in)>dead)
	return 1;
else
	return 0;
}

float fast_atan_table[257] = 
{
	0.000000e+00, 3.921549e-03, 7.842976e-03, 1.176416e-02,
	1.568499e-02, 1.960533e-02, 2.352507e-02, 2.744409e-02,
	3.136226e-02, 3.527947e-02, 3.919560e-02, 4.311053e-02,
	4.702413e-02, 5.093629e-02, 5.484690e-02, 5.875582e-02,
	6.266295e-02, 6.656816e-02, 7.047134e-02, 7.437238e-02,
	7.827114e-02, 8.216752e-02, 8.606141e-02, 8.995267e-02,
	9.384121e-02, 9.772691e-02, 1.016096e-01, 1.054893e-01,
	1.093658e-01, 1.132390e-01, 1.171087e-01, 1.209750e-01,
	1.248376e-01, 1.286965e-01, 1.325515e-01, 1.364026e-01,
	1.402496e-01, 1.440924e-01, 1.479310e-01, 1.517652e-01,
	1.555948e-01, 1.594199e-01, 1.632403e-01, 1.670559e-01,
	1.708665e-01, 1.746722e-01, 1.784728e-01, 1.822681e-01,
	1.860582e-01, 1.898428e-01, 1.936220e-01, 1.973956e-01,
	2.011634e-01, 2.049255e-01, 2.086818e-01, 2.124320e-01,
	2.161762e-01, 2.199143e-01, 2.236461e-01, 2.273716e-01,
	2.310907e-01, 2.348033e-01, 2.385093e-01, 2.422086e-01,
	2.459012e-01, 2.495869e-01, 2.532658e-01, 2.569376e-01,
	2.606024e-01, 2.642600e-01, 2.679104e-01, 2.715535e-01,
	2.751892e-01, 2.788175e-01, 2.824383e-01, 2.860514e-01,
	2.896569e-01, 2.932547e-01, 2.968447e-01, 3.004268e-01,
	3.040009e-01, 3.075671e-01, 3.111252e-01, 3.146752e-01,
	3.182170e-01, 3.217506e-01, 3.252758e-01, 3.287927e-01,
	3.323012e-01, 3.358012e-01, 3.392926e-01, 3.427755e-01,
	3.462497e-01, 3.497153e-01, 3.531721e-01, 3.566201e-01,
	3.600593e-01, 3.634896e-01, 3.669110e-01, 3.703234e-01,
	3.737268e-01, 3.771211e-01, 3.805064e-01, 3.838825e-01,
	3.872494e-01, 3.906070e-01, 3.939555e-01, 3.972946e-01,
	4.006244e-01, 4.039448e-01, 4.072558e-01, 4.105574e-01,
	4.138496e-01, 4.171322e-01, 4.204054e-01, 4.236689e-01,
	4.269229e-01, 4.301673e-01, 4.334021e-01, 4.366272e-01,
	4.398426e-01, 4.430483e-01, 4.462443e-01, 4.494306e-01,
	4.526070e-01, 4.557738e-01, 4.589307e-01, 4.620778e-01,
	4.652150e-01, 4.683424e-01, 4.714600e-01, 4.745676e-01,
	4.776654e-01, 4.807532e-01, 4.838312e-01, 4.868992e-01,
	4.899573e-01, 4.930055e-01, 4.960437e-01, 4.990719e-01,
	5.020902e-01, 5.050985e-01, 5.080968e-01, 5.110852e-01,
	5.140636e-01, 5.170320e-01, 5.199904e-01, 5.229388e-01,
	5.258772e-01, 5.288056e-01, 5.317241e-01, 5.346325e-01,
	5.375310e-01, 5.404195e-01, 5.432980e-01, 5.461666e-01,
	5.490251e-01, 5.518738e-01, 5.547124e-01, 5.575411e-01,
	5.603599e-01, 5.631687e-01, 5.659676e-01, 5.687566e-01,
	5.715357e-01, 5.743048e-01, 5.770641e-01, 5.798135e-01,
	5.825531e-01, 5.852828e-01, 5.880026e-01, 5.907126e-01,
	5.934128e-01, 5.961032e-01, 5.987839e-01, 6.014547e-01,
	6.041158e-01, 6.067672e-01, 6.094088e-01, 6.120407e-01,
	6.146630e-01, 6.172755e-01, 6.198784e-01, 6.224717e-01,
	6.250554e-01, 6.276294e-01, 6.301939e-01, 6.327488e-01,
	6.352942e-01, 6.378301e-01, 6.403565e-01, 6.428734e-01,
	6.453808e-01, 6.478788e-01, 6.503674e-01, 6.528466e-01,
	6.553165e-01, 6.577770e-01, 6.602282e-01, 6.626701e-01,
	6.651027e-01, 6.675261e-01, 6.699402e-01, 6.723452e-01,
	6.747409e-01, 6.771276e-01, 6.795051e-01, 6.818735e-01,
	6.842328e-01, 6.865831e-01, 6.889244e-01, 6.912567e-01,
	6.935800e-01, 6.958943e-01, 6.981998e-01, 7.004964e-01,
	7.027841e-01, 7.050630e-01, 7.073330e-01, 7.095943e-01,
	7.118469e-01, 7.140907e-01, 7.163258e-01, 7.185523e-01,
	7.207701e-01, 7.229794e-01, 7.251800e-01, 7.273721e-01,
	7.295557e-01, 7.317307e-01, 7.338974e-01, 7.360555e-01,
	7.382053e-01, 7.403467e-01, 7.424797e-01, 7.446045e-01,
	7.467209e-01, 7.488291e-01, 7.509291e-01, 7.530208e-01,
	7.551044e-01, 7.571798e-01, 7.592472e-01, 7.613064e-01,
	7.633576e-01, 7.654008e-01, 7.674360e-01, 7.694633e-01,
	7.714826e-01, 7.734940e-01, 7.754975e-01, 7.774932e-01,
	7.794811e-01, 7.814612e-01, 7.834335e-01, 7.853983e-01,
	7.853983e-01
};

float my_abs(float f)
{
	if (f >= 0.0f)
	{
		return f;
	}

	return -f;
}

REAL fast_atan2(REAL y, REAL x) 
{
	REAL x_abs, y_abs, z;
	REAL alpha, angle, base_angle;
	int index;

	/* don't divide by zero! */
	if ((y == 0.0f) && (x == 0.0f))
		angle = 0.0f;
	else 
	{
		/* normalize to +/- 45 degree range */
		y_abs = my_abs(y);
		x_abs = my_abs(x);
		//z = (y_abs < x_abs ? y_abs / x_abs : x_abs / y_abs);
		if (y_abs < x_abs)
			z = y_abs / x_abs;
		else
			z = x_abs / y_abs;
		/* when ratio approaches the table resolution, the angle is */
		/*      best approximated with the argument itself...       */
		if (z < TAN_MAP_RES)
			base_angle = z;
		else 
		{
			/* find index and interpolation value */
			alpha = z * (REAL) TAN_MAP_SIZE - .5f;
			index = (int) alpha;
			alpha -= (REAL) index;
			/* determine base angle based on quadrant and */
			/* add or subtract table value from base angle based on quadrant */
			base_angle = fast_atan_table[index];
			base_angle += (fast_atan_table[index + 1] - fast_atan_table[index]) * alpha;
		}

		if (x_abs > y_abs) 
		{        /* -45 -> 45 or 135 -> 225 */
			if (x >= 0.0f) 
			{           /* -45 -> 45 */
				if (y >= 0.0f)
					angle = base_angle;   /* 0 -> 45, angle OK */
				else
					angle = -base_angle;  /* -45 -> 0, angle = -angle */
			} 
			else
			{                  /* 135 -> 180 or 180 -> -135 */
				angle = 3.14159265358979323846;

				if (y >= 0.0f)
					angle -= base_angle;  /* 135 -> 180, angle = 180 - angle */
				else
					angle = base_angle - angle;   /* 180 -> -135, angle = angle - 180 */
			}
		} 
		else 
		{                    /* 45 -> 135 or -135 -> -45 */
			if (y >= 0.0f) 
			{           /* 45 -> 135 */
				angle = 1.57079632679489661923;

				if (x >= 0.0f)
					angle -= base_angle;  /* 45 -> 90, angle = 90 - angle */
				else
					angle += base_angle;  /* 90 -> 135, angle = 90 + angle */
			} 
			else
			{                  /* -135 -> -45 */
				angle = -1.57079632679489661923;

				if (x >= 0.0f)
					angle += base_angle;  /* -90 -> -45, angle = -90 + angle */
				else
					angle -= base_angle;  /* -135 -> -90, angle = -90 - angle */
			}
		}
	}


#ifdef ZERO_TO_TWOPI
	if (angle < 0)
		return (angle + TWOPI);
	else
		return (angle);
#else
	return (angle);
#endif
}

float my_atan(float x, float y)
{
	return fast_atan2(y, x);
}

//计算浮点数平方
float my_pow(float a)
{
	return a*a;
}

//快速平方根算法
float my_sqrt(float number)
{
	long i;
	float x, y;
	const float f = 1.5F;
	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );

	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) );
	y = y * ( f - ( x * y * y ) );
	return number * y;
}

#define ONE_PI   (3.14159265)
#define TWO_PI   (2.0 * 3.14159265)
#define ANGLE_UNIT (TWO_PI/10.0)

double mx_sin(double rad)
{   
	double sine;
	if (rad < 0)
		sine = rad*(1.27323954f + 0.405284735f * rad);
	else
		sine = rad * (1.27323954f - 0.405284735f * rad);
	if (sine < 0)
		sine = sine*(-0.225f * (sine + 1) + 1);
	else
		sine = sine * (0.225f *( sine - 1) + 1);
	return sine;
}

double my_sin(double rad)
{
	s8 flag = 1;

	if (rad >= ONE_PI)
	{
		rad -= ONE_PI;
		flag = -1;
	}

	return mx_sin(rad) * flag;
}

float my_cos(double rad)
{
	s8 flag = 1;
	rad += ONE_PI/2.0;

	if (rad >= ONE_PI)
	{
		flag = -1;
		rad -= ONE_PI;
	}

	return my_sin(rad)*flag;
}

float my_deathzoom(float x,float zoom)
{
	float t;
	if(x>0)
	{
		t = x - zoom;
		if(t<0)
		{
			t = 0;
		}
	}
	else
	{
		t = x + zoom;
		if(t>0)
		{
			t = 0;
		}
	}
  return (t);
}

float my_deathzoom_2(float x,float zoom)
{
	float t;
	
	if( x> -zoom && x < zoom )
	{
		t = 0;
	}
	else
	{
		t = x;
	}

  return (t);
}

float limit_mine(float x,float zoom)
{
	float t;
	
	if( x< -zoom)
	{
		t = -zoom;
	}
	else if( x>zoom)
	{
		t = zoom;
	}

  return (t);
}


float limit_mine2(float x,float min,float max)
{
	float t;
	
	if( x<min)
	{
		t = min;
	}
	else if( x>max)
	{
		t = max;
	}

  return (t);
}

float To_180_degrees(float x)
{
	return (x>180?(x-360):(x<-180?(x+360):x));
}

float my_pow_2_curve(float in,float a,float max)
{
	if( a > 1 || a < 0 )
	{
		return 0;
	}
	return( (1.0f - a) + a *ABS(in / max) * in );

}

void body_to_NEZ(float *vr, float *v, float *q) {
    float w, x, y, z;

    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];

    vr[0] = w*w*v[0] + 2.0f*y*w*v[2] - 2.0f*z*w*v[1] + x*x*v[0] + 2.0f*y*x*v[1] + 2.0f*z*x*v[2] - z*z*v[0] - y*y*v[0];
    vr[1] = 2.0f*x*y*v[0] + y*y*v[1] + 2.0f*z*y*v[2] + 2.0f*w*z*v[0] - z*z*v[1] + w*w*v[1] - 2.0f*x*w*v[2] - x*x*v[1];
    vr[2] = 2.0f*x*z*v[0] + 2.0f*y*z*v[1] + z*z*v[2] - 2.0f*w*y*v[0] - y*y*v[2] + 2.0f*w*x*v[1] - x*x*v[2] + w*w*v[2];
}


void Quaternion_FromRotationMatrix(float *R, float *Q)
{
#if 1
	// calculate the trace of the matrix
	float trace = R[0] + R[4] + R[8];
	float s;
	if(trace > 0){
		s = 0.5f * FastSqrt(trace + 1.0f);
		Q[0] = 0.25f / s;
		Q[1] = (R[7] - R[5]) * s;
		Q[2] = (R[2] - R[6]) * s;
		Q[3] = (R[3] - R[1]) * s;
	}
	else{
		if(R[0] > R[4] && R[0] > R[8] ){
			s = 0.5f * FastSqrtI(1.0f + R[0] - R[4] - R[8]);
			Q[0] = (R[7] - R[5]) * s;
			Q[1] = 0.25f / s;
			Q[2] = (R[1] + R[3]) * s;
			Q[3] = (R[2] + R[6]) * s;
		}
		else if(R[4] > R[8]) {
			s = 0.5f * FastSqrtI(1.0f + R[4] - R[0] - R[8]);
			Q[0] = (R[2] - R[6]) * s;
			Q[1] = (R[1] + R[3]) * s;
			Q[2] = 0.25f / s;
			Q[3] = (R[5] + R[7]) * s;
		}
		else{
			s = 0.5f * FastSqrtI(1.0f + R[8] - R[0] - R[4]);
			Q[0] = (R[3] - R[1]) * s;
			Q[1] = (R[2] + R[6]) * s;
			Q[2] = (R[5] + R[7]) * s;
			Q[3] = 0.25f / s;
		}
	}
#else
	// get the instantaneous orientation quaternion
	float fq0sq; // q0^2
	float recip4q0; // 1/4q0
	float fmag; // quaternion magnitude
#define SMALLQ0 0.01F // limit where rounding errors may appear
	// get q0^2 and q0
	fq0sq = 0.25f * (1.0f + R[0] + R[4] + R[8]);
	Q[0] = (float)FastSqrt(ABS(fq0sq));
	// normal case when q0 is not small meaning rotation angle not near 180 deg
	if (Q[0] > SMALLQ0){
		// calculate q1 to q3
		recip4q0 = 0.25F / Q[0];
		Q[1] = recip4q0 * (R[5] - R[7]);
		Q[2] = recip4q0 * (R[6] - R[2]);
		Q[3] = recip4q0 * (R[1] - R[3]);
	}
	else{
		// special case of near 180 deg corresponds to nearly symmetric matrix
		// which is not numerically well conditioned for division by small q0
		// instead get absolute values of q1 to q3 from leading diagonal
		Q[1] = FastSqrt(fabs(0.5f * (1.0f + R[0]) - fq0sq));
		Q[2] = FastSqrt(fabs(0.5f * (1.0f + R[4]) - fq0sq));
		Q[3] = FastSqrt(fabs(0.5f * (1.0f + R[8]) - fq0sq));
		// first assume q1 is positive and ensure q2 and q3 are consistent with q1
		if ((R[1] + R[3]) < 0.0f){
			// q1*q2 < 0 so q2 is negative
			Q[2] = -Q[2];
			if ((R[5] + R[7]) > 0.0f){
				// q1*q2 < 0 and q2*q3 > 0 so q3 is also both negative
				Q[3] = -Q[3];
			}
		}
		else if ((R[1] + R[3]) > 0.0f){
			if ((R[5] + R[7]) < 0.0f){
				// q1*q2 > 0 and q2*q3 < 0 so q3 is negative
				Q[3] = -Q[3];
			}
		}
		// negate the vector components if q1 should be negative
		if ((R[5] - R[7]) < 0.0f){
			Q[1] = -Q[1];
			Q[2] = -Q[2];
			Q[3] = -Q[3];
		}
	}
	// finally re-normalize
	fmag = FastSqrtI(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
	Q[0] *= fmag;
	Q[1] *= fmag;
	Q[2] *= fmag;
	Q[3] *= fmag;
#endif
}