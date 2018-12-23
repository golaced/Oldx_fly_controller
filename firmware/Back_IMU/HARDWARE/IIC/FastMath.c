/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "FastMath.h"
//////////////////////////////////////////////////////////////////////////
//transmit double precision float to my own Double
 Double intToDouble(int A)
{
	Double B;

	B.hi = (float)A;
	B.lo = 0.0f;
	
	return B;
}
//
 Double floatToDouble(float A)
{
	Double B;

	B.hi = A;
	B.lo = 0.0f;
	
	return B;
}
//
 Double doubleToDouble(double A)
{
	Double B;

	B.hi = (float)A;
	B.lo = (float)(A - (double)B.hi);
	
	return B;
}
//transmit my own Double to double precision float
 double DoubleTodouble(Double B)
{
	double A;

	A = B.hi;
	A += B.lo;

	return A;
}

//addition: Double + Double
 Double DoubleAdd(Double A, Double B)
{
	Double C;
	float t1, t2, e;

	//Compute high order sum and error
	t1 = A.hi + B.hi;
	e = t1 - A.hi;
	//Compute low order term, including error and overflows
	t2 = ((B.hi - e) + ( A.hi - (t1 - e))) + A.lo + B.lo;
	//Normalise to get final result
	C.hi = t1 + t2;
	C.lo = t2 -(C.hi - t1);

	return C;
}

//Subtraction: Double - Double
 Double DoubleSub(Double A, Double B)
{
	Double C;
	float t1, t2, e;

	//Compute high order sub and error
	t1 = A.hi - B.hi;
	e = t1 - A.hi;
	//Compute low order term, including error and overflows
	t2 = ((-B.hi - e) + ( A.hi - (t1 - e))) + A.lo - B.lo;
	//Normalise to get final result
	C.hi = t1 + t2;
	C.lo = t2 -(C.hi - t1);

	return C;
}

//multiplication: Double * Double
 Double DoubleMul(Double A, Double B)
{
	Double C;
	float cona, conb, a1, a2, b1, b2;
	float c11, c21, c2, e, t1, t2;

	//Compute initial high order approximation and error
	//If a fused multiply-add is available
	//c11 = A.hi * B.hi;
	//c21 = A.hi * B.hi - c11;

	//If no fused multiply-add is available
	cona = A.hi * 8193.0f;
	conb = B.hi * 8193.0f;
	a1 = cona - (cona - A.hi);
	b1 = conb - (conb - B.hi);
	a2 = A.hi - a1;
	b2 = B.hi - b1;
	c11 = A.hi * B.hi;
	c21 = (((a1 * b1 - c11) + a1 * b2) + a2 * b1) + a2 * b2;

	//Compute high order word of mixed term:
	c2 = A.hi * B.lo + A.lo * B.hi;
	//Compute (c11, c21) + c2 using Knuth's trick, including low order product
	t1 = c11 + c2;
	e = t1 - c11;
	t2 = ((c2-e) + (c11 - (t1 -e))) + c21 + A.lo * B.lo;
	//Normalise to get final result
	C.hi = t1 + t2;
	C.lo = t2 - ( C.hi - t1);
	return C;
}

//divides: Double / Double
 Double DoubleDiv(Double A, Double B)
{
	Double C;
	float a1, a2, b1, b2, cona, conb, c11, c2, c21, e, s1, s2;
	float t1, t2, t11, t12, t21, t22;

	// Compute a DP approximation to the quotient.
	s2 = 1.0f / B.hi;
	s1 = A.hi * s2;

	//This splits s1 and b.x into high-order and low-order words.
	cona = s1 * 8193.0f;
	conb = B.hi * 8193.0f;
	a1 = cona - (cona - s1);
	b1 = conb - (conb - B.hi);
	a2 = s1 -a1;
	b2 = B.hi - b1;
	//Multiply s1 * dsb(1) using Dekker's method.
	c11 = s1 * B.hi;
	c21 = (((a1 * b1 - c11) + a1 * b2) + a2 * b1) + a2 * b2;
	//Compute s1 * b.lo (only high-order word is needed).
	c2 = s1 * B.lo;
	//Compute (c11, c21) + c2 using Knuth's trick.
	t1 = c11 + c2;
	e = t1 - c11;
	t2 = ((c2 - e) + (c11 - (t1 - e))) + c21;
	//The result is t1 + t2, after normalization.
	t12 = t1 + t2;
	t22 = t2 - (t12 - t1);
	//Compute dsa - (t12, t22) using Knuth's trick.
	t11 = A.hi - t12;
	e = t11 - A.hi;
	t21 = ((-t12 - e) + (A.hi - (t11 - e))) + A.lo - t22;
	//Compute high-order word of (t11, t21) and divide by b.hi.
	s2 *= (t11 + t21);
	//The result is s1 + s2, after normalization.
	C.hi = s1 + s2;
	C.lo = s2 - (C.hi - s1);

	return C;
}
//translate from ADI's dsp library.
//////////////////////////////////////////////////////////////////////////
//Get fraction and integer parts of floating point
float Modf(float x, float *i)
{
	float y;
	float fract;

	y = x;
	if (x < (float)0.0){
		y = -y;
	}

	if (y >= (float)16777216.0f){
		*i = x;
		return (float)0.0f;
	}

	if (y < (float)1.0f){
		*i = (float)0.0f;
		return x;
	}

	y = (float)((long)(y));

	if (x < (float)0.0f){
		y = -y;
	}

	fract = x - y;
	*i = y;

	return fract;
}

float FastPow(float x,float y)
{
	float tmp;
	float znum, zden, result;
	float g, r, u1, u2, v, z;
	long m, p, negate, y_int, n;
	long mi, pi, iw1;
	float y1, y2, w1, w2, w;
	float *a1, *a2;
	float xtmp;
	long *lPtr = (long *)&xtmp;
	float *fPtr = &xtmp;
	static const long a1l[] =  {0,            /* 0.0 */
		0x3f800000,   /* 1.0 */
		0x3f75257d,   /* 0.9576032757759 */
		0x3f6ac0c6,   /* 0.9170039892197 */
		0x3f60ccde,   /* 0.8781260251999 */
		0x3f5744fc,   /* 0.8408963680267 */
		0x3f4e248c,   /* 0.8052451610565 */
		0x3f45672a,   /* 0.7711054086685 */
		0x3f3d08a3,   /* 0.7384130358696 */
		0x3f3504f3,   /* 0.7071067690849 */
		0x3f2d583e,   /* 0.6771277189255 */
		0x3f25fed6,   /* 0.6484197378159 */
		0x3f1ef532,   /* 0.6209288835526 */
		0x3f1837f0,   /* 0.5946035385132 */
		0x3f11c3d3,   /* 0.5693942904472 */
		0x3f0b95c1,   /* 0.5452538132668 */
		0x3f05aac3,   /* 0.5221368670464 */
		0x3f000000};  /* 0.5 */
	static const long a2l[] =  {0,            /* 0.0 */
		0x31a92436,   /* 4.922664054163e-9 */
		0x336c2a94,   /* 5.498675648141e-8 */
		0x31a8fc24,   /* 4.918108587049e-9 */
		0x331f580c,   /* 3.710015050729e-8 */
		0x336a42a1,   /* 5.454296925222e-8 */
		0x32c12342,   /* 2.248419050943e-8 */
		0x32e75623,   /* 2.693110978669e-8 */
		0x32cf9890};  /* 2.41673490109e-8 */

	a1 = (float *)a1l;
	a2 = (float *)a2l; 
	negate = 0;

	if (x == (float)0.0){
		if (y == (float)0.0){
			return (float)1.0;
		}
		else if (y > (float)0.0){
			return (float)0.0;
		}
		else{
			return (float)FLT_MAX;
		}
	}
	else if (x < (float)0.0){
		y_int = (long)(y);
		if ((float)(y_int) != y){
			return (float)0.0;
		}

		x = -x;
		negate = y_int & 0x1;
	}

	xtmp = x;
	m = (*lPtr >> 23);
	m = m - 126;

	*lPtr = (*lPtr & 0x807fffff) | (126 << 23);
	g = *fPtr;
	
	p = 1;
	if (g <= a1[9]){
		p = 9;
	}
	if (g <= a1[p + 4]){
		p = p + 4;
	}
	if (g <= a1[p + 2]){
		p = p + 2;
	}
	
	p = p + 1;
	znum = g - a1[p];
	znum = znum - a2[p >> 1];

	zden = g + a1[p];

	p = p - 1;

	z = znum / zden;
	z = z + z;

	v = z * z;

	r = POWP_COEF2 * v;
	r = r + POWP_COEF1;
	r = r * v;
	r = r * z;

	r = r + LOG2E_MINUS1 * r;
	u2 = LOG2E_MINUS1 * z;
	u2 = r + u2;
	u2 = u2 + z;

	u1 = (float)((m * 16) - p);
	u1 = u1 * 0.0625f;

	Modf(16.0f * y, &(y1));
	y1 = y1 * 0.0625f;

	y2 = y - y1;

	w = u1 * y2;
	tmp = u2 * y;
	w = tmp + w;

	Modf(16.0f * w, &(w1));
	w1 = w1 * 0.0625f;

	w2 = w - w1;

	w = u1 * y1;
	w = w + w1;

	Modf(16.0f * w, &(w1));
	w1 = w1 * 0.0625f;

	tmp = w - w1;
	w2 = w2 + tmp;

	Modf(16.0f * w2, &(w));
	w = w * 0.0625f;

	tmp = w1 + w;
	tmp = 16.0f * tmp;
	iw1 = (long)(tmp);

	w2 = w2 - w;

	if (iw1 > POW_BIGNUM){
		result = (float)FLT_MAX;
		if (negate == 1){
			result = -result;
		}
		return result;
	}

	if (w2 > 0){
		w2 = w2 - 0.0625f;
		iw1++;
	}

	if (iw1 < POW_SMALLNUM){
		return (float)0.0;
	}

	if (iw1 < 0){
		mi = 0;
	}
	else{
		mi = 1;
	}
	n = iw1 / 16;
	mi = mi + n;
	pi = (mi * 16) - iw1;

	z = POWQ_COEF5 * w2;
	z = z + POWQ_COEF4;
	z = z * w2;
	z = z + POWQ_COEF3;
	z = z * w2;
	z = z + POWQ_COEF2;
	z = z * w2;
	z = z + POWQ_COEF1;
	z = z * w2;

	z = z * a1[pi + 1];
	z = a1[pi + 1] + z;

	fPtr = &z;
	lPtr = (long *)fPtr;
	n = (*lPtr >> 23) & 0xff;
	n = n - 127;
	mi = mi + n;
	mi = mi + 127;

	mi = mi & 0xff;
	*lPtr = *lPtr & (0x807fffff);
	*lPtr = *lPtr | mi << 23;

	result = *fPtr;

	if (negate){
		result = -result;
	}

	return result;
}
//
float FastTan(float x)
{
    long n;
    float xn;
    float f, g;
    float x_int, x_fract;
    float result;
    float xnum, xden;

    if ((x > (float)X_MAX) || (x < (float)-X_MAX)){
        return (float)0.0;
    }

    x_int = (float)((long)(x));
    x_fract = x - x_int;

    g = (float)0.5;
    if (x <= (float)0.0){
        g = -g;
    }
    n = (long)(x * (float)INV_PI_2 + g);
    xn = (float)(n);

    f = x_int - xn * PI_2_C1;
    f = f + x_fract;
    f = f - xn * PI_2_C2;
    f = f - xn * PI_2_C3;

    if (f < (float)0.0){
        g = -f;
    }
    else{
        g = f;
    }
    if (g < (float)EPS_FLOAT){
        if (n & 0x0001){
            result = -1.0f / f;
        }
        else{
            result = f;
        }            
        return result;
    }

    g = f * f;
    xnum = g * TANP_COEF2;
    xnum = xnum + TANP_COEF1;
    xnum = xnum * g;
    xnum = xnum * f;
    xnum = xnum + f;

    xden = g * TANQ_COEF2;
    xden = xden + TANQ_COEF1;
    xden = xden * g;
    xden = xden + TANQ_COEF0;

    if (n & 0x0001){
        result = xnum;
        xnum = -xden;
        xden = result;
    }
    result = xnum / xden;
    return result;
}
//
float FastLn(float x)
{
	union { unsigned int i; float f;} e;
	float xn;
	float	z;
	float	w;
	float	a;
	float	b;
	float	r;
	float	result;
	float znum, zden;

	int exponent = (*((int*)&x) & 0x7F800000) >> 23;
	e.i = (*((int*)&x) & 0x3F800000);

	if(e.f > ROOT_HALF){
		znum = e.f - 1.0f;
		zden = e.f * 0.5f + 0.5f;
	}
	else{
		exponent -= 1;
		znum = e.f - 0.5f;
		zden = e.f * 0.5f + 0.5f;
	}
	xn = (float)exponent;
	z = znum / zden;
	w = z * z;
	a = (LOGDA_COEF2 * w + LOGDA_COEF1) * w + LOGDA_COEF0;
	b = ((w + LOGDB_COEF2) * w + LOGDB_COEF1) * w + LOGDB_COEF0;
	r = a / b * w * z + z;
	result = xn * LN2_DC1 + r;
	r = xn * LN2_DC2;
	result += r;
	r = xn * LN2_DC3;
	result += r;
	return result;
}

float FastAsin(float x)
{
	float y, g;
	float num, den, result;
	long i;
	float sign = 1.0;

	y = x;
	if (y < (float)0.0){
		y = -y;
		sign = -sign;
	}

	if (y > (float)0.5){
		i = 1;
		if (y > (float)1.0){
			result = 0.0;
			return result;
		}    
		g = (1.0f - y) * 0.5f;
		y = -2.0f * FastSqrt(g);
	}
	else{
		i = 0;
		if (y < (float)EPS_FLOAT){
			result = y;
			if (sign < (float)0.0){
				result = -result;
			}
			return result;
		}
		g = y * y;
	}
	num = ((ASINP_COEF3 * g + ASINP_COEF2) * g + ASINP_COEF1) * g;
	den = ((g + ASINQ_COEF2) * g + ASINQ_COEF1) * g + ASINQ_COEF0;
	result = num / den;
	result = result * y + y;
	if (i == 1){
		result = result + (float)PI_2;
	}
	if (sign < (float)0.0){
		result = -result;
	}
	return result;
}

float FastAtan2(float y, float x)
{
	float f, g;
	float num, den;
	float result;
	int n;

	static const float a[4] = {0, (float)PI_6, (float)PI_2, (float)PI_3};

	if (x == (float)0.0){
		if (y == (float)0.0){
			result = 0.0;
			return result;
		}

		result = (float)PI_2;
		if (y > (float)0.0){
			return result;
		}
		if (y < (float)0.0){
			result = -result;
			return result;
		}
	}
	n = 0;
	num = y;
	den = x;

	if (num < (float)0.0){
		num = -num;
	}
	if (den < (float)0.0){
		den = -den;
	}
	if (num > den){
		f = den;
		den = num;
		num = f;
		n = 2;
	}
	f = num / den;

	if (f > (float)TWO_MINUS_ROOT3){
		num = f * (float)SQRT3_MINUS_1 - 1.0f + f;
		den = (float)SQRT3 + f;
		f = num / den;
		n = n + 1;
	}

	g = f;
	if (g < (float)0.0){
		g = -g;
	}

	if (g < (float)EPS_FLOAT){
		result = f;
	}
	else{
		g = f * f;
		num = (ATANP_COEF1 * g + ATANP_COEF0) * g;
		den = (g + ATANQ_COEF1) * g + ATANQ_COEF0;
		result = num / den;
		result = result * f + f;
	}
	if (n > 1){
		result = -result;
	}
	result = result + a[n];

	if (x < (float)0.0){
		result = PI - result;
	}
	if (y < (float)0.0){
		result = -result;
	}
	return result;
}

// Quake inverse square root
float FastSqrtI(float x)
{
	//////////////////////////////////////////////////////////////////////////
	//less accuracy, more faster
	/*
	L2F l2f;
	float xhalf = 0.5f * x;
	l2f.f = x;

	l2f.i = 0x5f3759df - (l2f.i >> 1);
	x = l2f.f * (1.5f - xhalf * l2f.f * l2f.f);
	return x;
	*/
	//////////////////////////////////////////////////////////////////////////
	union { unsigned int i; float f;} l2f;
	l2f.f = x;
	l2f.i = 0x5F1F1412 - (l2f.i >> 1);
	return l2f.f * (1.69000231f - 0.714158168f * x * l2f.f * l2f.f);
}

float FastSqrt(float x)
{
	return x * FastSqrtI(x);
}

#define FAST_SIN_TABLE_SIZE 512

const float sinTable[FAST_SIN_TABLE_SIZE + 1] = {
	0.00000000f, 0.01227154f, 0.02454123f, 0.03680722f, 0.04906767f, 0.06132074f,
	0.07356456f, 0.08579731f, 0.09801714f, 0.11022221f, 0.12241068f, 0.13458071f,
	0.14673047f, 0.15885814f, 0.17096189f, 0.18303989f, 0.19509032f, 0.20711138f,
	0.21910124f, 0.23105811f, 0.24298018f, 0.25486566f, 0.26671276f, 0.27851969f,
	0.29028468f, 0.30200595f, 0.31368174f, 0.32531029f, 0.33688985f, 0.34841868f,
	0.35989504f, 0.37131719f, 0.38268343f, 0.39399204f, 0.40524131f, 0.41642956f,
	0.42755509f, 0.43861624f, 0.44961133f, 0.46053871f, 0.47139674f, 0.48218377f,
	0.49289819f, 0.50353838f, 0.51410274f, 0.52458968f, 0.53499762f, 0.54532499f,
	0.55557023f, 0.56573181f, 0.57580819f, 0.58579786f, 0.59569930f, 0.60551104f,
	0.61523159f, 0.62485949f, 0.63439328f, 0.64383154f, 0.65317284f, 0.66241578f,
	0.67155895f, 0.68060100f, 0.68954054f, 0.69837625f, 0.70710678f, 0.71573083f,
	0.72424708f, 0.73265427f, 0.74095113f, 0.74913639f, 0.75720885f, 0.76516727f,
	0.77301045f, 0.78073723f, 0.78834643f, 0.79583690f, 0.80320753f, 0.81045720f,
	0.81758481f, 0.82458930f, 0.83146961f, 0.83822471f, 0.84485357f, 0.85135519f,
	0.85772861f, 0.86397286f, 0.87008699f, 0.87607009f, 0.88192126f, 0.88763962f,
	0.89322430f, 0.89867447f, 0.90398929f, 0.90916798f, 0.91420976f, 0.91911385f,
	0.92387953f, 0.92850608f, 0.93299280f, 0.93733901f, 0.94154407f, 0.94560733f,
	0.94952818f, 0.95330604f, 0.95694034f, 0.96043052f, 0.96377607f, 0.96697647f,
	0.97003125f, 0.97293995f, 0.97570213f, 0.97831737f, 0.98078528f, 0.98310549f,
	0.98527764f, 0.98730142f, 0.98917651f, 0.99090264f, 0.99247953f, 0.99390697f,
	0.99518473f, 0.99631261f, 0.99729046f, 0.99811811f, 0.99879546f, 0.99932238f,
	0.99969882f, 0.99992470f, 1.00000000f, 0.99992470f, 0.99969882f, 0.99932238f,
	0.99879546f, 0.99811811f, 0.99729046f, 0.99631261f, 0.99518473f, 0.99390697f,
	0.99247953f, 0.99090264f, 0.98917651f, 0.98730142f, 0.98527764f, 0.98310549f,
	0.98078528f, 0.97831737f, 0.97570213f, 0.97293995f, 0.97003125f, 0.96697647f,
	0.96377607f, 0.96043052f, 0.95694034f, 0.95330604f, 0.94952818f, 0.94560733f,
	0.94154407f, 0.93733901f, 0.93299280f, 0.92850608f, 0.92387953f, 0.91911385f,
	0.91420976f, 0.90916798f, 0.90398929f, 0.89867447f, 0.89322430f, 0.88763962f,
	0.88192126f, 0.87607009f, 0.87008699f, 0.86397286f, 0.85772861f, 0.85135519f,
	0.84485357f, 0.83822471f, 0.83146961f, 0.82458930f, 0.81758481f, 0.81045720f,
	0.80320753f, 0.79583690f, 0.78834643f, 0.78073723f, 0.77301045f, 0.76516727f,
	0.75720885f, 0.74913639f, 0.74095113f, 0.73265427f, 0.72424708f, 0.71573083f,
	0.70710678f, 0.69837625f, 0.68954054f, 0.68060100f, 0.67155895f, 0.66241578f,
	0.65317284f, 0.64383154f, 0.63439328f, 0.62485949f, 0.61523159f, 0.60551104f,
	0.59569930f, 0.58579786f, 0.57580819f, 0.56573181f, 0.55557023f, 0.54532499f,
	0.53499762f, 0.52458968f, 0.51410274f, 0.50353838f, 0.49289819f, 0.48218377f,
	0.47139674f, 0.46053871f, 0.44961133f, 0.43861624f, 0.42755509f, 0.41642956f,
	0.40524131f, 0.39399204f, 0.38268343f, 0.37131719f, 0.35989504f, 0.34841868f,
	0.33688985f, 0.32531029f, 0.31368174f, 0.30200595f, 0.29028468f, 0.27851969f,
	0.26671276f, 0.25486566f, 0.24298018f, 0.23105811f, 0.21910124f, 0.20711138f,
	0.19509032f, 0.18303989f, 0.17096189f, 0.15885814f, 0.14673047f, 0.13458071f,
	0.12241068f, 0.11022221f, 0.09801714f, 0.08579731f, 0.07356456f, 0.06132074f,
	0.04906767f, 0.03680722f, 0.02454123f, 0.01227154f, 0.00000000f, -0.01227154f,
	-0.02454123f, -0.03680722f, -0.04906767f, -0.06132074f, -0.07356456f,
	-0.08579731f, -0.09801714f, -0.11022221f, -0.12241068f, -0.13458071f,
	-0.14673047f, -0.15885814f, -0.17096189f, -0.18303989f, -0.19509032f, 
	-0.20711138f, -0.21910124f, -0.23105811f, -0.24298018f, -0.25486566f, 
	-0.26671276f, -0.27851969f, -0.29028468f, -0.30200595f, -0.31368174f, 
	-0.32531029f, -0.33688985f, -0.34841868f, -0.35989504f, -0.37131719f, 
	-0.38268343f, -0.39399204f, -0.40524131f, -0.41642956f, -0.42755509f, 
	-0.43861624f, -0.44961133f, -0.46053871f, -0.47139674f, -0.48218377f, 
	-0.49289819f, -0.50353838f, -0.51410274f, -0.52458968f, -0.53499762f, 
	-0.54532499f, -0.55557023f, -0.56573181f, -0.57580819f, -0.58579786f, 
	-0.59569930f, -0.60551104f, -0.61523159f, -0.62485949f, -0.63439328f, 
	-0.64383154f, -0.65317284f, -0.66241578f, -0.67155895f, -0.68060100f, 
	-0.68954054f, -0.69837625f, -0.70710678f, -0.71573083f, -0.72424708f, 
	-0.73265427f, -0.74095113f, -0.74913639f, -0.75720885f, -0.76516727f, 
	-0.77301045f, -0.78073723f, -0.78834643f, -0.79583690f, -0.80320753f, 
	-0.81045720f, -0.81758481f, -0.82458930f, -0.83146961f, -0.83822471f, 
	-0.84485357f, -0.85135519f, -0.85772861f, -0.86397286f, -0.87008699f, 
	-0.87607009f, -0.88192126f, -0.88763962f, -0.89322430f, -0.89867447f, 
	-0.90398929f, -0.90916798f, -0.91420976f, -0.91911385f, -0.92387953f, 
	-0.92850608f, -0.93299280f, -0.93733901f, -0.94154407f, -0.94560733f, 
	-0.94952818f, -0.95330604f, -0.95694034f, -0.96043052f, -0.96377607f, 
	-0.96697647f, -0.97003125f, -0.97293995f, -0.97570213f, -0.97831737f, 
	-0.98078528f, -0.98310549f, -0.98527764f, -0.98730142f, -0.98917651f, 
	-0.99090264f, -0.99247953f, -0.99390697f, -0.99518473f, -0.99631261f, 
	-0.99729046f, -0.99811811f, -0.99879546f, -0.99932238f, -0.99969882f, 
	-0.99992470f, -1.00000000f, -0.99992470f, -0.99969882f, -0.99932238f, 
	-0.99879546f, -0.99811811f, -0.99729046f, -0.99631261f, -0.99518473f, 
	-0.99390697f, -0.99247953f, -0.99090264f, -0.98917651f, -0.98730142f, 
	-0.98527764f, -0.98310549f, -0.98078528f, -0.97831737f, -0.97570213f, 
	-0.97293995f, -0.97003125f, -0.96697647f, -0.96377607f, -0.96043052f, 
	-0.95694034f, -0.95330604f, -0.94952818f, -0.94560733f, -0.94154407f, 
	-0.93733901f, -0.93299280f, -0.92850608f, -0.92387953f, -0.91911385f, 
	-0.91420976f, -0.90916798f, -0.90398929f, -0.89867447f, -0.89322430f, 
	-0.88763962f, -0.88192126f, -0.87607009f, -0.87008699f, -0.86397286f, 
	-0.85772861f, -0.85135519f, -0.84485357f, -0.83822471f, -0.83146961f, 
	-0.82458930f, -0.81758481f, -0.81045720f, -0.80320753f, -0.79583690f, 
	-0.78834643f, -0.78073723f, -0.77301045f, -0.76516727f, -0.75720885f, 
	-0.74913639f, -0.74095113f, -0.73265427f, -0.72424708f, -0.71573083f, 
	-0.70710678f, -0.69837625f, -0.68954054f, -0.68060100f, -0.67155895f, 
	-0.66241578f, -0.65317284f, -0.64383154f, -0.63439328f, -0.62485949f, 
	-0.61523159f, -0.60551104f, -0.59569930f, -0.58579786f, -0.57580819f, 
	-0.56573181f, -0.55557023f, -0.54532499f, -0.53499762f, -0.52458968f, 
	-0.51410274f, -0.50353838f, -0.49289819f, -0.48218377f, -0.47139674f, 
	-0.46053871f, -0.44961133f, -0.43861624f, -0.42755509f, -0.41642956f, 
	-0.40524131f, -0.39399204f, -0.38268343f, -0.37131719f, -0.35989504f, 
	-0.34841868f, -0.33688985f, -0.32531029f, -0.31368174f, -0.30200595f, 
	-0.29028468f, -0.27851969f, -0.26671276f, -0.25486566f, -0.24298018f, 
	-0.23105811f, -0.21910124f, -0.20711138f, -0.19509032f, -0.18303989f, 
	-0.17096189f, -0.15885814f, -0.14673047f, -0.13458071f, -0.12241068f, 
	-0.11022221f, -0.09801714f, -0.08579731f, -0.07356456f, -0.06132074f, 
	-0.04906767f, -0.03680722f, -0.02454123f, -0.01227154f, -0.00000000f
};

void FastSinCos(float x, float *sinVal, float *cosVal)
{
	float fract, in; // Temporary variables for input, output
	unsigned short indexS, indexC; // Index variable
	float f1, f2, d1, d2; // Two nearest output values
	int n;
	float findex, Dn, Df, temp;

	// input x is in radians
	//Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi, for cosine add 0.25 (pi/2) to read sine table
	in = x * 0.159154943092f;

	// Calculation of floor value of input
	n = (int) in;

	// Make negative values towards -infinity
	if(in < 0.0f){
		n--;
	}
	// Map input value to [0 1]
	in = in - (float) n;

	// Calculation of index of the table
	findex = (float) FAST_SIN_TABLE_SIZE * in;
	indexS = ((unsigned short)findex) & 0x1ff;
	indexC = (indexS + (FAST_SIN_TABLE_SIZE / 4)) & 0x1ff;

	// fractional value calculation
	fract = findex - (float) indexS;

	// Read two nearest values of input value from the cos & sin tables
	f1 = sinTable[indexC+0];
	f2 = sinTable[indexC+1];
	d1 = -sinTable[indexS+0];
	d2 = -sinTable[indexS+1];

	Dn = 0.0122718463030f; // delta between the two points (fixed), in this case 2*pi/FAST_SIN_TABLE_SIZE
	Df = f2 - f1; // delta between the values of the functions
	temp = Dn*(d1 + d2) - 2*Df;
	temp = fract*temp + (3*Df - (d2 + 2*d1)*Dn);
	temp = fract*temp + d1*Dn;

	// Calculation of cosine value
	*cosVal = fract*temp + f1;

	// Read two nearest values of input value from the cos & sin tables
	f1 = sinTable[indexS+0];
	f2 = sinTable[indexS+1];
	d1 = sinTable[indexC+0];
	d2 = sinTable[indexC+1];

	Df = f2 - f1; // delta between the values of the functions
	temp = Dn*(d1 + d2) - 2*Df;
	temp = fract*temp + (3*Df - (d2 + 2*d1)*Dn);
	temp = fract*temp + d1*Dn;

	// Calculation of sine value
	*sinVal = fract*temp + f1;
}

float FastSin(float x)
{
	float sinVal, fract, in; // Temporary variables for input, output
	unsigned short index; // Index variable
	float a, b; // Two nearest output values
	int n;
	float findex;

	// input x is in radians
	// Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi
	in = x * 0.159154943092f;

	// Calculation of floor value of input
	n = (int) in;

	// Make negative values towards -infinity
	if(x < 0.0f){
		n--;
	}

	// Map input value to [0 1]
	in = in - (float) n;

	// Calculation of index of the table
	findex = (float) FAST_SIN_TABLE_SIZE * in;
	index = ((unsigned short)findex) & 0x1ff;

	// fractional value calculation
	fract = findex - (float) index;

	// Read two nearest values of input value from the sin table
	a = sinTable[index];
	b = sinTable[index+1];

	// Linear interpolation process
	sinVal = (1.0f-fract)*a + fract*b;

	// Return the output value
	return (sinVal);
}

float FastCos(float x)
{
	float cosVal, fract, in; // Temporary variables for input, output
	unsigned short index; // Index variable
	float a, b; // Two nearest output values
	int n;
	float findex;

	// input x is in radians
	// Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi, add 0.25 (pi/2) to read sine table
	in = x * 0.159154943092f + 0.25f;

	// Calculation of floor value of input
	n = (int) in;

	// Make negative values towards -infinity
	if(in < 0.0f){
		n--;
	}

	// Map input value to [0 1]
	in = in - (float) n;

	// Calculation of index of the table
	findex = (float) FAST_SIN_TABLE_SIZE * in;
	index = ((unsigned short)findex) & 0x1ff;

	// fractional value calculation
	fract = findex - (float) index;

	// Read two nearest values of input value from the cos table
	a = sinTable[index];
	b = sinTable[index+1];

	// Linear interpolation process
	cosVal = (1.0f-fract)*a + fract*b;

	// Return the output value
	return (cosVal);
}

Double FastSqrtID(Double dx)
{
	Double dy;
	Double dhalfx = DoubleMul(doubleToDouble(0.5), dx);
	union { double d; unsigned __int64 i; } u;
	//
	u.d = DoubleTodouble(dx);
	u.i = 0x5fe6ec85e7de30daLL - (u.i >> 1);

	dy = doubleToDouble(u.d);
	dy = DoubleMul(dy, DoubleSub(doubleToDouble(1.5), DoubleMul(DoubleMul(dhalfx, dy), dy)));
	//return DoubleDiv(doubleToDouble(1.0), dx);
	return dy;
}

Double FastSqrtD(Double dx)
{
	Double dy;
	Double dhalfx = DoubleMul(doubleToDouble(0.5), dx);
	union { double d; unsigned __int64 i; } u;
	//
	u.d = DoubleTodouble(dx);
	u.i = 0x5fe6ec85e7de30daLL - (u.i >> 1);

	dy = doubleToDouble(u.d);
	dy = DoubleMul(dy, DoubleSub(doubleToDouble(1.5), DoubleMul(DoubleMul(dhalfx, dy), dy)));
	return DoubleDiv(doubleToDouble(1.0), dx);
}
