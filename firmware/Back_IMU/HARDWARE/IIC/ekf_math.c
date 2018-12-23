
#include "ekf_math.h"

// 四元数转化成一个将向量由NED坐标系旋转到机体坐标系的矩阵
void Quaternion2Tnb(const float q[4], float Tnb[3][3])
{    
		// Calculate the ned to body cosine matrix
	  float q00 = q[0] * q[0];
    float q11 = q[1] * q[1];
    float q22 = q[2] * q[2];
    float q33 = q[3] * q[3];
    float q01 = q[0] * q[1];
    float q02 = q[0] * q[2];
    float q03 = q[0] * q[3];
    float q12 = q[1] * q[2];
    float q13 = q[1] * q[3];
    float q23 = q[2] * q[3];
	
    Tnb[0][0] = q00 + q11 - q22 - q33;
		Tnb[1][1] = q00 - q11 + q22 - q33;
		Tnb[2][2] = q00 - q11 - q22 + q33;	
    Tnb[0][1] = 2 * (q12 + q03);
    Tnb[0][2] = 2 * (q13 - q02);
    Tnb[1][0] = 2 * (q12 - q03);    
    Tnb[1][2] = 2 * (q23 + q01);
    Tnb[2][0] = 2 * (q13 + q02);
    Tnb[2][1] = 2 * (q23 - q01);
    
}

//四元数转化成一个将向量由机体坐标系旋转到NED坐标系的矩阵
void Quaternion2Tbn(const float q[4], float Tbn[3][3])
{
    // Calculate the body to ned cosine matrix
    float q00 = q[0] * q[0];
    float q11 = q[1] * q[1];
    float q22 = q[2] * q[2];
    float q33 = q[3] * q[3];
    float q01 = q[0] * q[1];
    float q02 = q[0] * q[2];
    float q03 = q[0] * q[3];
    float q12 = q[1] * q[2];
    float q13 = q[1] * q[3];
    float q23 = q[2] * q[3];

    Tbn[0][0] = q00 + q11 - q22 - q33;
    Tbn[1][1] = q00 - q11 + q22 - q33;
    Tbn[2][2] = q00 - q11 - q22 + q33;
    Tbn[0][1] = 2 * (q12 - q03);
    Tbn[0][2] = 2 * (q13 + q02);
    Tbn[1][0] = 2 * (q12 + q03);
    Tbn[1][2] = 2 * (q23 - q01);
    Tbn[2][0] = 2 * (q13 - q02);
    Tbn[2][1] = 2 * (q23 + q01);
}

// 由四元数转成欧拉角
void Quaternion2Euler(const float q[4], float euler[3])
{
    float R13, R11, R12, R23, R33;
    float q0s = q[0] * q[0];
    float q1s = q[1] * q[1];
    float q2s = q[2] * q[2];
    float q3s = q[3] * q[3];

    R13    = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    R11    = q0s + q1s - q2s - q3s;
    R12    = 2.0f * (q[1] * q[2] + q[0] * q[3]);
    R23    = 2.0f * (q[2] * q[3] + q[0] * q[1]);
    R33    = q0s - q1s - q2s + q3s;

		euler[0] = RAD_DEG * atan2f(R23, R33);
    euler[1] = RAD_DEG * asinf(-R13); // pitch always between -pi/2 to pi/2
    euler[2] = RAD_DEG * atan2f(R12, R11);       
}

// 由欧拉角转成四元数
void Euler2Quaternion(const float euler[3], float q[4])
{
    float u1 = cos(0.5f * euler[0] * DEG_RAD);
    float u2 = cos(0.5f * euler[1] * DEG_RAD);
    float u3 = cos(0.5f * euler[2] * DEG_RAD);
    float u4 = sin(0.5f * euler[0] * DEG_RAD);
    float u5 = sin(0.5f * euler[1] * DEG_RAD);
    float u6 = sin(0.5f * euler[2] * DEG_RAD);
	
    q[0] = u1 * u2 * u3 + u4 * u5 * u6;
    q[1] = u4 * u2 * u3 - u1 * u5 * u6;
    q[2] = u1 * u5 * u3 + u4 * u2 * u6;
    q[3] = u1 * u2 * u6 - u4 * u5 * u3;   
}

// 将一个向量由A坐标系旋转至B坐标系
void A_Fixed2B_Fixed(float TAB[3][3], float V[3])
{
		float Vtmp[3];
		Vtmp[0] = V[0];
		Vtmp[1] = V[1];
		Vtmp[2] = V[2];
	
		V[0]  = TAB[0][0] * Vtmp[0] + TAB[0][1] * Vtmp[1] + TAB[0][2] * Vtmp[2];
    V[1]  = TAB[1][0] * Vtmp[0] + TAB[1][1] * Vtmp[1] + TAB[1][2] * Vtmp[2];
    V[2]  = TAB[2][0] * Vtmp[0] + TAB[2][1] * Vtmp[1] + TAB[2][2] * Vtmp[2];
}

// 由四元数将机体参考系的向量旋转至NED坐标系下
void Quaternion2NED(const float q[4], float V_NED[3])
{
		float Tbn[3][3];
		Quaternion2Tbn(q, Tbn);
		A_Fixed2B_Fixed(Tbn, V_NED);
}

//??NED???
void Cal_Vel_NED(double velNED[3], double gpsCourse, double gpsGndSpd, double gpsVelD)
{
    velNED[0] = gpsGndSpd*cos(gpsCourse * DEG_RAD);
    velNED[1] = gpsGndSpd*sin(gpsCourse * DEG_RAD);
    velNED[2] = gpsVelD;
}

// 参数说明: posNEDr[3] 存放转换后的NED位置坐标 
// lat卫星定位的纬度*单位是rad/s* lon卫星定位的经度*单位是rad/s*  hgt卫星定位的高度*单位是m*
// latReference 参考的纬度(比如可以是飞机起飞的那个纬度)*单位是rad/s*
// lonReference 参考的经度(比如可以是飞机起飞的那个经度)*单位是rad/s*
// hgtReference 参考高度(比如可以是飞机起飞的海拔高度)*单位是m*
void Cal_Pos_NED(double posNEDr[3], double lat, double lon, float hgt, double latReference, double lonReference, float hgtReference)
{
    posNEDr[0] = earthRadius * (lat - latReference);
    posNEDr[1] = earthRadius * cos(latReference) * (lon - lonReference);
    posNEDr[2] = -(hgt - hgtReference);
}

// 3*3 y = x^-1
void Matrix_3X3_Inv(const real_T a[9], real_T c[9])
{
  real_T x[9];
  int32_T p1;
  int32_T p2;
  int32_T p3;
  real_T absx11;
  real_T absx21;
  real_T absx31;
  int32_T itmp;
  memcpy(&x[0], &a[0], 9U * sizeof(real_T));
  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = fabs(a[0]);
  absx21 = fabs(a[1]);
  absx31 = fabs(a[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    x[0] = a[1];
    x[1] = a[0];
    absx11 = x[3];
    x[3] = x[4];
    x[4] = absx11;
    absx11 = x[6];
    x[6] = x[7];
    x[7] = absx11;
  } else {
    if (absx31 > absx11) {
      p1 = 6;
      p3 = 0;
      x[0] = a[2];
      x[2] = a[0];
      absx11 = x[3];
      x[3] = x[5];
      x[5] = absx11;
      absx11 = x[6];
      x[6] = x[8];
      x[8] = absx11;
    }
  }

  x[1] /= x[0];
  x[2] /= x[0];
  x[4] -= x[1] * x[3];
  x[5] -= x[2] * x[3];
  x[7] -= x[1] * x[6];
  x[8] -= x[2] * x[6];
  if (fabs(x[5]) > fabs(x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    absx11 = x[1];
    x[1] = x[2];
    x[2] = absx11;
    absx11 = x[4];
    x[4] = x[5];
    x[5] = absx11;
    absx11 = x[7];
    x[7] = x[8];
    x[8] = absx11;
  }

  x[5] /= x[4];
  x[8] -= x[5] * x[7];
  absx11 = (x[5] * x[1] - x[2]) / x[8];
  absx21 = -(x[1] + x[7] * absx11) / x[4];
  c[p1] = ((1.0f - x[3] * absx21) - x[6] * absx11) / x[0];
  c[p1 + 1] = absx21;
  c[p1 + 2] = absx11;
  absx11 = -x[5] / x[8];
  absx21 = (1.0f - x[7] * absx11) / x[4];
  c[p2] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  c[p2 + 1] = absx21;
  c[p2 + 2] = absx11;
  absx11 = 1.0f / x[8];
  absx21 = -x[7] * absx11 / x[4];
  c[p3] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  c[p3 + 1] = absx21;
  c[p3 + 2] = absx11;
}

// 4*4 y = x^-1
void Matrix_4X4_Inv(const real_T x[16], real_T y[16])
{
		real_T A[16];
		int32_T i0;
		int8_T ipiv[4];
		int32_T j;
		int32_T jj;
		int32_T jp1j;
		int32_T pipk;
		int32_T ix;
		real_T smax;
		int32_T jA;
		real_T s;
		int32_T i;
		int8_T p[4];
		for (i0 = 0; i0 < 16; i0++) 
		{
			y[i0] = 0.0f;
			A[i0] = x[i0];
		}

		for (i0 = 0; i0 < 4; i0++) 
		{
			ipiv[i0] = (int8_T)(1 + i0);
		}

		for (j = 0; j < 3; j++) 
		{
			jj = j * 5;
			jp1j = jj + 2;
			pipk = 1;
			ix = jj;
			smax = fabs(A[jj]);
			for (jA = 2; jA <= 4 - j; jA++) 
			{
				ix++;
				s = fabs(A[ix]);
				if (s > smax) 
				{
					pipk = jA;
					smax = s;
				}
			}

			if (A[(jj + pipk) - 1] != 0.0f) 
			{
				if (pipk - 1 != 0) 
				{
					ipiv[j] = (int8_T)(j + pipk);
					ix = j;
					pipk = (j + pipk) - 1;
					for (jA = 0; jA < 4; jA++) 
					{
						smax = A[ix];
						A[ix] = A[pipk];
						A[pipk] = smax;
						ix += 4;
						pipk += 4;
					}
				}

				i0 = (jp1j - j) + 2;
				for (i = jp1j; i <= i0; i++) 
				{
					A[i - 1] /= A[jj];
				}
			}

			jA = jj + 5;
			pipk = jj + 4;
			for (jj = 1; jj <= 3 - j; jj++) 
			{
				smax = A[pipk];
				if (A[pipk] != 0.0f) 
				{
					ix = jp1j;
					i0 = (jA - j) + 3;
					for (i = jA; i + 1 <= i0; i++) 
					{
						A[i] += A[ix - 1] * -smax;
						ix++;
					}
				}

				pipk += 4;
				jA += 4;
			}
		}

		for (i0 = 0; i0 < 4; i0++) 
		{
			p[i0] = (int8_T)(1 + i0);
		}

		for (jA = 0; jA < 3; jA++) 
		{
			if (ipiv[jA] > 1 + jA) 
			{
				pipk = p[ipiv[jA] - 1];
				p[ipiv[jA] - 1] = p[jA];
				p[jA] = (int8_T)pipk;
			}
		}

		for (jA = 0; jA < 4; jA++) 
		{
			y[jA + ((p[jA] - 1) << 2)] = 1.0f;
			for (j = jA; j + 1 < 5; j++) 
			{
				if (y[j + ((p[jA] - 1) << 2)] != 0.0f) 
				{
					for (i = j + 1; i + 1 < 5; i++) 
					{
						y[i + ((p[jA] - 1) << 2)] -= y[j + ((p[jA] - 1) << 2)] * A[i + (j << 2)];
					}
				}
			}
		}

		for (j = 0; j < 4; j++) 
		{
			pipk = j << 2;
			for (jA = 3; jA > -1; jA += -1) 
			{
				jj = jA << 2;
				if (y[jA + pipk] != 0.0f) 
				{
					y[jA + pipk] /= A[jA + jj];
					for (i = 0; i + 1 <= jA; i++) 
					{
						y[i + pipk] -= y[jA + pipk] * A[i + jj];
					}
				}
			}
		}
}

// 6*6 y = x^-1
void Matrix_6X6_Inv(const real_T x[36], real_T y[36])
{
  real_T A[36];
  int32_T i0;
  int8_T ipiv[6];
  int32_T j;
  int32_T jj;
  int32_T jp1j;
  int32_T pipk;
  int32_T ix;
  real_T smax;
  int32_T jA;
  real_T s;
  int32_T i;
  int8_T p[6];
  for (i0 = 0; i0 < 36; i0++) 
	{
    y[i0] = 0.0f;
    A[i0] = x[i0];
  }

  for (i0 = 0; i0 < 6; i0++) 
	{
    ipiv[i0] = (int8_T)(1 + i0);
  }

  for (j = 0; j < 5; j++) 
	{
    jj = j * 7;
    jp1j = jj + 2;
    pipk = 1;
    ix = jj;
    smax = fabs(A[jj]);
    for (jA = 2; jA <= 6 - j; jA++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        pipk = jA;
        smax = s;
      }
    }

    if (A[(jj + pipk) - 1] != 0.0f) {
      if (pipk - 1 != 0) {
        ipiv[j] = (int8_T)(j + pipk);
        ix = j;
        pipk = (j + pipk) - 1;
        for (jA = 0; jA < 6; jA++) {
          smax = A[ix];
          A[ix] = A[pipk];
          A[pipk] = smax;
          ix += 6;
          pipk += 6;
        }
      }

      i0 = (jp1j - j) + 4;
      for (i = jp1j; i <= i0; i++) {
        A[i - 1] /= A[jj];
      }
    }

    jA = jj + 7;
    pipk = jj + 6;
    for (jj = 1; jj <= 5 - j; jj++) {
      smax = A[pipk];
      if (A[pipk] != 0.0f) {
        ix = jp1j;
        i0 = (jA - j) + 5;
        for (i = jA; i + 1 <= i0; i++) {
          A[i] += A[ix - 1] * -smax;
          ix++;
        }
      }

      pipk += 6;
      jA += 6;
    }
  }

  for (i0 = 0; i0 < 6; i0++) {
    p[i0] = (int8_T)(1 + i0);
  }

  for (jA = 0; jA < 5; jA++) {
    if (ipiv[jA] > 1 + jA) {
      pipk = p[ipiv[jA] - 1];
      p[ipiv[jA] - 1] = p[jA];
      p[jA] = (int8_T)pipk;
    }
  }

  for (jA = 0; jA < 6; jA++) {
    y[jA + 6 * (p[jA] - 1)] = 1.0f;
    for (j = jA; j + 1 < 7; j++) {
      if (y[j + 6 * (p[jA] - 1)] != 0.0f) {
        for (i = j + 1; i + 1 < 7; i++) {
          y[i + 6 * (p[jA] - 1)] -= y[j + 6 * (p[jA] - 1)] * A[i + 6 * j];
        }
      }
    }
  }

  for (j = 0; j < 6; j++) 
	{
    pipk = 6 * j;
    for (jA = 5; jA > -1; jA += -1) 
		{
      jj = 6 * jA;
      if (y[jA + pipk] != 0.0f) 
				{
        y[jA + pipk] /= A[jA + jj];
        for (i = 0; i + 1 <= jA; i++) 
				{
          y[i + pipk] -= y[jA + pipk] * A[i + jj];
        }
      }
    }
  }
}

// 7*7 y = x^-1
void Matrix_7X7_Inv(const real_T x[49], real_T y[49])
{
  real_T A[49];
  int32_T i0;
  int8_T ipiv[7];
  int32_T j;
  int32_T jj;
  int32_T jp1j;
  int32_T pipk;
  int32_T ix;
  real_T smax;
  int32_T jA;
  real_T s;
  int32_T i;
  int8_T p[7];
  for (i0 = 0; i0 < 49; i0++) {
    y[i0] = 0.0;
    A[i0] = x[i0];
  }

  for (i0 = 0; i0 < 7; i0++) {
    ipiv[i0] = (int8_T)(1 + i0);
  }

  for (j = 0; j < 6; j++) {
    jj = j << 3;
    jp1j = jj + 2;
    pipk = 1;
    ix = jj;
    smax = fabs(A[jj]);
    for (jA = 2; jA <= 7 - j; jA++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        pipk = jA;
        smax = s;
      }
    }

    if (A[(jj + pipk) - 1] != 0.0f) {
      if (pipk - 1 != 0) {
        ipiv[j] = (int8_T)(j + pipk);
        ix = j;
        pipk = (j + pipk) - 1;
        for (jA = 0; jA < 7; jA++) {
          smax = A[ix];
          A[ix] = A[pipk];
          A[pipk] = smax;
          ix += 7;
          pipk += 7;
        }
      }

      i0 = (jp1j - j) + 5;
      for (i = jp1j; i <= i0; i++) {
        A[i - 1] /= A[jj];
      }
    }

    jA = jj + 8;
    pipk = jj + 7;
    for (jj = 1; jj <= 6 - j; jj++) {
      smax = A[pipk];
      if (A[pipk] != 0.0f) {
        ix = jp1j;
        i0 = (jA - j) + 6;
        for (i = jA; i + 1 <= i0; i++) {
          A[i] += A[ix - 1] * -smax;
          ix++;
        }
      }

      pipk += 7;
      jA += 7;
    }
  }

  for (i0 = 0; i0 < 7; i0++) {
    p[i0] = (int8_T)(1 + i0);
  }

  for (jA = 0; jA < 6; jA++) {
    if (ipiv[jA] > 1 + jA) {
      pipk = p[ipiv[jA] - 1];
      p[ipiv[jA] - 1] = p[jA];
      p[jA] = (int8_T)pipk;
    }
  }

  for (jA = 0; jA < 7; jA++) {
    y[jA + 7 * (p[jA] - 1)] = 1.0f;
    for (j = jA; j + 1 < 8; j++) {
      if (y[j + 7 * (p[jA] - 1)] != 0.0f) {
        for (i = j + 1; i + 1 < 8; i++) {
          y[i + 7 * (p[jA] - 1)] -= y[j + 7 * (p[jA] - 1)] * A[i + 7 * j];
        }
      }
    }
  }

  for (j = 0; j < 7; j++) {
    pipk = 7 * j;
    for (jA = 6; jA > -1; jA += -1) {
      jj = 7 * jA;
      if (y[jA + pipk] != 0.0f) {
        y[jA + pipk] /= A[jA + jj];
        for (i = 0; i + 1 <= jA; i++) {
          y[i + pipk] -= y[jA + pipk] * A[i + jj];
        }
      }
    }
  }
}

// 10*10 y = x^-1
void Matrix_10X10_Inv(const real_T x[100], real_T y[100])
{
		real_T A[100];
		int32_T i0;
		int8_T ipiv[10];
		int32_T j;
		int32_T c;
		int32_T pipk;
		int32_T ix;
		real_T smax;
		int32_T k;
		real_T s;
		int32_T jy;
		int32_T ijA;
		int8_T p[10];
		for (i0 = 0; i0 < 100; i0++) 
		{
				y[i0] = 0.0f;
				A[i0] = x[i0];
		}

		for (i0 = 0; i0 < 10; i0++) 
		{
				ipiv[i0] = (int8_T)(1 + i0);
		}

		for (j = 0; j < 9; j++) 
		{
			c = j * 11;
			pipk = 0;
			ix = c;
			smax = fabs(A[c]);
			for (k = 2; k <= 10 - j; k++) 
			{
					ix++;
					s = fabs(A[ix]);
					if (s > smax) 
					{
							pipk = k - 1;
							smax = s;
					}
			}

			if (A[c + pipk] != 0.0f) 
			{
					if (pipk != 0) 
					{
							ipiv[j] = (int8_T)((j + pipk) + 1);
							ix = j;
							pipk += j;
							for (k = 0; k < 10; k++) 
							{
									smax = A[ix];
									A[ix] = A[pipk];
									A[pipk] = smax;
									ix += 10;
									pipk += 10;
							}
					}

					i0 = (c - j) + 10;
					for (jy = c + 1; jy + 1 <= i0; jy++) 
					{
							A[jy] /= A[c];
					}
			}

			pipk = c;
			jy = c + 10;
			for (k = 1; k <= 9 - j; k++) 
			{
					smax = A[jy];
					if (A[jy] != 0.0f) 
					{
							ix = c + 1;
							i0 = (pipk - j) + 20;
							for (ijA = 11 + pipk; ijA + 1 <= i0; ijA++) 
							{
									A[ijA] += A[ix] * -smax;
									ix++;
							}
					}

					jy += 10;
					pipk += 10;
			}
		}

		for (i0 = 0; i0 < 10; i0++) 
		{
				p[i0] = (int8_T)(1 + i0);
		}

		for (k = 0; k < 9; k++) 
		{
				if (ipiv[k] > 1 + k) 
				{
						pipk = p[ipiv[k] - 1];
						p[ipiv[k] - 1] = p[k];
						p[k] = (int8_T)pipk;
				}
		}

		for (k = 0; k < 10; k++) 
		{
				y[k + 10 * (p[k] - 1)] = 1.0f;
				for (j = k; j + 1 < 11; j++) 
				{
						if (y[j + 10 * (p[k] - 1)] != 0.0f) 
						{
								for (jy = j + 1; jy + 1 < 11; jy++) 
								{
										y[jy + 10 * (p[k] - 1)] -= y[j + 10 * (p[k] - 1)] * A[jy + 10 * j];
								}
						}
				}
		}

		for (j = 0; j < 10; j++) 
		{
				c = 10 * j;
				for (k = 9; k > -1; k += -1) 
				{
						pipk = 10 * k;
						if (y[k + c] != 0.0f) 
						{
								y[k + c] /= A[k + pipk];
								for (jy = 0; jy + 1 <= k; jy++) 
								{
										y[jy + c] -= y[k + c] * A[jy + pipk];
								}
						}
				}
		}
}


//Mc = Ma' Ma = line * row
void Matrix_Tran(const real_T *a, real_T *c, int line, int row)
{
		int i0;
		int i1;
		for (i0 = 0; i0 < line; i0++) 
		{		  
				for (i1 = 0; i1 < row; i1++) 
				{				
						c[i1*line + i0] = 0;
				    c[i1*line + i0] = a[i0*row + i1];
				}
		}
}

// Ma * Mb = Mc ; Ma ==> al * ar ; Mb ==> bl * br 
void ML_R_X_ML_R(const real_T *a, const real_T *b, real_T *c, int al, int ar, int bl, int br)
{
		int i0;
		int i1;
		int i2;
		#if 1
		for(i2 = 0; i2 < al * br; i2++)
		c[i2] = 0;
		#endif
		for(i2 = 0; i2 < br; i2++)
		{
				for (i0 = 0; i0 < al; i0++) 
				{
						for (i1 = 0; i1 < bl; i1++) 
						{
								c[i0 * br + i2] += a[i0 * ar + i1] * b[i1 * br + i2];
						}
				}
		}
}

void swap(float *a, float *b)
{  
    float c;
    c = *a;
    *a = *b;
    *b = c;
}

int inv(float *p,int n)
{
  int *is,*js,i,j,k;
  float temp, fmax;
  is=(int *)malloc(n*sizeof(int));
  js=(int *)malloc(n*sizeof(int));
  for(k=0;k<n;k++)
	{
				fmax=0.0f;
				for(i=k;i<n;i++)
					for(j=k;j<n;j++)
					{ 
							temp=fabs(*(p+i*n+j));//????
							if(temp>fmax)
							{ 
									fmax=temp;
									is[k]=i;js[k]=j;
							}
					}
     if((fmax+1.0f)==1.0f)
		 {  
				free(is);
				free(js);				
				return 0;
		 }
    if((i=is[k])!=k)
      for(j=0;j<n;j++)
        swap((p+k*n+j),(p+i*n+j));//????
   if((j=js[k])!=k)
     for(i=0;i<n;i++)
        swap((p+i*n+k),(p+i*n+j));  //????
   p[k*n+k]=1.0f/p[k*n+k];
   for(j=0;j<n;j++)
     if(j!=k)
        p[k*n+j]*=p[k*n+k];
   for(i=0;i<n;i++)
      if(i!=k)
        for(j=0;j<n;j++)
          if(j!=k)
             p[i*n+j]=p[i*n+j]-p[i*n+k]*p[k*n+j];
   for(i=0;i<n;i++)
     if(i!=k)
       p[i*n+k]*=-p[k*n+k];
 }
 for(k=n-1;k>=0;k--)
 {
     if((j=js[k])!=k)
        for(i=0;i<n;i++)
          swap((p+j*n+i),(p+k*n+i));
     if((i=is[k])!=k)
        for(j=0;j<n;j++)
          swap((p+j*n+i),(p+j*n+k));
  }
  free(is);
  free(js);
  return 1;
}

