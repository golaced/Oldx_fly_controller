#ifndef CommonConversions_H_
#define CommonConversions_H_

#include "include.h"

typedef float real_T;
typedef double real64_T;
typedef int8_t int8_T;
typedef int32_t int32_T;
// 四元数转化成一个将向量由NED坐标系旋转到机体坐标系的矩阵
void Quaternion2Tnb(const float q[4], float Tnb[3][3]);

// 四元数转化成一个将向量由机体坐标系旋转到NED坐标系的矩阵
void Quaternion2Tbn(const float q[4], float Tbn[3][3]);

// 由四元数转成欧拉角
void Quaternion2Euler(const float q[4], float euler[3]);

// 由欧拉角转成四元数
void Euler2Quaternion(const float euler[3], float q[4]);

// 将一个向量由A坐标系旋转至B坐标系
void A_Fixed2B_Fixed(float TAB[3][3], float V[3]);

// 由四元数将机体参考系的向量旋转至NED坐标系下
void Quaternion2NED(const float q[4], float V_NED[3]);

void Cal_Vel_NED(double velNED[3], double gpsCourse, double gpsGndSpd, double gpsVelD);

//将经纬度转化成NED
void Cal_Pos_NED(double posNEDr[3], double lat, double lon, float hgt, double latReference, double lonReference, float hgtReference);

// 求3*3矩阵的逆矩阵
void Matrix_3X3_Inv(const real_T a[9], real_T c[9]);

// 求4*4矩阵的逆矩阵
void Matrix_4X4_Inv(const real_T x[16], real_T y[16]);

// 求6*6矩阵的逆矩阵
void Matrix_6X6_Inv(const real_T x[36], real_T y[36]);

// 求7*7矩阵的逆矩阵
void Matrix_7X7_Inv(const real_T x[49], real_T y[49]);

// 求10*10矩阵的逆矩阵
void Matrix_10X10_Inv(const real_T x[100], real_T y[100]);

// 待转置矩阵a  c=a' line 为 a的行数 row为a的列数
void Matrix_Tran(const real_T *a, real_T *c, int line, int row);

// 矩阵相乘 需要输入 a矩阵 和 b矩阵 a的行数 al 列数 ar b矩阵的 行数 bl 列数br (a b 矩阵都是一维数组)
void ML_R_X_ML_R(const real_T *a, const real_T *b, real_T *c, int al, int ar, int bl, int br);

int inv(float *p,int n);

#endif
