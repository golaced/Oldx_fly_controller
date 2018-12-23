#ifndef _UWB_H
#define _UWB_H
#include "stm32f4xx.h"

#define TRILATERATION (1)
#define REGRESSION_NUM (10)
#define SPEED_OF_LIGHT      (299702547.0)   
#define NUM_ANCHORS (5)
#define REF_ANCHOR (5)	
#define		TRIL_3SPHERES							3
#define		TRIL_4SPHERES							4
typedef struct vec3d	vec3d;
struct vec3d {
	double	x;
	double	y;
	double	z;
};
#define   MAXZERO  0.001
#define		ERR_TRIL_CONCENTRIC						-1
#define		ERR_TRIL_COLINEAR_2SOLUTIONS			-2
#define		ERR_TRIL_SQRTNEGNUMB					-3
#define		ERR_TRIL_NOINTERSECTION_SPHERE4			-4
#define		ERR_TRIL_NEEDMORESPHERE					-5
void uwb_pos_cal(float dis[4]);
int GetLocation(vec3d *best_solution, int use4thAnchor, vec3d* anchorArray, int *distanceArray);
extern u8 uwb_good;
extern float uwb_pos[3];
#endif
