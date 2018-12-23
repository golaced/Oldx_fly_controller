
#include "stm32f4xx.h"
#include <math.h>

typedef struct  
 { u8 right;
	 float x_sumplain ;
	 float x_sumsq ;
	 float x_sumcube ;

	 float y_sumplain ;
	 float y_sumsq ;
	 float y_sumcube ;

	 float z_sumplain ;
	 float z_sumsq ;
	 float z_sumcube ;

	 float xy_sum ;
	 float xz_sum ;
	 float yz_sum ;

	 float x2y_sum ;
	 float x2z_sum ;
	 float y2x_sum ;
	 float y2z_sum ;
	 float z2x_sum ;
	 float z2y_sum ;

	 unsigned int size;
	 float Min[3];
	 float Max[3];
	 float Off[3];
	 float Gain[3];
	 float Off_Min_Max[3];
   float Off_Cycle[3];
   float Sum[3];
   float temp_max;
 }Cal_Cycle_OLDX;

extern Cal_Cycle_OLDX hml_lsq,acc_lsq;

void cycle_init_oldx(Cal_Cycle_OLDX * pLSQ);
unsigned int cycle_data_add_oldx(Cal_Cycle_OLDX * pLSQ, float x, float y, float z);
void cycle_cal_oldx(Cal_Cycle_OLDX * pLSQ, unsigned int max_iterations, float delta, float *sphere_x, float *sphere_y, float *sphere_z, float *sphere_radius);
