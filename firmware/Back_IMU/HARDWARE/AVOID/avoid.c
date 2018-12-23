#include "avoid.h"
double DIS_IN[20] = { 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
double A = 0.2*100;
double A_dead = 0.001*100;
int max_try = 200;	
float avoid_trace[3];	
	double x_mid = 0; 
	double y_mid = 0;
	double r_mid = 0;
void oldx_avoid(void)
{
	//OLD_X_AVOID(DIS_IN, A, A_dead,max_try, &x_mid, &y_mid, &r_mid);
  avoid_trace[0]=(float)x_mid/1000.;
	avoid_trace[1]=(float)y_mid/1000.;
	avoid_trace[2]=(float)r_mid/1000.;
}
