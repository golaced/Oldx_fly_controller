#include "cycle_cal_oldx.h"

Cal_Cycle_OLDX hml_lsq,hml_lsq1;
void cycle_init_oldx(Cal_Cycle_OLDX * pLSQ)
{
   pLSQ-> x_sumplain =0;
	 pLSQ-> x_sumsq =0;
	 pLSQ-> x_sumcube =0;
	 pLSQ-> y_sumplain =0;
	 pLSQ-> y_sumsq =0;
	 pLSQ-> y_sumcube =0;
	 pLSQ-> z_sumplain =0;
	 pLSQ-> z_sumsq =0;
	 pLSQ-> z_sumcube =0;
	 pLSQ-> xy_sum =0;
	 pLSQ-> xz_sum =0;
	 pLSQ-> yz_sum =0;
	 pLSQ-> x2y_sum =0;
	 pLSQ-> x2z_sum =0;
	 pLSQ-> y2x_sum =0;
	 pLSQ-> y2z_sum =0;
	 pLSQ-> z2x_sum =0;
	 pLSQ-> z2y_sum =0;
	 pLSQ->size=0;
	 pLSQ->right=0;	
	 pLSQ->Min[0]= pLSQ->Min[1]= pLSQ->Min[2]=-0.1;
	 pLSQ->Max[0]= pLSQ->Max[1]= pLSQ->Max[2]=0.1;
}

#define MIN_C_OLDX(a, b) ((a) < (b) ? (a) : (b))
#define MAX_C_OLDX(a, b) ((a) > (b) ? (a) : (b))
unsigned int cycle_data_add_oldx(Cal_Cycle_OLDX * pLSQ, float x, float y, float z)
{
float x2 ;//= x * x;
float y2 ;//= y * y;
float z2 ;//= z * z;
 x2 = x * x;
 y2 = y * y;
 z2 = z * z;
pLSQ->Min[0] = MIN_C_OLDX(x, pLSQ->Min[0]);
pLSQ->Min[1] = MIN_C_OLDX(y, pLSQ->Min[1]);
pLSQ->Min[2] = MIN_C_OLDX(z, pLSQ->Min[2]);

pLSQ->Max[0] = MAX_C_OLDX(x, pLSQ->Max[0]);
pLSQ->Max[1] = MAX_C_OLDX(y, pLSQ->Max[1]);
pLSQ->Max[2] = MAX_C_OLDX(z, pLSQ->Max[2]);
	
pLSQ->x_sumplain += x;
pLSQ->x_sumsq += x2;
pLSQ->x_sumcube += x2 * x;

pLSQ->y_sumplain += y;
pLSQ->y_sumsq += y2;
pLSQ->y_sumcube += y2 * y;

pLSQ->z_sumplain += z;
pLSQ->z_sumsq += z2;
pLSQ->z_sumcube += z2 * z;

pLSQ->xy_sum += x * y;
pLSQ->xz_sum += x * z;
pLSQ->yz_sum += y * z;

pLSQ->x2y_sum += x2 * y;
pLSQ->x2z_sum += x2 * z;

pLSQ->y2x_sum += y2 * x;
pLSQ->y2z_sum += y2 * z;

pLSQ->z2x_sum += z2 * x;
pLSQ->z2y_sum += z2 * y;

pLSQ->size++;

return pLSQ->size;
}


void cycle_cal_oldx(Cal_Cycle_OLDX * pLSQ, unsigned int max_iterations, float delta, float *sphere_x, float *sphere_y, float *sphere_z, float *sphere_radius)
{
float x_sum ;//= pLSQ->x_sumplain / pLSQ->size;        //sum( X[n] )
float x_sum2 ;//= pLSQ->x_sumsq / pLSQ->size;    //sum( X[n]^2 )
float x_sum3 ;//= pLSQ->x_sumcube / pLSQ->size;    //sum( X[n]^3 )
float y_sum ;//= pLSQ->y_sumplain / pLSQ->size;        //sum( Y[n] )
float y_sum2 ;//= pLSQ->y_sumsq / pLSQ->size;    //sum( Y[n]^2 )
float y_sum3;// = pLSQ->y_sumcube / pLSQ->size;    //sum( Y[n]^3 )
float z_sum;// = pLSQ->z_sumplain / pLSQ->size;        //sum( Z[n] )
float z_sum2 ;//= pLSQ->z_sumsq / pLSQ->size;    //sum( Z[n]^2 )
float z_sum3 ;//= pLSQ->z_sumcube / pLSQ->size;    //sum( Z[n]^3 )

float XY ;//= pLSQ->xy_sum / pLSQ->size;        //sum( X[n] * Y[n] )
float XZ ;//= pLSQ->xz_sum / pLSQ->size;        //sum( X[n] * Z[n] )
float YZ ;//= pLSQ->yz_sum / pLSQ->size;        //sum( Y[n] * Z[n] )
float X2Y ;//= pLSQ->x2y_sum / pLSQ->size;    //sum( X[n]^2 * Y[n] )
float X2Z ;//= pLSQ->x2z_sum / pLSQ->size;    //sum( X[n]^2 * Z[n] )
float Y2X ;//= pLSQ->y2x_sum / pLSQ->size;    //sum( Y[n]^2 * X[n] )
float Y2Z ;//= pLSQ->y2z_sum / pLSQ->size;    //sum( Y[n]^2 * Z[n] )
float Z2X ;//= pLSQ->z2x_sum / pLSQ->size;    //sum( Z[n]^2 * X[n] )
float Z2Y ;//= pLSQ->z2y_sum / pLSQ->size;    //sum( Z[n]^2 * Y[n] )

//Reduction of multiplications
float F0 ;//= x_sum2 + y_sum2 + z_sum2;
float F1 ;//=  0.5f * F0;
float F2 ;//= -8.0f * (x_sum3 + Y2X + Z2X);
float F3 ;//= -8.0f * (X2Y + y_sum3 + Z2Y);
float F4 ;//= -8.0f * (X2Z + Y2Z + z_sum3);

//Set initial conditions:
float A ;//= x_sum;
float B ;//= y_sum;
float C ;//= z_sum;

//First iteration computation:
float A2 ;//= A * A;
float B2 ;//= B * B;
float C2 ;//= C * C;
float QS ;//= A2 + B2 + C2;
float QB ;//= -2.0f * (A * x_sum + B * y_sum + C * z_sum);

//Set initial conditions:
float Rsq;// = F0 + QB + QS;

//First iteration computation:
float Q0 ;//= 0.5f * (QS - Rsq);
float Q1 ;//= F1 + Q0;
float Q2 ;//= 8.0f * (QS - Rsq + QB + F0);
float aA, aB, aC, nA, nB, nC, dA, dB, dC;


float sx,sy,sz,sr;
//Iterate N times, ignore stop condition.
unsigned int n = 0;

 x_sum = pLSQ->x_sumplain / pLSQ->size;        //sum( X[n] )
 x_sum2 = pLSQ->x_sumsq / pLSQ->size;    //sum( X[n]^2 )
 x_sum3 = pLSQ->x_sumcube / pLSQ->size;    //sum( X[n]^3 )
 y_sum = pLSQ->y_sumplain / pLSQ->size;        //sum( Y[n] )
 y_sum2 = pLSQ->y_sumsq / pLSQ->size;    //sum( Y[n]^2 )
 y_sum3 = pLSQ->y_sumcube / pLSQ->size;    //sum( Y[n]^3 )
 z_sum = pLSQ->z_sumplain / pLSQ->size;        //sum( Z[n] )
 z_sum2 = pLSQ->z_sumsq / pLSQ->size;    //sum( Z[n]^2 )
 z_sum3 = pLSQ->z_sumcube / pLSQ->size;    //sum( Z[n]^3 )

 XY = pLSQ->xy_sum / pLSQ->size;        //sum( X[n] * Y[n] )
 XZ = pLSQ->xz_sum / pLSQ->size;        //sum( X[n] * Z[n] )
 YZ = pLSQ->yz_sum / pLSQ->size;        //sum( Y[n] * Z[n] )
 X2Y = pLSQ->x2y_sum / pLSQ->size;    //sum( X[n]^2 * Y[n] )
 X2Z = pLSQ->x2z_sum / pLSQ->size;    //sum( X[n]^2 * Z[n] )
 Y2X = pLSQ->y2x_sum / pLSQ->size;    //sum( Y[n]^2 * X[n] )
 Y2Z = pLSQ->y2z_sum / pLSQ->size;    //sum( Y[n]^2 * Z[n] )
 Z2X = pLSQ->z2x_sum / pLSQ->size;    //sum( Z[n]^2 * X[n] )
 Z2Y = pLSQ->z2y_sum / pLSQ->size;    //sum( Z[n]^2 * Y[n] )

//Reduction of multiplications
 F0 = x_sum2 + y_sum2 + z_sum2;
 F1 =  0.5f * F0;
 F2 = -8.0f * (x_sum3 + Y2X + Z2X);
 F3 = -8.0f * (X2Y + y_sum3 + Z2Y);
 F4 = -8.0f * (X2Z + Y2Z + z_sum3);

//Set initial conditions:
 A = x_sum;
 B = y_sum;
 C = z_sum;

//First iteration computation:
 A2 = A * A;
 B2 = B * B;
 C2 = C * C;
 QS = A2 + B2 + C2;
 QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);

//Set initial conditions:
 Rsq = F0 + QB + QS;

//First iteration computation:
 Q0 = 0.5f * (QS - Rsq);
 Q1 = F1 + Q0;
 Q2 = 8.0f * (QS - Rsq + QB + F0);



while (n < max_iterations) {
		n++;

		//Compute denominator:
		aA = Q2 + 16.0f * (A2 - 2.0f * A * x_sum + x_sum2);
		aB = Q2 + 16.0f * (B2 - 2.0f * B * y_sum + y_sum2);
		aC = Q2 + 16.0f * (C2 - 2.0f * C * z_sum + z_sum2);
		aA = (aA == 0.0f) ? 1.0f : aA;
		aB = (aB == 0.0f) ? 1.0f : aB;
		aC = (aC == 0.0f) ? 1.0f : aC;

		//Compute next iteration
		nA = A - ((F2 + 16.0f * (B * XY + C * XZ + x_sum * (-A2 - Q0) + A * (x_sum2 + Q1 - C * z_sum - B * y_sum))) / aA);
		nB = B - ((F3 + 16.0f * (A * XY + C * YZ + y_sum * (-B2 - Q0) + B * (y_sum2 + Q1 - A * x_sum - C * z_sum))) / aB);
		nC = C - ((F4 + 16.0f * (A * XZ + B * YZ + z_sum * (-C2 - Q0) + C * (z_sum2 + Q1 - A * x_sum - B * y_sum))) / aC);

		//Check for stop condition
		dA = (nA - A);
		dB = (nB - B);
		dC = (nC - C);

		if ((dA * dA + dB * dB + dC * dC) <= delta) { break; }

		//Compute next iteration's values
		A = nA;
		B = nB;
		C = nC;
		A2 = A * A;
		B2 = B * B;
		C2 = C * C;
		QS = A2 + B2 + C2;
		QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
		Rsq = F0 + QB + QS;
		Q0 = 0.5f * (QS - Rsq);
		Q1 = F1 + Q0;
		Q2 = 8.0f * (QS - Rsq + QB + F0);
}


sx = A;
sy = B;
sz = C;
sr = sqrt(Rsq);

pLSQ->Off_Min_Max[0] = ((pLSQ->Max[0] + pLSQ->Min[0]) * 0.5f);
pLSQ->Off_Min_Max[1] = ((pLSQ->Max[1] + pLSQ->Min[1]) * 0.5f);
pLSQ->Off_Min_Max[2] = ((pLSQ->Max[2] + pLSQ->Min[2]) * 0.5f);

pLSQ->Sum[0] = fabs(pLSQ->Max[0] - pLSQ->Min[0]);
pLSQ->Sum[1] = fabs(pLSQ->Max[1] - pLSQ->Min[1]);
pLSQ->Sum[2] = fabs(pLSQ->Max[2] - pLSQ->Min[2]);


if(sr>0){
pLSQ->right=1;	
pLSQ->Off_Cycle[0]=(sx);
pLSQ->Off_Cycle[1]=(sy);
pLSQ->Off_Cycle[2]=(sz);
	
pLSQ->Off[0]=pLSQ->Off_Cycle[0];
pLSQ->Off[1]=pLSQ->Off_Cycle[1];
pLSQ->Off[2]=pLSQ->Off_Cycle[2];
pLSQ->Gain[0] =  (sr*2)/pLSQ->Sum[0] ;
pLSQ->Gain[1] =  (sr*2)/pLSQ->Sum[1] ;
pLSQ->Gain[2] =  (sr*2)/pLSQ->Sum[2] ;	

}
else{
pLSQ->right=2;
pLSQ->temp_max=pLSQ->Sum[0] ;
if( pLSQ->Sum[1]>pLSQ->temp_max)
pLSQ->temp_max=pLSQ->Sum[1];
if( pLSQ->Sum[2]>pLSQ->temp_max)
pLSQ->temp_max=pLSQ->Sum[2];

pLSQ->Off[0]=pLSQ->Off_Min_Max[0];
pLSQ->Off[1]=pLSQ->Off_Min_Max[1];
pLSQ->Off[2]=pLSQ->Off_Min_Max[2];
pLSQ->Gain[0] =  pLSQ->temp_max/pLSQ->Sum[0] ;
pLSQ->Gain[1] =  pLSQ->temp_max/pLSQ->Sum[1] ;
pLSQ->Gain[2] =  pLSQ->temp_max/pLSQ->Sum[2] ;	
}

*sphere_x = A;
*sphere_y = B;
*sphere_z = C;
*sphere_radius = sqrt(Rsq);
}
