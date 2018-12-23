#include "include.h"
#include "uwb.h"

vec3d vdiff(const vec3d vector1, const vec3d vector2)
{
	vec3d v;
	v.x = vector1.x - vector2.x;
	v.y = vector1.y - vector2.y;
	v.z = vector1.z - vector2.z;
	return v;
}
vec3d vsum(const vec3d vector1, const vec3d vector2)
{
	vec3d v;
	v.x = vector1.x + vector2.x;
	v.y = vector1.y + vector2.y;
	v.z = vector1.z + vector2.z;
	return v;
}
vec3d vmul(const vec3d vector, const float n)
{
	vec3d v;
	v.x = vector.x * n;
	v.y = vector.y * n;
	v.z = vector.z * n;
	return v;
}
vec3d vdiv(const vec3d vector, const float n)
{
	vec3d v;
	v.x = vector.x / n;
	v.y = vector.y / n;
	v.z = vector.z / n;
	return v;
}
float vdist(const vec3d v1, const vec3d v2)
{
	float xd = v1.x - v2.x;
	float yd = v1.y - v2.y;
	float zd = v1.z - v2.z;
	return sqrt(xd * xd + yd * yd + zd * zd);
}
float vnorm(const vec3d vector)
{
	return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}
float dot(const vec3d vector1, const vec3d vector2)
{
	return vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z;
}

vec3d crossuwb(const vec3d vector1, const vec3d vector2)
{
	vec3d v;
	v.x = vector1.y * vector2.z - vector1.z * vector2.y;
	v.y = vector1.z * vector2.x - vector1.x * vector2.z;
	v.z = vector1.x * vector2.y - vector1.y * vector2.x;
	return v;
}

float gdoprate(const vec3d tag, const vec3d p1, const vec3d p2, const vec3d p3)
{
	vec3d ex, t1, t2, t3;
	float h, gdop1, gdop2, gdop3, result;

	ex = vdiff(p1, tag);
	h = vnorm(ex);
	t1 = vdiv(ex, h);

	ex = vdiff(p2, tag);
	h = vnorm(ex);
	t2 = vdiv(ex, h);

	ex = vdiff(p3, tag);
	h = vnorm(ex);
	t3 = vdiv(ex, h);

	gdop1 = fabs(dot(t1, t2));
	gdop2 = fabs(dot(t2, t3));
	gdop3 = fabs(dot(t3, t1));

	if (gdop1 < gdop2) result = gdop2; else result = gdop1;
	if (result < gdop3) result = gdop3;

	return result;
}
int sphereline(const vec3d p1, const vec3d p2, const vec3d sc, float r, float *const mu1, float *const mu2)
{
   float a,b,c;
   float bb4ac;
   vec3d dp;

   dp.x = p2.x - p1.x;
   dp.y = p2.y - p1.y;
   dp.z = p2.z - p1.z;

   a = dp.x * dp.x + dp.y * dp.y + dp.z * dp.z;

   b = 2 * (dp.x * (p1.x - sc.x) + dp.y * (p1.y - sc.y) + dp.z * (p1.z - sc.z));

   c = sc.x * sc.x + sc.y * sc.y + sc.z * sc.z;
   c += p1.x * p1.x + p1.y * p1.y + p1.z * p1.z;
   c -= 2 * (sc.x * p1.x + sc.y * p1.y + sc.z * p1.z);
   c -= r * r;

   bb4ac = b * b - 4 * a * c;

   if (fabs(a) == 0 || bb4ac < 0) {
      *mu1 = 0;
      *mu2 = 0;
      return -1;
   }

   *mu1 = (-b + sqrt(bb4ac)) / (2 * a);
   *mu2 = (-b - sqrt(bb4ac)) / (2 * a);

   return 0;
}

int trilateration(vec3d *const result1,
				  vec3d *const result2,
				  vec3d *const best_solution,
				  const vec3d p1, const float r1,
                  const vec3d p2, const float r2,
                  const vec3d p3, const float r3,
                  const vec3d p4, const float r4,
                  const float maxzero)
{
	vec3d	ex, ey, ez, t1, t2, t3;
	float	h, i, j, x, y, z, t;
	float	mu1, mu2, mu;
	int result;

	ex = vdiff(p3, p1); 
	h = vnorm(ex); 
	if (h <= maxzero) {
		return ERR_TRIL_CONCENTRIC;
	}

	ex = vdiff(p3, p2);
	h = vnorm(ex);
	if (h <= maxzero) {
		return ERR_TRIL_CONCENTRIC;
	}

	ex = vdiff(p2, p1); 
	h = vnorm(ex); 
	if (h <= maxzero) {
		return ERR_TRIL_CONCENTRIC;
	}
	ex = vdiv(ex, h); 

	
	t1 = vdiff(p3, p1); 
	i = dot(ex, t1); 
	t2 = vmul(ex, i); 

	ey = vdiff(t1, t2); 
	t = vnorm(ey); 
	if (t > maxzero) {
		ey = vdiv(ey, t); 
		
		j = dot(ey, t1); 
	} else
		j = 0.0;

	if (fabs(j) <= maxzero) {
		t2 = vsum(p1, vmul(ex, r1));
		if (fabs(vnorm(vdiff(p2, t2)) - r2) <= maxzero &&
		    fabs(vnorm(vdiff(p3, t2)) - r3) <= maxzero) {
			if (result1)
				*result1 = t2;
			if (result2)
				*result2 = t2;
			return TRIL_3SPHERES;
		}
		t2 = vsum(p1, vmul(ex, -r1));
		if (fabs(vnorm(vdiff(p2, t2)) - r2) <= maxzero &&
		    fabs(vnorm(vdiff(p3, t2)) - r3) <= maxzero) {
			
			if (result1)
				*result1 = t2;
			if (result2)
				*result2 = t2;
			return TRIL_3SPHERES;
		}
		
		return ERR_TRIL_COLINEAR_2SOLUTIONS;
	}

	
	ez = crossuwb(ex, ey); 

	x = (r1*r1 - r2*r2) / (2*h) + h / 2;
	y = (r1*r1 - r3*r3 + i*i) / (2*j) + j / 2 - x * i / j;
	z = r1*r1 - x*x - y*y;
	if (z < -maxzero) {
		
		return ERR_TRIL_SQRTNEGNUMB;
	} else
	if (z > 0.0)
		z = sqrt(z);
	else
		z = 0.0;

	
	t2 = vsum(p1, vmul(ex, x));
	t2 = vsum(t2, vmul(ey, y));

	
	if (result1)
		*result1 = vsum(t2, vmul(ez, z));

	
	if (result2)
		*result2 = vsum(t2, vmul(ez, -z));

	
	ex = vdiff(p4, p1);
	h = vnorm(ex); 
	if (h <= maxzero) {
		
		return TRIL_3SPHERES;
	}
	
	ex = vdiff(p4, p2); 
	h = vnorm(ex); 
	if (h <= maxzero) {
	
		return TRIL_3SPHERES;
	}
	
	ex = vdiff(p4, p3);
	h = vnorm(ex);
	if (h <= maxzero) {
	
		return TRIL_3SPHERES;
	}

	
	t3 = vdiff(*result1, p4);
	i = vnorm(t3);
	
	t3 = vdiff(*result2, p4);
	h = vnorm(t3);

	
	if (i > h) {
		*best_solution = *result1;
		*result1 = *result2;
		*result2 = *best_solution;
	}

	int count4 = 0;
	float rr4 = r4;
	result = 1;
	
	while(result && count4 < 10)
	{
		result=sphereline(*result1, *result2, p4, rr4, &mu1, &mu2);
		rr4+=0.1;
		count4++;
	}

	if (result) {
	
		*best_solution = *result1;
	
	} else {

		if (mu1 < 0 && mu2 < 0) {

			if (fabs(mu1) <= fabs(mu2)) mu = mu1; else mu = mu2;
			
			ex = vdiff(*result2, *result1); 
			h = vnorm(ex); 
			ex = vdiv(ex, h); 
			
			mu = 0.5*mu;
			
			t2 = vmul(ex, mu*h);
			t2 = vsum(*result1, t2);
			
			*best_solution = t2;

		} else if ((mu1 < 0 && mu2 > 1) || (mu2 < 0 && mu1 > 1)) {

			
			if (mu1 > mu2) mu = mu1; else mu = mu2;
			
			ex = vdiff(*result2, *result1); 
			h = vnorm(ex); 
			ex = vdiv(ex, h);
			
			t2 = vmul(ex, mu*h);
			t2 = vsum(*result1, t2);
			
			t3 = vmul(vdiff(*result2, t2),0.5);
			
			*best_solution = vsum(t2, t3);

		} else if (((mu1 > 0 && mu1 < 1) && (mu2 < 0 || mu2 > 1))
				|| ((mu2 > 0 && mu2 < 1) && (mu1 < 0 || mu1 > 1))) {

			
			if (mu1 >= 0 && mu1 <= 1) mu = mu1; else mu = mu2;
			
			if (mu <= 0.5) mu-=0.5*mu; else mu-=0.5*(1-mu);
			
			ex = vdiff(*result2, *result1); 
			h = vnorm(ex); 
			ex = vdiv(ex, h); 
			
			t2 = vmul(ex, mu*h);
			t2 = vsum(*result1, t2);
			
			*best_solution = t2;

		} else if (mu1 == mu2) {

			
			mu = mu1;
			
			if (mu <= 0.25) mu-=0.5*mu;
			else if (mu <=0.5) mu-=0.5*(0.5-mu);
			else if (mu <=0.75) mu-=0.5*(mu-0.5);
			else mu-=0.5*(1-mu);
			
			ex = vdiff(*result2, *result1); 
			h = vnorm(ex); 
			ex = vdiv(ex, h); 
			
			t2 = vmul(ex, mu*h);
			t2 = vsum(*result1, t2);
			
			*best_solution = t2;

		} else {

		

			mu = mu1 + mu2;
			
			ex = vdiff(*result2, *result1); 
			h = vnorm(ex); 
			ex = vdiv(ex, h);
			
			mu = 0.5*mu;
			
			t2 = vmul(ex, mu*h);
			t2 = vsum(*result1, t2);
			
			*best_solution = t2;

		}

	}

	return TRIL_4SPHERES;

	
}



int deca_3dlocate (	vec3d	*const solution1,
					vec3d	*const solution2,
					vec3d	*const best_solution,
					int		*const nosolution_count,
					float	*const best_3derror,
					float	*const best_gdoprate,
					vec3d p1, float r1,
					vec3d p2, float r2,
					vec3d p3, float r3,
					vec3d p4, float r4,
					int *combination)
{
	vec3d	o1, o2, solution, ptemp;
	vec3d	solution_compare1, solution_compare2;
    float	 rtemp;
	float	gdoprate_compare1, gdoprate_compare2;
	float	ovr_r1, ovr_r2, ovr_r3, ovr_r4;
	int		overlook_count, combination_counter;
	int		trilateration_errcounter, trilateration_mode34;
	int		success, concentric, result;

	trilateration_errcounter = 0;
	trilateration_mode34 = 0;

	combination_counter = 4; 

	*best_gdoprate = 1; 
	gdoprate_compare1 = 1; gdoprate_compare2 = 1;
	solution_compare1.x = 0; solution_compare1.y = 0; solution_compare1.z = 0;


	do {
		success = 0;
		concentric = 0;
		overlook_count = 0;
		ovr_r1 = r1; ovr_r2 = r2; ovr_r3 = r3; ovr_r4 = r4;

		do {

			result = trilateration(&o1, &o2, &solution, p1, ovr_r1, p2, ovr_r2, p3, ovr_r3, p4, ovr_r4, MAXZERO);

			switch (result)
			{
				case TRIL_3SPHERES:
					trilateration_mode34 = TRIL_3SPHERES;
					success = 1;
					break;

				case TRIL_4SPHERES: 
					trilateration_mode34 = TRIL_4SPHERES;
					success = 1;
					break;

				case ERR_TRIL_CONCENTRIC:
					concentric = 1;
					break;

				default: 
					ovr_r1 += 0.10;
					ovr_r2 += 0.10;
					ovr_r3 += 0.10;
					ovr_r4 += 0.10;
					overlook_count++;
					break;
			}

         

        } while (!success && (overlook_count <= 5) && !concentric);


		if (success)
		{
			switch (result)
			{
			case TRIL_3SPHERES:
				*solution1 = o1;
				*solution2 = o2;
				*nosolution_count = overlook_count;

                combination_counter = 0;
				break;

			case TRIL_4SPHERES:
				
				gdoprate_compare1	= gdoprate(solution, p1, p2, p3);

				
				if (gdoprate_compare1 <= gdoprate_compare2) {

					*solution1 = o1;
					*solution2 = o2;
					*best_solution	= solution;
					*nosolution_count = overlook_count;
					*best_3derror	= sqrt((vnorm(vdiff(solution, p1))-r1)*(vnorm(vdiff(solution, p1))-r1) +
										(vnorm(vdiff(solution, p2))-r2)*(vnorm(vdiff(solution, p2))-r2) +
										(vnorm(vdiff(solution, p3))-r3)*(vnorm(vdiff(solution, p3))-r3) +
										(vnorm(vdiff(solution, p4))-r4)*(vnorm(vdiff(solution, p4))-r4));
					*best_gdoprate	= gdoprate_compare1;

					
					solution_compare2 = solution_compare1;
                 
					gdoprate_compare2 = gdoprate_compare1;

					*combination = 5 - combination_counter;

                    ptemp = p1; p1 = p2; p2 = p3; p3 = p4; p4 = ptemp;
                    rtemp = r1; r1 = r2; r2 = r3; r3 = r4; r4 = rtemp;
                    combination_counter--;

				}
				break;

			default:
				break;
			}
		}
		else
		{
            
            trilateration_errcounter = 4;
            combination_counter = 0;
		}

  

    } while (combination_counter);

	
	if (trilateration_errcounter >= 4) return -1; else return trilateration_mode34;

}


int GetLocation(vec3d *best_solution, int use4thAnchor, vec3d* anchorArray, int *distanceArray)
{

	vec3d	o1, o2, p1, p2, p3, p4;
    float	r1 = 0, r2 = 0, r3 = 0, r4 = 0, best_3derror, best_gdoprate;
	int		result;
    int     error, combination;

	vec3d	t3;
	float	dist1, dist2;

	
    p1.x = anchorArray[0].x;		p1.y = anchorArray[0].y;	p1.z = anchorArray[0].z;
    p2.x = anchorArray[1].x;		p2.y = anchorArray[1].y;	p2.z = anchorArray[1].z;
    p3.x = anchorArray[2].x;		p3.y = anchorArray[2].y;	p3.z = anchorArray[2].z;
    p4.x = anchorArray[0].x;		p4.y = anchorArray[0].y;	p4.z = anchorArray[0].z; //4th same as 1st - only 3 used for trilateration

    r1 = (float) distanceArray[0] / 1000.0;
    r2 = (float) distanceArray[1] / 1000.0;
    r3 = (float) distanceArray[2] / 1000.0;

    r4 = (float) distanceArray[3] / 1000.0;

	
    result = deca_3dlocate (&o1, &o2, best_solution, &error, &best_3derror, &best_gdoprate,
                            p1, r1, p2, r2, p3, r3, p4, r1, &combination);

    if(result >= 0)
    {
        if (use4thAnchor == 1) 
        {
                float diff1, diff2;
                
                t3 = vdiff(o1, anchorArray[3]);
                dist1 = vnorm(t3);

                t3 = vdiff(o2, anchorArray[3]);
                dist2 = vnorm(t3);

               
                diff1 = fabs(r4 - dist1);
                diff2 = fabs(r4 - dist2);

              
                if (diff1 < diff2) *best_solution = o1; else *best_solution = o2;
        }
        else
        {
            
            if(o1.z < p1.z) *best_solution = o1; else *best_solution = o2;
        }
    }

	if (result >= 0)
	{
		return result;
	}

	
	return -1;
}


float uwb_pos[3];
u8 uwb_good;
void uwb_pos_cal(float dis[4])
{
    int result = 0;
    vec3d anchorArray[4];
    vec3d report;
    int Range_deca[4];

    if((dis[0]>0.05&&dis[0]<100&&
			 dis[1]>0.05&&dis[1]<100&&
		   dis[2]>0.05&&dis[2]<100&&
		   dis[3]>0.05&&dis[3]<100)||0){
	  uwb_good=1;			 
    anchorArray[0].x = 0.000; //anchor0.x uint:m
    anchorArray[0].y = 0.000; //anchor0.y uint:m
    anchorArray[0].z = 1.3500; //anchor0.z uint:m

    anchorArray[1].x = 0.0; //anchor1.x uint:m
    anchorArray[1].y = 4.000; //anchor1.y uint:m
    anchorArray[1].z = 1.300; //anchor1.z uint:m

    anchorArray[2].x = 4.000; //anchor2.x uint:m
    anchorArray[2].y = 4.; //anchor2.y uint:m
    anchorArray[2].z = 1.350; //anchor2.z uint:m

    anchorArray[3].x = 4.000; //anchor3.x uint:m
    anchorArray[3].y = 0.0; //anchor3.y uint:m
    anchorArray[3].z = 1.650; //anchor3.z uint:m
				 
		Range_deca[0] = dis[0]*1000;
		Range_deca[1] = dis[1]*1000;
		Range_deca[2] = dis[2]*1000;
		Range_deca[3] = dis[3]*1000;
    //Range_deca[0] = Range_deca[1]=Range_deca[2] = Range_deca[3]=3500;
		result = GetLocation(&report, 0, &anchorArray[0], &Range_deca[0]);	
    uwb_pos[0]=report.x;
		uwb_pos[1]=report.y;
		uwb_pos[2]=report.z;
	}else uwb_good=0;
}
