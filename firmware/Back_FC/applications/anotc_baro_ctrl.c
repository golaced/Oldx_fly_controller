#include "rc.h"
#include "anotc_baro_ctrl.h"
#include "filter.h"
#include "alt_fushion.h"
#include "bmp.h"
		
_height_st sonarf;		
float kp_sonar=0.008;
void baro_ctrl(float dT,_hc_value_st *height_value)
{ 
   float kp_sonar_use=1;
	    baro.h_dt = dT; 
	    ukf_baro_task1(dT)	;//¸ß¶ÈÈÚºÏ
		  float tilted_fix_sonar=LIMIT((ALT_POS_SONAR2/cos(LIMIT(my_deathzoom_21(Pitch,5),-45,45)/57.3)/
							cos(LIMIT(my_deathzoom_21(Roll,5),-45,45)/57.3)-ALT_POS_SONAR2),0,0.5);
		  sonarf.h_origin=ALT_POS_SONAR2+tilted_fix_sonar;
	    sonarf.h_flt=sonarf.h_flt+ALT_VEL_BMP_UKF_OLDX*dT;
	    if(ALT_VEL_BMP_UKF_OLDX>2.2)
				kp_sonar_use=0;
			else
				kp_sonar_use=kp_sonar;
	    sonarf.h_flt=sonarf.h_flt*(1-kp_sonar_use)+kp_sonar_use*sonarf.h_origin;
	
			height_value->fusion_speed = LIMIT( (ALT_VEL_BMP_UKF_OLDX*1000),-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);
			height_value->fusion_height = ALT_POS_BMP_UKF_OLDX*1000;			

			m100.H=(float)height_value->fusion_height/1000.;
			m100.H_Spd=(float)height_value->fusion_speed/1000.;
}




