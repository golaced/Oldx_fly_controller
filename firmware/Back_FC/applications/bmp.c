#include "include.h"
#include "bmp.h"
#include "iic_hml.h"
#include "imu.h"
#include "height_ctrl.h"
#include <math.h>
#include "rc.h"
#include "time.h"
float baro_alt_speed_ano,MS5611_Pressure;
int baroAlt,baroAlt_fc;
u8 baro_set;
_height_st baro;
u8 baro_update;




