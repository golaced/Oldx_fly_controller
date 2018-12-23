#include "ak8975.h"
#include "parameter.h"
#include "mymath.h"
#include "include.h"
#include "iic_hml.h"
#include "i2c_soft.h"
#include "filter.h"
#include "mpu9250.h"
ak8975_t ak8975 = { {0,0,0},{124,-449,369},{1,0.532,0.486},{0,0,0} };
ak8975_t ak8975_fc = { {0,0,0},{232,-221,-119},{1.17,1.339,1},{0,0,0} };

