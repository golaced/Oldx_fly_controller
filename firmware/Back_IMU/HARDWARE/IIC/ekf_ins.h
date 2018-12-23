#ifndef EKF_INS_GPS_Estimator_H_
#define EKF_INS_GPS_Estimator_H_

#include <stm32f4xx.h>	 
typedef struct 
{
    double Pos[3]; // Position in meters and relative to a local NED frame ???????? ?? m X[0] X[1] X[2]
    double Vel[3]; // Velocity in meters and in NED ????? ?? m/s X[3] X[4] X[5]
    float q[4]; // unit quaternion rotation relative to NED ??? ??? X[6] X[7] X[8] X[9]
    float gyro_bias[3]; // ????? X[10] X[11] X[12]
    float accel_bias[3];// ????? X[13] X[14] X[15]
} NavStruct;

typedef struct  
{
	double NED_Pos[3],NED_Posf[3],NED_Posf_reg[3],Off_NED_Posf[3],NED_Pos_reg[3];
	double NED_Vel[3],NED_Velf[3];
	double NED_Acc[3];
	float heading;
	float groundspeed;
	float quality;
	uint8_t updated;
}GPS_Sensor_Struct;
extern GPS_Sensor_Struct Global_GPS_Sensor;

void EKF_INS_GPS_Run(float T);
#define NOT_DEBUG_EKF 1
extern float  Global_Now_Euler[3];
extern NavStruct Nav;       //导航结构体
extern float YawR,PitchR,RollR,RollRm,PitchRm,YawRm;;
extern float fRPY[3] ;
void GPS_Dates_Deal(void);
#endif
