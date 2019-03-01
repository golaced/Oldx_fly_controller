#include "stm32f4xx.h"


//#define OSR  256  // 0.60 mSec conversion time (1666.67 Hz)
//#define OSR  512  // 1.17 mSec conversion time ( 854.70 Hz)
//#define OSR 1024  // 2.28 mSec conversion time ( 357.14 Hz)
//#define OSR 2048  // 4.54 mSec conversion time ( 220.26 Hz)
#define OSR 4096  // 9.04 mSec conversion time ( 110.62 Hz)

int32_t calculateTemperature(void);
void readTemperature();
void readPressure();
void requestTemperature();
void requestPressure();
void calculatePressureAltitude(int32_t *pressure, int32_t *temperature);
u8 ms5611DetectSpi(void);
void MS5611_ThreadNew_SPI(void) ;

extern int32_t ms5611Press,ms5611Temp;
extern float   ms5611Alt;

extern float windSpeed,windSpeedAcc;
void WindEstimate(float dt);