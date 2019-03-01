#include "spi.h"
#include "time.h"
#include "include.h"
#include "bmp.h"
#include "iic_hml.h"
#include "imu.h"
#include "height_ctrl.h"
#include <math.h>
#include "rc.h"
#include "time.h"
#define ENABLE_MS5611       SPI_CS(MS5611,0)
#define DISABLE_MS5611      SPI_CS(MS5611,1)
float baro_alt_speed_ano,MS5611_Pressure;
int baroAlt,baroAlt_fc;
u8 baro_set;
_height_st baro;
u8 baro_update;
typedef union {
    uint16_t value;
    uint8_t bytes[2];
} uint16andUint8_t;

typedef union {
    uint32_t value;
    uint8_t bytes[4];
} uint32andUint8_t;

//#define OSR  256  // 0.60 mSec conversion time (1666.67 Hz)
//#define OSR  512  // 1.17 mSec conversion time ( 854.70 Hz)
//#define OSR 1024  // 2.28 mSec conversion time ( 357.14 Hz)
//#define OSR 2048  // 4.54 mSec conversion time ( 220.26 Hz)
#define OSR 4096  // 9.04 mSec conversion time ( 110.62 Hz)


uint16andUint8_t c1, c2, c3, c4, c5, c6;

uint32andUint8_t d1, d2;

int32_t dT;

int32_t ms5611Temperature;
int32_t ms5611Press,ms5611Temp;
float   ms5611Alt;
///////////////////////////////////////////////////////////////////////////////
// Calculate Temperature
///////////////////////////////////////////////////////////////////////////////

int32_t calculateTemperature(void)
{
    dT = (int32_t)d2.value - ((int32_t)c5.value << 8);
    ms5611Temperature = 2000 + (int32_t)(((int64_t)dT * c6.value) >> 23);
    return ms5611Temperature;
}

///////////////////////////////////////////////////////////////////////////////
// Read Temperature Request Pressure
///////////////////////////////////////////////////////////////////////////////

void readTemperature()
{
    ENABLE_MS5611;
    Spi_RW(0x00);
    d2.bytes[2] = Spi_RW(0x00);
    d2.bytes[1] = Spi_RW(0x00);
    d2.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;
    calculateTemperature();
}

void readPressure()
{
    ENABLE_MS5611;
    Spi_RW(0x00);
    d1.bytes[2] = Spi_RW(0x00);
    d1.bytes[1] = Spi_RW(0x00);
    d1.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;
}

void requestTemperature()
{
    ENABLE_MS5611;                      // Request temperature conversion
#if   (OSR ==  256)
    Spi_RW(0x50);
#elif (OSR ==  512)
    Spi_RW(0x52);
#elif (OSR == 1024)
    Spi_RW(0x54);
#elif (OSR == 2048)
    Spi_RW(0x56);
#elif (OSR == 4096)
    Spi_RW(0x58);
#endif
    DISABLE_MS5611;
}

void requestPressure()
{
    ENABLE_MS5611;                      // Request pressure conversion
#if   (OSR ==  256)
    Spi_RW(0x40);
#elif (OSR ==  512)
    Spi_RW(0x42);
#elif (OSR == 1024)
    Spi_RW(0x44);
#elif (OSR == 2048)
    Spi_RW(0x46);
#elif (OSR == 4096)
    Spi_RW(0x48);
#endif
    DISABLE_MS5611;
}

float windSpeed,windSpeedAcc;
void WindEstimate(float dt)
{
    float angle[2];
    static float tiltAngle;
    static float lastWindSpeed;

    static uint64_t previousT;
    float deltaT = dt;

    //?????????????????????????

    if(fabs(CH_filter[ROL])<25&&fabs(CH_filter[PIT])<25)
    {
        //??????, ?????
        angle[0] = my_deathzoom_21(Pit_fc, 3);
        angle[1] = my_deathzoom_21(Rol_fc, 3);
        tiltAngle = tiltAngle * 0.995f + sqrt(angle[0]*angle[0]+angle[1]*angle[1]) * 0.005f;

        //????????????
        windSpeed = 100 * sinf(tiltAngle*0.0173);

        //??????,?????
        windSpeedAcc = windSpeedAcc * 0.95f + ((windSpeed - lastWindSpeed) / deltaT) * 0.05f;
        lastWindSpeed = windSpeed;
    }
    else
    {
        windSpeed    -= windSpeed * 0.005f;
        windSpeedAcc -= windSpeedAcc * 0.005f;
    }

}

int16_t BaroCompensate(void)
{
    int16_t velocity;

    if((m100.navigation_mode==2&&m100.GPS_STATUS<6)||(m100.navigation_mode==1&&m100.GPS_STATUS<50))
        return 0;

    velocity = sqrt(VEL_UKF_Y*VEL_UKF_Y+VEL_UKF_X*VEL_UKF_X);
    velocity = my_deathzoom_21(LIMIT(velocity, 0, 0.5), 0.02);

    return - velocity * 0.05f;
}

///////////////////////////////////////////////////////////////////////////////
// Calculate Pressure Altitude
///////////////////////////////////////////////////////////////////////////////
static float Alt_offset_Pa=0; 
void calculatePressureAltitude(int32_t *pressure, int32_t *temperature)
{
    int64_t offset;
    int64_t offset2 = 0;
    int64_t sensitivity;
    int64_t sensitivity2 = 0;
    int64_t f;
    int32_t p;

    int32_t ms5611Temp2 = 0;

    offset = ((int64_t)c2.value << 16) + (((int64_t)c4.value * dT) >> 7);
    sensitivity = ((int64_t)c1.value << 15) + (((int64_t)c3.value * dT) >> 8);

    if (ms5611Temperature < 2000) {
        ms5611Temp2 = (dT * dT) >> 31;

        f = ms5611Temperature - 2000;
        f = f * f;
        offset2 = 5 * f >> 1;
        sensitivity2 = 5 * f >> 2;

        if (ms5611Temperature < -1500) {
            f = (ms5611Temperature + 1500);
            f = f * f;
            offset2 += 7 * f;
            sensitivity2 += 11 * f >> 1;
        }

        ms5611Temperature -= ms5611Temp2;

        offset -= offset2;
        sensitivity -= sensitivity2;
    }

    p = (((d1.value * sensitivity) >> 21) - offset) >> 15;
    if (pressure)
        *pressure = p;
    if (temperature)
        *temperature = ms5611Temperature;
		
  static u16 paInitCnt;	
  static long paOffsetNum;		
	// 是否初始化过0米气压值？
	if(Alt_offset_Pa == 0)
	{ 
		if(paInitCnt > 50)
		{
			Alt_offset_Pa = paOffsetNum / paInitCnt;
			//paOffsetInited=1;
		}
		else
			paOffsetNum += p;
		
		paInitCnt++;
		
		ms5611Alt = 0; //高度 为 0
		
	}else
   ms5611Alt= (44330.0f * (1.0f - pow((float)p / Alt_offset_Pa, 1.0f / 5.255f)));
	// ms5611Alt = 4433000.0 * (1 - pow((p / Alt_offset_Pa), 0.1903))*0.01f;  
   
	int baroAltTemp;
	baroAltTemp=BaroCompensate();

	//????????
	ms5611Alt = ms5611Alt ;//* 0.5f + baroAltTemp * 0.5f; 
}

///////////////////////////////////////////////////////////////////////////////
// Pressure Initialization
///////////////////////////////////////////////////////////////////////////////

u8 ms5611DetectSpi(void)
{
    

    ENABLE_MS5611;   // Reset Device
    Spi_RW(0x1E);
    Delay_ms(3);
    DISABLE_MS5611;

    Delay_us(150);

    ENABLE_MS5611;   // Read Calibration Data C1
    Spi_RW(0xA2);
    c1.bytes[1] = Spi_RW(0x00);
    c1.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;

    Delay_ms(10);

    ENABLE_MS5611;   // Read Calibration Data C2
    Spi_RW(0xA4);
    c2.bytes[1] = Spi_RW(0x00);
    c2.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;

    Delay_ms(10);

    ENABLE_MS5611;   // Read Calibration Data C3
    Spi_RW(0xA6);
    c3.bytes[1] = Spi_RW(0x00);
    c3.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;

    Delay_ms(10);

    ENABLE_MS5611;   // Read Calibration Data C4
    Spi_RW(0xA8);
    c4.bytes[1] = Spi_RW(0x00);
    c4.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;

    Delay_ms(10);

    ENABLE_MS5611;   // Read Calibration Data C5
    Spi_RW(0xAA);
    c5.bytes[1] = Spi_RW(0x00);
    c5.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;

    Delay_ms(10);

    ENABLE_MS5611;   // Read Calibration Data C6
    Spi_RW(0xAC);
    c6.bytes[1] = Spi_RW(0x00);
    c6.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;

 
    return 1;
}

#include "bmp.h"
static uint32_t MS5611_Delay_us[9] = {
	1500,//MS561101BA_OSR_256 0.9ms  0x00
	1500,//MS561101BA_OSR_256 0.9ms  
	2000,//MS561101BA_OSR_512 1.2ms  0x02
	2000,//MS561101BA_OSR_512 1.2ms
	3000,//MS561101BA_OSR_1024 2.3ms 0x04
	3000,//MS561101BA_OSR_1024 2.3ms
	5000,//MS561101BA_OSR_2048 4.6ms 0x06
	5000,//MS561101BA_OSR_2048 4.6ms
	11000,//MS561101BA_OSR_4096 9.1ms 0x08
	//20000, // 16.44ms
};
//#define MS5611Press_OSR  MS561101BA_OSR_2048  //气压采样精度
//#define MS5611Temp_OSR   MS561101BA_OSR_2048 //温度采样精度
#define MS5611Press_OSR  MS561101BA_OSR_4096  //气压采样精度
#define MS5611Temp_OSR   MS561101BA_OSR_4096 //温度采样精度
// 气压计状态机
#define SCTemperature  0x01	  //开始 温度转换
#define CTemperatureing  0x02  //正在转换温度
#define SCPressure  0x03	  //开始转换 气压
#define SCPressureing  0x04	  //正在转换气压值
uint8_t  Now_doing = SCTemperature;	//当前转换状态
static uint32_t Current_delay=0;	    //转换延时时间 us 
static uint32_t Start_Convert_Time; //启动转换时的 时间 us 
static uint8_t Baro_ALT_Updated = 0; //气压计高度更新完成标志。
void MS5611_ThreadNew_SPI(void) 
{
	switch(Now_doing)
	{ //查询状态 看看我们现在 该做些什么？
 		case SCTemperature:  //启动温度转换
			//开启温度转换		  
        requestTemperature(); 
				Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//转换时间
				Start_Convert_Time =GetSysTime_us();// micros(); //计时开始
				Now_doing = CTemperatureing;//下一个状态
 		break;
		
		case CTemperatureing:  //正在转换中 
			if((GetSysTime_us()-Start_Convert_Time) > Current_delay)
			{ //延时时间到了吗？
				readTemperature(); 
				//启动气压转换
				requestPressure(); 
				Current_delay = MS5611_Delay_us[MS5611Press_OSR];//转换时间
				Start_Convert_Time = GetSysTime_us();//计时开始
				Now_doing = SCPressureing;//下一个状态
			}
			break;
 
		case SCPressureing:	 //正在转换气压值
			if((GetSysTime_us()-Start_Convert_Time) > Current_delay)
			{ //延时时间到了吗？
				readPressure();
				baro_update=1;
				calculatePressureAltitude(&ms5611Press,&ms5611Temp);
				Baro_ALT_Updated = 0xff; 	//高度更新 完成。
				//开启温度转换
				requestTemperature(); 
				Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//转换时间
				Start_Convert_Time = GetSysTime_us(); //计时开始
				Now_doing = CTemperatureing;//下一个状态
			}
			break;
		default: 
			Now_doing = CTemperatureing;
			break;
	}
}