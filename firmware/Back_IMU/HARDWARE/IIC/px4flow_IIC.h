//22个字节，每个字节对应地址的宏定义
#ifndef  PX
#define  PX


#include "board.h"
# define frame_count_lower8 0x00
# define frame_count_upper8 0x01
# define pixel_flow_x_sum_lower8 0x02
# define pixel_flow_x_sum_upper8 0x03
# define pixel_flow_y_sum_lower8 0x04
# define pixel_flow_y_sum_upper8 0x05
# define flow_comp_m_x_lower8 0x06
# define flow_comp_m_x_upper8 0x07
# define flow_comp_m_y_lower8 0x08
# define flow_comp_m_y_upper8 0x09
# define flow_quality_lower8 0xA
# define flow_quality_upper8 0xB
# define gyro_x_rate_lower8 0xC
# define gyro_x_rate_upper8 0xD
# define gyro_y_rate_lower8 0xE
# define gyro_y_rate_upper8 0xF
# define gyro_z_rate_lower8 0x10
# define gyro_z_rate_upper8 0x11
# define gyro_range_8 0x12
# define sonar_timestamp_8 0x13
# define distance_lower8 0x14
# define distance_upper8 0x15

# define px_read 0x85
# define px_write 0x84



typedef struct FX4FLOW_TypeDef

{
	
    int16_t frame_count;// counts created I2C frames [#frames]
    int16_t pixel_flow_x_sum;// latest x flow measurement in pixels*10 [pixels]
    int16_t pixel_flow_y_sum;// latest y flow measurement in pixels*10 [pixels]
    int16_t flow_comp_m_x;// x velocity*1000 [meters/sec]
    int16_t flow_comp_m_y;// y velocity*1000 [meters/sec]
    int16_t qual;// Optical flow quality / confidence [0: bad, 255: maximum quality]
    int16_t gyro_x_rate; // latest gyro x rate [rad/sec]
    int16_t gyro_y_rate; // latest gyro y rate [rad/sec]
    int16_t gyro_z_rate; // latest gyro z rate [rad/sec]
    u8  gyro_range; // gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec] 
    u8   sonar_timestamp;// time since last sonar update [milliseconds]
    int16_t ground_distance;// Ground distance in meters*1000 [meters]. Positive value: distance known. Negative value: Unknown distance

    
		
		//25字节数据定义


   uint16_t frame_count_since_last_readout;//number of flow measurements since last I2C readout [#frames]
    int16_t pixel_flow_x_integral; //accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
    int16_t pixel_flow_y_integral;//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    int16_t gyro_x_rate_integral;//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_y_rate_integral;//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_z_rate_integral;//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000] 
    uint32_t integration_timespan;//accumulation timespan in microseconds since last I2C readout [microseconds]
    uint32_t sonar_timestamp25;// time since last sonar update [microseconds]
    int16_t ground_distance25;// Ground distance in meters*1000 [meters*1000]
    int16_t gyro_temperature;// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    uint8_t quality;// averaged quality of accumulated flow values [0:bad quality;255: max quality]
		
}FX4FLOW_TypeDef;
   


extern FX4FLOW_TypeDef FLOW;


void PX4FLOW_22date_compound(void);
void PX4FLOW_25date_compound(void);
void Read_Px4flow(void);

#endif
