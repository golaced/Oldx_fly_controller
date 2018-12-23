// MESSAGE SENSOR_OFFSETS PACKING

#ifndef _MAVLINK_MSG_SENSOR_OFFSETS_H_
#define _MAVLINK_MSG_SENSOR_OFFSETS_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_SENSOR_OFFSETS 150

typedef struct __mavlink_sensor_offsets_t
{
 float mag_declination; /*< magnetic declination (radians)*/
 int32_t raw_press; /*< raw pressure from barometer*/
 int32_t raw_temp; /*< raw temperature from barometer*/
 float gyro_cal_x; /*< gyro X calibration*/
 float gyro_cal_y; /*< gyro Y calibration*/
 float gyro_cal_z; /*< gyro Z calibration*/
 float accel_cal_x; /*< accel X calibration*/
 float accel_cal_y; /*< accel Y calibration*/
 float accel_cal_z; /*< accel Z calibration*/
 int16_t mag_ofs_x; /*< magnetometer X offset*/
 int16_t mag_ofs_y; /*< magnetometer Y offset*/
 int16_t mag_ofs_z; /*< magnetometer Z offset*/
} mavlink_sensor_offsets_t;

#define MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN 42
#define MAVLINK_MSG_ID_150_LEN 42

#define MAVLINK_MSG_ID_SENSOR_OFFSETS_CRC 134
#define MAVLINK_MSG_ID_150_CRC 134



#define MAVLINK_MESSAGE_INFO_SENSOR_OFFSETS { \
	"SENSOR_OFFSETS", \
	12, \
	{  { "mag_declination", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sensor_offsets_t, mag_declination) }, \
         { "raw_press", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_sensor_offsets_t, raw_press) }, \
         { "raw_temp", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_sensor_offsets_t, raw_temp) }, \
         { "gyro_cal_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensor_offsets_t, gyro_cal_x) }, \
         { "gyro_cal_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sensor_offsets_t, gyro_cal_y) }, \
         { "gyro_cal_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sensor_offsets_t, gyro_cal_z) }, \
         { "accel_cal_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sensor_offsets_t, accel_cal_x) }, \
         { "accel_cal_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sensor_offsets_t, accel_cal_y) }, \
         { "accel_cal_z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_sensor_offsets_t, accel_cal_z) }, \
         { "mag_ofs_x", NULL, MAVLINK_TYPE_INT16_T, 0, 36, offsetof(mavlink_sensor_offsets_t, mag_ofs_x) }, \
         { "mag_ofs_y", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_sensor_offsets_t, mag_ofs_y) }, \
         { "mag_ofs_z", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_sensor_offsets_t, mag_ofs_z) }, \
         } \
}


/**
 * @brief Pack a sensor_offsets message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 * @param mag_declination magnetic declination (radians)
 * @param raw_press raw pressure from barometer
 * @param raw_temp raw temperature from barometer
 * @param gyro_cal_x gyro X calibration
 * @param gyro_cal_y gyro Y calibration
 * @param gyro_cal_z gyro Z calibration
 * @param accel_cal_x accel X calibration
 * @param accel_cal_y accel Y calibration
 * @param accel_cal_z accel Z calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_sensor_offsets_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z);
									 
uint16_t mavlink_msg_sensor_offsets_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t mag_ofs_x,int16_t mag_ofs_y,int16_t mag_ofs_z,float mag_declination,int32_t raw_press,int32_t raw_temp,float gyro_cal_x,float gyro_cal_y,float gyro_cal_z,float accel_cal_x,float accel_cal_y,float accel_cal_z);
											 
uint16_t mavlink_msg_sensor_offsets_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_offsets_t* sensor_offsets);

uint16_t mavlink_msg_sensor_offsets_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sensor_offsets_t* sensor_offsets);


#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_sensor_offsets_send(mavlink_channel_t chan, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z);

#if MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_sensor_offsets_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z);

#endif

#endif

// MESSAGE SENSOR_OFFSETS UNPACKING


/**
 * @brief Get field mag_ofs_x from sensor_offsets message
 *
 * @return magnetometer X offset
 */
int16_t mavlink_msg_sensor_offsets_get_mag_ofs_x(const mavlink_message_t* msg);
int16_t mavlink_msg_sensor_offsets_get_mag_ofs_y(const mavlink_message_t* msg);
int16_t mavlink_msg_sensor_offsets_get_mag_ofs_z(const mavlink_message_t* msg);
float mavlink_msg_sensor_offsets_get_mag_declination(const mavlink_message_t* msg);
int32_t mavlink_msg_sensor_offsets_get_raw_press(const mavlink_message_t* msg);
int32_t mavlink_msg_sensor_offsets_get_raw_temp(const mavlink_message_t* msg);
float mavlink_msg_sensor_offsets_get_gyro_cal_x(const mavlink_message_t* msg);
float mavlink_msg_sensor_offsets_get_gyro_cal_y(const mavlink_message_t* msg);
float mavlink_msg_sensor_offsets_get_gyro_cal_z(const mavlink_message_t* msg);
float mavlink_msg_sensor_offsets_get_accel_cal_x(const mavlink_message_t* msg);
float mavlink_msg_sensor_offsets_get_accel_cal_y(const mavlink_message_t* msg);
float mavlink_msg_sensor_offsets_get_accel_cal_z(const mavlink_message_t* msg);
void mavlink_msg_sensor_offsets_decode(const mavlink_message_t* msg, mavlink_sensor_offsets_t* sensor_offsets);

#endif /*_x_h_*/
