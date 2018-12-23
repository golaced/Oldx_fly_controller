// MESSAGE GLOBAL_POSITION_INT PACKING

#ifndef _MAVLINK_MSG_GLOBAL_POSITION_INT_H_
#define _MAVLINK_MSG_GLOBAL_POSITION_INT_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33

typedef struct __mavlink_global_position_int_t
{
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 int32_t lat; /*< Latitude, expressed as * 1E7*/
 int32_t lon; /*< Longitude, expressed as * 1E7*/
 int32_t alt; /*< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)*/
 int32_t relative_alt; /*< Altitude above ground in meters, expressed as * 1000 (millimeters)*/
 int16_t vx; /*< Ground X Speed (Latitude), expressed as m/s * 100*/
 int16_t vy; /*< Ground Y Speed (Longitude), expressed as m/s * 100*/
 int16_t vz; /*< Ground Z Speed (Altitude), expressed as m/s * 100*/
 uint16_t hdg; /*< Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
} mavlink_global_position_int_t;

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN 28
#define MAVLINK_MSG_ID_33_LEN 28

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_CRC 104
#define MAVLINK_MSG_ID_33_CRC 104



#define MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT { \
	"GLOBAL_POSITION_INT", \
	9, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_global_position_int_t, time_boot_ms) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_global_position_int_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_global_position_int_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_global_position_int_t, alt) }, \
         { "relative_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_global_position_int_t, relative_alt) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_global_position_int_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_global_position_int_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_global_position_int_t, vz) }, \
         { "hdg", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_global_position_int_t, hdg) }, \
         } \
}


/**
 * @brief Pack a global_position_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_global_position_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg);
uint16_t mavlink_msg_global_position_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,int32_t lat,int32_t lon,int32_t alt,int32_t relative_alt,int16_t vx,int16_t vy,int16_t vz,uint16_t hdg);
uint16_t mavlink_msg_global_position_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_int_t* global_position_int);
uint16_t mavlink_msg_global_position_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_global_position_int_t* global_position_int);


#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
void mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg);

#if MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
void mavlink_msg_global_position_int_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg);
#endif

#endif

// MESSAGE GLOBAL_POSITION_INT UNPACKING


/**
 * @brief Get field time_boot_ms from global_position_int message
 *
 * @return Timestamp (milliseconds since system boot)
 */
uint32_t mavlink_msg_global_position_int_get_time_boot_ms(const mavlink_message_t* msg);
int32_t mavlink_msg_global_position_int_get_lat(const mavlink_message_t* msg);
int32_t mavlink_msg_global_position_int_get_lon(const mavlink_message_t* msg);
int32_t mavlink_msg_global_position_int_get_alt(const mavlink_message_t* msg);
int32_t mavlink_msg_global_position_int_get_relative_alt(const mavlink_message_t* msg);
int16_t mavlink_msg_global_position_int_get_vx(const mavlink_message_t* msg);
int16_t mavlink_msg_global_position_int_get_vy(const mavlink_message_t* msg);
int16_t mavlink_msg_global_position_int_get_vz(const mavlink_message_t* msg);
uint16_t mavlink_msg_global_position_int_get_hdg(const mavlink_message_t* msg);
void mavlink_msg_global_position_int_decode(const mavlink_message_t* msg, mavlink_global_position_int_t* global_position_int);

#endif /*_xx_h_*/

