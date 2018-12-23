// MESSAGE GPS_RAW_INT PACKING

#ifndef _MAVLINK_MSG_GPS_RAW_INT_H_
#define _MAVLINK_MSG_GPS_RAW_INT_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_GPS_RAW_INT 24

typedef struct __mavlink_gps_raw_int_t
{
 uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
 int32_t lat; /*< Latitude (WGS84), in degrees * 1E7*/
 int32_t lon; /*< Longitude (WGS84), in degrees * 1E7*/
 int32_t alt; /*< Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.*/
 uint16_t eph; /*< GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX*/
 uint16_t epv; /*< GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX*/
 uint16_t vel; /*< GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX*/
 uint16_t cog; /*< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
 uint8_t fix_type; /*< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
 uint8_t satellites_visible; /*< Number of satellites visible. If unknown, set to 255*/
} mavlink_gps_raw_int_t;

#define MAVLINK_MSG_ID_GPS_RAW_INT_LEN 30
#define MAVLINK_MSG_ID_24_LEN 30

#define MAVLINK_MSG_ID_GPS_RAW_INT_CRC 24
#define MAVLINK_MSG_ID_24_CRC 24



#define MAVLINK_MESSAGE_INFO_GPS_RAW_INT { \
	"GPS_RAW_INT", \
	10, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps_raw_int_t, time_usec) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gps_raw_int_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gps_raw_int_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_gps_raw_int_t, alt) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_gps_raw_int_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_gps_raw_int_t, epv) }, \
         { "vel", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_gps_raw_int_t, vel) }, \
         { "cog", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_gps_raw_int_t, cog) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_gps_raw_int_t, fix_type) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_gps_raw_int_t, satellites_visible) }, \
         } \
}


/**
 * @brief Pack a gps_raw_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param epv GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_gps_raw_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible);
uint16_t mavlink_msg_gps_raw_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,uint16_t cog,uint8_t satellites_visible);
uint16_t mavlink_msg_gps_raw_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_raw_int_t* gps_raw_int);
uint16_t mavlink_msg_gps_raw_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_raw_int_t* gps_raw_int);

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_gps_raw_int_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible);

#if MAVLINK_MSG_ID_GPS_RAW_INT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
void mavlink_msg_gps_raw_int_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible);
#endif

#endif

// MESSAGE GPS_RAW_INT UNPACKING

uint64_t mavlink_msg_gps_raw_int_get_time_usec(const mavlink_message_t* msg);
uint8_t mavlink_msg_gps_raw_int_get_fix_type(const mavlink_message_t* msg);
int32_t mavlink_msg_gps_raw_int_get_lat(const mavlink_message_t* msg);
int32_t mavlink_msg_gps_raw_int_get_lon(const mavlink_message_t* msg);
int32_t mavlink_msg_gps_raw_int_get_alt(const mavlink_message_t* msg);
uint16_t mavlink_msg_gps_raw_int_get_eph(const mavlink_message_t* msg);
uint16_t mavlink_msg_gps_raw_int_get_epv(const mavlink_message_t* msg);
uint16_t mavlink_msg_gps_raw_int_get_vel(const mavlink_message_t* msg);
uint16_t mavlink_msg_gps_raw_int_get_cog(const mavlink_message_t* msg);
uint8_t mavlink_msg_gps_raw_int_get_satellites_visible(const mavlink_message_t* msg);
void mavlink_msg_gps_raw_int_decode(const mavlink_message_t* msg, mavlink_gps_raw_int_t* gps_raw_int);


#endif /*_xx_h_*/

