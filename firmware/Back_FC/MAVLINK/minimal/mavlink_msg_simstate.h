// MESSAGE SIMSTATE PACKING

#ifndef _MAVLINK_MSG_SIMSTATE_H_
#define _MAVLINK_MSG_SIMSTATE_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_SIMSTATE 164

typedef struct __mavlink_simstate_t
{
 float roll; /*< Roll angle (rad)*/
 float pitch; /*< Pitch angle (rad)*/
 float yaw; /*< Yaw angle (rad)*/
 float xacc; /*< X acceleration m/s/s*/
 float yacc; /*< Y acceleration m/s/s*/
 float zacc; /*< Z acceleration m/s/s*/
 float xgyro; /*< Angular speed around X axis rad/s*/
 float ygyro; /*< Angular speed around Y axis rad/s*/
 float zgyro; /*< Angular speed around Z axis rad/s*/
 int32_t lat; /*< Latitude in degrees * 1E7*/
 int32_t lng; /*< Longitude in degrees * 1E7*/
} mavlink_simstate_t;

#define MAVLINK_MSG_ID_SIMSTATE_LEN 44
#define MAVLINK_MSG_ID_164_LEN 44

#define MAVLINK_MSG_ID_SIMSTATE_CRC 154
#define MAVLINK_MSG_ID_164_CRC 154



#define MAVLINK_MESSAGE_INFO_SIMSTATE { \
	"SIMSTATE", \
	11, \
	{  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_simstate_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_simstate_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_simstate_t, yaw) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_simstate_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_simstate_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_simstate_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_simstate_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_simstate_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_simstate_t, zgyro) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_simstate_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_simstate_t, lng) }, \
         } \
}


/**
 * @brief Pack a simstate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param xacc X acceleration m/s/s
 * @param yacc Y acceleration m/s/s
 * @param zacc Z acceleration m/s/s
 * @param xgyro Angular speed around X axis rad/s
 * @param ygyro Angular speed around Y axis rad/s
 * @param zgyro Angular speed around Z axis rad/s
 * @param lat Latitude in degrees * 1E7
 * @param lng Longitude in degrees * 1E7
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_simstate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng);
uint16_t mavlink_msg_simstate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float roll,float pitch,float yaw,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,int32_t lat,int32_t lng);
uint16_t mavlink_msg_simstate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_simstate_t* simstate);
uint16_t mavlink_msg_simstate_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_simstate_t* simstate);

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_simstate_send(mavlink_channel_t chan, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng);

#if MAVLINK_MSG_ID_SIMSTATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_simstate_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng);

#endif

#endif

// MESSAGE SIMSTATE UNPACKING


/**
 * @brief Get field roll from simstate message
 *
 * @return Roll angle (rad)
 */
float mavlink_msg_simstate_get_roll(const mavlink_message_t* msg);
float mavlink_msg_simstate_get_pitch(const mavlink_message_t* msg);
float mavlink_msg_simstate_get_yaw(const mavlink_message_t* msg);
float mavlink_msg_simstate_get_xacc(const mavlink_message_t* msg);
float mavlink_msg_simstate_get_yacc(const mavlink_message_t* msg);
float mavlink_msg_simstate_get_zacc(const mavlink_message_t* msg);
float mavlink_msg_simstate_get_xgyro(const mavlink_message_t* msg);
float mavlink_msg_simstate_get_ygyro(const mavlink_message_t* msg);
float mavlink_msg_simstate_get_zgyro(const mavlink_message_t* msg);
int32_t mavlink_msg_simstate_get_lat(const mavlink_message_t* msg);
int32_t mavlink_msg_simstate_get_lng(const mavlink_message_t* msg);
void mavlink_msg_simstate_decode(const mavlink_message_t* msg, mavlink_simstate_t* simstate);

#endif /*_x_h_*/

