// MESSAGE ATTITUDE PACKING

#ifndef _MAVLINK_MSG_ATTITUDE_H_
#define _MAVLINK_MSG_ATTITUDE_H_

#include "../mavlink_helpers.h" 


#define MAVLINK_MSG_ID_ATTITUDE 30

typedef struct __mavlink_attitude_t
{
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 float roll; /*< Roll angle (rad, -pi..+pi)*/
 float pitch; /*< Pitch angle (rad, -pi..+pi)*/
 float yaw; /*< Yaw angle (rad, -pi..+pi)*/
 float rollspeed; /*< Roll angular speed (rad/s)*/
 float pitchspeed; /*< Pitch angular speed (rad/s)*/
 float yawspeed; /*< Yaw angular speed (rad/s)*/
} mavlink_attitude_t;

#define MAVLINK_MSG_ID_ATTITUDE_LEN 28
#define MAVLINK_MSG_ID_30_LEN 28

#define MAVLINK_MSG_ID_ATTITUDE_CRC 39
#define MAVLINK_MSG_ID_30_CRC 39



#define MAVLINK_MESSAGE_INFO_ATTITUDE { \
	"ATTITUDE", \
	7, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_attitude_t, time_boot_ms) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_attitude_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_t, yawspeed) }, \
         } \
}


/**
 * @brief Pack a attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);
uint16_t mavlink_msg_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,float roll,float pitch,float yaw,float rollspeed,float pitchspeed,float yawspeed);
uint16_t mavlink_msg_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_t* attitude);
uint16_t mavlink_msg_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_attitude_t* attitude);

/**
 * @brief Send a attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_attitude_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);

#if MAVLINK_MSG_ID_ATTITUDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
void mavlink_msg_attitude_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);
#endif
#endif

// MESSAGE ATTITUDE UNPACKING


/**
 * @brief Get field time_boot_ms from attitude message
 *
 * @return Timestamp (milliseconds since system boot)
 */
uint32_t mavlink_msg_attitude_get_time_boot_ms(const mavlink_message_t* msg);
float mavlink_msg_attitude_get_roll(const mavlink_message_t* msg);
float mavlink_msg_attitude_get_pitch(const mavlink_message_t* msg);
float mavlink_msg_attitude_get_yaw(const mavlink_message_t* msg);
float mavlink_msg_attitude_get_rollspeed(const mavlink_message_t* msg);
float mavlink_msg_attitude_get_pitchspeed(const mavlink_message_t* msg);
float mavlink_msg_attitude_get_yawspeed(const mavlink_message_t* msg);
void mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude);


#endif /*_xx_h_*/

