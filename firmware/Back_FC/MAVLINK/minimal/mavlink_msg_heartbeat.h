
#ifndef _MAVLINK_MSG_HEARTBEAT_H_
#define _MAVLINK_MSG_HEARTBEAT_H_

#include "../mavlink_helpers.h"

#define MAVLINK_MSG_ID_HEARTBEAT 0

typedef struct __mavlink_heartbeat_t
{
 uint32_t custom_mode;    /*< A bitfield for use for autopilot-specific flags.*/
 uint8_t type;            /*< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)*/
 uint8_t autopilot;       /*< Autopilot type / class. defined in MAV_AUTOPILOT ENUM*/
 uint8_t base_mode;       /*< System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h*/
 uint8_t system_status;   /*< System status flag, see MAV_STATE ENUM*/
 uint8_t mavlink_version; /*< MAVLink version*/
} mavlink_heartbeat_t;

#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9
#define MAVLINK_MSG_ID_0_LEN 9

#define MAVLINK_MSG_ID_HEARTBEAT_CRC 50
#define MAVLINK_MSG_ID_0_CRC 50


#define MAVLINK_MESSAGE_INFO_HEARTBEAT { \
	"HEARTBEAT", \
	6, \
	{  { "custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_heartbeat_t, custom_mode) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_heartbeat_t, type) }, \
         { "autopilot", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_heartbeat_t, autopilot) }, \
         { "base_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_heartbeat_t, base_mode) }, \
         { "system_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_heartbeat_t, system_status) }, \
         { "mavlink_version", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_heartbeat_t, mavlink_version) }, \
         } \
}


/**
 * @brief Pack a heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
 * @param base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
 * @param custom_mode A bitfield for use for autopilot-specific flags.
 * @param system_status System status flag, see MAV_STATE ENUM
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status);
uint16_t mavlink_msg_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg,
						       uint8_t type,uint8_t autopilot,uint8_t base_mode,uint32_t custom_mode,uint8_t system_status);
uint16_t mavlink_msg_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat);
uint16_t mavlink_msg_heartbeat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat);

/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
 * @param base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
 * @param custom_mode A bitfield for use for autopilot-specific flags.
 * @param system_status System status flag, see MAV_STATE ENUM
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
void mavlink_msg_heartbeat_send(mavlink_channel_t chan, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status);
#if MAVLINK_MSG_ID_HEARTBEAT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
void mavlink_msg_heartbeat_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status);
#endif
#endif


// MESSAGE HEARTBEAT UNPACKING
uint8_t mavlink_msg_heartbeat_get_type(const mavlink_message_t* msg);
uint8_t mavlink_msg_heartbeat_get_autopilot(const mavlink_message_t* msg);
uint8_t mavlink_msg_heartbeat_get_base_mode(const mavlink_message_t* msg);
//uint32_t mavlink_msg_heartbeat_get_custom_mode(const mavlink_message_t* msg);
uint8_t mavlink_msg_heartbeat_get_system_status(const mavlink_message_t* msg);
uint8_t mavlink_msg_heartbeat_get_mavlink_version(const mavlink_message_t* msg);

/**
 * @brief Decode a heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param heartbeat C-struct to decode the message contents into
 */
void mavlink_msg_heartbeat_decode(const mavlink_message_t* msg, mavlink_heartbeat_t* heartbeat);


#endif /*_MAVLINK_MSG_HEARTBEAT_H_*/

