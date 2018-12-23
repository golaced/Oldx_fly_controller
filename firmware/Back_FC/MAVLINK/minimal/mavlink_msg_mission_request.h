// MESSAGE MISSION_REQUEST PACKING

#ifndef _MAVLINK_MSG_MISSION_REQUEST_H_
#define _MAVLINK_MSG_MISSION_REQUEST_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_MISSION_REQUEST 40

typedef struct __mavlink_mission_request_t
{
 uint16_t seq; /*< Sequence*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
} mavlink_mission_request_t;

#define MAVLINK_MSG_ID_MISSION_REQUEST_LEN 4
#define MAVLINK_MSG_ID_40_LEN 4

#define MAVLINK_MSG_ID_MISSION_REQUEST_CRC 230
#define MAVLINK_MSG_ID_40_CRC 230



#define MAVLINK_MESSAGE_INFO_MISSION_REQUEST { \
	"MISSION_REQUEST", \
	3, \
	{  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mission_request_t, seq) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mission_request_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mission_request_t, target_component) }, \
         } \
}


/**
 * @brief Pack a mission_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_mission_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint16_t seq);
uint16_t mavlink_msg_mission_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint16_t seq);
uint16_t mavlink_msg_mission_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_request_t* mission_request);
uint16_t mavlink_msg_mission_request_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_request_t* mission_request);

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_mission_request_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq);

#if MAVLINK_MSG_ID_MISSION_REQUEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_mission_request_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t seq);
#endif

#endif

// MESSAGE MISSION_REQUEST UNPACKING


/**
 * @brief Get field target_system from mission_request message
 *
 * @return System ID
 */
uint8_t mavlink_msg_mission_request_get_target_system(const mavlink_message_t* msg);
uint8_t mavlink_msg_mission_request_get_target_component(const mavlink_message_t* msg);
uint16_t mavlink_msg_mission_request_get_seq(const mavlink_message_t* msg);
void mavlink_msg_mission_request_decode(const mavlink_message_t* msg, mavlink_mission_request_t* mission_request);

#endif  /*_xx_h*/

