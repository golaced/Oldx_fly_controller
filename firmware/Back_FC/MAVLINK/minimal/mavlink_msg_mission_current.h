// MESSAGE MISSION_CURRENT PACKING

#ifndef _MAVLINK_MSG_MISSION_CURRENT_H_
#define _MAVLINK_MSG_MISSION_CURRENT_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_MISSION_CURRENT 42

typedef struct __mavlink_mission_current_t
{
 uint16_t seq; /*< Sequence*/
} mavlink_mission_current_t;

#define MAVLINK_MSG_ID_MISSION_CURRENT_LEN 2
#define MAVLINK_MSG_ID_42_LEN 2

#define MAVLINK_MSG_ID_MISSION_CURRENT_CRC 28
#define MAVLINK_MSG_ID_42_CRC 28



#define MAVLINK_MESSAGE_INFO_MISSION_CURRENT { \
	"MISSION_CURRENT", \
	1, \
	{  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mission_current_t, seq) }, \
         } \
}


/**
 * @brief Pack a mission_current message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_mission_current_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t seq);
uint16_t mavlink_msg_mission_current_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t seq);
uint16_t mavlink_msg_mission_current_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_current_t* mission_current);

uint16_t mavlink_msg_mission_current_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_current_t* mission_current);


#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_mission_current_send(mavlink_channel_t chan, uint16_t seq);

#if MAVLINK_MSG_ID_MISSION_CURRENT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_mission_current_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t seq);
#endif

#endif

// MESSAGE MISSION_CURRENT UNPACKING


/**
 * @brief Get field seq from mission_current message
 *
 * @return Sequence
 */
uint16_t mavlink_msg_mission_current_get_seq(const mavlink_message_t* msg);
void mavlink_msg_mission_current_decode(const mavlink_message_t* msg, mavlink_mission_current_t* mission_current);

#endif /*_xx_h_*/

