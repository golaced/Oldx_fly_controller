// MESSAGE MEMINFO PACKING

#ifndef _MAVLINK_MSG_MEMINFO_H_
#define _MAVLINK_MSG_MEMINFO_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_MEMINFO 152

typedef struct __mavlink_meminfo_t
{
 uint16_t brkval; /*< heap top*/
 uint16_t freemem; /*< free memory*/
} mavlink_meminfo_t;

#define MAVLINK_MSG_ID_MEMINFO_LEN 4
#define MAVLINK_MSG_ID_152_LEN 4

#define MAVLINK_MSG_ID_MEMINFO_CRC 208
#define MAVLINK_MSG_ID_152_CRC 208



#define MAVLINK_MESSAGE_INFO_MEMINFO { \
	"MEMINFO", \
	2, \
	{  { "brkval", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_meminfo_t, brkval) }, \
         { "freemem", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_meminfo_t, freemem) }, \
         } \
}


/**
 * @brief Pack a meminfo message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param brkval heap top
 * @param freemem free memory
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_meminfo_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t brkval, uint16_t freemem);
uint16_t mavlink_msg_meminfo_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t brkval,uint16_t freemem);
uint16_t mavlink_msg_meminfo_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_meminfo_t* meminfo);
uint16_t mavlink_msg_meminfo_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_meminfo_t* meminfo);

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_meminfo_send(mavlink_channel_t chan, uint16_t brkval, uint16_t freemem);
#if MAVLINK_MSG_ID_MEMINFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_meminfo_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t brkval, uint16_t freemem);
#endif

#endif

// MESSAGE MEMINFO UNPACKING


/**
 * @brief Get field brkval from meminfo message
 *
 * @return heap top
 */
uint16_t mavlink_msg_meminfo_get_brkval(const mavlink_message_t* msg);
uint16_t mavlink_msg_meminfo_get_freemem(const mavlink_message_t* msg);
void mavlink_msg_meminfo_decode(const mavlink_message_t* msg, mavlink_meminfo_t* meminfo);

#endif /*_xx_h_*/

