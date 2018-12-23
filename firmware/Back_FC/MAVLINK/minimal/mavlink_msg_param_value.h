// MESSAGE PARAM_VALUE PACKING

#ifndef _MAVLINK_MSG_PARAM_VALUE_H_
#define _MAVLINK_MSG_PARAM_VALUE_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_PARAM_VALUE 22

typedef struct __mavlink_param_value_t
{
 float param_value; /*< Onboard parameter value*/
 uint16_t param_count; /*< Total number of onboard parameters*/
 uint16_t param_index; /*< Index of this onboard parameter*/
 char param_id[16]; /*< Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string*/
 uint8_t param_type; /*< Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.*/
} mavlink_param_value_t;

#define MAVLINK_MSG_ID_PARAM_VALUE_LEN 25
#define MAVLINK_MSG_ID_22_LEN 25

#define MAVLINK_MSG_ID_PARAM_VALUE_CRC 220
#define MAVLINK_MSG_ID_22_CRC 220

#define MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN 16

#define MAVLINK_MESSAGE_INFO_PARAM_VALUE { \
	"PARAM_VALUE", \
	5, \
	{  { "param_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_param_value_t, param_value) }, \
         { "param_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_param_value_t, param_count) }, \
         { "param_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_param_value_t, param_index) }, \
         { "param_id", NULL, MAVLINK_TYPE_CHAR, 16, 8, offsetof(mavlink_param_value_t, param_id) }, \
         { "param_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_param_value_t, param_type) }, \
         } \
}


/**
 * @brief Pack a param_value message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param_id Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
 * @param param_count Total number of onboard parameters
 * @param param_index Index of this onboard parameter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_param_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index);
uint16_t mavlink_msg_param_value_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const char *param_id,float param_value,uint8_t param_type,uint16_t param_count,uint16_t param_index);
uint16_t mavlink_msg_param_value_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_value_t* param_value);
uint16_t mavlink_msg_param_value_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_param_value_t* param_value);

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_param_value_send(mavlink_channel_t chan, const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index);

#if MAVLINK_MSG_ID_PARAM_VALUE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_param_value_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index);
#endif

#endif

// MESSAGE PARAM_VALUE UNPACKING


/**
 * @brief Get field param_id from param_value message
 *
 * @return Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 */
uint16_t mavlink_msg_param_value_get_param_id(const mavlink_message_t* msg, char *param_id);
float mavlink_msg_param_value_get_param_value(const mavlink_message_t* msg);
uint8_t mavlink_msg_param_value_get_param_type(const mavlink_message_t* msg);
uint16_t mavlink_msg_param_value_get_param_count(const mavlink_message_t* msg);
uint16_t mavlink_msg_param_value_get_param_index(const mavlink_message_t* msg);
void mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value);

#endif /*_xx_h_*/

