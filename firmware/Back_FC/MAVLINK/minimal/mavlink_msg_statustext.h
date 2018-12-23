
// MESSAGE STATUSTEXT PACKING

#ifndef _MAVLINK_MSG_STATUSTEXT_H_ 
#define _MAVLINK_MSG_STATUSTEXT_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_STATUSTEXT 253

typedef struct __mavlink_statustext_t
{
 uint8_t severity; /*< Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.*/
 char text[50]; /*< Status text message, without null termination character*/
} mavlink_statustext_t;

#define MAVLINK_MSG_ID_STATUSTEXT_LEN 51
#define MAVLINK_MSG_ID_253_LEN 51

#define MAVLINK_MSG_ID_STATUSTEXT_CRC 83
#define MAVLINK_MSG_ID_253_CRC 83

#define MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN 50

#define MAVLINK_MESSAGE_INFO_STATUSTEXT { \
	"STATUSTEXT", \
	2, \
	{  { "severity", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_statustext_t, severity) }, \
         { "text", NULL, MAVLINK_TYPE_CHAR, 50, 1, offsetof(mavlink_statustext_t, text) }, \
         } \
}


/**
 * @brief Pack a statustext message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param severity Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 * @param text Status text message, without null termination character
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_statustext_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t severity,  char *text);
uint16_t mavlink_msg_statustext_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t severity, char *text);
uint16_t mavlink_msg_statustext_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, mavlink_statustext_t* statustext);
uint16_t mavlink_msg_statustext_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, mavlink_statustext_t* statustext);


/**
 * @brief Send a statustext message
 * @param chan MAVLink channel to send the message
 *
 * @param severity Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 * @param text Status text message, without null termination character
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_statustext_send(mavlink_channel_t chan, uint8_t severity, char *text);

#if MAVLINK_MSG_ID_STATUSTEXT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_statustext_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t severity, char *text);
#endif

#endif

// MESSAGE STATUSTEXT UNPACKING
/**
 * @brief Get field severity from statustext message
 *
 * @return Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 */
uint8_t mavlink_msg_statustext_get_severity(const mavlink_message_t* msg);
uint16_t mavlink_msg_statustext_get_text(const mavlink_message_t* msg, char *text);
void mavlink_msg_statustext_decode(const mavlink_message_t* msg, mavlink_statustext_t* statustext);


#endif /*_MAVLINK_MSG_STATUSTEXT_H_*/


