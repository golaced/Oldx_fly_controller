// MESSAGE HWSTATUS PACKING

#ifndef _MAVLINK_MSG_HWSTATUS_H_
#define _MAVLINK_MSG_HWSTATUS_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_HWSTATUS 165

typedef struct __mavlink_hwstatus_t
{
 uint16_t Vcc; /*< board voltage (mV)*/
 uint8_t I2Cerr; /*< I2C error count*/
} mavlink_hwstatus_t;

#define MAVLINK_MSG_ID_HWSTATUS_LEN 3
#define MAVLINK_MSG_ID_165_LEN 3

#define MAVLINK_MSG_ID_HWSTATUS_CRC 21
#define MAVLINK_MSG_ID_165_CRC 21



#define MAVLINK_MESSAGE_INFO_HWSTATUS { \
	"HWSTATUS", \
	2, \
	{  { "Vcc", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_hwstatus_t, Vcc) }, \
         { "I2Cerr", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_hwstatus_t, I2Cerr) }, \
         } \
}


/**
 * @brief Pack a hwstatus message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Vcc board voltage (mV)
 * @param I2Cerr I2C error count
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_hwstatus_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t Vcc, uint8_t I2Cerr);
uint16_t mavlink_msg_hwstatus_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t Vcc,uint8_t I2Cerr);
uint16_t mavlink_msg_hwstatus_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hwstatus_t* hwstatus);
uint16_t mavlink_msg_hwstatus_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hwstatus_t* hwstatus);

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_hwstatus_send(mavlink_channel_t chan, uint16_t Vcc, uint8_t I2Cerr);

#if MAVLINK_MSG_ID_HWSTATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
void mavlink_msg_hwstatus_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t Vcc, uint8_t I2Cerr);
#endif

#endif

// MESSAGE HWSTATUS UNPACKING

uint16_t mavlink_msg_hwstatus_get_Vcc(const mavlink_message_t* msg);
uint8_t mavlink_msg_hwstatus_get_I2Cerr(const mavlink_message_t* msg);
void mavlink_msg_hwstatus_decode(const mavlink_message_t* msg, mavlink_hwstatus_t* hwstatus);

#endif /*_xx_h_*/

