// MESSAGE RC_CHANNELS_RAW PACKING

#ifndef _MAVLINK_MSG_RC_CHANNELS_RAW_H_
#define _MAVLINK_MSG_RC_CHANNELS_RAW_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW 35

typedef struct __mavlink_rc_channels_raw_t
{
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 uint16_t chan1_raw; /*< RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan2_raw; /*< RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan3_raw; /*< RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan4_raw; /*< RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan5_raw; /*< RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan6_raw; /*< RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan7_raw; /*< RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan8_raw; /*< RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint8_t port; /*< Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.*/
 uint8_t rssi; /*< Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.*/
} mavlink_rc_channels_raw_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN 22
#define MAVLINK_MSG_ID_35_LEN 22

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_CRC 244
#define MAVLINK_MSG_ID_35_CRC 244



#define MAVLINK_MESSAGE_INFO_RC_CHANNELS_RAW { \
	"RC_CHANNELS_RAW", \
	11, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_rc_channels_raw_t, time_boot_ms) }, \
         { "chan1_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_rc_channels_raw_t, chan1_raw) }, \
         { "chan2_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_rc_channels_raw_t, chan2_raw) }, \
         { "chan3_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_rc_channels_raw_t, chan3_raw) }, \
         { "chan4_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_rc_channels_raw_t, chan4_raw) }, \
         { "chan5_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_rc_channels_raw_t, chan5_raw) }, \
         { "chan6_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_rc_channels_raw_t, chan6_raw) }, \
         { "chan7_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_rc_channels_raw_t, chan7_raw) }, \
         { "chan8_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_rc_channels_raw_t, chan8_raw) }, \
         { "port", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_rc_channels_raw_t, port) }, \
         { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_rc_channels_raw_t, rssi) }, \
         } \
}


/**
 * @brief Pack a rc_channels_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
 * @param chan1_raw RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan2_raw RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan3_raw RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan4_raw RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan5_raw RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan6_raw RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan7_raw RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan8_raw RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param rssi Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_rc_channels_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi);
									 
uint16_t mavlink_msg_rc_channels_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t port,uint16_t chan1_raw,uint16_t chan2_raw,uint16_t chan3_raw,uint16_t chan4_raw,uint16_t chan5_raw,uint16_t chan6_raw,uint16_t chan7_raw,uint16_t chan8_raw,uint8_t rssi);
											 
uint16_t mavlink_msg_rc_channels_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_raw_t* rc_channels_raw);
uint16_t mavlink_msg_rc_channels_raw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rc_channels_raw_t* rc_channels_raw);
	
	
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_rc_channels_raw_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi);

#if MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_rc_channels_raw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi);
#endif

#endif

// MESSAGE RC_CHANNELS_RAW UNPACKING


/**
 * @brief Get field time_boot_ms from rc_channels_raw message
 *
 * @return Timestamp (milliseconds since system boot)
 */
uint32_t mavlink_msg_rc_channels_raw_get_time_boot_ms(const mavlink_message_t* msg);
uint8_t mavlink_msg_rc_channels_raw_get_port(const mavlink_message_t* msg);
uint16_t mavlink_msg_rc_channels_raw_get_chan1_raw(const mavlink_message_t* msg);
uint16_t mavlink_msg_rc_channels_raw_get_chan2_raw(const mavlink_message_t* msg);
uint16_t mavlink_msg_rc_channels_raw_get_chan3_raw(const mavlink_message_t* msg);
uint16_t mavlink_msg_rc_channels_raw_get_chan4_raw(const mavlink_message_t* msg);
uint16_t mavlink_msg_rc_channels_raw_get_chan5_raw(const mavlink_message_t* msg);
uint16_t mavlink_msg_rc_channels_raw_get_chan6_raw(const mavlink_message_t* msg);
uint16_t mavlink_msg_rc_channels_raw_get_chan7_raw(const mavlink_message_t* msg);
uint16_t mavlink_msg_rc_channels_raw_get_chan8_raw(const mavlink_message_t* msg);
uint8_t mavlink_msg_rc_channels_raw_get_rssi(const mavlink_message_t* msg);
void mavlink_msg_rc_channels_raw_decode(const mavlink_message_t* msg, mavlink_rc_channels_raw_t* rc_channels_raw);

#endif /**/



