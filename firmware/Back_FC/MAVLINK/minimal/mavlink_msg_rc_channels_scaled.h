// MESSAGE RC_CHANNELS_SCALED PACKING

#ifndef _MAVLINK_MSG_RC_CHANNELS_SCALED_H_
#define _MAVLINK_MSG_RC_CHANNELS_SCALED_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_RC_CHANNELS_SCALED 34

typedef struct __mavlink_rc_channels_scaled_t
{
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 int16_t chan1_scaled; /*< RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.*/
 int16_t chan2_scaled; /*< RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.*/
 int16_t chan3_scaled; /*< RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.*/
 int16_t chan4_scaled; /*< RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.*/
 int16_t chan5_scaled; /*< RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.*/
 int16_t chan6_scaled; /*< RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.*/
 int16_t chan7_scaled; /*< RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.*/
 int16_t chan8_scaled; /*< RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.*/
 uint8_t port; /*< Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.*/
 uint8_t rssi; /*< Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.*/
} mavlink_rc_channels_scaled_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN 22
#define MAVLINK_MSG_ID_34_LEN 22

#define MAVLINK_MSG_ID_RC_CHANNELS_SCALED_CRC 237
#define MAVLINK_MSG_ID_34_CRC 237



#define MAVLINK_MESSAGE_INFO_RC_CHANNELS_SCALED { \
	"RC_CHANNELS_SCALED", \
	11, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_rc_channels_scaled_t, time_boot_ms) }, \
         { "chan1_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_rc_channels_scaled_t, chan1_scaled) }, \
         { "chan2_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_rc_channels_scaled_t, chan2_scaled) }, \
         { "chan3_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_rc_channels_scaled_t, chan3_scaled) }, \
         { "chan4_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_rc_channels_scaled_t, chan4_scaled) }, \
         { "chan5_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_rc_channels_scaled_t, chan5_scaled) }, \
         { "chan6_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_rc_channels_scaled_t, chan6_scaled) }, \
         { "chan7_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_rc_channels_scaled_t, chan7_scaled) }, \
         { "chan8_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_rc_channels_scaled_t, chan8_scaled) }, \
         { "port", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_rc_channels_scaled_t, port) }, \
         { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_rc_channels_scaled_t, rssi) }, \
         } \
}


/**
 * @brief Pack a rc_channels_scaled message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
 * @param chan1_scaled RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @param chan2_scaled RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @param chan3_scaled RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @param chan4_scaled RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @param chan5_scaled RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @param chan6_scaled RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @param chan7_scaled RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @param chan8_scaled RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
 * @param rssi Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_rc_channels_scaled_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t port, int16_t chan1_scaled, int16_t chan2_scaled, int16_t chan3_scaled, int16_t chan4_scaled, int16_t chan5_scaled, int16_t chan6_scaled, int16_t chan7_scaled, int16_t chan8_scaled, uint8_t rssi);
									 
uint16_t mavlink_msg_rc_channels_scaled_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t port,int16_t chan1_scaled,int16_t chan2_scaled,int16_t chan3_scaled,int16_t chan4_scaled,int16_t chan5_scaled,int16_t chan6_scaled,int16_t chan7_scaled,int16_t chan8_scaled,uint8_t rssi);
											 
uint16_t mavlink_msg_rc_channels_scaled_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_scaled_t* rc_channels_scaled);
uint16_t mavlink_msg_rc_channels_scaled_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rc_channels_scaled_t* rc_channels_scaled);


#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_rc_channels_scaled_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t port, int16_t chan1_scaled, int16_t chan2_scaled, int16_t chan3_scaled, int16_t chan4_scaled, int16_t chan5_scaled, int16_t chan6_scaled, int16_t chan7_scaled, int16_t chan8_scaled, uint8_t rssi);

#if MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_rc_channels_scaled_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t port, int16_t chan1_scaled, int16_t chan2_scaled, int16_t chan3_scaled, int16_t chan4_scaled, int16_t chan5_scaled, int16_t chan6_scaled, int16_t chan7_scaled, int16_t chan8_scaled, uint8_t rssi);
#endif

#endif

// MESSAGE RC_CHANNELS_SCALED UNPACKING


/**
 * @brief Get field time_boot_ms from rc_channels_scaled message
 *
 * @return Timestamp (milliseconds since system boot)
 */
uint32_t mavlink_msg_rc_channels_scaled_get_time_boot_ms(const mavlink_message_t* msg);
uint8_t mavlink_msg_rc_channels_scaled_get_port(const mavlink_message_t* msg);
int16_t mavlink_msg_rc_channels_scaled_get_chan1_scaled(const mavlink_message_t* msg);
int16_t mavlink_msg_rc_channels_scaled_get_chan2_scaled(const mavlink_message_t* msg);
int16_t mavlink_msg_rc_channels_scaled_get_chan3_scaled(const mavlink_message_t* msg);
int16_t mavlink_msg_rc_channels_scaled_get_chan4_scaled(const mavlink_message_t* msg);
int16_t mavlink_msg_rc_channels_scaled_get_chan5_scaled(const mavlink_message_t* msg);
int16_t mavlink_msg_rc_channels_scaled_get_chan6_scaled(const mavlink_message_t* msg);
int16_t mavlink_msg_rc_channels_scaled_get_chan7_scaled(const mavlink_message_t* msg);
int16_t mavlink_msg_rc_channels_scaled_get_chan8_scaled(const mavlink_message_t* msg);
uint8_t mavlink_msg_rc_channels_scaled_get_rssi(const mavlink_message_t* msg);
void mavlink_msg_rc_channels_scaled_decode(const mavlink_message_t* msg, mavlink_rc_channels_scaled_t* rc_channels_scaled);

#endif /*_xx_h_*/
