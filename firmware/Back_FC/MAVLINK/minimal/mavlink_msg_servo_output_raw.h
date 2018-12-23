// MESSAGE SERVO_OUTPUT_RAW PACKING

#ifndef _MAVLINK_MSG_SERVO_OUTPUT_RAW_H_
#define _MAVLINK_MSG_SERVO_OUTPUT_RAW_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW 36

typedef struct __mavlink_servo_output_raw_t
{
 uint32_t time_usec; /*< Timestamp (microseconds since system boot)*/
 uint16_t servo1_raw; /*< Servo output 1 value, in microseconds*/
 uint16_t servo2_raw; /*< Servo output 2 value, in microseconds*/
 uint16_t servo3_raw; /*< Servo output 3 value, in microseconds*/
 uint16_t servo4_raw; /*< Servo output 4 value, in microseconds*/
 uint16_t servo5_raw; /*< Servo output 5 value, in microseconds*/
 uint16_t servo6_raw; /*< Servo output 6 value, in microseconds*/
 uint16_t servo7_raw; /*< Servo output 7 value, in microseconds*/
 uint16_t servo8_raw; /*< Servo output 8 value, in microseconds*/
 uint8_t port; /*< Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.*/
} mavlink_servo_output_raw_t;

#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN 21
#define MAVLINK_MSG_ID_36_LEN 21

#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_CRC 222
#define MAVLINK_MSG_ID_36_CRC 222



#define MAVLINK_MESSAGE_INFO_SERVO_OUTPUT_RAW { \
	"SERVO_OUTPUT_RAW", \
	10, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_servo_output_raw_t, time_usec) }, \
         { "servo1_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_servo_output_raw_t, servo1_raw) }, \
         { "servo2_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_servo_output_raw_t, servo2_raw) }, \
         { "servo3_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_servo_output_raw_t, servo3_raw) }, \
         { "servo4_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_servo_output_raw_t, servo4_raw) }, \
         { "servo5_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_servo_output_raw_t, servo5_raw) }, \
         { "servo6_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_servo_output_raw_t, servo6_raw) }, \
         { "servo7_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_servo_output_raw_t, servo7_raw) }, \
         { "servo8_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_servo_output_raw_t, servo8_raw) }, \
         { "port", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_servo_output_raw_t, port) }, \
         } \
}


/**
 * @brief Pack a servo_output_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
 * @param servo1_raw Servo output 1 value, in microseconds
 * @param servo2_raw Servo output 2 value, in microseconds
 * @param servo3_raw Servo output 3 value, in microseconds
 * @param servo4_raw Servo output 4 value, in microseconds
 * @param servo5_raw Servo output 5 value, in microseconds
 * @param servo6_raw Servo output 6 value, in microseconds
 * @param servo7_raw Servo output 7 value, in microseconds
 * @param servo8_raw Servo output 8 value, in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_servo_output_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_usec, uint8_t port, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw);
									 
uint16_t mavlink_msg_servo_output_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_usec,uint8_t port,uint16_t servo1_raw,uint16_t servo2_raw,uint16_t servo3_raw,uint16_t servo4_raw,uint16_t servo5_raw,uint16_t servo6_raw,uint16_t servo7_raw,uint16_t servo8_raw);
											 
uint16_t mavlink_msg_servo_output_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_servo_output_raw_t* servo_output_raw);

uint16_t mavlink_msg_servo_output_raw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_servo_output_raw_t* servo_output_raw);


#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_servo_output_raw_send(mavlink_channel_t chan, uint32_t time_usec, uint8_t port, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw);

#if MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_servo_output_raw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_usec, uint8_t port, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw);

#endif

#endif

// MESSAGE SERVO_OUTPUT_RAW UNPACKING


/**
 * @brief Get field time_usec from servo_output_raw message
 *
 * @return Timestamp (microseconds since system boot)
 */
uint32_t mavlink_msg_servo_output_raw_get_time_usec(const mavlink_message_t* msg);
uint8_t mavlink_msg_servo_output_raw_get_port(const mavlink_message_t* msg);
uint16_t mavlink_msg_servo_output_raw_get_servo1_raw(const mavlink_message_t* msg);
uint16_t mavlink_msg_servo_output_raw_get_servo2_raw(const mavlink_message_t* msg);
uint16_t mavlink_msg_servo_output_raw_get_servo3_raw(const mavlink_message_t* msg);
uint16_t mavlink_msg_servo_output_raw_get_servo4_raw(const mavlink_message_t* msg);
uint16_t mavlink_msg_servo_output_raw_get_servo5_raw(const mavlink_message_t* msg);
uint16_t mavlink_msg_servo_output_raw_get_servo6_raw(const mavlink_message_t* msg);
uint16_t mavlink_msg_servo_output_raw_get_servo7_raw(const mavlink_message_t* msg);
uint16_t mavlink_msg_servo_output_raw_get_servo8_raw(const mavlink_message_t* msg);
void mavlink_msg_servo_output_raw_decode(const mavlink_message_t* msg, mavlink_servo_output_raw_t* servo_output_raw);

#endif /*_xx_h_*/

