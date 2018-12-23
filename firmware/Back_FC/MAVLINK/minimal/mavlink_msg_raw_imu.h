// MESSAGE RAW_IMU PACKING

#ifndef _MAVLINK_MSG_RAW_IMU_H_
#define _MAVLINK_MSG_RAW_IMU_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_RAW_IMU 27

typedef struct __mavlink_raw_imu_t
{
 uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
 int16_t xacc; /*< X acceleration (raw)*/
 int16_t yacc; /*< Y acceleration (raw)*/
 int16_t zacc; /*< Z acceleration (raw)*/
 int16_t xgyro; /*< Angular speed around X axis (raw)*/
 int16_t ygyro; /*< Angular speed around Y axis (raw)*/
 int16_t zgyro; /*< Angular speed around Z axis (raw)*/
 int16_t xmag; /*< X Magnetic field (raw)*/
 int16_t ymag; /*< Y Magnetic field (raw)*/
 int16_t zmag; /*< Z Magnetic field (raw)*/
} mavlink_raw_imu_t;

#define MAVLINK_MSG_ID_RAW_IMU_LEN 26
#define MAVLINK_MSG_ID_27_LEN 26

#define MAVLINK_MSG_ID_RAW_IMU_CRC 144
#define MAVLINK_MSG_ID_27_CRC 144



#define MAVLINK_MESSAGE_INFO_RAW_IMU { \
	"RAW_IMU", \
	10, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_raw_imu_t, time_usec) }, \
         { "xacc", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_raw_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_raw_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_raw_imu_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_raw_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_raw_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_raw_imu_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_raw_imu_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_raw_imu_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_raw_imu_t, zmag) }, \
         } \
}


/**
 * @brief Pack a raw_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param xacc X acceleration (raw)
 * @param yacc Y acceleration (raw)
 * @param zacc Z acceleration (raw)
 * @param xgyro Angular speed around X axis (raw)
 * @param ygyro Angular speed around Y axis (raw)
 * @param zgyro Angular speed around Z axis (raw)
 * @param xmag X Magnetic field (raw)
 * @param ymag Y Magnetic field (raw)
 * @param zmag Z Magnetic field (raw)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_raw_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag);
uint16_t mavlink_msg_raw_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,int16_t xacc,int16_t yacc,int16_t zacc,int16_t xgyro,int16_t ygyro,int16_t zgyro,int16_t xmag,int16_t ymag,int16_t zmag);
uint16_t mavlink_msg_raw_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_imu_t* raw_imu);
uint16_t mavlink_msg_raw_imu_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_raw_imu_t* raw_imu);

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_raw_imu_send(mavlink_channel_t chan, uint64_t time_usec, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag);


#if MAVLINK_MSG_ID_RAW_IMU_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_raw_imu_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag);

#endif

#endif

// MESSAGE RAW_IMU UNPACKING


/**
 * @brief Get field time_usec from raw_imu message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
uint64_t mavlink_msg_raw_imu_get_time_usec(const mavlink_message_t* msg);
int16_t mavlink_msg_raw_imu_get_xacc(const mavlink_message_t* msg);
int16_t mavlink_msg_raw_imu_get_yacc(const mavlink_message_t* msg);
int16_t mavlink_msg_raw_imu_get_zacc(const mavlink_message_t* msg);
int16_t mavlink_msg_raw_imu_get_xgyro(const mavlink_message_t* msg);
int16_t mavlink_msg_raw_imu_get_ygyro(const mavlink_message_t* msg);
int16_t mavlink_msg_raw_imu_get_zgyro(const mavlink_message_t* msg);
int16_t mavlink_msg_raw_imu_get_xmag(const mavlink_message_t* msg);
int16_t mavlink_msg_raw_imu_get_ymag(const mavlink_message_t* msg);
int16_t mavlink_msg_raw_imu_get_zmag(const mavlink_message_t* msg);
void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu);

#endif /*_xx_h_*/
