// MESSAGE AHRS PACKING

#ifndef _MAVLINK_MSG_AHRS_H_
#define _MAVLINK_MSG_AHRS_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_AHRS 163

typedef struct __mavlink_ahrs_t
{
 float omegaIx; /*< X gyro drift estimate rad/s*/
 float omegaIy; /*< Y gyro drift estimate rad/s*/
 float omegaIz; /*< Z gyro drift estimate rad/s*/
 float accel_weight; /*< average accel_weight*/
 float renorm_val; /*< average renormalisation value*/
 float error_rp; /*< average error_roll_pitch value*/
 float error_yaw; /*< average error_yaw value*/
} mavlink_ahrs_t;

#define MAVLINK_MSG_ID_AHRS_LEN 28
#define MAVLINK_MSG_ID_163_LEN 28

#define MAVLINK_MSG_ID_AHRS_CRC 127
#define MAVLINK_MSG_ID_163_CRC 127



#define MAVLINK_MESSAGE_INFO_AHRS { \
	"AHRS", \
	7, \
	{  { "omegaIx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ahrs_t, omegaIx) }, \
         { "omegaIy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ahrs_t, omegaIy) }, \
         { "omegaIz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ahrs_t, omegaIz) }, \
         { "accel_weight", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ahrs_t, accel_weight) }, \
         { "renorm_val", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ahrs_t, renorm_val) }, \
         { "error_rp", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ahrs_t, error_rp) }, \
         { "error_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ahrs_t, error_yaw) }, \
         } \
}


/**
 * @brief Pack a ahrs message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param omegaIx X gyro drift estimate rad/s
 * @param omegaIy Y gyro drift estimate rad/s
 * @param omegaIz Z gyro drift estimate rad/s
 * @param accel_weight average accel_weight
 * @param renorm_val average renormalisation value
 * @param error_rp average error_roll_pitch value
 * @param error_yaw average error_yaw value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_ahrs_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw);
uint16_t mavlink_msg_ahrs_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float omegaIx,float omegaIy,float omegaIz,float accel_weight,float renorm_val,float error_rp,float error_yaw);
uint16_t mavlink_msg_ahrs_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ahrs_t* ahrs);
uint16_t mavlink_msg_ahrs_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ahrs_t* ahrs);

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_ahrs_send(mavlink_channel_t chan, float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw);

#if MAVLINK_MSG_ID_AHRS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_ahrs_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw);

#endif

#endif

// MESSAGE AHRS UNPACKING


/**
 * @brief Get field omegaIx from ahrs message
 *
 * @return X gyro drift estimate rad/s
 */
float mavlink_msg_ahrs_get_omegaIx(const mavlink_message_t* msg);
float mavlink_msg_ahrs_get_omegaIy(const mavlink_message_t* msg);
float mavlink_msg_ahrs_get_omegaIz(const mavlink_message_t* msg);
float mavlink_msg_ahrs_get_accel_weight(const mavlink_message_t* msg);
float mavlink_msg_ahrs_get_renorm_val(const mavlink_message_t* msg);
float mavlink_msg_ahrs_get_error_rp(const mavlink_message_t* msg);
float mavlink_msg_ahrs_get_error_yaw(const mavlink_message_t* msg);
void mavlink_msg_ahrs_decode(const mavlink_message_t* msg, mavlink_ahrs_t* ahrs);

#endif /*_MAVLINK_MSG_AHRS_H_*/

