// MESSAGE NAV_CONTROLLER_OUTPUT PACKING

#ifndef _MAVLINK_MSG_NAV_CONTROLLER_OUTPUT_H_
#define _MAVLINK_MSG_NAV_CONTROLLER_OUTPUT_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT 62

typedef struct __mavlink_nav_controller_output_t
{
 float nav_roll; /*< Current desired roll in degrees*/
 float nav_pitch; /*< Current desired pitch in degrees*/
 float alt_error; /*< Current altitude error in meters*/
 float aspd_error; /*< Current airspeed error in meters/second*/
 float xtrack_error; /*< Current crosstrack error on x-y plane in meters*/
 int16_t nav_bearing; /*< Current desired heading in degrees*/
 int16_t target_bearing; /*< Bearing to current MISSION/target in degrees*/
 uint16_t wp_dist; /*< Distance to active MISSION in meters*/
} mavlink_nav_controller_output_t;

#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN 26
#define MAVLINK_MSG_ID_62_LEN 26

#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_CRC 183
#define MAVLINK_MSG_ID_62_CRC 183



#define MAVLINK_MESSAGE_INFO_NAV_CONTROLLER_OUTPUT { \
	"NAV_CONTROLLER_OUTPUT", \
	8, \
	{  { "nav_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_nav_controller_output_t, nav_roll) }, \
         { "nav_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_nav_controller_output_t, nav_pitch) }, \
         { "alt_error", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_nav_controller_output_t, alt_error) }, \
         { "aspd_error", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_nav_controller_output_t, aspd_error) }, \
         { "xtrack_error", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_nav_controller_output_t, xtrack_error) }, \
         { "nav_bearing", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_nav_controller_output_t, nav_bearing) }, \
         { "target_bearing", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_nav_controller_output_t, target_bearing) }, \
         { "wp_dist", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_nav_controller_output_t, wp_dist) }, \
         } \
}


/**
 * @brief Pack a nav_controller_output message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param nav_roll Current desired roll in degrees
 * @param nav_pitch Current desired pitch in degrees
 * @param nav_bearing Current desired heading in degrees
 * @param target_bearing Bearing to current MISSION/target in degrees
 * @param wp_dist Distance to active MISSION in meters
 * @param alt_error Current altitude error in meters
 * @param aspd_error Current airspeed error in meters/second
 * @param xtrack_error Current crosstrack error on x-y plane in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_nav_controller_output_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error);
uint16_t mavlink_msg_nav_controller_output_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float nav_roll,float nav_pitch,int16_t nav_bearing,int16_t target_bearing,uint16_t wp_dist,float alt_error,float aspd_error,float xtrack_error);
uint16_t mavlink_msg_nav_controller_output_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_nav_controller_output_t* nav_controller_output);

uint16_t mavlink_msg_nav_controller_output_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_nav_controller_output_t* nav_controller_output);

/**
 * @brief Send a nav_controller_output message
 * @param chan MAVLink channel to send the message
 *
 * @param nav_roll Current desired roll in degrees
 * @param nav_pitch Current desired pitch in degrees
 * @param nav_bearing Current desired heading in degrees
 * @param target_bearing Bearing to current MISSION/target in degrees
 * @param wp_dist Distance to active MISSION in meters
 * @param alt_error Current altitude error in meters
 * @param aspd_error Current airspeed error in meters/second
 * @param xtrack_error Current crosstrack error on x-y plane in meters
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_nav_controller_output_send(mavlink_channel_t chan, float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error);


#if MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_nav_controller_output_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error);

#endif

#endif

// MESSAGE NAV_CONTROLLER_OUTPUT UNPACKING


/**
 * @brief Get field nav_roll from nav_controller_output message
 *
 * @return Current desired roll in degrees
 */
float mavlink_msg_nav_controller_output_get_nav_roll(const mavlink_message_t* msg);
float mavlink_msg_nav_controller_output_get_nav_pitch(const mavlink_message_t* msg);
int16_t mavlink_msg_nav_controller_output_get_nav_bearing(const mavlink_message_t* msg);
int16_t mavlink_msg_nav_controller_output_get_target_bearing(const mavlink_message_t* msg);
uint16_t mavlink_msg_nav_controller_output_get_wp_dist(const mavlink_message_t* msg);
float mavlink_msg_nav_controller_output_get_alt_error(const mavlink_message_t* msg);
float mavlink_msg_nav_controller_output_get_aspd_error(const mavlink_message_t* msg);
float mavlink_msg_nav_controller_output_get_xtrack_error(const mavlink_message_t* msg);
void mavlink_msg_nav_controller_output_decode(const mavlink_message_t* msg, mavlink_nav_controller_output_t* nav_controller_output);

#endif /*_xx_h_*/
