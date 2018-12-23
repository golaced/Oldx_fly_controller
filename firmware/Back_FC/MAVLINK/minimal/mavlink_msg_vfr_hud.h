// MESSAGE VFR_HUD PACKING

#ifndef _MAVLINK_MSG_VFR_HUD_H_
#define _MAVLINK_MSG_VFR_HUD_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_VFR_HUD 74

typedef struct __mavlink_vfr_hud_t
{
 float airspeed; /*< Current airspeed in m/s*/
 float groundspeed; /*< Current ground speed in m/s*/
 float alt; /*< Current altitude (MSL), in meters*/
 float climb; /*< Current climb rate in meters/second*/
 int16_t heading; /*< Current heading in degrees, in compass units (0..360, 0=north)*/
 uint16_t throttle; /*< Current throttle setting in integer percent, 0 to 100*/
} mavlink_vfr_hud_t;

#define MAVLINK_MSG_ID_VFR_HUD_LEN 20
#define MAVLINK_MSG_ID_74_LEN 20

#define MAVLINK_MSG_ID_VFR_HUD_CRC 20
#define MAVLINK_MSG_ID_74_CRC 20



#define MAVLINK_MESSAGE_INFO_VFR_HUD { \
	"VFR_HUD", \
	6, \
	{  { "airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_vfr_hud_t, airspeed) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_vfr_hud_t, groundspeed) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vfr_hud_t, alt) }, \
         { "climb", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vfr_hud_t, climb) }, \
         { "heading", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_vfr_hud_t, heading) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_vfr_hud_t, throttle) }, \
         } \
}


/**
 * @brief Pack a vfr_hud message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param airspeed Current airspeed in m/s
 * @param groundspeed Current ground speed in m/s
 * @param heading Current heading in degrees, in compass units (0..360, 0=north)
 * @param throttle Current throttle setting in integer percent, 0 to 100
 * @param alt Current altitude (MSL), in meters
 * @param climb Current climb rate in meters/second
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_vfr_hud_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb);
uint16_t mavlink_msg_vfr_hud_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float airspeed,float groundspeed,int16_t heading,uint16_t throttle,float alt,float climb);
uint16_t mavlink_msg_vfr_hud_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vfr_hud_t* vfr_hud);
uint16_t mavlink_msg_vfr_hud_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vfr_hud_t* vfr_hud);

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_vfr_hud_send(mavlink_channel_t chan, float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb);

#if MAVLINK_MSG_ID_VFR_HUD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_vfr_hud_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb);
#endif

#endif

// MESSAGE VFR_HUD UNPACKING


/**
 * @brief Get field airspeed from vfr_hud message
 *
 * @return Current airspeed in m/s
 */
float mavlink_msg_vfr_hud_get_airspeed(const mavlink_message_t* msg);
float mavlink_msg_vfr_hud_get_groundspeed(const mavlink_message_t* msg);
int16_t mavlink_msg_vfr_hud_get_heading(const mavlink_message_t* msg);
uint16_t mavlink_msg_vfr_hud_get_throttle(const mavlink_message_t* msg);
float mavlink_msg_vfr_hud_get_alt(const mavlink_message_t* msg);
float mavlink_msg_vfr_hud_get_climb(const mavlink_message_t* msg);
void mavlink_msg_vfr_hud_decode(const mavlink_message_t* msg, mavlink_vfr_hud_t* vfr_hud);

#endif /*_x_h_*/

