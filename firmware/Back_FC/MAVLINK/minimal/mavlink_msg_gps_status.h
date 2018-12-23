// MESSAGE GPS_STATUS PACKING

#ifndef _MAVLINK_MSG_GPS_STATUS_H_
#define _MAVLINK_MSG_GPS_STATUS_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_GPS_STATUS 25

typedef struct __mavlink_gps_status_t
{
 uint8_t satellites_visible; /*< Number of satellites visible*/
 uint8_t satellite_prn[20]; /*< Global satellite ID*/
 uint8_t satellite_used[20]; /*< 0: Satellite not used, 1: used for localization*/
 uint8_t satellite_elevation[20]; /*< Elevation (0: right on top of receiver, 90: on the horizon) of satellite*/
 uint8_t satellite_azimuth[20]; /*< Direction of satellite, 0: 0 deg, 255: 360 deg.*/
 uint8_t satellite_snr[20]; /*< Signal to noise ratio of satellite*/
} mavlink_gps_status_t;

#define MAVLINK_MSG_ID_GPS_STATUS_LEN 101
#define MAVLINK_MSG_ID_25_LEN 101

#define MAVLINK_MSG_ID_GPS_STATUS_CRC 23
#define MAVLINK_MSG_ID_25_CRC 23

#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_PRN_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_USED_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_ELEVATION_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_AZIMUTH_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_SNR_LEN 20

#define MAVLINK_MESSAGE_INFO_GPS_STATUS { \
	"GPS_STATUS", \
	6, \
	{  { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gps_status_t, satellites_visible) }, \
         { "satellite_prn", NULL, MAVLINK_TYPE_UINT8_T, 20, 1, offsetof(mavlink_gps_status_t, satellite_prn) }, \
         { "satellite_used", NULL, MAVLINK_TYPE_UINT8_T, 20, 21, offsetof(mavlink_gps_status_t, satellite_used) }, \
         { "satellite_elevation", NULL, MAVLINK_TYPE_UINT8_T, 20, 41, offsetof(mavlink_gps_status_t, satellite_elevation) }, \
         { "satellite_azimuth", NULL, MAVLINK_TYPE_UINT8_T, 20, 61, offsetof(mavlink_gps_status_t, satellite_azimuth) }, \
         { "satellite_snr", NULL, MAVLINK_TYPE_UINT8_T, 20, 81, offsetof(mavlink_gps_status_t, satellite_snr) }, \
         } \
}


/**
 * @brief Pack a gps_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param satellites_visible Number of satellites visible
 * @param satellite_prn Global satellite ID
 * @param satellite_used 0: Satellite not used, 1: used for localization
 * @param satellite_elevation Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 * @param satellite_azimuth Direction of satellite, 0: 0 deg, 255: 360 deg.
 * @param satellite_snr Signal to noise ratio of satellite
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_gps_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t satellites_visible, const uint8_t *satellite_prn, const uint8_t *satellite_used, const uint8_t *satellite_elevation, const uint8_t *satellite_azimuth, const uint8_t *satellite_snr);
uint16_t mavlink_msg_gps_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t satellites_visible,const uint8_t *satellite_prn,const uint8_t *satellite_used,const uint8_t *satellite_elevation,const uint8_t *satellite_azimuth,const uint8_t *satellite_snr);
uint16_t mavlink_msg_gps_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_status_t* gps_status);
uint16_t mavlink_msg_gps_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_status_t* gps_status);

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_gps_status_send(mavlink_channel_t chan, uint8_t satellites_visible, const uint8_t *satellite_prn, const uint8_t *satellite_used, const uint8_t *satellite_elevation, const uint8_t *satellite_azimuth, const uint8_t *satellite_snr);
#if MAVLINK_MSG_ID_GPS_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
void mavlink_msg_gps_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t satellites_visible, const uint8_t *satellite_prn, const uint8_t *satellite_used, const uint8_t *satellite_elevation, const uint8_t *satellite_azimuth, const uint8_t *satellite_snr);
#endif

#endif

// MESSAGE GPS_STATUS UNPACKING

uint8_t mavlink_msg_gps_status_get_satellites_visible(const mavlink_message_t* msg);
uint16_t mavlink_msg_gps_status_get_satellite_prn(const mavlink_message_t* msg, uint8_t *satellite_prn);
uint16_t mavlink_msg_gps_status_get_satellite_used(const mavlink_message_t* msg, uint8_t *satellite_used);
uint16_t mavlink_msg_gps_status_get_satellite_elevation(const mavlink_message_t* msg, uint8_t *satellite_elevation);
uint16_t mavlink_msg_gps_status_get_satellite_azimuth(const mavlink_message_t* msg, uint8_t *satellite_azimuth);
uint16_t mavlink_msg_gps_status_get_satellite_snr(const mavlink_message_t* msg, uint8_t *satellite_snr);
void mavlink_msg_gps_status_decode(const mavlink_message_t* msg, mavlink_gps_status_t* gps_status);

#endif  /*_xx_h_*/


