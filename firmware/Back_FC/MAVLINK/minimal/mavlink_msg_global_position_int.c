// MESSAGE GLOBAL_POSITION_INT PACKING
#include "mavlink_msg_global_position_int.h"


/**
 * @brief Pack a global_position_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_global_position_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_int16_t(buf, 20, vx);
	_mav_put_int16_t(buf, 22, vy);
	_mav_put_int16_t(buf, 24, vz);
	_mav_put_uint16_t(buf, 26, hdg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
#else
	mavlink_global_position_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
#endif
}

/**
 * @brief Pack a global_position_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_global_position_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,int32_t lat,int32_t lon,int32_t alt,int32_t relative_alt,int16_t vx,int16_t vy,int16_t vz,uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_int16_t(buf, 20, vx);
	_mav_put_int16_t(buf, 22, vy);
	_mav_put_int16_t(buf, 24, vz);
	_mav_put_uint16_t(buf, 26, hdg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
#else
	mavlink_global_position_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
#endif
}

/**
 * @brief Encode a global_position_int struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position_int C-struct to read the message contents from
 */
uint16_t mavlink_msg_global_position_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_int_t* global_position_int)
{
	return mavlink_msg_global_position_int_pack(system_id, component_id, msg, global_position_int->time_boot_ms, global_position_int->lat, global_position_int->lon, global_position_int->alt, global_position_int->relative_alt, global_position_int->vx, global_position_int->vy, global_position_int->vz, global_position_int->hdg);
}

/**
 * @brief Encode a global_position_int struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param global_position_int C-struct to read the message contents from
 */
uint16_t mavlink_msg_global_position_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_global_position_int_t* global_position_int)
{
	return mavlink_msg_global_position_int_pack_chan(system_id, component_id, chan, msg, global_position_int->time_boot_ms, global_position_int->lat, global_position_int->lon, global_position_int->alt, global_position_int->relative_alt, global_position_int->vx, global_position_int->vy, global_position_int->vz, global_position_int->hdg);
}

/**
 * @brief Send a global_position_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_int16_t(buf, 20, vx);
	_mav_put_int16_t(buf, 22, vy);
	_mav_put_int16_t(buf, 24, vz);
	_mav_put_uint16_t(buf, 26, hdg);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
#endif
#else
	mavlink_global_position_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_global_position_int_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_int16_t(buf, 20, vx);
	_mav_put_int16_t(buf, 22, vy);
	_mav_put_int16_t(buf, 24, vz);
	_mav_put_uint16_t(buf, 26, hdg);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
#endif
#else
	mavlink_global_position_int_t *packet = (mavlink_global_position_int_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->lat = lat;
	packet->lon = lon;
	packet->alt = alt;
	packet->relative_alt = relative_alt;
	packet->vx = vx;
	packet->vy = vy;
	packet->vz = vz;
	packet->hdg = hdg;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, (const char *)packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, (const char *)packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GLOBAL_POSITION_INT UNPACKING


/**
 * @brief Get field time_boot_ms from global_position_int message
 *
 * @return Timestamp (milliseconds since system boot)
 */
uint32_t mavlink_msg_global_position_int_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lat from global_position_int message
 *
 * @return Latitude, expressed as * 1E7
 */
int32_t mavlink_msg_global_position_int_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from global_position_int message
 *
 * @return Longitude, expressed as * 1E7
 */
int32_t mavlink_msg_global_position_int_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from global_position_int message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 */
int32_t mavlink_msg_global_position_int_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field relative_alt from global_position_int message
 *
 * @return Altitude above ground in meters, expressed as * 1000 (millimeters)
 */
int32_t mavlink_msg_global_position_int_get_relative_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field vx from global_position_int message
 *
 * @return Ground X Speed (Latitude), expressed as m/s * 100
 */
int16_t mavlink_msg_global_position_int_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field vy from global_position_int message
 *
 * @return Ground Y Speed (Longitude), expressed as m/s * 100
 */
int16_t mavlink_msg_global_position_int_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field vz from global_position_int message
 *
 * @return Ground Z Speed (Altitude), expressed as m/s * 100
 */
int16_t mavlink_msg_global_position_int_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field hdg from global_position_int message
 *
 * @return Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
uint16_t mavlink_msg_global_position_int_get_hdg(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Decode a global_position_int message into a struct
 *
 * @param msg The message to decode
 * @param global_position_int C-struct to decode the message contents into
 */
void mavlink_msg_global_position_int_decode(const mavlink_message_t* msg, mavlink_global_position_int_t* global_position_int)
{
#if MAVLINK_NEED_BYTE_SWAP
	global_position_int->time_boot_ms = mavlink_msg_global_position_int_get_time_boot_ms(msg);
	global_position_int->lat = mavlink_msg_global_position_int_get_lat(msg);
	global_position_int->lon = mavlink_msg_global_position_int_get_lon(msg);
	global_position_int->alt = mavlink_msg_global_position_int_get_alt(msg);
	global_position_int->relative_alt = mavlink_msg_global_position_int_get_relative_alt(msg);
	global_position_int->vx = mavlink_msg_global_position_int_get_vx(msg);
	global_position_int->vy = mavlink_msg_global_position_int_get_vy(msg);
	global_position_int->vz = mavlink_msg_global_position_int_get_vz(msg);
	global_position_int->hdg = mavlink_msg_global_position_int_get_hdg(msg);
#else
	memcpy(global_position_int, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
#endif
}
