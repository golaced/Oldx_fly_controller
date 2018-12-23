// MESSAGE LIMITS_STATUS PACKING

#ifndef _MAVLINK_MSG_LIMITS_STATUS_H_
#define _MAVLINK_MSG_LIMITS_STATUS_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_LIMITS_STATUS 167

typedef struct __mavlink_limits_status_t
{
 uint32_t last_trigger; /*< time of last breach in milliseconds since boot*/
 uint32_t last_action; /*< time of last recovery action in milliseconds since boot*/
 uint32_t last_recovery; /*< time of last successful recovery in milliseconds since boot*/
 uint32_t last_clear; /*< time of last all-clear in milliseconds since boot*/
 uint16_t breach_count; /*< number of fence breaches*/
 uint8_t limits_state; /*< state of AP_Limits, (see enum LimitState, LIMITS_STATE)*/
 uint8_t mods_enabled; /*< AP_Limit_Module bitfield of enabled modules, (see enum moduleid or LIMIT_MODULE)*/
 uint8_t mods_required; /*< AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE)*/
 uint8_t mods_triggered; /*< AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE)*/
} mavlink_limits_status_t;

#define MAVLINK_MSG_ID_LIMITS_STATUS_LEN 22
#define MAVLINK_MSG_ID_167_LEN 22

#define MAVLINK_MSG_ID_LIMITS_STATUS_CRC 144
#define MAVLINK_MSG_ID_167_CRC 144



#define MAVLINK_MESSAGE_INFO_LIMITS_STATUS { \
	"LIMITS_STATUS", \
	9, \
	{  { "last_trigger", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_limits_status_t, last_trigger) }, \
         { "last_action", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_limits_status_t, last_action) }, \
         { "last_recovery", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_limits_status_t, last_recovery) }, \
         { "last_clear", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_limits_status_t, last_clear) }, \
         { "breach_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_limits_status_t, breach_count) }, \
         { "limits_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_limits_status_t, limits_state) }, \
         { "mods_enabled", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_limits_status_t, mods_enabled) }, \
         { "mods_required", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_limits_status_t, mods_required) }, \
         { "mods_triggered", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_limits_status_t, mods_triggered) }, \
         } \
}


/**
 * @brief Pack a limits_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param limits_state state of AP_Limits, (see enum LimitState, LIMITS_STATE)
 * @param last_trigger time of last breach in milliseconds since boot
 * @param last_action time of last recovery action in milliseconds since boot
 * @param last_recovery time of last successful recovery in milliseconds since boot
 * @param last_clear time of last all-clear in milliseconds since boot
 * @param breach_count number of fence breaches
 * @param mods_enabled AP_Limit_Module bitfield of enabled modules, (see enum moduleid or LIMIT_MODULE)
 * @param mods_required AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE)
 * @param mods_triggered AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_limits_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t limits_state, uint32_t last_trigger, uint32_t last_action, uint32_t last_recovery, uint32_t last_clear, uint16_t breach_count, uint8_t mods_enabled, uint8_t mods_required, uint8_t mods_triggered);
uint16_t mavlink_msg_limits_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t limits_state,uint32_t last_trigger,uint32_t last_action,uint32_t last_recovery,uint32_t last_clear,uint16_t breach_count,uint8_t mods_enabled,uint8_t mods_required,uint8_t mods_triggered);
uint16_t mavlink_msg_limits_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_limits_status_t* limits_status);
uint16_t mavlink_msg_limits_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_limits_status_t* limits_status);

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
void mavlink_msg_limits_status_send(mavlink_channel_t chan, uint8_t limits_state, uint32_t last_trigger, uint32_t last_action, uint32_t last_recovery, uint32_t last_clear, uint16_t breach_count, uint8_t mods_enabled, uint8_t mods_required, uint8_t mods_triggered);

#if MAVLINK_MSG_ID_LIMITS_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
void mavlink_msg_limits_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t limits_state, uint32_t last_trigger, uint32_t last_action, uint32_t last_recovery, uint32_t last_clear, uint16_t breach_count, uint8_t mods_enabled, uint8_t mods_required, uint8_t mods_triggered);
#endif

#endif

// MESSAGE LIMITS_STATUS UNPACKING


/**
 * @brief Get field limits_state from limits_status message
 *
 * @return state of AP_Limits, (see enum LimitState, LIMITS_STATE)
 */
uint8_t mavlink_msg_limits_status_get_limits_state(const mavlink_message_t* msg);
uint32_t mavlink_msg_limits_status_get_last_trigger(const mavlink_message_t* msg);
uint32_t mavlink_msg_limits_status_get_last_action(const mavlink_message_t* msg);
uint32_t mavlink_msg_limits_status_get_last_recovery(const mavlink_message_t* msg);
uint32_t mavlink_msg_limits_status_get_last_clear(const mavlink_message_t* msg);
uint16_t mavlink_msg_limits_status_get_breach_count(const mavlink_message_t* msg);
uint8_t mavlink_msg_limits_status_get_mods_enabled(const mavlink_message_t* msg);
uint8_t mavlink_msg_limits_status_get_mods_required(const mavlink_message_t* msg);
uint8_t mavlink_msg_limits_status_get_mods_triggered(const mavlink_message_t* msg);
void mavlink_msg_limits_status_decode(const mavlink_message_t* msg, mavlink_limits_status_t* limits_status);


#endif /*__xx_h_*/

