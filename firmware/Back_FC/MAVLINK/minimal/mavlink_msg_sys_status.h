// MESSAGE SYS_STATUS PACKING

#ifndef _MAVLINK_MSG_SYS_STATUS_H_
#define _MAVLINK_MSG_SYS_STATUS_H_

#include "../mavlink_helpers.h" 

#define MAVLINK_MSG_ID_SYS_STATUS 1

typedef struct __mavlink_sys_status_t
{
 uint32_t onboard_control_sensors_present; /*< Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
 uint32_t onboard_control_sensors_enabled; /*< Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
 uint32_t onboard_control_sensors_health; /*< Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
 uint16_t load; /*< Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000*/
 uint16_t voltage_battery; /*< Battery voltage, in millivolts (1 = 1 millivolt)*/
 int16_t current_battery; /*< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current*/
 uint16_t drop_rate_comm; /*< Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
 uint16_t errors_comm; /*< Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
 uint16_t errors_count1; /*< Autopilot-specific errors*/
 uint16_t errors_count2; /*< Autopilot-specific errors*/
 uint16_t errors_count3; /*< Autopilot-specific errors*/
 uint16_t errors_count4; /*< Autopilot-specific errors*/
 int8_t battery_remaining; /*< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery*/
} mavlink_sys_status_t;

#define MAVLINK_MSG_ID_SYS_STATUS_LEN 31
#define MAVLINK_MSG_ID_1_LEN 31

#define MAVLINK_MSG_ID_SYS_STATUS_CRC 124
#define MAVLINK_MSG_ID_1_CRC 124



#define MAVLINK_MESSAGE_INFO_SYS_STATUS { \
	"SYS_STATUS", \
	13, \
	{  { "onboard_control_sensors_present", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_sys_status_t, onboard_control_sensors_present) }, \
         { "onboard_control_sensors_enabled", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_sys_status_t, onboard_control_sensors_enabled) }, \
         { "onboard_control_sensors_health", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sys_status_t, onboard_control_sensors_health) }, \
         { "load", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_sys_status_t, load) }, \
         { "voltage_battery", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_sys_status_t, voltage_battery) }, \
         { "current_battery", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_sys_status_t, current_battery) }, \
         { "drop_rate_comm", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_sys_status_t, drop_rate_comm) }, \
         { "errors_comm", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_sys_status_t, errors_comm) }, \
         { "errors_count1", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_sys_status_t, errors_count1) }, \
         { "errors_count2", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_sys_status_t, errors_count2) }, \
         { "errors_count3", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_sys_status_t, errors_count3) }, \
         { "errors_count4", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_sys_status_t, errors_count4) }, \
         { "battery_remaining", NULL, MAVLINK_TYPE_INT8_T, 0, 30, offsetof(mavlink_sys_status_t, battery_remaining) }, \
         } \
}


/**
 * @brief Pack a sys_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param onboard_control_sensors_present Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
 * @param onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
 * @param onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
 * @param load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 * @param voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
 * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
 * @param drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_count1 Autopilot-specific errors
 * @param errors_count2 Autopilot-specific errors
 * @param errors_count3 Autopilot-specific errors
 * @param errors_count4 Autopilot-specific errors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
uint16_t mavlink_msg_sys_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4);
									 
uint16_t mavlink_msg_sys_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t onboard_control_sensors_present,uint32_t onboard_control_sensors_enabled,uint32_t onboard_control_sensors_health,uint16_t load,uint16_t voltage_battery,int16_t current_battery,int8_t battery_remaining,uint16_t drop_rate_comm,uint16_t errors_comm,uint16_t errors_count1,uint16_t errors_count2,uint16_t errors_count3,uint16_t errors_count4);
											 
uint16_t mavlink_msg_sys_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sys_status_t* sys_status);
uint16_t mavlink_msg_sys_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sys_status_t* sys_status);

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

void mavlink_msg_sys_status_send(mavlink_channel_t chan, uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4);

#if MAVLINK_MSG_ID_SYS_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
void mavlink_msg_sys_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4);

#endif

#endif

// MESSAGE SYS_STATUS UNPACKING


/**
 * @brief Get field onboard_control_sensors_present from sys_status message
 *
 * @return Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
 */
uint32_t mavlink_msg_sys_status_get_onboard_control_sensors_present(const mavlink_message_t* msg);
uint32_t mavlink_msg_sys_status_get_onboard_control_sensors_enabled(const mavlink_message_t* msg);
uint32_t mavlink_msg_sys_status_get_onboard_control_sensors_health(const mavlink_message_t* msg);
uint16_t mavlink_msg_sys_status_get_load(const mavlink_message_t* msg);
uint16_t mavlink_msg_sys_status_get_voltage_battery(const mavlink_message_t* msg);
int16_t mavlink_msg_sys_status_get_current_battery(const mavlink_message_t* msg);
int8_t mavlink_msg_sys_status_get_battery_remaining(const mavlink_message_t* msg);
uint16_t mavlink_msg_sys_status_get_drop_rate_comm(const mavlink_message_t* msg);
uint16_t mavlink_msg_sys_status_get_errors_comm(const mavlink_message_t* msg);
uint16_t mavlink_msg_sys_status_get_errors_count1(const mavlink_message_t* msg);
uint16_t mavlink_msg_sys_status_get_errors_count2(const mavlink_message_t* msg);
uint16_t mavlink_msg_sys_status_get_errors_count3(const mavlink_message_t* msg);
uint16_t mavlink_msg_sys_status_get_errors_count4(const mavlink_message_t* msg);
void mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status);

#endif /*_x_h_*/
