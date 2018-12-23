
#ifndef  _MAVLINK_HELPERS_H_
#define  _MAVLINK_HELPERS_H_

#include "protocol.h"

#ifndef MAVLINK_HELPER
#define MAVLINK_HELPER
#endif

/* decls in sync with those in mavlink_helpers.h */
#ifndef MAVLINK_GET_CHANNEL_STATUS
   MAVLINK_HELPER mavlink_status_t* mavlink_get_channel_status(uint8_t chan);
#endif
 
MAVLINK_HELPER void mavlink_reset_channel_status(uint8_t chan);
 
#if MAVLINK_CRC_EXTRA
   MAVLINK_HELPER uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
                           uint8_t chan, uint8_t length, uint8_t crc_extra);
   MAVLINK_HELPER uint16_t mavlink_finalize_message(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
                           uint8_t length, uint8_t crc_extra);
   #ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
      MAVLINK_HELPER void _mav_finalize_message_chan_send(mavlink_channel_t chan, uint8_t msgid, const char *packet,
                           uint8_t length, uint8_t crc_extra);
   #endif
#else
   MAVLINK_HELPER uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
                           uint8_t chan, uint8_t length);
   MAVLINK_HELPER uint16_t mavlink_finalize_message(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
                           uint8_t length);
   #ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
      MAVLINK_HELPER void _mav_finalize_message_chan_send(mavlink_channel_t chan, uint8_t msgid, const char *packet, uint8_t length);
   #endif
#endif // MAVLINK_CRC_EXTRA
 
MAVLINK_HELPER uint16_t mavlink_msg_to_send_buffer(uint8_t *buffer, const mavlink_message_t *msg);
MAVLINK_HELPER void mavlink_start_checksum(mavlink_message_t* msg);
MAVLINK_HELPER void mavlink_update_checksum(mavlink_message_t* msg, uint8_t c);
MAVLINK_HELPER uint8_t mavlink_frame_char_buffer(mavlink_message_t* rxmsg, 
                                 						     mavlink_status_t* status,
						                                     uint8_t c, 
						                                     mavlink_message_t* r_message, 
						                                     mavlink_status_t* r_mavlink_status);
MAVLINK_HELPER uint8_t mavlink_frame_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);
MAVLINK_HELPER uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);
MAVLINK_HELPER uint8_t put_bitfield_n_by_index(int32_t b, uint8_t bits, uint8_t packet_index, uint8_t bit_index,
                               uint8_t* r_bit_index, uint8_t* buffer);

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
   void comm_send_ch(mavlink_channel_t chan, uint8_t ch);
   MAVLINK_HELPER void _mavlink_send_uart(mavlink_channel_t chan, const char *buf, uint16_t len);
   MAVLINK_HELPER void _mavlink_resend_uart(mavlink_channel_t chan, const mavlink_message_t *msg);
#endif


#endif /* _MAVLINK_HELPERS_H_ */




