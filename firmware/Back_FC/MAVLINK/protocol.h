#ifndef  _MAVLINK_PROTOCOL_H_
#define  _MAVLINK_PROTOCOL_H_

#include "./minimal/mavlink.h"

/* 
   If you want MAVLink on a system that is native big-endian,
   you need to define NATIVE_BIG_ENDIAN
*/
#ifdef NATIVE_BIG_ENDIAN
# define MAVLINK_NEED_BYTE_SWAP (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN)
#else
# define MAVLINK_NEED_BYTE_SWAP (MAVLINK_ENDIAN != MAVLINK_LITTLE_ENDIAN)
#endif

#ifndef MAVLINK_STACK_BUFFER
#define MAVLINK_STACK_BUFFER 0
#endif

#ifndef MAVLINK_AVOID_GCC_STACK_BUG
# define MAVLINK_AVOID_GCC_STACK_BUG defined(__GNUC__)
#endif

#ifndef MAVLINK_ASSERT
#define MAVLINK_ASSERT(x)
#endif

#ifndef MAVLINK_START_UART_SEND
#define MAVLINK_START_UART_SEND(chan, length)
#endif

#ifndef MAVLINK_END_UART_SEND
#define MAVLINK_END_UART_SEND(chan, length)
#endif

/* option to provide alternative implementation of mavlink_helpers.h */
#ifdef MAVLINK_SEPARATE_HELPERS
    #define MAVLINK_HELPER
#else
    //#define MAVLINK_HELPER static inline
    #define MAVLINK_HELPER
#endif // MAVLINK_SEPARATE_HELPERS



uint16_t mavlink_msg_get_send_buffer_length(const mavlink_message_t* msg);
#if MAVLINK_NEED_BYTE_SWAP
void byte_swap_2(char *dst, const char *src);
void byte_swap_4(char *dst, const char *src);
void byte_swap_8(char *dst, const char *src);
#elif !MAVLINK_ALIGNED_FIELDS
void byte_copy_2(char *dst, const char *src);
void byte_copy_4(char *dst, const char *src);
void byte_copy_8(char *dst, const char *src);
#endif

#define _mav_put_uint8_t(buf, wire_offset, b) buf[wire_offset] = (uint8_t)b
#define _mav_put_int8_t(buf, wire_offset, b)  buf[wire_offset] = (int8_t)b
#define _mav_put_char(buf, wire_offset, b)    buf[wire_offset] = b

#if MAVLINK_NEED_BYTE_SWAP
#define _mav_put_uint16_t(buf, wire_offset, b) byte_swap_2(&buf[wire_offset], (const char *)&b)
#define _mav_put_int16_t(buf, wire_offset, b)  byte_swap_2(&buf[wire_offset], (const char *)&b)
#define _mav_put_uint32_t(buf, wire_offset, b) byte_swap_4(&buf[wire_offset], (const char *)&b)
#define _mav_put_int32_t(buf, wire_offset, b)  byte_swap_4(&buf[wire_offset], (const char *)&b)
#define _mav_put_uint64_t(buf, wire_offset, b) byte_swap_8(&buf[wire_offset], (const char *)&b)
#define _mav_put_int64_t(buf, wire_offset, b)  byte_swap_8(&buf[wire_offset], (const char *)&b)
#define _mav_put_float(buf, wire_offset, b)    byte_swap_4(&buf[wire_offset], (const char *)&b)
#define _mav_put_double(buf, wire_offset, b)   byte_swap_8(&buf[wire_offset], (const char *)&b)
#elif !MAVLINK_ALIGNED_FIELDS
#define _mav_put_uint16_t(buf, wire_offset, b) byte_copy_2(&buf[wire_offset], (const char *)&b)
#define _mav_put_int16_t(buf, wire_offset, b)  byte_copy_2(&buf[wire_offset], (const char *)&b)
#define _mav_put_uint32_t(buf, wire_offset, b) byte_copy_4(&buf[wire_offset], (const char *)&b)
#define _mav_put_int32_t(buf, wire_offset, b)  byte_copy_4(&buf[wire_offset], (const char *)&b)
#define _mav_put_uint64_t(buf, wire_offset, b) byte_copy_8(&buf[wire_offset], (const char *)&b)
#define _mav_put_int64_t(buf, wire_offset, b)  byte_copy_8(&buf[wire_offset], (const char *)&b)
#define _mav_put_float(buf, wire_offset, b)    byte_copy_4(&buf[wire_offset], (const char *)&b)
#define _mav_put_double(buf, wire_offset, b)   byte_copy_8(&buf[wire_offset], (const char *)&b)
#else
#define _mav_put_uint16_t(buf, wire_offset, b) *(uint16_t *)&buf[wire_offset] = b
#define _mav_put_int16_t(buf, wire_offset, b)  *(int16_t *)&buf[wire_offset] = b
#define _mav_put_uint32_t(buf, wire_offset, b) *(uint32_t *)&buf[wire_offset] = b
#define _mav_put_int32_t(buf, wire_offset, b)  *(int32_t *)&buf[wire_offset] = b
#define _mav_put_uint64_t(buf, wire_offset, b) *(uint64_t *)&buf[wire_offset] = b
#define _mav_put_int64_t(buf, wire_offset, b)  *(int64_t *)&buf[wire_offset] = b
#define _mav_put_float(buf, wire_offset, b)    *(float *)&buf[wire_offset] = b
#define _mav_put_double(buf, wire_offset, b)   *(double *)&buf[wire_offset] = b
#endif

/*
  like memcpy(), but if src is NULL, do a memset to zero
*/

//add by BigW 2015-08-18
void memset(void *dest, int8_t data, uint32_t length);
void memcpy(void *dest, void *src, uint32_t length);
//end (add by ...)

void mav_array_memcpy(void *dest, const void *src, uint32_t n);
void _mav_put_char_array(char *buf, uint8_t wire_offset, char *b, uint8_t array_length);
void _mav_put_uint8_t_array(char *buf, uint8_t wire_offset, uint8_t *b, uint8_t array_length);
void _mav_put_int8_t_array(char *buf, uint8_t wire_offset, int8_t *b, uint8_t array_length);


#define _MAV_RETURN_char(msg, wire_offset)    (char)_MAV_PAYLOAD(msg)[wire_offset]
#define _MAV_RETURN_int8_t(msg, wire_offset)  (int8_t)_MAV_PAYLOAD(msg)[wire_offset]
#define _MAV_RETURN_uint8_t(msg, wire_offset) (uint8_t)_MAV_PAYLOAD(msg)[wire_offset]

uint16_t _MAV_RETURN_uint16_t(const mavlink_message_t *msg, uint8_t ofs);
int16_t _MAV_RETURN_int16_t(const mavlink_message_t *msg, uint8_t ofs);
uint32_t _MAV_RETURN_uint32_t(const mavlink_message_t *msg, uint8_t ofs);
int32_t _MAV_RETURN_int32_t(const mavlink_message_t *msg, uint8_t ofs);
uint64_t _MAV_RETURN_uint64_t(const mavlink_message_t *msg, uint8_t ofs);
int64_t _MAV_RETURN_int64_t(const mavlink_message_t *msg, uint8_t ofs);
float _MAV_RETURN_float(const mavlink_message_t *msg, uint8_t ofs);
double _MAV_RETURN_double(const mavlink_message_t *msg, uint8_t ofs);



#endif // _MAVLINK_PROTOCOL_H_



