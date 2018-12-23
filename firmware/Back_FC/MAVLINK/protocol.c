

#include "protocol.h"

/**
 * @brief Get the required buffer size for this message
 */
uint16_t mavlink_msg_get_send_buffer_length(const mavlink_message_t* msg)
{
	return msg->len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}

#if MAVLINK_NEED_BYTE_SWAP
void byte_swap_2(char *dst, const char *src)
{
	dst[0] = src[1];
	dst[1] = src[0];
}
void byte_swap_4(char *dst, const char *src)
{
	dst[0] = src[3];
	dst[1] = src[2];
	dst[2] = src[1];
	dst[3] = src[0];
}
void byte_swap_8(char *dst, const char *src)
{
	dst[0] = src[7];
	dst[1] = src[6];
	dst[2] = src[5];
	dst[3] = src[4];
	dst[4] = src[3];
	dst[5] = src[2];
	dst[6] = src[1];
	dst[7] = src[0];
}
#elif !MAVLINK_ALIGNED_FIELDS
void byte_copy_2(char *dst, const char *src)
{
	dst[0] = src[0];
	dst[1] = src[1];
}
void byte_copy_4(char *dst, const char *src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
	dst[3] = src[3];
}
void byte_copy_8(char *dst, const char *src)
{
	memcpy(dst, src, 8);
}
#endif


//add by BigW 2015-08-18
void memset(void *dest, int8_t data, uint32_t length) { 
	uint32_t i;
	uint8_t *point = dest;
	
	for(i=0; i<length; i++) point[i] = (uint8_t)data; 	
}  

void memcpy(void *dest, void *src, uint32_t length){
	uint32_t i;
	uint8_t *d=dest, *s=src; 
	
	for(i=0; i<length; i++) d[i] = s[i]; 		
}
//end (add by ...)


/*
  like memcpy(), but if src is NULL, do a memset to zero
*/
void mav_array_memcpy(void *dest, const void *src, uint32_t n)
{
	if (src == NULL) {
		memset(dest, 0, n);
	} else {
		memcpy(dest, (void*)src, n);
	}
}

/*
 * Place a char array into a buffer
 */
void _mav_put_char_array(char *buf, uint8_t wire_offset, char *b, uint8_t array_length)
{
	mav_array_memcpy(&buf[wire_offset], b, array_length);

}

/*
 * Place a uint8_t array into a buffer
 */
void _mav_put_uint8_t_array(char *buf, uint8_t wire_offset, uint8_t *b, uint8_t array_length)
{
	mav_array_memcpy(&buf[wire_offset], b, array_length);

}

/*
 * Place a int8_t array into a buffer
 */
void _mav_put_int8_t_array(char *buf, uint8_t wire_offset, int8_t *b, uint8_t array_length)
{
	mav_array_memcpy(&buf[wire_offset], b, array_length);

}


#if MAVLINK_NEED_BYTE_SWAP
#define _MAV_MSG_RETURN_TYPE(TYPE, SIZE) \
TYPE _MAV_RETURN_## TYPE(const mavlink_message_t *msg, uint8_t ofs) \
{ TYPE r; byte_swap_## SIZE((char*)&r, &_MAV_PAYLOAD(msg)[ofs]); return r; }

_MAV_MSG_RETURN_TYPE(uint16_t, 2)
_MAV_MSG_RETURN_TYPE(int16_t,  2)
_MAV_MSG_RETURN_TYPE(uint32_t, 4)
_MAV_MSG_RETURN_TYPE(int32_t,  4)
_MAV_MSG_RETURN_TYPE(uint64_t, 8)
_MAV_MSG_RETURN_TYPE(int64_t,  8)
_MAV_MSG_RETURN_TYPE(float,    4)
_MAV_MSG_RETURN_TYPE(double,   8)

#elif !MAVLINK_ALIGNED_FIELDS
#define _MAV_MSG_RETURN_TYPE(TYPE, SIZE) \
TYPE _MAV_RETURN_## TYPE(const mavlink_message_t *msg, uint8_t ofs) \
{ TYPE r; byte_copy_## SIZE((char*)&r, &_MAV_PAYLOAD(msg)[ofs]); return r; }

_MAV_MSG_RETURN_TYPE(uint16_t, 2)
_MAV_MSG_RETURN_TYPE(int16_t,  2)
_MAV_MSG_RETURN_TYPE(uint32_t, 4)
_MAV_MSG_RETURN_TYPE(int32_t,  4)
_MAV_MSG_RETURN_TYPE(uint64_t, 8)
_MAV_MSG_RETURN_TYPE(int64_t,  8)
_MAV_MSG_RETURN_TYPE(float,    4)
_MAV_MSG_RETURN_TYPE(double,   8)
#else // nicely aligned, no swap
#define _MAV_MSG_RETURN_TYPE(TYPE) \
TYPE _MAV_RETURN_## TYPE(const mavlink_message_t *msg, uint8_t ofs) \
{ return *(const TYPE *)(&_MAV_PAYLOAD(msg)[ofs]);}

_MAV_MSG_RETURN_TYPE(uint16_t)
_MAV_MSG_RETURN_TYPE(int16_t)
_MAV_MSG_RETURN_TYPE(uint32_t)
_MAV_MSG_RETURN_TYPE(int32_t)
_MAV_MSG_RETURN_TYPE(uint64_t)
_MAV_MSG_RETURN_TYPE(int64_t)
_MAV_MSG_RETURN_TYPE(float)
_MAV_MSG_RETURN_TYPE(double)
#endif // MAVLINK_NEED_BYTE_SWAP


