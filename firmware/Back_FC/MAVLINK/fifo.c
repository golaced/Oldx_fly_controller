
#include "fifo.h"


/** @brief 读FIFO
  * @param fifo 待读缓冲区
	*        *ch   读到的数据
	* @return 
	*        正确读取，1； 无数据，0
  */
uint8_t fifo_read_ch(fifo_t* fifo, uint8_t* ch)
{
	if(fifo->tail == fifo->head) return false;
	*ch = fifo->buf[fifo->tail];  
	
	if(++fifo->tail >= fifo->length) fifo->tail = 0;
  return true;
}


/** @brief 写一字节数据到FIFO
  * @param fifo 待写入缓冲区
	*        ch   待写入的数据
	* @return 
	*        正确，1； 缓冲区满，0
  */
uint8_t fifo_write_ch(fifo_t* fifo, uint8_t ch)
{
	uint16_t h = fifo->head;
	
	if(++h >= fifo->length) h = 0;
	if(h == fifo->tail) return false;
	
	fifo->buf[fifo->head] = ch;
	fifo->head = h;
  return true;
}


/** @brief 返回缓冲区剩余字节长度
  * @param fifo 
	* @return 
	*        剩余空间
  *
  * @note  剩余字节长度大于等于2时，才可写入数据
  */
uint16_t fifo_free(fifo_t* fifo)  
{
	uint16_t free;
	
	if(fifo->head >= fifo->tail) free = fifo->tail + (fifo->length - fifo->head);
	else free = fifo->tail - fifo->head;
	
  return free;
}

uint16_t fifo_used(fifo_t* fifo)
{
	uint16_t used;
	
	if(fifo->head >= fifo->tail) used = fifo->head - fifo->tail;
	else used = fifo->head + (fifo->length - fifo->tail);
	
	return used;	
}


/** @brief 初始化缓冲区
  * @param *fifo
  *        *buf 
  *        length
  */
void fifo_init(fifo_t* fifo, uint8_t* buf, uint16_t length)  
{
	uint16_t i;
	
	fifo->buf = buf;
	fifo->length = length;
	fifo->head = 0;
	fifo->tail = 0;
	
	for(i=0; i<length; i++) fifo->buf[i] = 0;	
}





