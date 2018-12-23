#ifndef _USBD_USER_HID_H
#define	_USBD_USER_HID_H

#include "stm32f4xx.h"

extern void Usb_Hid_Init(void) ;
void Usb_Hid_Adddata(u8 *dataToSend , u8 length);
void Usb_Hid_Send(void);

#endif

