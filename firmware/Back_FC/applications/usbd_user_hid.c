/*----------------------------------------------------------------------------
 *      RL-ARM - USB
 *----------------------------------------------------------------------------
 *      Name:    usbd_user_hid.c
 *      Purpose: Human Interface Device Class User module
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "RTL.h"
#include <rl_usb.h>
#include <stm32f4xx.h>                  /* STM32F4xx Definitions              */
#define __NO_USB_LIB_C
#include "usb_config.c"
#include "data_transfer.h"
#include "usbd_user_hid.h"

u8 HID_SEND_TIMEOUT = 5;			//hid发送不足一帧时，等待HID_SEND_TIMEOUT周期进行发送
u8 hid_datatemp[256];					//hid环形缓冲区
u8 hid_datatemp_begin = 0;		//环形缓冲区数据指针，指向应当发送的数据
u8 hid_datatemp_end = 0;			//环形缓冲区数据结尾
u8 hid_data2send[64];					//hid发送缓存，一个hid数据帧64字节，第一字节表示有效数据字节数，0-63，后面是数据，最多63字节

void Usb_Hid_Init (void) 
{
  usbd_init();
	usbd_connect(__TRUE);
}
void Usb_Hid_Adddata(u8 *dataToSend , u8 length)
{
	for(u8 i=0; i<length; i++)
	{
		hid_datatemp[hid_datatemp_end++] = dataToSend[i];
	}
}
void Usb_Hid_Send (void)
{
	static u8 notfull_timeout=0;
	
	if(hid_datatemp_end > hid_datatemp_begin)
	{
		if((hid_datatemp_end - hid_datatemp_begin) >= 63)
		{
			notfull_timeout = 0;
			hid_data2send[0] = 63;
			for(u8 i=0; i<63; i++)
			{
				hid_data2send[i+1] = hid_datatemp[hid_datatemp_begin++];
			}
			usbd_hid_get_report_trigger(0, hid_data2send, 64);		//send
		}
		else
		{
			notfull_timeout++;
			if(notfull_timeout == HID_SEND_TIMEOUT)
			{
				notfull_timeout = 0;
				hid_data2send[0] = hid_datatemp_end - hid_datatemp_begin;
				for(u8 i=0; i<63; i++)
				{
					if(i<hid_datatemp_end - hid_datatemp_begin)
						hid_data2send[i+1] = hid_datatemp[hid_datatemp_begin+i];
					else
						hid_data2send[i+1] = 0;
				}
				hid_datatemp_begin = hid_datatemp_end;
				usbd_hid_get_report_trigger(0, hid_data2send, 64);		//send
			}
		}
	}
	else if(hid_datatemp_end < hid_datatemp_begin)
	{
		if((256 - hid_datatemp_begin + hid_datatemp_end) >= 63)
		{
			notfull_timeout = 0;
			hid_data2send[0] = 63;
			for(u8 i=0; i<63; i++)
			{
				hid_data2send[i+1] = hid_datatemp[hid_datatemp_begin++];
			}
			usbd_hid_get_report_trigger(0, hid_data2send, 64);		//send
		}
		else
		{
			notfull_timeout++;
			if(notfull_timeout == HID_SEND_TIMEOUT)
			{
				notfull_timeout = 0;
				hid_data2send[0] = 256 - hid_datatemp_begin + hid_datatemp_end;
				for(u8 i=0; i<63; i++)
				{
					if(i<256 - hid_datatemp_begin + hid_datatemp_end)
						hid_data2send[i+1] = hid_datatemp[(u8)(hid_datatemp_begin+i)];
					else
						hid_data2send[i+1] = 0;
				}
				hid_datatemp_begin = hid_datatemp_end;
				usbd_hid_get_report_trigger(0, hid_data2send, 64);		//dend
			}
		}
	}
}

int usbd_hid_get_report (U8 rtype, U8 rid, U8 *buf, U8 req) 
{
  switch (rtype) 
	{
    case HID_REPORT_INPUT:
      switch (rid) 
			{
         case 0:
          switch (req) 
					{
            case USBD_HID_REQ_EP_CTRL:
            case USBD_HID_REQ_PERIOD_UPDATE:
							return 64;
            case USBD_HID_REQ_EP_INT:
              break;
          }
           break;
      }
      break;
    case HID_REPORT_FEATURE:
      return (1);
  }
  return (0);
}

void usbd_hid_set_report (U8 rtype, U8 rid, U8 *buf, int len, U8 req) {

  switch (rtype) {
    case HID_REPORT_OUTPUT:
      for(u8 i = 1; i<=(*(buf)); i++)
				ANO_DT_Data_Receive_Prepare(*(buf+i));		//hid接收到数据会调用此函数
      break;
    case HID_REPORT_FEATURE:
      //feat = buf[0];
      break;
  }
}
