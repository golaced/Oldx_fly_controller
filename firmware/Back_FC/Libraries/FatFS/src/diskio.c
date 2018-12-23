/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
//#include "driver_flash.h"	
#include "stm32f4xx.h"
#include  "drv_w25qxx.h"

/* Definitions of physical drive number for each drive */
#define SPI_FLASH		0	/*  Map spi nor flash to physical drive 0 */

static flash_info_t *flash_info;
/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{

	switch (pdrv) {
	case SPI_FLASH :
		if(flash_info->initialized)
		{
     return RES_OK;
		}
    break;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{

	switch (pdrv) {
	case SPI_FLASH :
		flash_info = Flash_GetInfo();
	  if(!flash_info->initialized)
		{
     Flash_Init();
		}

		return RES_OK;

	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	switch (pdrv) {
	case SPI_FLASH :
    if(flash_info->initialized)
		{
		DWORD  address   = sector*flash_info->sector_size;
  	Flash_SectorsRead(address,buff,count);
     return RES_OK;
		}
		break;
	}

 	return RES_PARERR;

}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count			/* Number of sectors to write */
)
{
	switch (pdrv) {
	case SPI_FLASH :
		if(flash_info->initialized)
		{
			DWORD  address   = sector*flash_info->sector_size;
			Flash_SectorsWrite(address,(uint8_t *)buff,count);

     return RES_OK;
		}
		break;
	}

	return RES_PARERR;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{

	switch (pdrv) {
	case SPI_FLASH :
	switch(cmd)
		{
			case CTRL_SYNC :  
				return RES_OK;
			
			case GET_SECTOR_SIZE:
				 *((WORD *)buff)   = flash_info->sector_size;   //返回扇区大小
				return RES_OK;
			
			case GET_SECTOR_COUNT:
				*(DWORD*)buff = flash_info->sector_count;  //FLASH总共的页数
				return RES_OK;
			
			case GET_BLOCK_SIZE:   
				//*(WORD*)buff = 1;
				return RES_OK;
			      
		}
		break;
	}

	return RES_PARERR;
}
#endif

DWORD get_fattime (void)
{
	/* Pack date and time into a DWORD variable */
	return	 0;
}
