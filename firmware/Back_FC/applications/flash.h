#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "stm32f4xx.h"

#define SIZE_FLASH_ROOM 80
//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
#define SIZE (SIZE_FLASH_ROOM/4+((SIZE_FLASH_ROOM%4)?1:0))*2
#define FLASH_SAVE_ADDR  0x080E0000 	//设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.

//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//扇区8起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//扇区9起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//扇区10起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//扇区11起始地址,128 Kbytes  

u32 STMFLASH_ReadWord(u32 faddr);		  	//读出字  
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead);   		//从指定地址开始读出指定长度的数据

extern void READ_PARM(void);
extern void WRITE_PARM(void);
void READ_WAY_POINTS(void); 
void WRITE_PARM_WAY_POINTS(void);
extern float SONAR_HEIGHT,yaw_off_qr;
extern u16 LENGTH_OF_DRONE;
extern int H_INT;
extern u8 need_init_mems;
extern float k_sensitivity[3];//感度

extern u16 SBUS_MIN ;//=954;
extern u16 SBUS_MAX ;//=2108;
extern u16 SBUS_MID ;//=1524;
extern u16 SBUS_MIN_A ;//=954;
extern u16 SBUS_MAX_A ;//=2108;
extern u16 SBUS_MID_A ;//=1524;
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//W25QXX 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

//W25X系列/Q系列芯片列表	   
//W25Q80  ID  0XEF13
//W25Q16  ID  0XEF14
//W25Q32  ID  0XEF15
//W25Q64  ID  0XEF16	
//W25Q128 ID  0XEF17	
#define W25Q80 	0XEF13 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16
#define W25Q128	0XEF17

extern u16 W25QXX_TYPE;					//定义W25QXX芯片型号		   

#define	W25QXX_CS 		PCout(1)  		//W25QXX的片选信号

////////////////////////////////////////////////////////////////////////////////// 
//指令表
#define W25X_WriteEnable		0x06 
#define W25X_WriteDisable		0x04 
#define W25X_ReadStatusReg		0x05 
#define W25X_WriteStatusReg		0x01 
#define W25X_ReadData			0x03 
#define W25X_FastReadData		0x0B 
#define W25X_FastReadDual		0x3B 
#define W25X_PageProgram		0x02 
#define W25X_BlockErase			0xD8 
#define W25X_SectorErase		0x20 
#define W25X_ChipErase			0xC7 
#define W25X_PowerDown			0xB9 
#define W25X_ReleasePowerDown	0xAB 
#define W25X_DeviceID			0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F 

void W25QXX_Init(void);
u16  W25QXX_ReadID(void);  	    		//读取FLASH ID
u8	 W25QXX_ReadSR(void);        		//读取状态寄存器 
void W25QXX_Write_SR(u8 sr);  			//写状态寄存器
void W25QXX_Write_Enable(void);  		//写使能 
void W25QXX_Write_Disable(void);		//写保护
void W25QXX_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void W25QXX_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);   //读取flash
void W25QXX_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);//写入flash
void W25QXX_Erase_Chip(void);    	  	//整片擦除
void W25QXX_Erase_Sector(u32 Dst_Addr);	//扇区擦除
void W25QXX_Wait_Busy(void);           	//等待空闲
void W25QXX_PowerDown(void);        	//进入掉电模式
void W25QXX_WAKEUP(void);				//唤醒

void SPI1_Init(void);			 //初始化SPI1口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI1速度   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI1总线读写一个字节

 typedef struct
{
  u16 cnt[10];
	u8 sel;
	u16 shorte;
	u16 longe;
	u16 idel_cnt;
	u8 new_fly;
	u16 temp_cnt;
	u8 state;
	u8 rst;
}TIME;
extern TIME time_fly;













#endif

















