
#include "include.h"
#include "flash.h"
#include "mpu6050.h"
#include "hml5833l.h"
#include "flash_w25.h"
#include "LIS3MDL.h" 
//读取指定地址的半字(16位数据) 
//faddr:读地址 
//返回值:对应数据.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
//获取某个地址所在的flash扇区
//addr:flash地址
//返回值:0~11,即addr所在的扇区
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}
//从指定地址开始写入指定长度的数据
//特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
//         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
//         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
//         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
//该函数对OTP区域也有效!可以用来写OTP区!
//OTP区域地址范围:0X1FFF7800~0X1FFF7A0F
//WriteAddr:起始地址(此地址必须为4的倍数!!)
//pBuffer:数据指针
//NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
	FLASH_Unlock();									//解锁 
  FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx=WriteAddr;				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
	if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)break;	//发生错误了
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//写数据
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//写入数据
			{ 
				break;	//写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
} 

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToRead:字(4位)数
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}
#define SIZE_PARAM 100
u8 FLASH_READ_BUF[SIZE_PARAM]={0};
u8 FLASH_Buffer[SIZE_PARAM]={0};
u32 FLASH_SIZE=16*1024*1024;	//FLASH 大小为16字节
void READ_PARM(void)
{
u16 cnt=1;
//STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)FLASH_READ_BUF,SIZE);	
W25QXX_Read(FLASH_READ_BUF,FLASH_SIZE-100,SIZE_PARAM);					//从倒数第100个地址处开始,读出SIZE个字节
mpu6050.Gyro_Offset.x=(vs16)(FLASH_READ_BUF[1]<<8|FLASH_READ_BUF[0]);
mpu6050.Gyro_Offset.y=(vs16)(FLASH_READ_BUF[3]<<8|FLASH_READ_BUF[2]);
mpu6050.Gyro_Offset.z=(vs16)(FLASH_READ_BUF[5]<<8|FLASH_READ_BUF[4]);
	
mpu6050.Acc_Offset.x=(vs16)(FLASH_READ_BUF[7]<<8|FLASH_READ_BUF[6]);
mpu6050.Acc_Offset.y=(vs16)(FLASH_READ_BUF[9]<<8|FLASH_READ_BUF[8]);
mpu6050.Acc_Offset.z=(vs16)(FLASH_READ_BUF[11]<<8|FLASH_READ_BUF[10]);
	
ak8975.Mag_Offset.x=(vs16)(FLASH_READ_BUF[13]<<8|FLASH_READ_BUF[12]);
ak8975.Mag_Offset.y=(vs16)(FLASH_READ_BUF[15]<<8|FLASH_READ_BUF[14]);
ak8975.Mag_Offset.z=(vs16)(FLASH_READ_BUF[17]<<8|FLASH_READ_BUF[16]);
	
ak8975.Mag_Gain.x =(float)((vs16)((FLASH_READ_BUF[19]<<8|FLASH_READ_BUF[18])))/100.;
ak8975.Mag_Gain.y=(float)((vs16)((FLASH_READ_BUF[21]<<8|FLASH_READ_BUF[20])))/100.;
ak8975.Mag_Gain.z =(float)((vs16)((FLASH_READ_BUF[23]<<8|FLASH_READ_BUF[22])))/100.;
	
	
lis3mdl.Gyro_Offset.x=(vs16)(FLASH_READ_BUF[25]<<8|FLASH_READ_BUF[24]);
lis3mdl.Gyro_Offset.y=(vs16)(FLASH_READ_BUF[27]<<8|FLASH_READ_BUF[26]);
lis3mdl.Gyro_Offset.z=(vs16)(FLASH_READ_BUF[29]<<8|FLASH_READ_BUF[28]);
	
lis3mdl.Acc_Offset.x=(vs16)(FLASH_READ_BUF[31]<<8|FLASH_READ_BUF[30]);
lis3mdl.Acc_Offset.y=(vs16)(FLASH_READ_BUF[33]<<8|FLASH_READ_BUF[32]);
lis3mdl.Acc_Offset.z=(vs16)(FLASH_READ_BUF[35]<<8|FLASH_READ_BUF[34]);
	
lis3mdl.Mag_Offset.x=(vs16)(FLASH_READ_BUF[37]<<8|FLASH_READ_BUF[36]);
lis3mdl.Mag_Offset.y=(vs16)(FLASH_READ_BUF[39]<<8|FLASH_READ_BUF[38]);
lis3mdl.Mag_Offset.z=(vs16)(FLASH_READ_BUF[41]<<8|FLASH_READ_BUF[40]);
	
lis3mdl.Mag_Gain.x =(float)((vs16)((FLASH_READ_BUF[43]<<8|FLASH_READ_BUF[42])))/100.;
lis3mdl.Mag_Gain.y=(float)((vs16)((FLASH_READ_BUF[45]<<8|FLASH_READ_BUF[44])))/100.;
lis3mdl.Mag_Gain.z =(float)((vs16)((FLASH_READ_BUF[47]<<8|FLASH_READ_BUF[46])))/100.;	

lis3mdl.Off_3d.x=(vs16)(FLASH_READ_BUF[49]<<8|FLASH_READ_BUF[48]);
lis3mdl.Off_3d.y=(vs16)(FLASH_READ_BUF[51]<<8|FLASH_READ_BUF[50]);
lis3mdl.Off_3d.z=(vs16)(FLASH_READ_BUF[53]<<8|FLASH_READ_BUF[52]);
	
lis3mdl.Gain_3d.x =(float)((vs16)((FLASH_READ_BUF[55]<<8|FLASH_READ_BUF[54])))/1000.;
lis3mdl.Gain_3d.y=(float)((vs16)((FLASH_READ_BUF[57]<<8|FLASH_READ_BUF[56])))/1000.;
lis3mdl.Gain_3d.z =(float)((vs16)((FLASH_READ_BUF[59]<<8|FLASH_READ_BUF[58])))/1000.;

lis3mdl.Mag_Offseto.x=(vs16)(FLASH_READ_BUF[61]<<8|FLASH_READ_BUF[60]);
lis3mdl.Mag_Offseto.y=(vs16)(FLASH_READ_BUF[63]<<8|FLASH_READ_BUF[62]);
lis3mdl.Mag_Offseto.z=(vs16)(FLASH_READ_BUF[65]<<8|FLASH_READ_BUF[64]);
	
lis3mdl.Mag_Gaino.x =(float)((vs16)((FLASH_READ_BUF[67]<<8|FLASH_READ_BUF[66])))/100.;
lis3mdl.Mag_Gaino.y=(float)((vs16)((FLASH_READ_BUF[69]<<8|FLASH_READ_BUF[68])))/100.;
lis3mdl.Mag_Gaino.z =(float)((vs16)((FLASH_READ_BUF[71]<<8|FLASH_READ_BUF[70])))/100.;	

if(fabs(lis3mdl.Off_3d.x)<400&&fabs(lis3mdl.Off_3d.y)<400&&fabs(lis3mdl.Off_3d.z)<400)
	module.acc=2;
else
	module.acc=1;

if(fabs(lis3mdl.Gyro_Offset.x)<200&&fabs(lis3mdl.Gyro_Offset.y)<200&&fabs(lis3mdl.Gyro_Offset.z)<200)
	module.gyro=2;
else
	module.gyro=1;

if((fabs(lis3mdl.Off_3d.x)>10||fabs(lis3mdl.Off_3d.y)>10||fabs(lis3mdl.Off_3d.z)>10)
	&&(fabs(lis3mdl.Off_3d.x)<600&&fabs(lis3mdl.Off_3d.y)<600&&fabs(lis3mdl.Off_3d.z)<600))
lis3mdl.Cali_3d=1;
}

void WRITE_PARM(void)
{ 

int16_t _temp;
u8 cnt=0;

_temp=(int16_t)mpu6050.Gyro_Offset.x;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mpu6050.Gyro_Offset.y;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mpu6050.Gyro_Offset.z;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);

_temp=(int16_t)mpu6050.Acc_Offset.x;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mpu6050.Acc_Offset.y;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mpu6050.Acc_Offset.z;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)ak8975.Mag_Offset.x;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)ak8975.Mag_Offset.y;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)ak8975.Mag_Offset.z;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);


_temp=(int16_t)(ak8975.Mag_Gain.x*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(ak8975.Mag_Gain.y*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(ak8975.Mag_Gain.z*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);


//------lis3mdl
_temp=(int16_t)lis3mdl.Gyro_Offset.x;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)lis3mdl.Gyro_Offset.y;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)lis3mdl.Gyro_Offset.z;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);

_temp=(int16_t)lis3mdl.Acc_Offset.x;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)lis3mdl.Acc_Offset.y;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)lis3mdl.Acc_Offset.z;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)lis3mdl.Mag_Offset.x;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)lis3mdl.Mag_Offset.y;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)lis3mdl.Mag_Offset.z;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);


_temp=(int16_t)(lis3mdl.Mag_Gain.x*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(lis3mdl.Mag_Gain.y*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(lis3mdl.Mag_Gain.z*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);



_temp=(int16_t)lis3mdl.Off_3d .x;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)lis3mdl.Off_3d.y;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)lis3mdl.Off_3d.z;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);

_temp=(int16_t)(lis3mdl.Gain_3d.x*1000);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(lis3mdl.Gain_3d.y*1000);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(lis3mdl.Gain_3d.z*1000);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);

_temp=(int16_t)lis3mdl.Mag_Offseto.x;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)lis3mdl.Mag_Offseto.y;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)lis3mdl.Mag_Offseto.z;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);


_temp=(int16_t)(lis3mdl.Mag_Gaino.x*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(lis3mdl.Mag_Gaino.y*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(lis3mdl.Mag_Gaino.z*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);

//_temp=(int16_t)(dj_angle_offset[0]*100);
//FLASH_Buffer[cnt++]=BYTE0(_temp);
//FLASH_Buffer[cnt++]=BYTE1(_temp);
//_temp=(int16_t)(dj_angle_offset[1]*100);
//FLASH_Buffer[cnt++]=BYTE0(_temp);
//FLASH_Buffer[cnt++]=BYTE1(_temp);
//_temp=(int16_t)(dj_angle_offset[2]*100);
//FLASH_Buffer[cnt++]=BYTE0(_temp);
//FLASH_Buffer[cnt++]=BYTE1(_temp);

//STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)FLASH_Buffer,SIZE);
	W25QXX_Write((u8*)FLASH_Buffer,FLASH_SIZE-100,SIZE_PARAM);		//从倒数第100个地址处开始,写入SIZE长度的数据
}










