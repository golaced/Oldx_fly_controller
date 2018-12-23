
#include "icm20602.h"
#include "spi.h"

static void icm20602_readbuf(u8 reg, u8 length, u8 *data)
{
	SPI_CS(CS_ACCGRO,0);
	Spi_RW(reg|0x80);
	SPI_Receive(data,length);
	SPI_CS(CS_ACCGRO,1);
}

static u8 icm20602_writebyte(u8 reg, u8 data)
{
	u8 status;
	
	SPI_CS(CS_ACCGRO,0);
	status = Spi_RW(reg);
	Spi_RW(data);
	SPI_CS(CS_ACCGRO,1);
	return status;
}
/**************************实现函数********************************************
*功　　能:	  读 修改 写 指定设备 指定寄存器一个字节 中的1个位
reg	   寄存器地址
bitNum  要修改目标字节的bitNum位
data  为0 时，目标位将被清0 否则将被置位
*******************************************************************************/
static void icm20602_writeBit(u8 reg, u8 bitNum, u8 data) 
{
	u8 b;
	icm20602_readbuf(reg, 1, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	icm20602_writebyte(reg, b);
}

static void icm20602_setIntEnabled ( void )
{
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTCFG_INT_LEVEL_BIT, ICM_INTMODE_ACTIVEHIGH );
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTCFG_INT_OPEN_BIT, ICM_INTDRV_PUSHPULL );
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTCFG_LATCH_INT_EN_BIT, ICM_INTLATCH_50USPULSE);//MPU6050_INTLATCH_WAITCLEAR );
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTCFG_INT_RD_CLEAR_BIT, ICM_INTCLEAR_ANYREAD );

	icm20602_writeBit ( MPUREG_INT_ENABLE, ICM_INTERRUPT_DATA_RDY_BIT, 1 );
}

/**************************实现函数********************************************
*功　　能:	    初始化icm进入可用状态。
*******************************************************************************/
u8 Icm20602Reg_Init(void)
{
	u8 tmp;
	icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x80);
	Delay_ms(10);
	icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x01);
	Delay_ms(10);
	
	icm20602_readbuf(MPUREG_WHOAMI, 1, &tmp);
	if(tmp != MPU_WHOAMI_20602)
	module.acc_imu=module.gyro_imu=0;
	else
  module.acc_imu=module.gyro_imu=2;
	/*复位reg*/
	icm20602_writebyte(MPU_RA_SIGNAL_PATH_RESET,0x03);
	Delay_ms(10);
  /*复位reg*/
	icm20602_writebyte(MPU_RA_USER_CTRL,0x01);	
	Delay_ms(10);

	icm20602_writebyte(0x70,0x40);//dmp 
	Delay_ms(10);
	icm20602_writebyte(MPU_RA_PWR_MGMT_2,0x00);
	Delay_ms(10);
	icm20602_writebyte(MPU_RA_SMPLRT_DIV,0);
	Delay_ms(10);

	icm20602_writebyte(MPU_RA_CONFIG,ICM20602_LPF_20HZ);
	Delay_ms(10);
	icm20602_writebyte(MPU_RA_GYRO_CONFIG,(3 << 3));
	Delay_ms(10);
	icm20602_writebyte(MPU_RA_ACCEL_CONFIG,(2 << 3));
	Delay_ms(10);
	/*加速度计LPF 20HZ*/
	icm20602_writebyte(0X1D,0x04);
	Delay_ms(10);
	/*关闭低功耗*/
	icm20602_writebyte(0X1E,0x00);
	Delay_ms(10);
	/*关闭FIFO*/
	icm20602_writebyte(0X23,0x00);

	Delay_ms(100);
	Icm20602_Read();
	return 1;

}



u8 mpu_buffer[14];

void Icm20602_Read()
{
	icm20602_readbuf(MPUREG_ACCEL_XOUT_H,14,mpu_buffer);
}


