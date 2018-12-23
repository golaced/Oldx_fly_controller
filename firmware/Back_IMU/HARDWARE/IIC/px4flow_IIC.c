#include "Soft_I2C_PX4.h"
#include "flow.h"
#include "usart_fc.h"
u8 buf25[25];
u8 buf22[23];

void PX4FLOW_22date_compound(void)
{ 
I2C_ReadBuffer_PX4(0x42,0,22,buf22);
//i2cWrite(0x42,0x85,0x00 );
	
//i2cRead(0x42, 0x00, 22, buf)	;
	
flow.hight.origin  =((buf22[21]<<8)+buf22[20]);

// 
flow.time_sec = buf22[19]*10;
	
//flow.gyro_range = buf22[18]*10000;

//flow.gyro_z_rate =((buf22[17]<<8)+buf22[16])*10;

//flow.gyro_y_rate	= ((buf22[15]<<8)+buf22[14])*10;
//	
//flow.gyro_x_rate	= ((buf22[13]<<8)+buf22[12])*10;
	
//flow.qual = ((buf22[11]<<8)+buf22[10])*10;

flow.flow_y.origin = ((buf22[9]<<8)+buf22[8]);
	
flow.flow_x.origin = ((buf22[7]<<8)+buf22[6]);	

flow.flow_comp_y.origin = ((buf22[5]<<8)+buf22[4]);

flow.flow_comp_x.origin = ((buf22[3]<<8)+buf22[2]);

flow.flow_cnt = ((buf22[1]<<8)+buf22[0]);
}


void PX4FLOW_25date_compound(void)
{
	
I2C_ReadBuffer_PX4(0x42,0x16, 25, buf25);
//i2cRead(0x42, 0x16, 25, buf25);	
	
	
//flow.frame_count_since_last_readout= ((buf25[1]<<8)+buf25[0]);

 
flow_rad.integrated_x =((buf25[3]<<8)+buf25[2]);
	
flow_rad.integrated_y= ((buf25[5]<<8)+buf25[4]);

flow_rad.integrated_xgyro =((buf25[7]<<8)+buf25[6]);

flow_rad.integrated_ygyro	= ((buf25[9]<<8)+buf25[8]);
	
flow_rad.integrated_zgyro	= ((buf25[11]<<8)+buf25[10]);
	
	
flow_rad.integration_time_us = ((buf25[15]<<24)+(buf25[14]<<16)+(buf25[13]<<8)+(buf25[12]));

flow_rad.time_delta_distance_us =  ((buf25[19]<<24)+(buf25[18]<<16)+(buf25[17]<<8)+(buf25[16]));
	
flow_rad.distance = ((buf25[21]<<8)+buf25[20]);	

flow_rad.temperature = ((buf25[23]<<8)+buf25[22])/100;

flow_rad.quality = buf25[24];

//frame_count = ((buf[1]<<8)+buf[0]);

}

void Read_Px4flow(void){


PX4FLOW_22date_compound();
PX4FLOW_25date_compound();

}




