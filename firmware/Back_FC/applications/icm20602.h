#ifndef _DRV_ICM20602_H_
#define _DRV_ICM20602_H_

#include "include.h"

#define MPU_WHOAMI_20602		0x12

#define MPUREG_XG_OFFS_TC                       0x00
#define MPUREG_YG_OFFS_TC                       0x01
#define MPUREG_ZG_OFFS_TC                       0x02
#define MPUREG_X_FINE_GAIN                      0x03
#define MPUREG_Y_FINE_GAIN                      0x04
#define MPUREG_Z_FINE_GAIN                      0x05
#define MPUREG_XA_OFFS_H                        0x06    // X axis accelerometer offset (high byte)
#define MPUREG_XA_OFFS_L                        0x07    // X axis accelerometer offset (low byte)
#define MPUREG_YA_OFFS_H                        0x08    // Y axis accelerometer offset (high byte)
#define MPUREG_YA_OFFS_L                        0x09    // Y axis accelerometer offset (low byte)
#define MPUREG_ZA_OFFS_H                        0x0A    // Z axis accelerometer offset (high byte)
#define MPUREG_ZA_OFFS_L                        0x0B    // Z axis accelerometer offset (low byte)
#define MPUREG_PRODUCT_ID                       0x0C    // Product ID Register
#define MPUREG_XG_OFFS_USRH                     0x13    // X axis gyro offset (high byte)
#define MPUREG_XG_OFFS_USRL                     0x14    // X axis gyro offset (low byte)
#define MPUREG_YG_OFFS_USRH                     0x15    // Y axis gyro offset (high byte)
#define MPUREG_YG_OFFS_USRL                     0x16    // Y axis gyro offset (low byte)
#define MPUREG_ZG_OFFS_USRH                     0x17    // Z axis gyro offset (high byte)
#define MPUREG_ZG_OFFS_USRL                     0x18    // Z axis gyro offset (low byte)
#define MPUREG_SMPLRT_DIV                       0x19    // sample rate.  Fsample= 1Khz/(<this value>+1) = 200Hz
#       define MPUREG_SMPLRT_1000HZ                 0x00
#       define MPUREG_SMPLRT_500HZ                  0x01
#       define MPUREG_SMPLRT_250HZ                  0x03
#       define MPUREG_SMPLRT_200HZ                  0x04
#       define MPUREG_SMPLRT_100HZ                  0x09
#       define MPUREG_SMPLRT_50HZ                   0x13
#define MPUREG_CONFIG                           0x1A
#       define MPUREG_CONFIG_EXT_SYNC_SHIFT            3
#       define MPUREG_CONFIG_EXT_SYNC_GX            0x02
#       define MPUREG_CONFIG_EXT_SYNC_GY            0x03
#       define MPUREG_CONFIG_EXT_SYNC_GZ            0x04
#       define MPUREG_CONFIG_EXT_SYNC_AX            0x05
#       define MPUREG_CONFIG_EXT_SYNC_AY            0x06
#       define MPUREG_CONFIG_EXT_SYNC_AZ            0x07
#       define MPUREG_CONFIG_FIFO_MODE_STOP         0x40
#define MPUREG_GYRO_CONFIG                      0x1B
// bit definitions for MPUREG_GYRO_CONFIG
#       define BITS_GYRO_FS_250DPS                  0x00
#       define BITS_GYRO_FS_500DPS                  0x01
#       define BITS_GYRO_FS_1000DPS                 0x02
#       define BITS_GYRO_FS_2000DPS                 0x03
#       define BITS_GYRO_FS_MASK                    0x18 // only bits 3 and 4 are used for gyro full scale so use this to mask off other bits
#       define BITS_GYRO_ZGYRO_SELFTEST             0x20
#       define BITS_GYRO_YGYRO_SELFTEST             0x40
#       define BITS_GYRO_XGYRO_SELFTEST             0x80
#define MPUREG_ACCEL_CONFIG                     0x1C
#		define BITS_ACCEL_FS_2          0x00
#		define BITS_ACCEL_FS_4          0x01
#		define BITS_ACCEL_FS_8          0x02
#		define BITS_ACCEL_FS_16         0x03
#define MPUREG_MOT_THR                          0x1F    // detection threshold for Motion interrupt generation.  Motion is detected when the absolute value of any of the accelerometer measurements exceeds this
#define MPUREG_MOT_DUR                          0x20    // duration counter threshold for Motion interrupt generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit of 1 LSB = 1 ms
#define MPUREG_ZRMOT_THR                        0x21    // detection threshold for Zero Motion interrupt generation.
#define MPUREG_ZRMOT_DUR                        0x22    // duration counter threshold for Zero Motion interrupt generation. The duration counter ticks at 16 Hz, therefore ZRMOT_DUR has a unit of 1 LSB = 64 ms.
#define MPUREG_FIFO_EN                          0x23
#       define BIT_TEMP_FIFO_EN                     0x80
#       define BIT_XG_FIFO_EN                       0x40
#       define BIT_YG_FIFO_EN                       0x20
#       define BIT_ZG_FIFO_EN                       0x10
#       define BIT_ACCEL_FIFO_EN                    0x08
#       define BIT_SLV2_FIFO_EN                     0x04
#       define BIT_SLV1_FIFO_EN                     0x02
#       define BIT_SLV0_FIFI_EN0                    0x01
#define MPUREG_I2C_MST_CTRL                     0x24
#       define BIT_I2C_MST_P_NSR                    0x10
#       define BIT_I2C_MST_CLK_400KHZ               0x0D
#define MPUREG_I2C_SLV0_ADDR                    0x25
#define MPUREG_I2C_SLV1_ADDR                    0x28
#define MPUREG_I2C_SLV2_ADDR                    0x2B
#define MPUREG_I2C_SLV3_ADDR                    0x2E
#define MPUREG_INT_PIN_CFG                      0x37
#       define BIT_BYPASS_EN                        0x02
#       define BIT_INT_RD_CLEAR                     0x10    // clear the interrupt when any read occurs
#       define BIT_LATCH_INT_EN                     0x20    // latch data ready pin
#define MPUREG_I2C_SLV4_CTRL                    0x34
#define MPUREG_INT_ENABLE                       0x38
// bit definitions for MPUREG_INT_ENABLE
#       define BIT_RAW_RDY_EN                       0x01
#       define BIT_DMP_INT_EN                       0x02    // enabling this bit (DMP_INT_EN) also enables RAW_RDY_EN it seems
#       define BIT_UNKNOWN_INT_EN                   0x04
#       define BIT_I2C_MST_INT_EN                   0x08
#       define BIT_FIFO_OFLOW_EN                    0x10
#       define BIT_ZMOT_EN                          0x20
#       define BIT_MOT_EN                           0x40
#       define BIT_FF_EN                            0x80
#define MPUREG_INT_STATUS                       0x3A
// bit definitions for MPUREG_INT_STATUS (same bit pattern as above because this register shows what interrupt actually fired)
#       define BIT_RAW_RDY_INT                      0x01
#       define BIT_DMP_INT                          0x02
#       define BIT_UNKNOWN_INT                      0x04
#       define BIT_I2C_MST_INT                      0x08
#       define BIT_FIFO_OFLOW_INT                   0x10
#       define BIT_ZMOT_INT                         0x20
#       define BIT_MOT_INT                          0x40
#       define BIT_FF_INT                           0x80
#define MPUREG_ACCEL_XOUT_H                     0x3B
#define MPUREG_ACCEL_XOUT_L                     0x3C
#define MPUREG_ACCEL_YOUT_H                     0x3D
#define MPUREG_ACCEL_YOUT_L                     0x3E
#define MPUREG_ACCEL_ZOUT_H                     0x3F
#define MPUREG_ACCEL_ZOUT_L                     0x40
#define MPUREG_TEMP_OUT_H                       0x41
#define MPUREG_TEMP_OUT_L                       0x42
#define MPUREG_GYRO_XOUT_H                      0x43
#define MPUREG_GYRO_XOUT_L                      0x44
#define MPUREG_GYRO_YOUT_H                      0x45
#define MPUREG_GYRO_YOUT_L                      0x46
#define MPUREG_GYRO_ZOUT_H                      0x47
#define MPUREG_GYRO_ZOUT_L                      0x48
#define MPUREG_EXT_SENS_DATA_00                 0x49
#define MPUREG_I2C_SLV0_DO                      0x63
#define MPUREG_I2C_MST_DELAY_CTRL               0x67
#       define BIT_I2C_SLV0_DLY_EN              0x01
#       define BIT_I2C_SLV1_DLY_EN              0x02
#       define BIT_I2C_SLV2_DLY_EN              0x04
#       define BIT_I2C_SLV3_DLY_EN              0x08
#define MPUREG_USER_CTRL                        0x6A
// bit definitions for MPUREG_USER_CTRL
#       define BIT_USER_CTRL_SIG_COND_RESET         0x01 // resets signal paths and results registers for all sensors (gyros, accel, temp)
#       define BIT_USER_CTRL_I2C_MST_RESET          0x02 // reset I2C Master (only applicable if I2C_MST_EN bit is set)
#       define BIT_USER_CTRL_FIFO_RESET             0x04 // Reset (i.e. clear) FIFO buffer
#       define BIT_USER_CTRL_DMP_RESET              0x08 // Reset DMP
#       define BIT_USER_CTRL_I2C_IF_DIS             0x10 // Disable primary I2C interface and enable hal.spi->interface
#       define BIT_USER_CTRL_I2C_MST_EN             0x20 // Enable MPU to act as the I2C Master to external slave sensors
#       define BIT_USER_CTRL_FIFO_EN                0x40 // Enable FIFO operations
#       define BIT_USER_CTRL_DMP_EN             0x80     // Enable DMP operations
#define MPUREG_PWR_MGMT_1                           0x6B
#       define BIT_PWR_MGMT_1_CLK_INTERNAL          0x00 // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_XGYRO             0x01 // PLL with X axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_YGYRO             0x02 // PLL with Y axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_ZGYRO             0x03 // PLL with Z axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_EXT32KHZ          0x04 // PLL with external 32.768kHz reference
#       define BIT_PWR_MGMT_1_CLK_EXT19MHZ          0x05 // PLL with external 19.2MHz reference
#       define BIT_PWR_MGMT_1_CLK_STOP              0x07 // Stops the clock and keeps the timing generator in reset
#       define BIT_PWR_MGMT_1_TEMP_DIS              0x08 // disable temperature sensor
#       define BIT_PWR_MGMT_1_CYCLE                 0x20 // put sensor into cycle mode.  cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL
#       define BIT_PWR_MGMT_1_SLEEP                 0x40 // put sensor into low power sleep mode
#       define BIT_PWR_MGMT_1_DEVICE_RESET          0x80 // reset entire device
#define MPUREG_PWR_MGMT_2                       0x6C    // allows the user to configure the frequency of wake-ups in Accelerometer Only Low Power Mode
#define MPUREG_BANK_SEL                         0x6D    // DMP bank selection register (used to indirectly access DMP registers)
#define MPUREG_MEM_START_ADDR                   0x6E    // DMP memory start address (used to indirectly write to dmp memory)
#define MPUREG_MEM_R_W                              0x6F // DMP related register
#define MPUREG_DMP_CFG_1                            0x70 // DMP related register
#define MPUREG_DMP_CFG_2                            0x71 // DMP related register
#define MPUREG_FIFO_COUNTH                          0x72
#define MPUREG_FIFO_COUNTL                          0x73
#define MPUREG_FIFO_R_W                             0x74
#define MPUREG_WHOAMI                               0x75

#define ICM_PWR1_DEVICE_RESET_BIT   7
#define ICM_PWR1_SLEEP_BIT          6
#define ICM_PWR1_CYCLE_BIT          5
#define ICM_PWR1_TEMP_DIS_BIT       3
#define ICM_PWR1_CLKSEL_BIT         2
#define ICM_PWR1_CLKSEL_LENGTH      3

#define ICM_GCONFIG_FS_SEL_BIT      4
#define ICM_GCONFIG_FS_SEL_LENGTH   2

#define ICM_ACONFIG_XA_ST_BIT           7
#define ICM_ACONFIG_YA_ST_BIT           6
#define ICM_ACONFIG_ZA_ST_BIT           5
#define ICM_ACONFIG_AFS_SEL_BIT         4
#define ICM_ACONFIG_AFS_SEL_LENGTH      2
#define ICM_ACONFIG_ACCEL_HPF_BIT       2
#define ICM_ACONFIG_ACCEL_HPF_LENGTH    3

#define ICM_CFG_DLPF_CFG_BIT    2
#define ICM_CFG_DLPF_CFG_LENGTH 3

#define ICM_DLPF_BW_256         0x00
#define ICM_DLPF_BW_188         0x01
#define ICM_DLPF_BW_98          0x02
#define ICM_DLPF_BW_42          0x03
#define ICM_DLPF_BW_20          0x04
#define ICM_DLPF_BW_10          0x05
#define ICM_DLPF_BW_5           0x06

#define ICM_INTCFG_INT_LEVEL_BIT        7
#define ICM_INTCFG_INT_OPEN_BIT         6
#define ICM_INTCFG_LATCH_INT_EN_BIT     5
#define ICM_INTCFG_INT_RD_CLEAR_BIT     4
#define ICM_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define ICM_INTCFG_FSYNC_INT_EN_BIT     2
#define ICM_INTCFG_I2C_BYPASS_EN_BIT    1
#define ICM_INTCFG_CLKOUT_EN_BIT        0

#define ICM_INTMODE_ACTIVEHIGH  0x00
#define ICM_INTMODE_ACTIVELOW   0x01

#define ICM_INTDRV_PUSHPULL     0x00
#define ICM_INTDRV_OPENDRAIN    0x01

#define ICM_INTLATCH_50USPULSE  0x00
#define ICM_INTLATCH_WAITCLEAR  0x01

#define ICM_INTCLEAR_STATUSREAD 0x00
#define ICM_INTCLEAR_ANYREAD    0x01

#define ICM_INTERRUPT_FF_BIT            7
#define ICM_INTERRUPT_MOT_BIT           6
#define ICM_INTERRUPT_ZMOT_BIT          5
#define ICM_INTERRUPT_FIFO_OFLOW_BIT    4
#define ICM_INTERRUPT_I2C_MST_INT_BIT   3
#define ICM_INTERRUPT_PLL_RDY_INT_BIT   2
#define ICM_INTERRUPT_DMP_INT_BIT       1
#define ICM_INTERRUPT_DATA_RDY_BIT      0


#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

#define ICM20602_LPF_250HZ       0
#define ICM20602_LPF_176HZ       1
#define ICM20602_LPF_92HZ        2
#define ICM20602_LPF_41HZ        3
#define ICM20602_LPF_20HZ        4
#define ICM20602_LPF_10HZ        5
#define ICM20602_LPF_5HZ         6
#define ICM20602_LPF_3281HZ      7

/////////////////////////////////////

#define RANGE_PN2000_TO_RAD 0.001065f
#define RANGE_PN8G_TO_CMSS  0.2395f

typedef struct
{
	float center_pos_cm[3];
	float gyro_rad[3];
	float gyro_rad_old[3];
	float gyro_rad_acc[3];
	float linear_acc[3];
}_center_pos_st;
extern _center_pos_st center_pos;

extern u8 mpu_buffer[14];

typedef struct
{
  u8 surface_CALIBRATE;
	float surface_vec[3];
	float surface_unitvec[3];
	
}_sensor_rotate_st;
extern _sensor_rotate_st sensor_rot ;


u8 Icm20602Reg_Init(void);
void Icm20602_Read(void);
void Sensor_Data_Prepare(u8 dT_ms);
void Center_Pos_Set(void);
void  Acc_Prepare(float Pitch,float Roll,float Yaw);
#endif
