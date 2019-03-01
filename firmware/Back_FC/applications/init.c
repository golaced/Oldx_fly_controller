
#include "include.h"
#include "pwm_out.h"
#include "mpu6050.h"
#include "i2c_soft.h"
#include "led.h"
#include "ctrl.h"
#include "ms5611.h"
#include "ak8975.h"
#include "ultrasonic.h"
#include "bmp.h"
#include "spi.h"
#include "nrf.h"
#include "iic_hml.h"
#include "mpu9250.h"
#include "ms5611_spi.h"
#include "beep.h"
#include "rng.h"
#include "icm20602.h"
#include "pos_ctrl.h"
#include "bat.h"
#include "mavl.h"
_SYS_INIT sys_init;
u8 mcuID[3];
int id_chip;
u8 ble_imu_force;
u8 CHE=44;
void cpuidGetId(void)
{
    mcuID[0] = *(__IO u32*)(0x1FFF7A10);
    mcuID[1] = *(__IO u32*)(0x1FFF7A14);
    mcuID[2] = *(__IO u32*)(0x1FFF7A18); 
	  id_chip=mcuID[0]+mcuID[1]+mcuID[2];
}
s16 motor_out_test[MAXMOTORS]={0};

u8 All_Init()
{ u8 i;
	NVIC_PriorityGroupConfig(NVIC_GROUP);		//中断优先级组别设置
	SysTick_Configuration(); 	//滴答时钟
	cpuidGetId();
	switch(id_chip)
	{
		case IMAV1: CHE=11;break;//MAP1
		case IMAV2: CHE=22;break;//Serach2
		case IMAV3: CHE=33;break;//Serach3
		default: CHE=11;break;
	}
	#if USE_SAFE_SWITCH
	
  #endif
	#if !USE_VER_3||USE_VER_6
	I2c_Soft_Init();					//初始化模拟I2C
	#endif
	#if MAXMOTORS==12
		PWM_AUX_Out_Init(400);	
	#else
		PWM_AUX_Out_Init(50);	
	#endif
	PWM_Out_Init(400);				//初始化电调输出功能	
	Delay_ms(100);						//延时
	LED_Init();								//LED功能初始
	Delay_ms(100);						//启动延时
	SPI2_Init();
	#if USE_VER_3&&!USE_VER_6
	#if USE_VER_7
	Icm20602Reg_Init();
	#else
	Mpu9250_Init();
	#endif
	ms5611DetectSpi();
	#else
	#if EN_ATT_CAL_FC
	MPU6050_Init(20);
	#if !USE_ZIN_BMP
	HMC5883L_SetUp();
	#else
	IIC_Init();
	#endif
	Delay_ms(100);						//延时
	MS5611_Init_FC();
	#endif
	#endif
	RNG_Init();
	
	Cycle_Time_Init();
	//Usart1_Init(115200);			//蓝牙
  //Usart1_Init(2000000);			//ANO_A
	Usart1_Init(38400L);			//3DR
	Usart1_Init(57600);			//WIFI
	mavlink_init();
	#if EN_DMA_UART1 
	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF_SIZE1+2,1);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
	Usart2_Init(576000L);			//IMU_LINK
	#if EN_DMA_UART2
	MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)SendBuff2,SEND_BUF_SIZE2+2,1);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
  Usart4_Init(256000L);     //接收机  SD卡
	#if EN_DMA_UART4 
	MYDMA_Config(DMA1_Stream4,DMA_Channel_4,(u32)&UART4->DR,(u32)SendBuff4,SEND_BUF_SIZE4+2,0);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
	#if !USE_ZIN_BMP	
	#if USE_PXY										
	Usart3_Init(115200L);  
	#else
	#if USE_ANO_FLOW
	Usart3_Init(500000L); 
	#else
	Usart3_Init(115200);     // 未使用 或者 超声波   230400LPX4
	#endif
	#if USE_VER_6
	Usart3_Init(115200L);
	#endif
	#endif
	#if PX4_SDK
	Usart3_Init(115200);//odroid
	#endif
	#if EN_DMA_UART3
	MYDMA_Config(DMA1_Stream3,DMA_Channel_4,(u32)&USART3->DR,(u32)SendBuff3,SEND_BUF_SIZE3+2,2);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
  #endif
	#endif
	#if USE_MINI_FC_FLOW_BOARD||USE_VER_3
	Uart5_Init(100000);	
  #else

	#if !SONAR_USE_FC1
  Uart5_Init(115200L);      // 图像Odroid 
	#else
	Uart5_Init(9600);      // 超声波
	#endif

	#endif
	Delay_ms(100);
	#if EN_DMA_UART4 
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);       
	MYDMA_Enable(DMA1_Stream4,SEND_BUF_SIZE4+2);    
	#endif
	#if EN_DMA_UART2
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);       
	MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);     	
	#endif
	#if USE_ZIN_BMP
	#if EN_DMA_UART3
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);      
	MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE3+2);  	
	#endif
	#endif
	#if EN_DMA_UART1 
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);     
	MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);     
	#endif	
	#if SONAR_USE_FC
	#if !USE_ZIN_BMP	
	Ultrasonic_Init();
	#endif
	#endif
	#if !FLASH_USE_STM32	
	W25QXX_Init();		
	if(W25QXX_ReadID()!=W25Q32&&W25QXX_ReadID()!=W25Q16)								//检测不到flash
		Delay_ms(100);
	#endif
	READ_PARM();//读取参数
	#if !FLASH_USE_STM32
	 READ_WAY_POINTS();
	#endif
	Para_Init();//参数初始
	#if USE_MINI_FC_FLOW_BOARD||USE_VER_3		
	Nrf24l01_Init(MODEL_RX2,CHE);
	Nrf24l01_Check();
	#endif
	#if USE_VER_3
	Adc_Init();
	#endif
	#if !defined(DEBUG_INROOM)
	Beep_Init(0,84-1);//2k=1M/500
  #endif
	//-------------系统默认参数
	mode_oldx.en_eso_h_in=1;
	mode_oldx.imu_use_mid_down=1;
	mode_oldx.flow_f_use_ukfm=2;
	mode_oldx.baro_f_use_ukfm=0;				
	mode_oldx.yaw_use_eso=0;
  mode_oldx.use_px4_err=1;
	
	mode_oldx.en_auto_home=1;//使用一键返航
	mode_oldx.auto_fly_up=1;//使用自动起飞
	mode_oldx.rc_loss_return_home=1;//使能油门丢失0-> shut motor 1->return home 2->directland  3->ignore
	
	three_wheel_car.car_mode=3;//3circle 2forwad
	three_wheel_car.gain[0]=1;//spd
	three_wheel_car.gain[1]=2;//r
	three_wheel_car.gain[2]=2;//length fb
	#if USE_VER_3
	while(1){
		Delay_ms(1000);
		if(NAV_BOARD_CONNECT&&module.hml_imu!=0)
			break;
	}
	#if defined(USE_LED)
		LED_INIT();
	#endif
	if(module.acc_imu==2&&module.gyro_imu==2&&module.hml_imu==2&&1)
		Play_Music_Direct(START_BEEP);
	else
		Play_Music_Direct(MEMS_ERROR_BEEP);
	#endif	
	

	time_fly.sel=time_fly.state=time_fly.temp_cnt=0;
	for(i=0;i<10;i++){			 
	time_fly.cnt[i]=0;
	}
 	return (1);
}
