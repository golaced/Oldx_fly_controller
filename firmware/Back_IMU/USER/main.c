#include "include.h" 
#include "iic_soft.h"
#include "iic_imu1.h"
#include "iic2.h"
#include "hml_sample.h"
#include "ms5611.h"
#include "alt_kf.h"
#include "flash.h"
#include "led_fc.h"
#include "bmp_adc.h"
#include "flash_w25.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "dog.h"
#include "ukf_task.h"
#include "stm32f4xx_dma.h"
#include "LSM303.h"
#include "Soft_I2C_PX4.h"
#include "LIS3MDL.h"
#include "nav_ukf.h"
#include "gps.h"
 /////////////////////////UCOSII启动任务设置///////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			20 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				64
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);	
//////////////////////////////////////////////////////////////////////////////
    
OS_EVENT * msg_key;			//按键邮箱事件块	  
OS_EVENT * q_msg;			//消息队列

OS_FLAG_GRP * flags_key;	//按键信号量集
void * MsgGrp[256];			//消息队列存储地址,最大支持256个消息
SYSTEM module;
uint16_t cpuGetFlashSize(void)
{
   return (*(__IO u16*)(0x1FFF7A22));
}

//读取ChipID
u32 mcuID[3];
void cpuidGetId(void)
{
    mcuID[0] = *(__IO u32*)(0x1FFF7A10);
    mcuID[1] = *(__IO u32*)(0x1FFF7A14);
    mcuID[2] = *(__IO u32*)(0x1FFF7A18);
}

float ekf_loop_time1;
int main(void)

{ 
	NVIC_PriorityGroupConfig(NVIC_GROUP);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	Initial_Timer_SYS();
	LED_Init();								//LED功能初始化
	Delay_ms(100);
	//-------------------------Para Init------------------------------------	
	W25QXX_Init();			//W25QXX初始化
	READ_PARM();//读取参数
	Delay_ms(4000);
  IIC_IMU1_Init();

	#if IMU_UPDATE
	LIS3MDL_enableDefault();
	Delay_ms(10);						//延时
	#endif
	Drv_GpsPin_Init6();
	Delay_ms(500);
//------------------------Uart Init-------------------------------------
	#if USE_OUTER_LINK
	Usart1_Init(57600);			//Outer Linker
	#else
	Drv_GpsPin_Init();
	#endif
	
	#if EN_DMA_UART1 
	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF_SIZE1+2,1);
	#endif
	Usart2_Init(576000L);			//IMU_LINK
	#if EN_DMA_UART2
	MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)SendBuff2,SEND_BUF_SIZE2+2,1);
	#endif
	#if defined(USE_WIFI_CONTROL)
	Usart4_Init(19200);
	#else
		#if USE_LASER_AVOID
		Usart4_Init(576000L);     //AVOID BOARD
		#else
			#if USE_M100_IMU
			Usart4_Init(115200L);     //IMU2 Link
			#else
			Usart4_Init(576000L);     //PI FLOW
			#endif
		#endif
	#endif
	#if USE_IMU_BACK_IO_AS_SONAR
	 #if defined (USE_LIDAR)
   	Usart4_Init(115200L); 
	 #else
	 	Usart4_Init(9600);
	#endif
	#endif
	
	#if EN_DMA_UART4 
	MYDMA_Config(DMA1_Stream4,DMA_Channel_4,(u32)&UART4->DR,(u32)SendBuff4,SEND_BUF_SIZE4+2,0);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
	#if defined(SONAR_USE_UART)  
		#if defined(URM07)
		Usart3_Init(19200); 
		#else
		Usart3_Init(9600L);    	
		#endif
		#if defined(USE_LIDAR)
		  Usart3_Init(115200);
		#endif
	#else
	Ultrasonic_Init();
	#endif
	#if EN_DMA_UART3
	MYDMA_Config(DMA1_Stream3,DMA_Channel_4,(u32)&USART3->DR,(u32)SendBuff3,SEND_BUF_SIZE3+2,2);
	#endif

	#if FLOW_USE_IIC
	Soft_I2C_Init_PX4();      //FLOW PX4 IIC
	#else
  Uart5_Init(576000);			//FLOW PX4
	#endif
	#if FLOW_USE_OPENMV
  Uart5_Init(576000L);			//FLOW PX4
  #endif
	#if USE_ANO_FLOW
	Uart5_Init(500000L);			
	#endif
	#if USE_UWB_AS_POS
		Usart4_Init(460800);
	#endif
	#if USE_GPS_U4
		Drv_GpsPin_Init();
	#endif

//-----------------------Mode &  Flag init--------------------	
//--system
	fly_ready=0;
	mode.en_imu_ekf=0;
	three_wheel_car.gain[0]=1.68;
	three_wheel_car.car_mode=1;//0 遥控   2 前进 1 往返 3 圆
	//-----------------DMA Init--------------------------
#if EN_DMA_UART4 
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);    
	MYDMA_Enable(DMA1_Stream4,SEND_BUF_SIZE4+2);     
#endif
#if EN_DMA_UART2
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE); 
	MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);    
#endif
#if EN_DMA_UART3
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);   
	MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE3+2);    
#endif
#if EN_DMA_UART1 
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);   
	MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);    
#endif	

  TIM3_Int_Init(50-1,8400-1);	
	Delay_ms(20);
	Outer_Compass_init();//外置罗盘初始化
	IWDG_Init(4,500*3); //与分频数为64,重载值为500,溢出时间为1s	
	OSInit();  	 				
	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();	    
}
 

//开始任务
void start_task(void *pdata)
{
  OS_CPU_SR cpu_sr=0;
	u8 err;	    	    
	pdata = pdata; 	
	msg_key=OSMboxCreate((void*)0);		//创建消息邮箱
	q_msg=OSQCreate(&MsgGrp[0],256);	//创建消息队列
 	flags_key=OSFlagCreate(0,&err); 	//创建信号量集		  
	  
	OSStatInit();					//初始化统计任务.这里会延时1秒钟左右
	//注册软件定时器
	tmr1=OSTmrCreate(10,10,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)tmr1_callback,0,"tmr1",&err);		//100ms执行一次  cpu使用率
	OSTmrStart(tmr1,&err);//启动软件定时器1				 	
	tmr2=OSTmrCreate(10,5,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)tmr2_callback,0,"tmr2",&err);		//50ms执行一次  LED&&MODE
	OSTmrStart(tmr2,&err);//启动软件定时器1				 	
 	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)    
 	//注册线程 	
	#if !USE_UKF_FROM_AUTOQUAD	
	OSTaskCreate(outer_task,(void *)0,(OS_STK*)&OUTER_TASK_STK[OUTER_STK_SIZE-1],OUTER_TASK_PRIO);
	#endif
	#if !EN_TIM_INNER
	OSTaskCreate(inner_task,(void *)0,(OS_STK*)&INNER_TASK_STK[INNER_STK_SIZE-1],INNER_TASK_PRIO);
	#endif
	#if !UKF_IN_ONE_THREAD
	OSTaskCreate(ekf_task,(void *)0,(OS_STK*)&EKF_TASK_STK[EKF_STK_SIZE-1],EKF_TASK_PRIO);
	#endif
	OSTaskCreate(flow_task1,(void *)0,(OS_STK*)&FLOW_TASK_STK[FLOW_STK_SIZE-1],FLOW_TASK_PRIO);
	#if !UKF_IN_ONE_THREAD
	OSTaskCreate(baro_task,(void *)0,(OS_STK*)&BARO_TASK_STK[BARO_STK_SIZE-1],BARO_TASK_PRIO);
	#endif
	OSTaskCreate(sonar_task,(void *)0,(OS_STK*)&SONAR_TASK_STK[SONAR_STK_SIZE-1],SONAR_TASK_PRIO);	
	OSTaskCreate(error_task,(void *)0,(OS_STK*)&ERROR_TASK_STK[ERROR_STK_SIZE-1],ERROR_TASK_PRIO);
	//--
 	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
}
   

//信号量集处理任务
void flags_task(void *pdata)
{	
	u16 flags;	
	u8 err;	    						 
	while(1)
	{
		flags=OSFlagPend(flags_key,0X001F,OS_FLAG_WAIT_SET_ANY,0,&err);//等待信号量 		
		OSFlagPost(flags_key,0X001F,OS_FLAG_CLR,&err);//全部信号量清零
 	}
}
   		    


