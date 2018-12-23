#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"


extern OS_TMR   * tmr1;			//软件定时器1
extern OS_TMR   * tmr2;			//软件定时器2
extern OS_TMR   * tmr3;			//软件定时器3
void tmr1_callback(OS_TMR *ptmr,void *p_arg); 		  	   
void tmr2_callback(OS_TMR *ptmr,void *p_arg); 	  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg); 
	
//设置任务优先级 0->Highest     
//10->开始任务的优先级设置为最低
#define MEMS_TASK_PRIO       			  1 //MEMS
#define INNER_TASK_PRIO       			2 //内环
#define OUTER_TASK_PRIO       			3 //外环
#define EKF_TASK_PRIO       			  4 //EKF
#define FLOW_TASK_PRIO       			  5 //EKF


#define POS_TASK_PRIO       			  14 //位置
#define BARO_TASK_PRIO       			  15 //气压计采集与UKF
#define SONAR_TASK_PRIO       			16 //超声波采集
#define NRF_TASK_PRIO       			  17 //射频通讯
#define UART_TASK_PRIO       			  18 //串口通讯

#define ERROR_TASK_PRIO       			19 //故障
//defin START_TASK_PRIO      			  20//开始任务的优先级设置为最低
//-----------------------MEMS解算线程
//设置任务堆栈大小
#define MEMS_STK_SIZE  					64*3
//任务堆栈	
extern OS_STK MEMS_TASK_STK[MEMS_STK_SIZE];
//任务函数
void mems_task(void *pdata);

//-----------------------INNER解算线程
//设置任务堆栈大小
#define INNER_STK_SIZE  					64*6
//任务堆栈	
extern OS_STK INNER_TASK_STK[INNER_STK_SIZE];
//任务函数
void inner_task(void *pdata);

//------------------------OUTER解算线程
//设置任务堆栈大小
#define OUTER_STK_SIZE  					64*4*6
//任务堆栈	
extern OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];
//任务函数
void outer_task(void *pdata);

//------------------------EKF解算线程
//设置任务堆栈大小
#define EKF_STK_SIZE  					64*9*9
//任务堆栈	
extern OS_STK EKF_TASK_STK[EKF_STK_SIZE];
//任务函数
void ekf_task(void *pdata);


//------------------------POS控制线程
//设置任务堆栈大小
#define POS_STK_SIZE  					64*2
//任务堆栈	
extern OS_STK POS_TASK_STK[POS_STK_SIZE];
//任务函数
void pos_task(void *pdata);

//------------------------NRF算线程
//设置任务堆栈大小
#define NRF_STK_SIZE  					64*2
//任务堆栈	
extern OS_STK NRF_TASK_STK[NRF_STK_SIZE];
//任务函数
void nrf_task(void *pdata);

//------------------------BARO线程
//设置任务堆栈大小
#define BARO_STK_SIZE  					64*8
//任务堆栈	
extern OS_STK BARO_TASK_STK[BARO_STK_SIZE];
//任务函数
void baro_task(void *pdata);

//-----------------------SONAR线程
//设置任务堆栈大小
#define SONAR_STK_SIZE  					64*2
//任务堆栈	
extern OS_STK SONAR_TASK_STK[SONAR_STK_SIZE];
//任务函数
void sonar_task(void *pdata);

//------------------------UART线程
//设置任务堆栈大小
#define UART_STK_SIZE  					64*2
//任务堆栈	
extern OS_STK UART_TASK_STK[UART_STK_SIZE];
//任务函数
void uart_task(void *pdata);
//

//-----------------------FLOW线程
//设置任务堆栈大小
#define FLOW_STK_SIZE  					64*3
//任务堆栈	
extern OS_STK FLOW_TASK_STK[FLOW_STK_SIZE];
//任务函数
void flow_task1(void *pdata);

//-----------------------ERROR线程
//设置任务堆栈大小
#define ERROR_STK_SIZE  					64
//任务堆栈	
extern OS_STK ERROR_TASK_STK[ERROR_STK_SIZE];
//任务函数
void error_task(void *pdata);
//
#endif

