#include "include.h"

void NMI_Handler(void)
{
}

void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void SysTick_Handler(void)
{
	sysTickUptime++;
	sys_time();
}



//void USART2_IRQHandler(void)
//{
//	Usart2_IRQ();
//}

//void UART5_IRQHandler(void)
//{
//	Uart5_IRQ();
//}
