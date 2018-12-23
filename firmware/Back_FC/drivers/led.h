#ifndef _LED_H_
#define	_LED_H_

#include "stm32f4xx.h"

#define LED1_OFF         ANO_GPIO_LED->BSRRL = ANO_Pin_LED1   //H
#define LED1_ON          ANO_GPIO_LED->BSRRH = ANO_Pin_LED1		//L
#define LED2_ON          ANO_GPIO_LED->BSRRL = ANO_Pin_LED2
#define LED2_OFF         ANO_GPIO_LED->BSRRH = ANO_Pin_LED2
#define LED3_ON          ANO_GPIO_LED->BSRRL = ANO_Pin_LED3
#define LED3_OFF         ANO_GPIO_LED->BSRRH = ANO_Pin_LED3
#define LED4_ON          ANO_GPIO_LED->BSRRL = ANO_Pin_LED4
#define LED4_OFF         ANO_GPIO_LED->BSRRH = ANO_Pin_LED4

#define LED1_OFF1         //GPIOA->BSRRL = GPIO_Pin_5   //H  UNUSED
#define LED1_ON1          //GPIOA->BSRRH = GPIO_Pin_5		//L
#define LED2_ON1          GPIOB->BSRRL = GPIO_Pin_12
#define LED2_OFF1         GPIOB->BSRRH = GPIO_Pin_12
#define LED4_ON1          GPIOA->BSRRL = GPIO_Pin_0
#define LED4_OFF1         GPIOA->BSRRH = GPIO_Pin_0
#define LED3_ON1          GPIOA->BSRRL = GPIO_Pin_11
#define LED3_OFF1         GPIOA->BSRRH = GPIO_Pin_11

/***************LED GPIO定义******************/
#define ANO_RCC_LED			RCC_AHB1Periph_GPIOC
#define ANO_GPIO_LED		GPIOC
#define ANO_Pin_LED1		GPIO_Pin_5//unused
#define ANO_Pin_LED2		GPIO_Pin_7
#define ANO_Pin_LED3		GPIO_Pin_11
#define ANO_Pin_LED4		GPIO_Pin_0
/*********************************************/
/*
//RGB_Info == 0，蓝色连续双闪1次：当前模式为3轴飞行器。
//RGB_Info == 1，蓝色连续双闪2次：当前模式为4轴飞行器。
//RGB_Info == 2，蓝色连续双闪3次：当前模式为6轴飞行器。
//RGB_Info == 3，蓝色连续双闪4次：当前模式为8轴飞行器。
//RGB_Info == 4，红色连续1次间隔闪烁：惯性传感器异常。
//RGB_Info == 5，红色连续2次间隔闪烁：电子罗盘异常。
//RGB_Info == 6，红色连续3次闪烁：气压计异常。
//RGB_Info == 7，绿色快速闪烁：传感器正常，自检校准中。
//RGB_Info == 8，开机红色常亮：自检失败，需重启。

//RGB_Info == 9，	呼吸白色：手动模式上锁（或姿态模式）。
//RGB_Info == 10，呼吸绿色：自动模式上锁（定点定高）。
//RGB_Info == 11，呼吸绿红色：GPS信号弱上锁（自动模式）。
//RGB_Info == 12，呼吸绿黄色：罗盘信号干扰上锁（自动模式）。

//RGB_Info == 13，双闪白白色：手动模式解锁（或姿态模式）。
//RGB_Info == 14，双闪绿绿色：自动模式解锁（定点定高）。
//RGB_Info == 15，双闪绿红色：GPS信号弱解锁（自动模式）。
//RGB_Info == 16，双闪绿黄色：罗盘信号干扰解锁（自动模式）。

//RGB_Info == 17，双闪蓝蓝色：返航模式。

//RGB_Info == 18，红色连续三闪：电量低。
//RGB_Info == 19，天蓝色快闪：罗盘校准中。
//RGB_Info == 20，深蓝色快闪：加速度计校准中。
//RGB_Info == 21，绿色快闪：陀螺校准中。
//RGB_Info == 22，红色常亮：校准失败。


RGB_Info == 9，	白色呼吸：手动油门模式上锁 
RGB_Info == 23, 绿色常亮：手动油门模式解锁
RGB_Info == 24, 黄色呼吸：气压定高上锁
RGB_Info == 25, 黄色常亮：气压定高解锁
RGB_Info == 26, 紫色呼吸：超声波定高上锁
RGB_Info == 27, 紫色常亮：超声波定高解锁
……………………
LED_Info == 待定
*/
enum  //led编号
{
	X1=0,
	B,
	R,
	G,
	LED_NUM,

};

typedef struct
{
	u8 RGB_Info;
	u8 LED_Info;

	u8 RGB_Info_old;
}LED_state;
extern LED_state light;

void LED_Init(void);
void LED_1ms_DRV(void );
void LED_Duty(float dt);
void LED_ON(u8 sel);
void LED_INIT(void);
void LED_MPU_Err(void);
void LED_Mag_Err(void);
void LED_MS5611_Err(void);
void aircraft_mode_led(u8 maxmotors);
void LEDRGB(void);
void LED_RGB_Init();
#endif
