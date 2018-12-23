#include "include.h"
#include "ultrasonic.h"
#include "usart.h"


void Ultrasonic_Init()
{
  Usart3_Init(9600);	
}

s8 ultra_start_f;

void Ultra_Duty()
{
	u8 temp[3];

	ultra.h_dt = 0.05f; 
	#if defined(USE_KS103)

	#elif defined(USE_US100)
	#if SONAR_USE_FC1
	  temp[0]=0x55;
	  USART_SendData(UART5, 0x55); 
	#else
		Uart3_Send(0x55);
	#endif
	#endif
///////////////////////////////////////////////
		ultra_start_f = 1;

		if(ultra.measure_ot_cnt<200) //200ms
		{
			ultra.measure_ot_cnt += ultra.h_dt *1000;
		}
		else
		{
			ultra.measure_ok = 0;//超时，复位
		}
}

u16 ultra_distance_old;

_height_st ultra;

static float sonar_values[3] = { 0.0f };
static unsigned insert_index = 0;

static void sonar_bubble_sort(float sonar_values[], unsigned n);

void sonar_bubble_sort(float sonar_values[], unsigned n)
{
	float t;

	for (unsigned i = 0; i < (n - 1); i++) {
		for (unsigned j = 0; j < (n - i - 1); j++) {
			if (sonar_values[j] > sonar_values[j+1]) {
				t = sonar_values[j];
				sonar_values[j] = sonar_values[j + 1];
				sonar_values[j + 1] = t;
			}
		}
	}
}

float insert_sonar_value_and_get_mode_value(float insert)
{
	const unsigned sonar_count = sizeof(sonar_values) / sizeof(sonar_values[0]);
	sonar_values[insert_index] = insert;
	insert_index++;
	if (insert_index == sonar_count) {
		insert_index = 0;
	}
	float sonar_temp[sonar_count];
	memcpy(sonar_temp, sonar_values, sizeof(sonar_values));
	sonar_bubble_sort(sonar_temp, sonar_count);
	return sonar_temp[sonar_count / 2];
}


void Ultra_Get(u8 com_data)
{
	static u8 ultra_tmp;
	
	if( ultra_start_f == 1 )
	{
		ultra_tmp = com_data;
		ultra_start_f = 2;
	}
	else if( ultra_start_f == 2 )
	{
		ultra.height =  ((ultra_tmp<<8) + com_data);
		
		if(ultra.height < 5000) // 5米范围内认为有效，跳变值约10米.
		{
			ultra.relative_height =insert_sonar_value_and_get_mode_value((float)ultra.height/1000.);
			ultra.measure_ok = 1;		
		}
		else
		{
			ultra.measure_ok = 2; //数据超范围
		}
		ultra_start_f = 0;
	}
	ultra.measure_ot_cnt = 0; //清除超时计数（喂狗）
	ultra.h_delta = ultra.relative_height - ultra_distance_old;
	ultra_distance_old = ultra.relative_height;
}

