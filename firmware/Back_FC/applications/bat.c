#include "bat.h"
#include "include.h"		 
#include "mymath.h"	
#include "ctrl.h"	
//初始化ADC															   
void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

  //先初始化ADC1通道5 IO口
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//PA5 通道5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化  
 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 
 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
 
	ADC_Cmd(ADC1, ENABLE);//开启AD转换器	

}				  
//获得ADC值
//ch: @ref ADC_channels 
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
float k_ad=0.00875;
float Get_Adc(u8 ch)   
{ 
	ch=7;
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1)*k_ad;	//返回最近一次ADC1规则组的转换结果
}
//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		Delay_ms(5);
	}
	return temp_val/times;
} 
	 
BAT bat;
float k_fix_bat=0.567;
void Bat_protect(float dt)
{
static u8 state;
static u16 cnt[5];	
static  float temp,temp1;
	switch(state)
	{
		case 0:
			  if(Get_Adc(7)!=0){
			  temp+=Get_Adc(7);
				cnt[0]++;	
				}
				if(cnt[0]>20){
					state=1;
					temp1=temp/cnt[0];
					if(temp1>11.1-1.5&&temp1<12.6+1.5)
					  bat.bat_s=3;
					else if(temp1>14.8-1.5&&temp1<16.4+1.5)
						bat.bat_s=4;
					else
						bat.bat_s=0;
					bat.full=4.2*bat.bat_s;
					temp=0;
				}	  
		break;
	  case 1:
			 if(Get_Adc(7)!=0){
			  bat.origin=LIMIT(Get_Adc(7)+(float)thr_value*fly_ready/1000.*bat.bat_s*k_fix_bat,0,bat.full);
				temp+=bat.origin;
				cnt[1]++;	
				}
				if(cnt[1]>2/dt){		
					bat.average=temp/cnt[1];cnt[1]=0;
					if(bat.full>3.7)
						bat.percent=LIMIT((bat.average-bat.bat_s*3.65)/(bat.full-bat.bat_s*3.65),0,1);
					if(px4.connect)
						 bat.percent=px4.Bat/100.;
          temp=0;		
          switch(bat.bat_s){
            case 3:
						 	if(bat.percent<0.1)//(bat.average<11.1-0.6||bat.average<11.1-0.6+(12.6-11.1)*bat.protect_percent)
							bat.low_bat=1;	
							else			
							bat.low_bat=0;
						break;
						case 4:
						if(bat.percent<0.25)//bat.average<14.8-0.6||bat.average<14.8-0.6+(16.4-14.8)*bat.protect_percent)
							bat.low_bat_cnt++;
						else
							bat.low_bat_cnt=0;
						 if(bat.low_bat_cnt>2/0.05)
						  bat.low_bat=1;	
							else			
							bat.low_bat=0;
						break;;
 					}
				}	  
		break;
	
	}
}	









