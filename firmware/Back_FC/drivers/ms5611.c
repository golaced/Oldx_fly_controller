

#include "ms5611.h"
#include "math.h"
#include "filter.h"

#define BARO_CAL_CNT 80


	s32 baro_height_old;
	
	s32 baro_Offset;
	
	uint32_t ms5611_ut;  // static result of temperature measurement
	uint32_t ms5611_up;  // static result of pressure measurement
	uint16_t ms5611_prom[PROM_NB];  // on-chip ROM
	uint8_t t_rxbuf[3],p_rxbuf[3];

void MS5611_Reset(void)
{
    IIC_Write_1Byte(MS5611_ADDR, CMD_RESET, 1);
}

u8 MS5611_Read_Prom(void)
{
	uint8_t rxbuf[2] = { 0, 0 };
	u8 check = 0;
	u8 i;

	for (i = 0; i < PROM_NB; i++)
	{
		check += IIC_Read_nByte(MS5611_ADDR, CMD_PROM_RD + i * 2, 2, rxbuf); // send PROM READ command
		ms5611_prom[i] = rxbuf[0] << 8 | rxbuf[1];
	}

	if(check==PROM_NB)
		return 1;
	else
		return 0;
}


void MS5611_Read_Adc_T(void)
{
	IIC_Read_nByte( MS5611_ADDR, CMD_ADC_READ, 3, t_rxbuf ); // read ADC
}

void MS5611_Read_Adc_P(void)
{
	IIC_Read_nByte(MS5611_ADDR, CMD_ADC_READ, 3, p_rxbuf); // read ADC
}

void MS5611_Start_T(void)
{
	IIC_Write_1Byte(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + MS5611_OSR, 1); // D2 (temperature) conversion start!
}

void MS5611_Start_P(void)
{
  IIC_Write_1Byte(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + MS5611_OSR, 1); // D1 (pressure) conversion start!
}

u8 ms5611_ok;
void MS5611_Init(void)
{
	
	Delay_ms(10);
	//传感器复位
	MS5611_Reset();
	Delay_ms(3);
	ms5611_ok = !( MS5611_Read_Prom() );
	//开始读取温度
	MS5611_Start_T();
}

int MS5611_Update(void)
{
	static int state = 0;
	
//	I2C_FastMode = 0;
	
	if (state) 
	{  
		 #if EN_ATT_CAL_FC
			MS5611_Read_Adc_P();
			MS5611_Start_T();
		 #endif
		 MS5611_BaroAltCalculate();
			state = 0;
		 
	} 
	else 
	{
		#if EN_ATT_CAL_FC
			MS5611_Read_Adc_T();
			MS5611_Start_P();
		#endif
			state = 1;
	}
	return (state);
}

float temperature_5611;

_height_st baro;

//Moving_Median(float moavarray[],u16 len ,u16 *fil_p,float in)

#define MO_LEN 5
	s32 mo_av_baro[MO_LEN];
	u16 moavcnt;

void MS5611_BaroAltCalculate(void)
{
	static u8 baro_start;
	
  int32_t temperature, off2 = 0, sens2 = 0, delt;
  int32_t pressure;
	float alt_3;
	
	int32_t dT;
	int64_t off;
	int64_t sens;
	
		static vs32 sum_tmp_5611 = 0;
		static u8 sum_cnt = BARO_CAL_CNT + 10;
	
	
		ms5611_ut = (t_rxbuf[0] << 16) | (t_rxbuf[1] << 8) | t_rxbuf[2];
		ms5611_up = (p_rxbuf[0] << 16) | (p_rxbuf[1] << 8) | p_rxbuf[2];
		
    dT = ms5611_ut - ((uint32_t)ms5611_prom[5] << 8);
    off = ((uint32_t)ms5611_prom[2] << 16) + (((int64_t)dT * ms5611_prom[4]) >> 7);
    sens = ((uint32_t)ms5611_prom[1] << 15) + (((int64_t)dT * ms5611_prom[3]) >> 8);
    temperature = 2000 + (((int64_t)dT * ms5611_prom[6]) >> 23);

    if (temperature < 2000) { // temperature lower than 20degC 
        delt = temperature - 2000;
        delt = delt * delt;
        off2 = (5 * delt) >> 1;
        sens2 = (5 * delt) >> 2;
        if (temperature < -1500) { // temperature lower than -15degC
            delt = temperature + 1500;
            delt = delt * delt;
            off2  += 7 * delt;
            sens2 += (11 * delt) >> 1;
        }
    }
    off  -= off2; 
    sens -= sens2;
    pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
		//pressure = (int)((1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f); // centimeter
		
		alt_3 = (101000 - pressure)/1000.0f;
		pressure = 0.82f *alt_3 * alt_3 *alt_3 + 0.09f *(101000 - pressure)*100.0f ;
							// - ( temperature_5611  ) *650 *(pressure/101000.0);
		
// 		if( pressure < 101000 )
// 		{
// 			pressure = 80 *my_pow( (101000 - pressure)/1000.0 ) + 0.08 *(101000 - pressure)*100.0;
// 			
// 		}
// 		else
// 		{
// 			pressure = baro_Offset;
// 		}
		
		//if(!(baro_Offset == 0))

				
			
			
			baro.height = (s32)( pressure  );//( my_deathzoom(( pressure - baro_Offset ), baroAlt ,baro_fix ) ) ; //cm + 
			//baro.relative_height = (pressure - baro_Offset);
			//baro_alt_speed += 10 *0.02 *3.14 *( 50 *( baroAlt - baroAltOld ) - baro_alt_speed ); // 20ms 一次 /0.02 = *50 单位cm/s
			
			baro.relative_height = Moving_Median(mo_av_baro,MO_LEN ,&moavcnt,(s32)(pressure - baro_Offset));
	///////////////////		
			baro.h_delta = ( baro.height - baro_height_old ) ; 
			//LPF_1_(1.0f,0.02f,( baro.height - baro_height_old ),baro.h_delta);
			
		  baro_height_old = baro.height;
			
			baro.measure_ok = 1;
			
			if( baro_start < 50 )
			{
				baro_start++;
				baro.h_delta = 0;

				baro.relative_height = 0;
				
				if(baro_start<10)
				{
					baro_Offset = pressure;
				}
				else
				{
					baro_Offset += 10.0f *3.14f *0.02f *(pressure - baro_Offset);
				}
			}	
			
		if(sum_cnt)
		{
			sum_cnt--;
// 			if(sum_cnt < BARO_CAL_CNT)
// 				sum_tmp_5611 += pressure;
// 			if(sum_cnt==0)
// 				baro_Offset = sum_tmp_5611 / (BARO_CAL_CNT - 1);
		}
		else
		{
			temperature_5611 += 0.01f *( ( 0.01f *temperature ) - temperature_5611 );
		}
			
		
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

