/*
 * current_measurement.c
 *
 *  Created on: Jul 12, 2024
 *      Author: Gerry Geyer
 *
 *
 */
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "parameter.h"
#include "foc_math.h"
#include "current_measurement.h"
#include "motor_events.h"



extern DMA_HandleTypeDef hdma_adc2;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

static void ntc_lut_init(void);
static void median_filter_ab(abi32_t *x);

uint32_t adc_buffer_voltage = 0, adc_buffer_temperature = 0;

abi32_t current_value_t;


abi32_t debug_current, debug_current_raw;
float IIR_DC_Filter_f, ONE_MINUS_IIR_DC_Filter_f;
int16_t IIR_Filter_q15, ONE_MINUS_IIR_Filter_q15;

int16_t max_voltage_value_q15;


void init_current_measurement(void){


	IIR_DC_Filter_f = (PI_MULTIPLY_2 * FOC_TS * 1.0f)/(PI_MULTIPLY_2 * FOC_TS * 1.0f + 1.0f);
	ONE_MINUS_IIR_DC_Filter_f = (float)(1.0f - IIR_DC_Filter_f);

	float fc = ((float)BANDWIDTH_CURRENT/(float)PI_MULTIPLY_2) * 5.0f; // cutoff frequency for current IIR filter, we set it to 5 times the bandwidth for good noise reduction but still good dynamic response
	float iir_f = ((PI_MULTIPLY_2 * FOC_TS * fc)/(PI_MULTIPLY_2 * FOC_TS * fc + 1.0f));
	float one_minus_iir_f = (float)(1.0f - iir_f);
	IIR_Filter_q15 = (int16_t)((iir_f * (float)Q15) + 0.5f);
	ONE_MINUS_IIR_Filter_q15 = (int16_t)((one_minus_iir_f * (float)Q15) + 0.5f);

	if(OVER_VOLTAGE_VALUE >= 32 || OVER_VOLTAGE_VALUE <= 0){
		max_voltage_value_q15 = 32767;
	}else{
		max_voltage_value_q15 = (int16_t)((float)OVER_VOLTAGE_VALUE * 32768.0f / 32.0f); 
	}
	ntc_lut_init();
}


/*
 * read the values from the ADCs. Then calculate the resulting Current and remove the DC component
 */
void execute_current_measurement(FOC_HandleTypeDef *pHandle_foc, CurrMeasState mode){
	ab32_t I;

	static ab32_t cur_last_val = {0, 0};
	static ab_f dc_value = {0.0f, 0.0f}, cur_val_dc = {0.0f, 0.0f};

	I.a = debug_current_raw.a =  (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	I.b =  debug_current_raw.b = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

	int32_t fixed_offset = 2553; // value comes from circuit -> offset = (0,892/(22,892))*(2^12)*16

	current_value_t.a = debug_current_raw.a = -((int32_t)I.a - fixed_offset);
	current_value_t.b = debug_current_raw.b = -((int32_t)I.b - fixed_offset);

	// Convert to Q15 current value -> gain compensation from circuit and resqale to Q15 (max current 32A [Q5])
	current_value_t.a = debug_current.a = ((int32_t)current_value_t.a * (int32_t)COUNT_TO_AMP_t) >> DIV_MAX_CURRENT_Q15;	// multiplicate with 0.02822876 * 2^15
	current_value_t.b = debug_current.b = ((int32_t)current_value_t.b * (int32_t)COUNT_TO_AMP_t) >> DIV_MAX_CURRENT_Q15;

	switch(mode){
		case (STATIONARY):
		/*
		* DC removal with IIR-Filter (float point unit) [in fixed point we need 64 bit precision for good results]
		* info:
		* IIR_DC_FILTER_A = (int16_t)((float)(IIR_ZERO_A * (float)UINT16_MAX_VALUE));
		* #define IIR_ZERO_A			((PI_MULTIPLY_2 * FOC_TS * 1.0f)/(PI_MULTIPLY_2 * FOC_TS * 1.0f + 1.0f))
		*/

		dc_value.a = IIR_DC_Filter_f * (float)current_value_t.a + ONE_MINUS_IIR_DC_Filter_f * cur_val_dc.a;
		dc_value.b = IIR_DC_Filter_f * (float)current_value_t.b + ONE_MINUS_IIR_DC_Filter_f * cur_val_dc.b;

		cur_val_dc.a = dc_value.a;
		cur_val_dc.b = dc_value.b;
		break;

		default:	
		// use estimatated DC value from stationary mode
		break;
	}

		current_value_t.a -= CLAMP_INT32_TO_INT16((int32_t)(cur_val_dc.a + 0.5f)); // +0.5 for correct fixed point rounding
		current_value_t.b -= CLAMP_INT32_TO_INT16((int32_t)(cur_val_dc.b + 0.5f));

		debug_current.a =(int32_t)(cur_val_dc.a + 0.5f);
		debug_current.b = (int32_t)(cur_val_dc.b + 0.5f);

	// median_filter_ab(&current_value_t);	
	

	int32_t x,y, out;
	x = (IIR_Filter_q15 * current_value_t.a);
	y = (ONE_MINUS_IIR_Filter_q15 * cur_last_val.a);
	out = (x + y + (1 << 14)) >> 15;
	current_value_t.a = CLAMP_INT32_TO_INT16(out);
	cur_last_val.a = current_value_t.a;

	x = (IIR_Filter_q15 * current_value_t.b);
	y = (ONE_MINUS_IIR_Filter_q15 * cur_last_val.b);
	out = (x + y + (1 << 14)) >> 15;
	current_value_t.b = CLAMP_INT32_TO_INT16(out);
	cur_last_val.b = current_value_t.b;

	pHandle_foc->I_ab_q15.a = CLAMP_INT32_TO_INT16(current_value_t.a);
	pHandle_foc->I_ab_q15.b = CLAMP_INT32_TO_INT16(current_value_t.b);

	// over current protection

}
// ############# VOLTAGE AND TEMPERATURE MEASUREMENT #############
/**
 * NTC temperature estimation using an ADC lookup table.
 *
 * The ADC raw value of an NTC voltage divider is converted to an approximate
 * temperature in °C. A precomputed lookup table defines ADC values for
 * temperature points from −10 °C to +140 °C in 10 °C steps.
 *
 * At runtime, the code selects the corresponding LUT segment and performs
 * linear interpolation. To minimize CPU load, the interpolation gain is
 * precomputed in Q15 fixed-point format, avoiding divisions and floating-
 * point operations.
 *
 * The returned value is an integer temperature in degrees Celsius.
 * Accuracy is sufficient for over-temperature detection and protection,
 * not for precise temperature measurement.
 */

int16_t adc_value, temperature_value;

void execute_voltage_measurement(void){
	HAL_ADC_Start_DMA(&hadc2, &adc_buffer_voltage, 1);
}

void execute_temperature_measurement(void){
	HAL_ADC_Start_DMA(&hadc1, &adc_buffer_temperature, 1);
}

void read_voltage_value(FOC_HandleTypeDef *pHandle_foc){
	pHandle_foc->source_voltage = adc_value;
}

void read_temperature_value(FOC_HandleTypeDef *pHandle_foc){
	pHandle_foc->temperature = temperature_value;
}


const uint16_t adc_lut_raw[16] = {
   306, 
   502, 
   774, 
  1117, 
  1511, 
  1924, 
  2322, 
  2678, 
  2979, 
  3224, 
  3416, 
  3566, 
  3681, 
  3769, 
  3836, 
  3888

};


static uint16_t k_q15[15]; // k_q15[i] = (10<<15)/(a2-a1)

static void ntc_lut_init(void)
{
    for (int i = 0; i < 15; i++) {
        uint32_t den = (uint32_t)(adc_lut_raw[i+1] - adc_lut_raw[i]);
        k_q15[i] = (uint16_t)(((uint32_t)10 << 15) / den);
    }
}

static inline int16_t adc_to_temp_q15(uint16_t adc_raw)
{
    const int16_t T0 = -10;

    if (adc_raw <= adc_lut_raw[0])  return T0;
    if (adc_raw >= adc_lut_raw[15]) return (int16_t)(T0 + 150);

    uint8_t i = 0;
    while (adc_raw >= adc_lut_raw[i+1]) i++;

    uint16_t a1 = adc_lut_raw[i];
    uint16_t dx = (uint16_t)(adc_raw - a1);

    // deltaT = (dx * k_q15[i]) >> 15   (entspricht 10*dx/(a2-a1))
    int16_t dT = (int16_t)(((int32_t)dx * (int32_t)k_q15[i]) >> 15);

    return (int16_t)(T0 + 10*i + dT);
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  	if (hadc->Instance == ADC1)
    {
        temperature_value = adc_to_temp_q15(adc_buffer_temperature);
		if(temperature_value >= OVER_TEMPERATURE_VALUE){
			ot_trip = true;	
		}
    }
	if (hadc->Instance == ADC2)
    {
        uint32_t x = (int32_t)adc_buffer_voltage * BAT_VOLT32_Q25; // Q12 + Q7 = Q19

		//	x = (x >> 5); // Divide with max Voltage (= V_max = 32V) (Q19)
		//	x = (x >> 4); // Scale back to Q15
		// simplify:
		x = (x >> 10);
		adc_value = CLAMP_INT32_TO_INT16((int32_t)x); // V = x* 32/Q15
		if(adc_value >= max_voltage_value_q15){
			ov_trip = true;
		}
    }

}



#define MEDIAN_WIN 3 

// Hilfsfunktion zum Sortieren eines kleinen Arrays
static void sort_int16(int16_t *v, uint8_t len){
    for(uint8_t i=0; i<len-1; i++){
        for(uint8_t j=i+1; j<len; j++){
            if(v[j]<v[i]){
                int16_t tmp = v[i];
                v[i] = v[j];
                v[j] = tmp;
            }
        }
    }
}


static void median_filter_ab(abi32_t *x)
{
    static int16_t buf_a[MEDIAN_WIN] = {0};
    static int16_t buf_b[MEDIAN_WIN] = {0};
    static uint8_t idx = 0;

    idx = (uint8_t)((idx + 1U) % MEDIAN_WIN);

    /* neuen Wert einspeisen */
    buf_a[idx] = CLAMP_INT32_TO_INT16(x->a);
    buf_b[idx] = CLAMP_INT32_TO_INT16(x->b);

    /* Kopien fürs Sortieren */
    int16_t tmp_a[MEDIAN_WIN];
    int16_t tmp_b[MEDIAN_WIN];

    for(uint8_t k = 0; k < MEDIAN_WIN; k++){
        tmp_a[k] = buf_a[k];
        tmp_b[k] = buf_b[k];
    }

    sort_int16(tmp_a, MEDIAN_WIN);
    sort_int16(tmp_b, MEDIAN_WIN);

    /* Median wählen (bei geradem WIN ist das das "obere" mittlere Element) */
    x->a = tmp_a[MEDIAN_WIN / 2U];
    x->b = tmp_b[MEDIAN_WIN / 2U];
}
