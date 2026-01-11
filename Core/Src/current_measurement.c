/*
 * current_measurement.c
 *
 *  Created on: Jul 12, 2024
 *      Author: Gerry Geyer
 *
 *
 *      https://deepbluembedded.com/stm32-adc-multi-channel-scan-dma-poll-single-conversion/
 *
 *
 */
 #include <stdio.h>
#include "main.h"
#include "parameter.h"
#include "foc_math.h"
#include "current_measurement.h"
#include "observer.h"
#include "task.h"

extern DMA_HandleTypeDef hdma_adc2;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

uint32_t adc_buffer_voltage;
int16_t adc_value;


abi32_t current_value_t,current_value_dc_t;
abi32_t debug_current_after_dc;



ab_f dc_value, lp_z_memory, current_debug_f;


ab_f cur_val_dc;
ab16_t offset;
int16_t cur_lp_a;
ab16_t I_buffer;

uint16_t offset_counter;
ab32_t offset_val;

abi32_t debug_current, debug_current_raw;
int16_t IIR_DC_FILTER_A;
float IIR_DC_Filter_f, ONE_MINUS_IIR_DC_Filter_f;




void init_current_measurement(void){

	offset_counter = 0;


	I_buffer.a = 0;
	I_buffer.b = 0;

	lp_z_memory.a = 0.0;
	lp_z_memory.b = 0.0;

	adc_value = 0;
	adc_buffer_voltage = 0;

	IIR_DC_FILTER_A = (int16_t)((2.0f * (float)IIR_ZERO_A * (float)Q15) + 0.5f);
	IIR_DC_Filter_f = IIR_ZERO_A;
	ONE_MINUS_IIR_DC_Filter_f = (1.0f - IIR_ZERO_A);

}


/*
 * read the values from the ADCs. Then calculate the resulting Current and remove the DC component
 */
void execute_current_measurement(void){
	ab32_t I;

	static ab32_t cur_last_val = {0, 0};

	I.a = debug_current_raw.a =  (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	I.b =  debug_current_raw.b = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

	int32_t fixed_offset = 2125; // value comes from circuit

	current_value_t.a = -((int32_t)I.a - fixed_offset);
	current_value_t.b = -((int32_t)I.b - fixed_offset);

	current_value_t.a = debug_current.a = ((int32_t)current_value_t.a * (int32_t)COUNT_TO_AMP_t) >> DIV_MAX_CURRENT_Q15;	// multiplicate with 0.02822876 * 2^15
	current_value_t.b = debug_current.b = ((int32_t)current_value_t.b * (int32_t)COUNT_TO_AMP_t) >> DIV_MAX_CURRENT_Q15;

	/*
	 * info:
	 * IIR_DC_FILTER_A = (int16_t)((float)(IIR_ZERO_A * (float)UINT16_MAX_VALUE));
	 * #define IIR_ZERO_A			((PI_MULTIPLY_2 * FOC_TS * 1.0f)/(PI_MULTIPLY_2 * FOC_TS * 1.0f + 1.0f))
	 */



	dc_value.a = IIR_DC_Filter_f * (float)current_value_t.a + ONE_MINUS_IIR_DC_Filter_f * cur_val_dc.a;
	dc_value.b = IIR_DC_Filter_f * (float)current_value_t.b + ONE_MINUS_IIR_DC_Filter_f * cur_val_dc.b;

	cur_val_dc.a = dc_value.a;
	cur_val_dc.b = dc_value.b;


	current_value_t.a -= CLAMP_INT32_TO_INT16((int32_t)(cur_val_dc.a + 0.5f));
	current_value_t.b -= CLAMP_INT32_TO_INT16((int32_t)(cur_val_dc.b + 0.5f));
	debug_current_after_dc.a = current_value_t.a;
	debug_current_after_dc.b = current_value_t.b;

	/*
	 * IIR-Filter with fixed parameter:
	 * 5500RPM max speed -> 5500 * (polepair/60) Hz
	 *
	 *
	 * a =  0.1678 -> a_q15 = 5497
	 * (1-a) = 0.8322 -> (1-a)_q15 = 27271
	 *
	 */


	int32_t x,y, out;
	x = (5497 * current_value_t.a);
	y = (27271 * cur_last_val.a);
	out = (x + y + (1 << 14)) >> 15;
	current_value_t.a = CLAMP_INT32_TO_INT16(out);
	cur_last_val.a = current_value_t.a;

	x = (5497 * current_value_t.b);
	y = (27271 * cur_last_val.b);
	out = (x + y + (1 << 14)) >> 15;
	current_value_t.b = CLAMP_INT32_TO_INT16(out);
	cur_last_val.b = current_value_t.b;

}

/*
 * get the current value from the last current measurement
 */

ab_t get_realCurrentQ15(void){
	ab_t Output;
	Output.a = CLAMP_INT32_TO_INT16(current_value_t.a);
	Output.b = CLAMP_INT32_TO_INT16(current_value_t.b);
	return (Output);
}

void execute_voltage_measurement(void){
	HAL_ADC_Start_DMA(&hadc2, &adc_buffer_voltage, 1);
}

int16_t get_voltage_value(void){
	return (adc_value);
}

void read_voltage_value(FOC_HandleTypeDef *pHandle_foc){
	pHandle_foc->source_voltage = adc_value;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//	int32_t x = (int32_t)adc_buffer * 404423;
//	x /= 11796;
//	uint32_t x = (int32_t)adc_buffer_voltage * Battery_Voltage_Factor_Q7; // Q12 + Q7 = Q19
//
////	x = (x >> 5); // Divide with max Voltage (= V_max = 32V) (Q19)
////	x = (x >> 4); // Scale back to Q15
//	// simplify:
//	x = (x >> 9);
//
	uint32_t x = (int32_t)adc_buffer_voltage * BAT_VOLT32_Q25; // Q12 + Q7 = Q19

//	x = (x >> 5); // Divide with max Voltage (= V_max = 32V) (Q19)
//	x = (x >> 4); // Scale back to Q15
	// simplify:
	x = (x >> 10);

	adc_value = CLAMP_INT32_TO_INT16((int32_t)x); // V = x* 32/Q15
}



