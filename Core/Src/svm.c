/*
 * svm.c
 *
 *  Created on: Jul 4, 2024
 *      Author: Gerry Geyer
 */

#include <stdio.h>
#include <stdlib.h>
#include <main.h>
#include <math.h>
#include <foc_math.h>
#include <foc.h>
#include <svm.h>
#include <settings.h>

extern TIM_HandleTypeDef htim1;

svm_output pwm_output;


svm_output_f voltage;

void init_SVM(void){
	pwm_output.Sa = 0;
	pwm_output.Sb = 0;
	pwm_output.Sc = 0;

}




void generate_svm_output(void){

	TIM1->CCR1 = pwm_output.Sa;
	TIM1->CCR2 = pwm_output.Sb;
	TIM1->CCR3 = pwm_output.Sc;
}

void set_output_to_zero(void){

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
}

svm_output_f get_abc_output_voltage(void){
	return (voltage);
}

abc_16t SVM_Q15(abc_16t v){
	abc_16t Output;
	int32_t x, min, max,mid;

	max = MAX(MAX(v.a, v.b), v.c);
	min = MIN(MIN(v.a, v.b), v.c);
	mid = ((max+min) >> 1);

	x = ((((int32_t)v.a - mid) * DIV_2_SQRT3Q14) >> 14);
	Output.a = CLAMP_INT32_TO_INT16(x);

	x = ((((int32_t)v.b - mid) * DIV_2_SQRT3Q14) >> 14);
	Output.b = CLAMP_INT32_TO_INT16(x);

	x = ((((int32_t)v.c - mid) * DIV_2_SQRT3Q14) >> 14);
	Output.c = CLAMP_INT32_TO_INT16(x);
	return (Output);
}

svm_output get_PWM_OUTPUT_Q15(abc_16t v){
	svm_output Output;

	if(SPACE_VECTOR_MODULAION == ON){
		v = SVM_Q15(v);
	}

	Output.Sa		= (((int32_t)v.a + Q15) >> 1);
	Output.Sb		= (((int32_t)v.b + Q15) >> 1);
	Output.Sc		= (((int32_t)v.c + Q15) >> 1);

	// Scale to max timer Output
	Output.Sa 		= (Output.Sa * TIM1_COUNTER) >> 15;
	Output.Sb 		= (Output.Sb * TIM1_COUNTER) >> 15;
	Output.Sc 		= (Output.Sc * TIM1_COUNTER) >> 15;

	Output.Sa 		= CLAMP(Output.Sa, 0, TIM1_COUNTER);
	Output.Sb 		= CLAMP(Output.Sb, 0, TIM1_COUNTER);
	Output.Sc 		= CLAMP(Output.Sc, 0, TIM1_COUNTER);

	return (Output);
}


