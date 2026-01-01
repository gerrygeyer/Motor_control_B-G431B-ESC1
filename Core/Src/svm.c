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

svm_output_f debug_svm; // DEBUG
svm_output pwm_output;


svm_output_f voltage;

void init_SVM(void){
	pwm_output.Sa = 0;
	pwm_output.Sb = 0;
	pwm_output.Sc = 0;

}


svm_output execute_SVM_matlab(abc_f v){
	svm_output Output;
	abc_f svm_voltage, v_norm;


	v.a = (v.a > MAX_VOLTAGE) ? MAX_VOLTAGE:v.a;
	v.a = (v.a < -MAX_VOLTAGE) ? -MAX_VOLTAGE:v.a;
	v.b = (v.b > MAX_VOLTAGE) ? MAX_VOLTAGE:v.b;
	v.b = (v.b < -MAX_VOLTAGE) ? -MAX_VOLTAGE:v.b;
	v.c = (v.c > MAX_VOLTAGE) ? MAX_VOLTAGE:v.c;
	v.c = (v.c < -MAX_VOLTAGE) ? -MAX_VOLTAGE:v.c;


	v_norm.a = v.a/MAX_VOLTAGE;
	v_norm.b = v.b/MAX_VOLTAGE;
	v_norm.c = v.c/MAX_VOLTAGE;

	if(SPACE_VECTOR_MODULAION == ENABLE){
		svm_voltage = SVM_input_Va_Vb_Vc(v_norm);
	}else{
		svm_voltage = Sinus_Va_Vb_Vc(v_norm);
	}

	voltage.Sa = svm_voltage.a; // DEBUG
	voltage.Sb = svm_voltage.b; // DEBUG
	voltage.Sc = svm_voltage.c; // DEBUG

	svm_voltage.a = (svm_voltage.a/2) + 0.5f;
	svm_voltage.b = (svm_voltage.b/2) + 0.5f;
	svm_voltage.c = (svm_voltage.c/2) + 0.5f;

	svm_voltage.a = (svm_voltage.a > 1) ? 1.0f:svm_voltage.a;
	svm_voltage.a = (svm_voltage.a < 0) ? 0.0f:svm_voltage.a;
	svm_voltage.b = (svm_voltage.b > 1) ? 1.0f:svm_voltage.b;
	svm_voltage.b = (svm_voltage.b < 0) ? 0.0f:svm_voltage.b;
	svm_voltage.c = (svm_voltage.c > 1) ? 1.0f:svm_voltage.c;
	svm_voltage.c = (svm_voltage.c < 0) ? 0.0f:svm_voltage.c;

	Output.Sa = (uint32_t)(TIM1_COUNTER * svm_voltage.a);
	Output.Sb = (uint32_t)(TIM1_COUNTER * svm_voltage.b);
	Output.Sc = (uint32_t)(TIM1_COUNTER * svm_voltage.c);

	Output.Sa = (Output.Sa > (TIM1_COUNTER-1))?(TIM1_COUNTER-1):Output.Sa;
	Output.Sb = (Output.Sb > (TIM1_COUNTER-1))?(TIM1_COUNTER-1):Output.Sb;
	Output.Sc = (Output.Sc > (TIM1_COUNTER-1))?(TIM1_COUNTER-1):Output.Sc;


	return (Output);
}



abc_f SVM_input_Va_Vb_Vc(abc_f v){

	abc_f Output;
	float min, max;

	if(v.a > v.b && v.a > v.c){
		max = v.a;
	}else{
		if(v.b > v.a && v.b > v.c){
			max = v.b;
		}else{
			max = v.c;
		}
	}

	if(v.a < v.b && v.a < v.c){
		min = v.a;
	}else{
		if(v.b < v.a && v.b < v.c){
			min = v.b;
		}else{
			min = v.c;
		}
	}
	Output.a = (-0.5f * (max+min) + v.a) * DIV_2_SQRT3; // DIV_2_SQRT3 = (2.0f /SQRT3);
	Output.b = (-0.5f * (max+min) + v.b) * DIV_2_SQRT3; //(2.0f /SQRT3);
	Output.c = (-0.5f * (max+min) + v.c) * DIV_2_SQRT3; //(2.0f /SQRT3);

	return (Output);
}

abc_f Sinus_Va_Vb_Vc(abc_f v){
	abc_f Output;

	Output.a = v.a;
	Output.b = v.b;
	Output.c = v.c;

	return Output;
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


