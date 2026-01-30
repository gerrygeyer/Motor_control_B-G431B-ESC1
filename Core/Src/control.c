/*
 * control.c
 *
 *  Created on: Nov 22, 2024
 *      Author: Gerry Geyer
 */


#include "parameter.h"
#include "settings.h"
#include "foc_math.h"

// PI Control Parameter (global)
int16_t Kp_Q14, Ki_Q15, Kp_s_Q10, Ki_s_Q13;
int32_t I_speed_buffer_q20;
dq_32t I_current_buffer_q20;
dq_t winduptQ15;
int16_t alpha_t;

// for PI Tuning (online tuning)
uint16_t bandwidth_speed = 0, bandwidth_current = 0, cutoff_freq_div = 0; 

// debug
int16_t debug_error_speed = 0;

void init_control_functions(void){


	cutoff_freq_div = CUTOFFF_FREQU_DIV;
	bandwidth_speed = BANDWIDTH_SPEED;
	bandwidth_current = BANDWIDTH_CURRENT;

	// check kp_c, ki_c , kp_s and ki_s in debug mode for good resolution 
	float kp_c = (PMSM_LS * (float)BANDWIDTH_CURRENT * (float)Q5)/(float)MAX_VOLTAGE;	// Q5 = MAX_CURRENT = 32A
	Kp_Q14 = (int16_t)((kp_c + 0.5f) * Q14); // we need a high resolution for low Kp values

	float ki_c = (PMSM_RS_OHM * (float)BANDWIDTH_CURRENT * (float)Q5)/((float)MAX_VOLTAGE * (float)FOC_FREQUENCY);
	Ki_Q15 = (int16_t)((ki_c + 0.5f) * Q15); 

	float kp_s = (PMSM_J_M_R * (float)BANDWIDTH_SPEED / PMSM_KE) * RPM_TO_RAD_S * (float)MAX_SPEED / 32.0f; // 32.0f = max current
	Kp_s_Q10 = (int16_t)(kp_s * (float)Q10);
	// (((KP_SPEED ) * BANDWIDTH_SPEED * DIVISION_M)/(CUTOFFF_FREQU_DIV * FOC_FREQUENCY))
	float speed_control_frequency = (float)FOC_FREQUENCY/(float)DIVISION_M;
	float ki_s = (kp_s * (float)BANDWIDTH_SPEED / ((float)CUTOFFF_FREQU_DIV * speed_control_frequency)) * RPM_TO_RAD_S * (float)MAX_SPEED / 32.0f;
	Ki_s_Q13 = (int16_t)(ki_s * (float)Q13); // ki is so small, that Q18 are better than Q15

	float alpha_f = PI_IP_ALPHA;
	alpha_t = (alpha_f * (float)Q15);

	I_speed_buffer_q20 = 0;
}

void clear_control_parameter(void){

	// clear all PI buffer
	I_current_buffer_q20.d = 0;
	I_current_buffer_q20.q = 0;
	I_speed_buffer_q20 = 0;
}

void update_PI_parameter(void){

	static uint16_t bandwidth_speed_last = 0;
	static uint16_t bandwidth_current_last = 0;
	static uint16_t cutoff_freq_div_last = 0;
	static uint8_t first_run = 1;



	if(first_run){
		bandwidth_speed_last = bandwidth_speed;
		bandwidth_current_last = bandwidth_current;
		cutoff_freq_div_last = cutoff_freq_div;
		first_run = 0;
	}

	// check if parameters changed (for online tuning)
	if((bandwidth_current != bandwidth_current_last) || (bandwidth_speed != bandwidth_speed_last) || (cutoff_freq_div != cutoff_freq_div_last)){

		bandwidth_current_last = bandwidth_current;
		bandwidth_speed_last = bandwidth_speed;
		cutoff_freq_div_last = cutoff_freq_div;

		Kp_Q14 = (int16_t)(((PMSM_LS * (float)bandwidth_current * (float)Q19)/(float)MAX_VOLTAGE) + 0.5f);
		Ki_Q15 = (int16_t)(((PMSM_RS_OHM * (float)bandwidth_current * (float)Q20)/((float)MAX_VOLTAGE * (float)FOC_FREQUENCY)) + 0.5f);

		float kp_s = (PMSM_J_M_R * (float)bandwidth_speed / PMSM_KE) * RPM_TO_RAD_S * (float)MAX_SPEED / 32.0f; // 32.0f = max current
		Kp_s_Q10 = (int16_t)(kp_s * (float)Q10);
		// (((KP_SPEED ) * BANDWIDTH_SPEED * DIVISION_M)/(CUTOFFF_FREQU_DIV * FOC_FREQUENCY))
		float speed_control_frequency = (float)FOC_FREQUENCY/(float)DIVISION_M;
		float ki_s = (kp_s * (float)bandwidth_speed / ((float)cutoff_freq_div * speed_control_frequency)) * RPM_TO_RAD_S * (float)MAX_SPEED / 32.0f;
		Ki_s_Q13 = (int16_t)(ki_s * (float)Q13); // ki is so small, that Q18 are better than Q15
	}

}



void reset_pi_integrator(void){

	I_current_buffer_q20.d = 0;
	I_current_buffer_q20.q = 0;
}


dq_t PI_id_iq_Q15(dq_t error, FOC_HandleTypeDef *pHandle_foc){

	dq_t Output, V;
	dq_f P;
	int32_t x;


	/*
	 *
	 * input "error" are in Q15 with error = (err/I_max) * Q15.
	 * output Output are in Q15 with out = (out/V_max) * Q15
	 * we take Q14 and Q15 for the best resolution of the parameter
	 * Kp_Q14 = ((I_max * PMSM_LS * BANDWIDTH_CURRENT * Q14)/V_max)
	 * Ki_Q15 = ((I_max * PMSM_RS_OHM * BANDWIDTH_CURRENT * Q15)/V_max)
	 *
	 * 	Kp_Q14 = (int16_t)((PMSM_LS * (float)BANDWIDTH_CURRENT * (float)Q19)/(float)MAX_VOLTAGE + 0.5f);
	 * 	Ki_Q15 = (int16_t)((PMSM_RS_OHM * (float)BANDWIDTH_CURRENT * (float)Q20)/((float)MAX_VOLTAGE * (float)FOC_FREQUENCY) + 0.5f);
	 *
	 * for exact rounding in int16 we add +0.5f
	 *
	 */

	// d-part
	P.d = ((error.d * Kp_Q14) >> 14);

	// Q15 * Q15 = Q30, >>10 results in Q20 for more accuracy in the integrator
	// Later >>5 again to get to Q15 for the output
	I_current_buffer_q20.d += ((error.d * Ki_Q15) >> 10);
	/* windup: note that the effect are Q2 times smaller (because
	 * the current buffer are in Q20 and the windup increased to Q16)
	 */
	I_current_buffer_q20.d -= (winduptQ15.d << 1);

	/* limit the buffer, protection of overrun. To limit to the max
	 * output voltage (Q15) set the limits to Q20 and -Q20, because the
	 * buffer are in Q20 and will be every time shift back to Q15 (for
	 * better resolution).
	 */
	I_current_buffer_q20.d = CLAMP(I_current_buffer_q20.d, -Q20, Q20);

	x = ((I_current_buffer_q20.d >> 5) + P.d);

	V.d = CLAMP_INT32_TO_INT16(x);


	// q-part
	P.q = ((error.q * Kp_Q14) >> 14);
	// Q15 * Q15 = Q30, >>110 results in Q20 for more accuracy in the integrator
	// Later >>5 again to get to Q15 for the output
	I_current_buffer_q20.q += ((error.q * Ki_Q15) >> 10);
	/* windup: note that the effect are Q2 times smaller (because
	 * the current buffer are in Q20 and the windup increased to Q16)
	 */
	I_current_buffer_q20.q -= (winduptQ15.q << 1);
	/* limit the buffer, protection of overrun. To limit to the max
	 * output voltage (Q15) set the limits to Q20 and -Q20, because the
	 * buffer are in Q20 and will be every time shift back to Q15 (for
	 * better resolution).
	 */
	I_current_buffer_q20.q = CLAMP(I_current_buffer_q20.q, -Q20, Q20);

	x = ((I_current_buffer_q20.q >> 5) + P.q);

	V.q = CLAMP_INT32_TO_INT16(x);


	Output = circle_limitation_Q15(V, (Q15-1));

	winduptQ15.d = (V.d - Output.d);
	winduptQ15.q = (V.q - Output.q);


	return (Output);
}
static int16_t multiplicate_two_q15(int16_t q1, int16_t q2){
	int32_t x = ((q1 * q2) + (1 << 14)) >> 15;
	return CLAMP_INT32_TO_INT16(x);
}


int16_t pi_speed_q15 (int32_t speed_rpm, int32_t speed_rpm_ref, FOC_HandleTypeDef *pHandle_foc){

	/*
	 * Structure of blendig IP/PI controller: P = speed_err * Kp * alpha - speed * Kp * (1-alpha)
	 * 		-> we can simplify it to P = (speed_ref * alpha - speed) * Kp
	 * 				-> proof: set alpha = 1 than: P = (speed_ref - speed) * Kp = speed_err * Kp  ☑️
	 */
	int32_t C, x, Output;
	x = (speed_rpm_ref << 15) / MAX_SPEED; // = speed_ref_q15
	int16_t speed_ref_alpha_q15 = multiplicate_two_q15(CLAMP_INT32_TO_INT16(x), alpha_t);
	x = (speed_rpm << 15) / MAX_SPEED; // = speed_rpm_q15
	x = ((int32_t)speed_ref_alpha_q15 - x );
	x = CLAMP_INT32_TO_INT16(x);

	int32_t P = (x * Kp_s_Q10) >> 10;

	x = (((speed_rpm_ref - speed_rpm) << 15)/MAX_SPEED); // speed_err_q15
	debug_error_speed = x;
	x = CLAMP_INT32_TO_INT16(x);

	I_speed_buffer_q20 += (x * Ki_s_Q13) >> 8;  // error Q15 mult with KiQ13 = I_buf_Q29. than bitshift 13 to Q20
	I_speed_buffer_q20 = CLAMP(I_speed_buffer_q20, -Q20, Q20);

	C = P + (I_speed_buffer_q20 >> 5);

	Output = CLAMP_INT32_TO_INT16(C);
	pHandle_foc->Iq_set_q15 = Output;

	return (Output);
}



