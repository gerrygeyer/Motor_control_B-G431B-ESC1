/*
 * control.c
 *
 *  Created on: Nov 22, 2024
 *      Author: Gerry Geyer
 */


#include "parameter.h"
#include "settings.h"
#include "foc_math.h"
#include <math.h>

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

static q_format_t find_Q_format(float value){
	float value_abs = fabsf(value);
	if(value_abs > 1.0f){
		if(value_abs > 128.0f){
			return Qerror; // value too high for fixed16_t
		}else if(value_abs > 64.0f){
			return q8; 
		}else if(value_abs > 32.0f){
			return q9; 
		}else if(value_abs > 16.0f){
			return q10;
		}else if(value_abs > 8.0f){
			return q11; 
		}else if (value_abs > 4.0f) {
			return q12;
		}else if (value_abs > 2.0f) {
			return q13;
		}else{
			return q14;
		}
	}else{

		if(value_abs > 0.5f){
			return q15;
	}else if (value_abs > 0.25f) {
			return q16;
	
	}else if (value_abs > 0.125) {
			return q17;
	}else{
			return q18;
		}
	if((value_abs * Q18)== 0.0f){
		return Qerror; 
	}

	}
}

static void calculate_PI_parameter(Control_Loops *ctrl){

	float kp_c = (ctrl->motor_params.Ls * ctrl->motor_params.bandwidth_current * ctrl->motor_params.max_current)/(float)ctrl->motor_params.max_voltage;	
	float ki_c = (ctrl->motor_params.Rs * (float)ctrl->motor_params.bandwidth_current * ctrl->motor_params.max_current)/((float)ctrl->motor_params.max_voltage * (float)FOC_FREQUENCY);

	float kp_s = (ctrl->motor_params.J * ctrl->motor_params.bandwidth_speed / ctrl->motor_params.Ke) * RPM_TO_RAD_S * (float)ctrl->motor_params.max_speed / ctrl->motor_params.max_current; 
	float speed_control_frequency = (float)FOC_FREQUENCY/(float)DIVISION_M;
	float ki_s = (kp_s * ctrl->motor_params.bandwidth_speed / (ctrl->motor_params.cutoff_freq_div * speed_control_frequency)) * RPM_TO_RAD_S * (float)ctrl->motor_params.max_speed / ctrl->motor_params.max_current;

	
	ctrl->current.Kp.q = find_Q_format(kp_c);
	if(ctrl->current.Kp.q == Qerror){
		// handle error
		return;
	}
	ctrl->current.Kp.value = (int16_t)((kp_c * (float)(1 << ctrl->current.Kp.q)) + 0.5f);

	ctrl->current.Ki.q = find_Q_format(ki_c);
		if(ctrl->current.Ki.q == Qerror){
		// handle error
		return;
	}
	ctrl->current.Ki.value = (int16_t)((ki_c * (float)(1 << ctrl->current.Ki.q)) + 0.5f);
	
	ctrl->speed.Kp.q = find_Q_format(kp_s);
			if(ctrl->speed.Kp.q == Qerror){
		// handle error
		return;
	}
	ctrl->speed.Kp.value = (int16_t)((kp_s * (float)(1 << ctrl->speed.Kp.q)) + 0.5f);
	ctrl->speed.Ki.q = find_Q_format(ki_s);
	if(ctrl->speed.Ki.q == Qerror){
		// handle error
		return;
	}
	ctrl->speed.Ki.value = (int16_t)((ki_s * (float)(1 << ctrl->speed.Ki.q)) + 0.5f);
}

void init_control_functions(Control_Loops *ctrl){

	ctrl->motor_params.Rs = PMSM_RS_OHM;
	ctrl->motor_params.Ls = PMSM_LS;
	ctrl->motor_params.J = PMSM_J_M_R;
	ctrl->motor_params.B = PMSM_B;
	ctrl->motor_params.Ke = PMSM_KE;
	ctrl->motor_params.max_current = MAX_CURRENT;
	ctrl->motor_params.max_voltage = MAX_VOLTAGE;
	ctrl->motor_params.max_speed = MAX_SPEED;
	ctrl->motor_params.bandwidth_current = BANDWIDTH_CURRENT;
	ctrl->motor_params.bandwidth_speed = BANDWIDTH_SPEED;
	ctrl->motor_params.cutoff_freq_div = CUTOFFF_FREQU_DIV;

	ctrl->current.I_buffer_q20 = (dq_32t){0,0};
	ctrl->speed.I_buffer_q20 = (dq_32t){0,0};
	ctrl->current.windup = (dq_t){0,0};
	ctrl->speed.windup = (dq_t){0,0};
	ctrl->alpha = (int16_t)(PI_IP_ALPHA * (float)Q15);
	ctrl->new_parameter_flag = 0;

	calculate_PI_parameter(ctrl);

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

void clear_control_parameter(Control_Loops *ctrl){

	// clear all PI buffer
	ctrl->current.I_buffer_q20 = (dq_32t){0,0};
	ctrl->speed.I_buffer_q20 = (dq_32t){0,0};
	ctrl->current.windup = (dq_t){0,0};
	ctrl->speed.windup = (dq_t){0,0};
}

void update_PI_parameter(Control_Loops *ctrl){

	if(ctrl->new_parameter_flag){
		calculate_PI_parameter(ctrl);
		ctrl->new_parameter_flag = 0;
	}

	// static uint16_t bandwidth_speed_last = 0;
	// static uint16_t bandwidth_current_last = 0;
	// static uint16_t cutoff_freq_div_last = 0;

	// check if parameters changed (for online tuning)
	// if((bandwidth_current != bandwidth_current_last) || (bandwidth_speed != bandwidth_speed_last) || (cutoff_freq_div != cutoff_freq_div_last)){

	// 	bandwidth_current_last = bandwidth_current;
	// 	bandwidth_speed_last = bandwidth_speed;
	// 	cutoff_freq_div_last = cutoff_freq_div;

	// 	Kp_Q14 = (int16_t)(((PMSM_LS * (float)bandwidth_current * (float)Q19)/(float)MAX_VOLTAGE) + 0.5f);
	// 	Ki_Q15 = (int16_t)(((PMSM_RS_OHM * (float)bandwidth_current * (float)Q20)/((float)MAX_VOLTAGE * (float)FOC_FREQUENCY)) + 0.5f);

	// 	float kp_s = (PMSM_J_M_R * (float)bandwidth_speed / PMSM_KE) * RPM_TO_RAD_S * (float)MAX_SPEED / 32.0f; // 32.0f = max current
	// 	Kp_s_Q10 = (int16_t)(kp_s * (float)Q10);
	// 	// (((KP_SPEED ) * BANDWIDTH_SPEED * DIVISION_M)/(CUTOFFF_FREQU_DIV * FOC_FREQUENCY))
	// 	float speed_control_frequency = (float)FOC_FREQUENCY/(float)DIVISION_M;
	// 	float ki_s = (kp_s * (float)bandwidth_speed / ((float)cutoff_freq_div * speed_control_frequency)) * RPM_TO_RAD_S * (float)MAX_SPEED / 32.0f;
	// 	Ki_s_Q13 = (int16_t)(ki_s * (float)Q13); // ki is so small, that Q18 are better than Q15
	// }

}



void reset_pi_integrator(Control_Loops *ctrl){

	ctrl->current.I_buffer_q20.d = 0;
	ctrl->current.I_buffer_q20.q = 0;
	ctrl->current.windup.d = 0;
	ctrl->current.windup.q = 0;
	ctrl->speed.I_buffer_q20.d = 0;
	ctrl->speed.I_buffer_q20.q = 0;
	ctrl->speed.windup.d = 0;
	ctrl->speed.windup.q = 0;
}


dq_t PI_id_iq_Q15(dq_t error, Control_Loops *ctrl, FOC_HandleTypeDef *pHandle_foc){

	dq_t Output, V;
	dq_t P;
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
	P.d = CLAMP_INT32_TO_INT16(((int32_t)error.d * ctrl->current.Kp.value) >> ctrl->current.Kp.q);

	// Q15 * Q15 = Q30, >>10 results in Q20 for more accuracy in the integrator
	// Later >>5 again to get to Q15 for the output
	ctrl->current.I_buffer_q20.d += (((int32_t)error.d * ctrl->current.Ki.value) >> (ctrl->current.Ki.q - 5));
	/* windup: note that the effect are Q2 times smaller (because
	 * the current buffer are in Q20 and the windup increased to Q16)
	 */
	ctrl->current.I_buffer_q20.d -= (ctrl->current.windup.d << 1);

	/* limit the buffer, protection of overrun. To limit to the max
	 * output voltage (Q15) set the limits to Q20 and -Q20, because the
	 * buffer are in Q20 and will be every time shift back to Q15 (for
	 * better resolution).
	 */
	ctrl->current.I_buffer_q20.d = CLAMP(ctrl->current.I_buffer_q20.d, -Q20, Q20);

	x = ((ctrl->current.I_buffer_q20.d >> 5) + P.d);

	V.d = CLAMP_INT32_TO_INT16(x);


	// q-part
	P.q = CLAMP_INT32_TO_INT16(((int32_t)error.q * ctrl->current.Kp.value) >> ctrl->current.Kp.q);
	// Q15 * Q15 = Q30, >>110 results in Q20 for more accuracy in the integrator
	// Later >>5 again to get to Q15 for the output
	ctrl->current.I_buffer_q20.q += (((int32_t)error.q * ctrl->current.Ki.value) >> (ctrl->current.Ki.q - 5));
	/* windup: note that the effect are Q2 times smaller (because
	 * the current buffer are in Q20 and the windup increased to Q16)
	 */
	ctrl->current.I_buffer_q20.q -= (ctrl->current.windup.q << 1);
	/* limit the buffer, protection of overrun. To limit to the max
	 * output voltage (Q15) set the limits to Q20 and -Q20, because the
	 * buffer are in Q20 and will be every time shift back to Q15 (for
	 * better resolution).
	 */
	ctrl->current.I_buffer_q20.q = CLAMP(ctrl->current.I_buffer_q20.q, -Q20, Q20);

	x = ((ctrl->current.I_buffer_q20.q >> 5) + P.q);

	V.q = CLAMP_INT32_TO_INT16(x);


	Output = circle_limitation_Q15(V, (Q15-1));

	ctrl->current.windup.d = (V.d - Output.d);
	ctrl->current.windup.q = (V.q - Output.q);


	return (Output);
}
static int16_t multiplicate_two_q15(int16_t q1, int16_t q2){
	int32_t x = ((q1 * q2) + (1 << 14)) >> 15;
	return CLAMP_INT32_TO_INT16(x);
}


int16_t pi_speed_q15 (int32_t speed_rpm, int32_t speed_rpm_ref, Control_Loops *ctrl, FOC_HandleTypeDef *pHandle_foc){

	if(pHandle_foc->foc_mode != FOC_CLOSELOOP){
		ctrl->speed.I_buffer_q20.d = 0;
		ctrl->speed.I_buffer_q20.q = 0;
		ctrl->speed.windup.d = 0;
		ctrl->speed.windup.q = 0;
		return 0;
	}
	/*
	 * Structure of blendig IP/PI controller: P = speed_err * Kp * alpha - speed * Kp * (1-alpha)
	 * 		-> we can simplify it to P = (speed_ref * alpha - speed) * Kp
	 * 				-> proof: set alpha = 1 than: P = (speed_ref - speed) * Kp = speed_err * Kp  ☑️
	 */
	int32_t C, x, Output;
	x = (speed_rpm_ref << 15) / ctrl->motor_params.max_speed; // = speed_ref_q15
	int16_t speed_ref_alpha_q15 = multiplicate_two_q15(CLAMP_INT32_TO_INT16(x), ctrl->alpha);
	x = (speed_rpm << 15) / ctrl->motor_params.max_speed; // = speed_rpm_q15
	x = ((int32_t)speed_ref_alpha_q15 - x );
	x = CLAMP_INT32_TO_INT16(x);

	int32_t P = ((int32_t)x * ctrl->speed.Kp.value) >> ctrl->speed.Kp.q; // P in Q15

	x = (((speed_rpm_ref - speed_rpm) << 15)/ctrl->motor_params.max_speed); // speed_err_q15
	debug_error_speed = x;
	x = CLAMP_INT32_TO_INT16(x);

	ctrl->speed.I_buffer_q20.q += (((int32_t)x * ctrl->speed.Ki.value) >> (ctrl->speed.Ki.q - 5));  // error Q15 mult with KiQ13 = I_buf_Q29. than bitshift 13 to Q20
	ctrl->speed.I_buffer_q20.q = CLAMP(ctrl->speed.I_buffer_q20.q, -Q20, Q20);

	C = P + (ctrl->speed.I_buffer_q20.q >> 5);

	Output = CLAMP_INT32_TO_INT16(C);

	pHandle_foc->I_ref_q15.q = Output;
	pHandle_foc->I_ref_q15.d = 0;

	return (Output);
}



