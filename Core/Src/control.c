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
#include <stdlib.h>

// PI Control Parameter (global)
int16_t Kp_Q14, Ki_Q15, Kp_s_Q10, Ki_s_Q13;
int32_t I_speed_buffer_q20;
dq_32t I_current_buffer_q20;
dq_t winduptQ15;
int16_t alpha_t;

// for PI Tuning (online tuning)
uint16_t bandwidth_speed = 0, bandwidth_current = 0, cutoff_freq_div = 0; 

/**
 * @brief Find the optimal Q Format from Q8 to Q18
 * 
 * @details allows only a smaller area than 'get_q_format()', made for get q format of Ki and Kp of 
 * current and speed control. in future this function can be replaced by 'get_q_format()' but needs 
 * aditional logic for reaseble Parameter.
 *
 * @see get_q_format()
 */
static q_format_t get_limited_q_format(float value){
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
/**
 * @brief calculate all PI Parameter (Speed & Current); made for initalisation */
void calculate_PI_parameter(Control_Loops *ctrl){

	float kp_c = (ctrl->motor_params.Ls * ctrl->motor_params.bandwidth_current * 32.0f)/(float)ctrl->motor_params.max_voltage;	
	float ki_c = (ctrl->motor_params.Rs * (float)ctrl->motor_params.bandwidth_current * 32.0f)/((float)ctrl->motor_params.max_voltage * (float)FOC_FREQUENCY);

	float feed_forward_f = ctrl->motor_params.Ls * RPM_TO_RAD_S * (float)Q5 / (float)ctrl->motor_params.max_voltage; 		// -> *Q5 are the result of (I_q10/Q10) / (V_max/Q15) ; note: sometimes i write I_q15 but its all time in Q10 with +-Q15 range
	float feed_forward_EMF_f = ctrl->motor_params.Ke * (float)Q15 / (float)ctrl->motor_params.max_voltage; 					// -> *Q15 to transform directly in Q15/Vmax by using (w_rpm * val) >> 15

	float kp_s = (ctrl->motor_params.J * ctrl->motor_params.bandwidth_speed / ctrl->motor_params.Ke) * RPM_TO_RAD_S * (float)ctrl->motor_params.max_speed / 32.0f; 
	float speed_control_frequency = (float)FOC_FREQUENCY/(float)DIVISION_M;
	float ki_s = (kp_s * ctrl->motor_params.bandwidth_speed / (ctrl->motor_params.cutoff_freq_div * speed_control_frequency)) * RPM_TO_RAD_S * (float)ctrl->motor_params.max_speed / 32.0f;

// ############### KP_c #######################
	ctrl->current.Kp.q = get_limited_q_format(kp_c);
		if(ctrl->current.Kp.q == Qerror){
			ctrl->nonvalid_values_flag = true;
			return;
		}
	ctrl->current.Kp.value = (int16_t)((kp_c * (float)(1 << ctrl->current.Kp.q)) + 0.5f);
// ############### KI_c #######################
	ctrl->current.Ki.q = get_limited_q_format(ki_c);
		if(ctrl->current.Ki.q == Qerror){
			ctrl->nonvalid_values_flag = true;
			return;
		}
	ctrl->current.Ki.value = (int16_t)((ki_c * (float)(1 << ctrl->current.Ki.q)) + 0.5f);
// ############### c1_ff #######################
	ctrl->current.ff_c1 = get_q_format(feed_forward_f);
		if(ctrl->current.ff_c1.q == Qerror){
			ctrl->nonvalid_values_flag = true;
			return;
		}
	ctrl->current.ff_c1.value = (int16_t)((feed_forward_f * (float)(1 << ctrl->current.ff_c1.q)) + 0.5f);
// ############### c2_ff #######################
	ctrl->current.ff_c2 = get_q_format(feed_forward_EMF_f);
		if(ctrl->current.ff_c2.q == Qerror){
			ctrl->nonvalid_values_flag = true;
			return;
		}
	ctrl->current.ff_c2.value = (int16_t)((feed_forward_EMF_f * (float)(1 << ctrl->current.ff_c2.q)) + 0.5f);
// ############### KP_s #######################
	ctrl->speed.Kp.q = get_limited_q_format(kp_s);
		if(ctrl->speed.Kp.q == Qerror){
			ctrl->nonvalid_values_flag = true;
			return;
	}
	ctrl->speed.Kp.value = (int16_t)((kp_s * (float)(1 << ctrl->speed.Kp.q)) + 0.5f);
// ############### KI_s #######################
	ctrl->speed.Ki.q = get_limited_q_format(ki_s);
	if(ctrl->speed.Ki.q == Qerror){
		ctrl->nonvalid_values_flag = true;
		return;
	}
	ctrl->speed.Ki.value = (int16_t)((ki_s * (float)(1 << ctrl->speed.Ki.q)) + 0.5f);
// ######################################
	ctrl->nonvalid_values_flag = false;	
}
/**
 * @brief Calculate PI (current) values in non-interrupt area 
 */
void calculate_PI_parameter_small(Control_Loops *ctrl){
	if(fabsf(ctrl->motor_params.max_voltage) < 7.0f){
		// error, voltage too low -> only USB voltage (battery not connected)
		return;
	}
	float kp_c = (ctrl->motor_params.Ls * ctrl->motor_params.bandwidth_current * 32.0f)/(float)ctrl->motor_params.max_voltage;	
	float ki_c = (ctrl->motor_params.Rs * (float)ctrl->motor_params.bandwidth_current * 32.0f)/((float)ctrl->motor_params.max_voltage * (float)FOC_FREQUENCY);

	float feed_forward_f = ctrl->motor_params.Ls * RPM_TO_RAD_S * (float)Q5 / (float)(float)ctrl->motor_params.max_voltage; 		// -> *Q5 are the result of (I_q10/Q10) / (V_max/Q15) ; note: sometimes i write I_q15 but its all time in Q10 with +-Q15 range
	float feed_forward_EMF_f = ctrl->motor_params.Ke * (float)Q15 / (float)(float)ctrl->motor_params.max_voltage; 				// -> *Q15 to transform directly in Q15/Vmax by using (w_rpm * val) >> 15


	q_format_t kp_q = get_limited_q_format(kp_c);
	q_format_t ki_q = get_limited_q_format(ki_c);
	q_format_t c1_ff = get_limited_q_format(feed_forward_f);
	q_format_t c2_ff = get_limited_q_format(feed_forward_EMF_f);

	if(kp_q == Qerror || ki_q == Qerror || c1_ff == Qerror || c2_ff == Qerror ){
		// value are not correct, lets use the old one and stop here
		return;
	}

// ############### buffer the results to eleminate read/write conflicts #######################
	ctrl->current_new.Kp.q = (q_format_t)kp_q;
	ctrl->current_new.Kp.value = (int16_t)((kp_c * (float)(1 << ctrl->current_new.Kp.q)) + 0.5f);
	
	ctrl->current_new.Ki.q = (q_format_t)ki_q;
	ctrl->current_new.Ki.value = (int16_t)((ki_c * (float)(1 << ctrl->current_new.Ki.q)) + 0.5f);

	ctrl->current_new.ff_c1.q = (q_format_t)c1_ff;
	ctrl->current_new.ff_c1.value = (int16_t)((feed_forward_f * (float)(1 << ctrl->current_new.ff_c1.q)) + 0.5f);

	ctrl->current_new.ff_c2.q = (q_format_t)c2_ff;
	ctrl->current_new.ff_c2.value = (int16_t)((feed_forward_EMF_f * (float)(1 << ctrl->current_new.ff_c2.q)) + 0.5f);

	// finish calculaton
	ctrl->new_parameter_flag = true;
	
}

void init_control_functions(Control_Loops *ctrl){

	ctrl->motor_params.Rs = PMSM_RS_OHM;
	ctrl->motor_params.Ls = PMSM_LS;
	ctrl->motor_params.J = PMSM_J_M_R;
	ctrl->motor_params.B = PMSM_B;
	ctrl->motor_params.Ke = PMSM_KE;
	ctrl->motor_params.max_current = MAX_CURRENT;	// not used. nicht verwendet. 
	ctrl->motor_params.max_voltage = MAX_VOLTAGE;
	ctrl->motor_params.max_speed = MAX_SPEED;
	ctrl->motor_params.max_speed_rad = (float)MAX_SPEED * RPM_TO_RAD_S; // convert to rad/s in q15 format
	ctrl->motor_params.bandwidth_current = BANDWIDTH_CURRENT;
	ctrl->motor_params.bandwidth_speed = BANDWIDTH_SPEED;
	ctrl->motor_params.cutoff_freq_div = CUTOFFF_FREQU_DIV;

	ctrl->current.I_buffer = (dq_32t){0,0};
	ctrl->speed.I_buffer = (dq_32t){0,0};
	ctrl->current.windup = (dq_t){0,0};
	ctrl->speed.windup = (dq_t){0,0};
	ctrl->alpha = (int16_t)(PI_IP_ALPHA * (float)Q15);
	ctrl->new_parameter_flag = 0;
	ctrl->nonvalid_values_flag = false;

	if(SPACE_VECTOR_MODULAION == ON){
		ctrl->modulation_index_q15 = (int16_t)CLAMP_INT32_TO_INT16(MODULATION_INDEX_SVM * (float)Q15);
	}else{
		ctrl->modulation_index_q15 = (int16_t)CLAMP_INT32_TO_INT16(MODULATION_INDEX_SINUS * (float)Q15);
	}

	calculate_PI_parameter(ctrl);
}

void clear_control_parameter(Control_Loops *ctrl){

	// clear all PI buffer
	ctrl->current.I_buffer = (dq_32t){0,0};
	ctrl->speed.I_buffer = (dq_32t){0,0};
	ctrl->current.windup = (dq_t){0,0};
	ctrl->speed.windup = (dq_t){0,0};
}

void update_PI_parameter(Control_Loops *ctrl, FOC_HandleTypeDef *pHandle_foc){

	if(ctrl->new_user_parameter_flag){
		calculate_PI_parameter(ctrl);
		ctrl->new_user_parameter_flag = 0;
		return;
	}

	static int16_t battery_voltage = 0;
	int32_t voltage_drop = abs(pHandle_foc->source_voltage - battery_voltage);

	if(voltage_drop > 102){ // voltage change of 10 % or more, we update the PI parameters
		battery_voltage = abs(pHandle_foc->source_voltage);
		ctrl->motor_params.max_voltage = (float)battery_voltage / (float)Q10; // convert back to real voltage in V
		calculate_PI_parameter_small(ctrl);
	}

}

/**
 * @brief Update new valid values during interrupt routine, which calculate in non-interrupt area */
void do_update_pi_parameter(Control_Loops *ctrl){
		if(ctrl->new_parameter_flag){
		ctrl->new_parameter_flag = 0;
		ctrl->motor_params = ctrl->motor_params_new;	
		ctrl->current		= ctrl->current_new;
		ctrl->speed			= ctrl->speed_new;
		ctrl->alpha			= ctrl->alpha_new;
	}
}



void reset_pi_integrator(Control_Loops *ctrl){

	ctrl->current.I_buffer.d = 0;
	ctrl->current.I_buffer.q = 0;
	ctrl->current.windup.d = 0;
	ctrl->current.windup.q = 0;
	ctrl->speed.I_buffer.d = 0;
	ctrl->speed.I_buffer.q = 0;
	ctrl->speed.windup.d = 0;
	ctrl->speed.windup.q = 0;
}


dq_t PI_id_iq_Q15(dq_t error, Control_Loops *ctrl, FOC_HandleTypeDef *pHandle_foc){


	dq_t Output, V;
	dq_t P;
	int32_t x;

	/* Check for nonvalid values*/
	if(ctrl->nonvalid_values_flag) return (dq_t){0,0};


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
	ctrl->current.I_buffer.d += (((int32_t)error.d * ctrl->current.Ki.value) >> (ctrl->current.Ki.q - 5));
	/* windup: note that the effect are Q2 times smaller (because
	 * the current buffer are in Q20 and the windup increased to Q16)
	 */
	ctrl->current.I_buffer.d -= (ctrl->current.windup.d << 1);

	/* limit the buffer, protection of overrun. To limit to the max
	 * output voltage (Q15) set the limits to Q20 and -Q20, because the
	 * buffer are in Q20 and will be every time shift back to Q15 (for
	 * better resolution).
	 */
	ctrl->current.I_buffer.d = CLAMP(ctrl->current.I_buffer.d, -Q20, Q20);

	x = ((ctrl->current.I_buffer.d >> 5) + P.d);

	if(pHandle_foc->current_ff_flag){
		int32_t ff_coupling_term = CLAMP_INT32_TO_INT16(((int64_t)pHandle_foc->I_ref_q15.q * (int64_t)ctrl->current.ff_c1.value * (int64_t)pHandle_foc->speed_ref) >> ctrl->current.ff_c1.q); // feed forward term for coupling between iq and id
		int32_t ff_emf_term = CLAMP_INT32_TO_INT16(((int64_t)pHandle_foc->speed_ref * (int64_t)ctrl->current.ff_c2.value) >> ctrl->current.ff_c2.q); // feed forward term for back EMF compensation
		x += ff_coupling_term;
		x += ff_emf_term;
	}

	V.d = CLAMP_INT32_TO_INT16(x);


	// q-part
	P.q = CLAMP_INT32_TO_INT16(((int32_t)error.q * ctrl->current.Kp.value) >> ctrl->current.Kp.q);
	// Q15 * Q15 = Q30, >>110 results in Q20 for more accuracy in the integrator
	// Later >>5 again to get to Q15 for the output
	ctrl->current.I_buffer.q += (((int32_t)error.q * ctrl->current.Ki.value) >> (ctrl->current.Ki.q - 5));
	/* windup: note that the effect are Q2 times smaller (because
	 * the current buffer are in Q20 and the windup increased to Q16)
	 */
	ctrl->current.I_buffer.q -= (ctrl->current.windup.q << 1);
	/* limit the buffer, protection of overrun. To limit to the max
	 * output voltage (Q15) set the limits to Q20 and -Q20, because the
	 * buffer are in Q20 and will be every time shift back to Q15 (for
	 * better resolution).
	 */
	ctrl->current.I_buffer.q = CLAMP(ctrl->current.I_buffer.q, -Q20, Q20);

	x = ((ctrl->current.I_buffer.q >> 5) + P.q);

	if(pHandle_foc->current_ff_flag){
		int32_t ff_coupling_term = CLAMP_INT32_TO_INT16(((int64_t)pHandle_foc->I_ref_q15.d * (int64_t)ctrl->current.ff_c1.value * (int64_t)pHandle_foc->speed_ref) >> ctrl->current.ff_c1.q); // feed forward term for coupling between iq and id
		x += ff_coupling_term;
	}

	V.q = CLAMP_INT32_TO_INT16(x);


	Output = circle_limitation_Q15(V, (ctrl->modulation_index_q15));

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
		ctrl->speed.I_buffer.d = 0;
		ctrl->speed.I_buffer.q = 0;
		ctrl->speed.windup.d = 0;
		ctrl->speed.windup.q = 0;
		return 0;
	}
	/* Check for nonvalid values*/
	if(ctrl->nonvalid_values_flag) return (int16_t){0};

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
	x = CLAMP_INT32_TO_INT16(x);

	
	ctrl->speed.I_buffer.q += (((int32_t)x * ctrl->speed.Ki.value) >> (ctrl->speed.Ki.q - 5));  
	ctrl->speed.I_buffer.q = CLAMP(ctrl->speed.I_buffer.q, -Q30, Q30);

	C = P + (ctrl->speed.I_buffer.q >> 5); // shift back to Q15 for the output

	Output = CLAMP_INT32_TO_INT16(C);

	pHandle_foc->I_ref_q15.q = Output;
	pHandle_foc->I_ref_q15.d = 0;

	return (Output);
}


