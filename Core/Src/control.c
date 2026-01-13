/*
 * control.c
 *
 *  Created on: Nov 22, 2024
 *      Author: Gerry Geyer
 */


#include "parameter.h"
#include "settings.h"
#include "foc_math.h"


float Pi_d_buffer, Pi_d_windup, Pi_q_buffer,Pi_q_windup, Pi_speed_buffer, Pi_speed_windup, Pi_pos_windup, Pi_pos_buffer; // Integration for PI control
float Kp,Ki, Ki_speed, Kp_speed, Kd_speed, alpha_speed, Kd_buffer, max_torque;
float cutoff_freq_div;
uint16_t bandwidth_speed, bandwidth_current; // FOR TUNINGMODE

float debug_Te, debug_P, debug_I, debug_D;


int16_t Kp_Q14, Ki_Q15, Kp_s_Q10, Ki_s_Q13;
int32_t I_speed_buffer_q20;
dq_32t I_current_buffer_q20;
dq_t winduptQ15;
int16_t alpha_t;



int16_t debug_error_speed = 0;


void init_control_functions(void){

	Pi_d_buffer = 0.0;
	Pi_q_buffer = 0.0;
	Pi_speed_buffer = 0;

	Kp = KP_CURRENT;
	Ki = KI_CURRENT;

	Kp_speed = KP_SPEED;
	Ki_speed = KI_SPEED;
	Kd_speed = 0.0;

	cutoff_freq_div = CUTOFFF_FREQU_DIV;

	alpha_speed = PI_IP_ALPHA;

	bandwidth_speed = BANDWIDTH_SPEED;
	bandwidth_current = BANDWIDTH_CURRENT;

//	max_torque = ((float)MAX_CURRENT * PMSM_KT);
	max_torque = ((float)MAX_CURRENT);


	Kp_Q14 = (int16_t)(((PMSM_LS * (float)BANDWIDTH_CURRENT * (float)Q19)/(float)MAX_VOLTAGE) + 0.5f);
	Ki_Q15 = (int16_t)(((PMSM_RS_OHM * (float)BANDWIDTH_CURRENT * (float)Q20)/((float)MAX_VOLTAGE * (float)FOC_FREQUENCY)) + 0.5f);

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
	Pi_d_buffer = 0.0;
	Pi_q_buffer = 0.0;
	Pi_speed_buffer = 1.0;

	I_current_buffer_q20.d = 0;
	I_current_buffer_q20.q = 0;

	I_speed_buffer_q20 = 0;



	/*
	 * if tuning mode is enable, than the control parameter are not fixed
	 * and can be changed online.
	 */
	if(PI_TUNING_MODE == ON){
//		bandwidth_speed = BANDWIDTH_SPEED;
//		bandwidth_current = BANDWIDTH_CURRENT;
		Kp = (PMSM_LS * bandwidth_current);
		Ki = (PMSM_RS_OHM * bandwidth_current * FOC_TS);

//		Kp_speed = (PMSM_J * bandwidth_speed)/(PMSM_KE * (float)PMSM_DIV_KT);

// commend out for debugging
//		Kp_speed = (PMSM_J_MOTOR * bandwidth_speed)/(PMSM_KE);
//		Ki_speed = (Kp_speed * bandwidth_speed * DIVISION_M)/(cutoff_freq_div * FOC_FREQUENCY);
//		Kd_speed = 0.0;

	}else{

		Kp = KP_CURRENT;
		Ki = KI_CURRENT;

		Kp_speed = KP_SPEED;
		Ki_speed = KI_SPEED;
		Kd_speed = 0.0;

		bandwidth_speed = BANDWIDTH_SPEED;
		bandwidth_current = BANDWIDTH_CURRENT;

	}

}

void update_PI_parameter(void){
	Kp = (PMSM_LS * bandwidth_current);
	Ki = (PMSM_RS_OHM * bandwidth_current * FOC_TS);
	Kp_speed = (PMSM_J_M_R * bandwidth_speed)/PMSM_KE;
	Ki_speed = (Kp_speed * (float)bandwidth_speed * (float)DIVISION_M)/(cutoff_freq_div * (float)FOC_FREQUENCY);
	Kd_speed = 0.0;
}

float PI_controller(float errorstate, uint16_t frequency, float Kp, float Ki,float limit, float old_value){
	// Ki * Ts
	float Output;

	float controller_i = ((Ki/((float)frequency)) * errorstate) + old_value;
	float controller_p = Kp * errorstate;

	Output = controller_i + controller_p;
	//simple output limiter
		if (Output < -limit){
			Output = -limit;
		}
		if (Output > limit){
			Output = limit;
		}
	return Output;

}

float PI_iq (float error,FOC_HandleTypeDef *pHandle_foc){

	float Output;
	float P;


	P = (error * Kp);
	Pi_q_buffer += (error * Ki);


	if(ANTI_WINDUP == ON){
		Pi_q_buffer -= (Pi_q_windup/(3*Kp));
	}

	Output = Pi_q_buffer + P;


	Pi_q_windup = (float)(Output - pHandle_foc->v_circle.q);

	if (Output > MAX_VOLTAGE){
		Pi_q_windup = (Output - MAX_VOLTAGE);
		Output = MAX_VOLTAGE;
	}
	if (Output < -MAX_VOLTAGE){
		Pi_q_windup = (Output + MAX_VOLTAGE);
		Output = -MAX_VOLTAGE;
	}
	if (Output < MAX_VOLTAGE && Output > -MAX_VOLTAGE){
		Pi_q_windup = 0.0f;
	}


	return (Output);
}

float PI_id (float error, FOC_HandleTypeDef *pHandle_foc){

	float Output;
	float P;

	P = (error * Kp);
	Pi_d_buffer += (error * Ki);

	if(ANTI_WINDUP == ON){
		Pi_d_buffer -= (Pi_d_windup/(3*Kp));
	}

	Output = Pi_d_buffer + P;
	// Feed Forward:


	if (Output > MAX_VOLTAGE){
		Pi_d_windup = (Output - MAX_VOLTAGE);
		Output = MAX_VOLTAGE;
	}
	if (Output < -MAX_VOLTAGE){
		Pi_d_windup = (Output + MAX_VOLTAGE);
		Output = -MAX_VOLTAGE;
	}
	if (Output < MAX_VOLTAGE && Output > -MAX_VOLTAGE){
		Pi_d_windup = 0.0f;
	}


	return (Output);
}

dq_f PI_id_iq (dq_f error, FOC_HandleTypeDef *pHandle_foc){

	dq_f Output, V;
	dq_f P;

	// d-part
	P.d = (error.d * Kp);



	Pi_d_buffer += (error.d * Ki);
	if(ANTI_WINDUP == ON){
		Pi_d_buffer -= (Pi_d_windup/(Kp));
	}


	V.d = Pi_d_buffer + P.d;


	// q-part
	P.q = (error.q * Kp);


	Pi_q_buffer += (error.q * Ki);
	if(ANTI_WINDUP == ON){
		Pi_q_buffer -= (Pi_q_windup/(Kp));
	}

	V.q = Pi_q_buffer + P.q;

	Output = circle_limiter_maxVoltage(V);

	Pi_d_windup = (V.d - Output.d);
	Pi_q_windup = (V.q - Output.q);




	return (Output);
}

/*
 * PI Speed Controller with Feed Forward term:
 *  if BLENDING_PI_IP are ENABLE than they use the blending of PI and IP Regulator. (Figure 4.44)
 *  if FEED_FORWARD_SPEED are ENABLE than they use the acceleration feed-forward term.(Figure 4.46)
 * 	source:  Control of Electric Machine Drive Systems | Wiley Online Books. https://onlinelibrary.wiley.com/doi/book/10.1002/ 9780470876541
 *
 */
float PI_speed (float speed_rad, int32_t speed_ref, FOC_HandleTypeDef *pHandle_foc){

	float T,Output, P, error, alpha;


	if (BLENDING_PI_IP == ON){
		alpha = alpha_speed;
	}else{
		alpha = 1.0f;
	}
	error = ((float)speed_ref - speed_rad);

	P = (error * Kp_speed) * alpha;
	Pi_speed_buffer += (error * Ki_speed);

//	if(ANTI_WINDUP_SPEED == ON){
//		Pi_speed_buffer -= (Pi_speed_windup * 3.0f / Kp_speed);
//	}
//	D = Kd_speed * (error - Kd_buffer) * (float)M_FREQUENCY;
	T = Pi_speed_buffer + P;

	debug_P = P;

	T -= (speed_rad * Kp_speed * (1.0 - alpha));

	if (Pi_speed_buffer > max_torque){
		Pi_speed_buffer = max_torque;
	}
	if (Pi_speed_buffer < -max_torque){
		Pi_speed_buffer = -max_torque;
	}
	Kd_buffer = error;
	// end of classic PI control

	T /= (float)PMSM_DIV_KT;

	T *= (float)PMSM_DIV_KT;
	debug_Te = T;

//	Output = T* PMSM_DIV_KT; // Torque to current
	Output = T;

	return (Output);
}




void reset_pi_integrator(void){

	Pi_pos_buffer = 0;
	Pi_speed_buffer = 0;
	Pi_d_buffer = 0;
	Pi_q_buffer = 0;

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



