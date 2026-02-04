/*
 * encoder.c
 *
 *  Created on: Aug 6, 2024
 *      Author: Gerry Geyer
 */
#include <encoder.h>
#include <main.h>
#include <observer.h>
#include <stdlib.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern uint32_t operation_time_us;

static int16_t iir_lp_speed_q15(int16_t val);

uint16_t encoder_count;// speed_lp_buffer;
uint32_t count_to_angle; // pre-calc

int32_t rotor_position = 0;


void init_encoder(void){
	count_to_angle = COUNT_TO_EL_ANGLE;
}

// set the encoder (timer4) to 0, needed for zero position calibration
void set_encoder_to_zero(FOC_HandleTypeDef *pHandle_foc){
	uint32_t counter_value;
	counter_value = 0;
	__HAL_TIM_SET_COUNTER(&htim4,counter_value);
}


/*
 * calculate the electrical angle from the rotor position.
 * 
 * Timer gives the position with a scale: 1 revolution from 0 to MAX_TIMER_VALUE are equal to 16 Rotations
 * so we can use differntial calculation in Q15 to get the speed very smooth. 
 * But we have to rescale for the position information.
 *
 */
void calc_rotor_position(FOC_HandleTypeDef *pHandle_foc){

	uint32_t encoder_count_large;
	uint16_t encoder_count = encoder_count_large = __HAL_TIM_GET_COUNTER(&htim4);
	uint16_t rotation = encoder_count/ENCODER_PULS_PER_REVOLUTION;
	encoder_count = encoder_count - (rotation * ENCODER_PULS_PER_REVOLUTION);
	// now we have the real encoder count within one revolution

	// ############## TRANSFORM POSITION TO Q16 SCALE  #################
	uint32_t x 		= encoder_count_large * (uint32_t)Q16; // Q16 = 2^16
	x 				/= (16 * ENCODER_PULS_PER_REVOLUTION); // ENCODER_PULS_PER_REVOLUTION = 4000
	rotor_position 	= (int32_t)x;	// we need the global variable rotor_position for speed calculation

	// ############## TRANSFORM TO ELECTRICAL ANGLE #################
	uint16_t encoder_count_uint = (encoder_count * count_to_angle); 
	pHandle_foc->theta = (uint16_t)encoder_count_uint;
}


int16_t speed_calculation(FOC_HandleTypeDef *pHandle_foc){

	static int16_t encoder_last_value_q15 = 0;

	// fist: scale the encoder count to Q16 (UINT16_MAX).
	// @note: if the encoder have 4096 steps, its not necessary.

	int32_t speed_t_large = (int16_t)((int16_t)rotor_position - encoder_last_value_q15);

	speed_t_large *= (FOC_FREQUENCY/DIVISION_M); // -> rps
	speed_t_large >>= 2; // normaly we make bitshift 16 but lets split it in two parts for protecting against overflow
	speed_t_large *= 16; // multiply 16 to cancel out the "virtual" rotation of the Timer (which is 12 times slower)
	speed_t_large *= 60; // multiply 60 to rpm
	speed_t_large >>= 14; // part 2 of bitshift 16
	// save last value for next step
	encoder_last_value_q15 = (int16_t)rotor_position;

	int32_t speed_filter =speed_t_large;
	speed_filter = CLAMP_INT32_TO_INT16(speed_filter);
	speed_filter = iir_lp_speed_q15((int16_t)speed_filter);

	pHandle_foc->speed = (int16_t)speed_filter;
	return (pHandle_foc->speed); 
}


/**
 * @brief       First-order IIR low-pass filter for a Q15 signal.
 *
 * @details     Implements:
 *              \f[
 *                  y[n] = a \, x[n] + (1-a)\, y[n-1]
 *              \f]
 *              with all quantities in Q15 fixed-point. The coefficient @p a_iir
 *              (~200 rad/s ⇒ ≈32 Hz cutoff given @c FOC_FREQUENCY/@c DIVISION_M)
 *              is computed once at startup in Q15.
 *
 * @param[in]   val     Current input sample (Q15).
 * @return              Filtered output sample (Q15).
 *
 * @note
 * - Uses a static state (@c filter_val) and is therefore not re-entrant/thread-safe.
 * - 32-bit intermediates are used; output is rounded and clamped to int16_t.
 * - Stability requires 0 ≤ @p a_iir ≤ Q15.
 *
 * @see         CLAMP_INT32_TO_INT16, Q15
 */
static int16_t iir_lp_speed_q15(int16_t val){
	static int32_t filter_val = 0;
	// filter coefficient with 200rad cutoff (\apporx fc = 32 Hz) depending on frequency in q15 format
	static int16_t a_iir = CLAMP_INT32_TO_INT16((int32_t)((float)((PI_MULTIPLY_2 * ((float)DIVISION_M/(float)FOC_FREQUENCY)*25)/(PI_MULTIPLY_2 * ((float)DIVISION_M/(float)FOC_FREQUENCY)*25 + 1.0f))*(float)Q15));

	filter_val = (((int32_t)a_iir * val) >> 14) +  ((((int32_t)Q15 - (int32_t)a_iir) * filter_val) >> 14) + 1; // add 1 for correct rounding
	filter_val = CLAMP_INT32_TO_INT16((filter_val >> 1));
	return (filter_val);
}
