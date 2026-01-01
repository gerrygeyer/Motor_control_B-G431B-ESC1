/*
 * encoder.c
 *
 *  Created on: Aug 6, 2024
 *      Author: Gerry Geyer
 */
#include <encoder.h>
#include <main.h>
#include <foc.h>
#include <observer.h>
#include <stdlib.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

static int16_t iir_lp_speed_q15(int16_t val);

uint16_t encoder_count;// speed_lp_buffer;
uint16_t encoder_angle_last_value;
uint16_t init_angle_encoder;
uint32_t count_to_angle; // pre-calc

uint32_t encoder_count_large;
uint16_t encoder_speed_last_value;
uint16_t iir_a_speed;

int16_t speed_last_value,encoder_count_large_lv;
uint8_t counter;
int32_t speed_rad_memory;

int16_t speed_t;
int32_t speed_t_large;
uint16_t encoder_count_uint, encoder_angle, encoder_debug;

int16_t slow_speed_t, slow_encoder_angle, slow_encoder_angle_last_value, slow_speed_last_value;
int16_t debug_speed_error;

extern uint32_t operation_time_us;

uint32_t scale_encoder_to_u16_debug;

int32_t rotor_position = 0;


float speed_iir_a = 0.7585f;





void init_encoder(void){
	init_angle_encoder = INIT_EL_ANGLE;
	counter = 0;
	speed_rad_memory = 0;
	encoder_speed_last_value = 0;

	iir_a_speed = 1000;
	encoder_count_large_lv = 0;

	count_to_angle = COUNT_TO_EL_ANGLE;
}

// set the encoder (timer4) to 0
void set_encoder_to_zero(FOC_HandleTypeDef *pHandle_foc){
	uint32_t counter_value;
	counter_value = 0;
	__HAL_TIM_SET_COUNTER(&htim4,counter_value);
}


/*
 * get_el_angle(): gives the angle and the speed of the Rotor.
 */
void calc_speed_and_position(void){

	encoder_count = encoder_count_large = __HAL_TIM_GET_COUNTER(&htim4);
	encoder_debug = encoder_count;
	uint16_t virtual_rotations = encoder_count/ENCODER_PULS_PER_REVOLUTION;
	encoder_count = encoder_count - (virtual_rotations * ENCODER_PULS_PER_REVOLUTION);

	uint32_t x 		= encoder_count_large * (uint32_t)Q16; // Q16 = 2^16
	x 				/= (16 * ENCODER_PULS_PER_REVOLUTION); // ENCODER_PULS_PER_REVOLUTION = 4000
	rotor_position 	= (int32_t)x;

	// ############## TRANSFORM TO ELECTRICAL ANGLE #################
//	x = (encoder_count * 65536)/4000;
//	x = (x * PMSM_POLEPAIR);
	encoder_count_uint = (encoder_count * count_to_angle); // ((65536 * 6)/ 4000) ;
	encoder_count = (uint16_t)encoder_count_uint;
//	encoder_count += (uint16_t)((uint32_t)(init_angle_encoder * UINT16_MAX_VALUE)/ENCODER_PULS_PER_REVOLUTION);

}


int16_t get_el_angle(FOC_HandleTypeDef *pHandle_foc){


	pHandle_foc->theta = encoder_count;

	return encoder_count;
}

int16_t speed_calculation(FOC_HandleTypeDef *pHandle_foc){

	static int16_t encoder_last_value_q15 = 0;

	// fist: scale the encoder count to Q16 (UINT16_MAX).
	// @note: if the encoder have 4096 steps, its not necessary.

	int16_t encoder_value = (int16_t)rotor_position;

	speed_t_large = (int16_t)(encoder_value - encoder_last_value_q15);

	speed_t_large *= (FOC_FREQUENCY/DIVISION_M); // -> rps
	speed_t_large >>= 2; // normaly we make bitshift 16 but lets split it in two parts for protecting against overflow
	speed_t_large *= 16; // multiply 16 to cancel out the "virtual" rotation of the Timer (which is 12 times slower)
	speed_t_large *= 60; // multiply 60 to rpm
	speed_t_large >>= 14; // part 2 of bitshift 16
	// save last value for next step
//	encoder_count_large_lv = (int16_t)scale_encoder_to_u16;

	encoder_last_value_q15 = (int16_t)rotor_position;




//	int32_t speed_filter =((speed_t_large + speed_t_large_old) >> 1);
	int32_t speed_filter =speed_t_large;
	speed_filter = CLAMP_INT32_TO_INT16(speed_filter);

	speed_filter = iir_lp_speed_q15((int16_t)speed_filter);

//	float speed_filter_f = speed_iir_a * (float)speed_filter + (1.0f - speed_iir_a) * speed_iir_old;
//	speed_iir_old = speed_filter_f;





//	if(abs(speed_t_large) < 20000){
//		pHandle_foc->speed = (int16_t)speed_filter_f;
	pHandle_foc->speed = (int16_t)speed_filter;
//	}
	return (0); // <- war zu testzwecken deaktiviert


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



















