#include <stdint.h>

#include <foc_math.h>
#include <main.h>
#include <observer.h>
#include <current_measurement.h>

#include "control.h"
#include <encoder.h>
#include <svm.h>
#include <foc.h>

// variables/structs

int32_t debug_speed;
float debug_output_Vq;

abc_16t V_abc_q15;
alphabeta_t I_alph_bet_q15, V_alph_bet_q15;

dq_t Idq_q15, Vdq_q15, Idq_set_q15;
ab_t Iab_q15;

//debug
dq_t Idq_q15_scope, Idq_set_q15_scope;

// ########## STATIC FUNCTIONS BEGIN ##########
static dq_t park_transformation_q15(alphabeta_t I, angle_t el_theta);
static alphabeta_t clark_transformation_q15(ab_t Input);
static alphabeta_t inverse_park_transformation_q15(dq_t V, angle_t elec_theta);
static abc_16t inverse_clark_transformation_q15(alphabeta_t v);
static dq_t error_fct(dq_t ref, dq_t val);
static dq_t iir_current_dq_filter_q15(dq_t i);
static void median_filter_d_t(int16_t *data);
static void median_filter_q_t(int16_t *data);
// ########## STATIC FUNCTIONS END ##########

// Debug
svm_output debug_PWM_OUT;
float debug_current_Iq;

// init function
void init_foc(FOC_HandleTypeDef *pHandle){

	Vdq_q15.q			= 0;
	Vdq_q15.d = (Q15 >>3); // 0.125 * Vmax

}

void clear_foc(FOC_HandleTypeDef *pHandle){ // need this for start and stop without setting new Kp,Ki,.. values

		clear_control_parameter();

}

void execute_FOC(FOC_HandleTypeDef *pHandle_foc){
	angle_t el_theta_q15;

	// for change the parameter online
	update_PI_parameter();

	switch (pHandle_foc->foc_mode){
	case FOC_OPENLOOP: // generates openloop voltage vector without control
		

		pHandle_foc->elec_theta_q15.sin = sin_t(pHandle_foc->theta_openloop);
		pHandle_foc->elec_theta_q15.cos = cos_t(pHandle_foc->theta_openloop);

		pHandle_foc->V_dq_q15.q = 0;
		pHandle_foc->V_dq_q15.d = (Q15 >>4); // 0.125 * Vmax
		pHandle_foc->V_alph_bet_q15 = inverse_park_transformation_q15(pHandle_foc->V_dq_q15,pHandle_foc->elec_theta_q15);
		pHandle_foc->V_abc_q15 = inverse_clark_transformation_q15(pHandle_foc->V_alph_bet_q15);
		debug_PWM_OUT = get_PWM_OUTPUT_Q15(pHandle_foc->V_abc_q15);

		// dont use PI control, clear the integral parts
		clear_control_parameter();

	break;

	default: // FOC_CLOSELOOP

		el_theta_q15.sin = sin_t(pHandle_foc->theta);		// get sine and cosine in Q15
		el_theta_q15.cos = cos_t(pHandle_foc->theta);

		pHandle_foc->I_alph_bet_q15 = clark_transformation_q15(pHandle_foc->I_ab_q15);	// Clarke transformation
		pHandle_foc->I_dq_q15 = park_transformation_q15(pHandle_foc->I_alph_bet_q15, el_theta_q15);	// Park transformation

		/*
			*  	Median Filter (solve Problem of noise spikes)
			*/

		median_filter_d_t(&pHandle_foc->I_dq_q15.d);
		median_filter_q_t(&pHandle_foc->I_dq_q15.q);

		pHandle_foc->V_dq_q15 = PI_id_iq_Q15(error_fct(pHandle_foc->I_ref_q15, pHandle_foc->I_dq_q15), pHandle_foc); // PI current control in Q15
		pHandle_foc->V_alph_bet_q15 = inverse_park_transformation_q15(pHandle_foc->V_dq_q15,pHandle_foc->elec_theta_q15);	// Inverse Park transformation
		pHandle_foc->V_abc_q15 = inverse_clark_transformation_q15(pHandle_foc->V_alph_bet_q15);	// Inverse Clarke transformation

		pHandle_foc->V_q15.a = pHandle_foc->V_abc_q15.a;
		pHandle_foc->V_q15.b = pHandle_foc->V_abc_q15.b;
		pHandle_foc->V_q15.c = pHandle_foc->V_abc_q15.c;

		Idq_q15_scope.d = Idq_q15.d;
		Idq_q15_scope.q = Idq_q15.q;

	break;
	}

	pHandle_foc->V_q15.a = pHandle_foc->V_abc_q15.a;
	pHandle_foc->V_q15.b = pHandle_foc->V_abc_q15.b;
	pHandle_foc->V_q15.c = pHandle_foc->V_abc_q15.c;

	// for
	pHandle_foc->Iq_value = Idq_q15.q;

}


svm_output goto_position(dq_t V){
	svm_output Output;

	/*
	 * inverse park transformation with \theta = 0:
	 * 		V_\alpha = V_d, V_\beta = V_q;
	 */
	alphabeta_t V_alpha_beta = {V.d,V.q};
	abc_16t V_abc_q15 = inverse_clark_transformation_q15(V_alpha_beta);
	Output = get_PWM_OUTPUT_Q15(V_abc_q15);
	return (Output);
}



/**
 * @brief       Computes the Clarke transformation (abc -> αβ) in Q15 format.
 *
 * @details     Transforms two-phase stator currents from the a-b coordinate system into the
 *              αβ-coordinate system. Assumes a balanced system where i_c = −(i_a + i_b).
 *
 * @param       Input       Stator current vector in a-b-frame (Q15).
 *
 * @return      Transformed current vector in αβ-frame (Q15).
 *
 * @note        Uses fixed-point arithmetic (Q15). Constant DIV_1_SQRT3_Q15 ≈ 1/√3 in Q15.
 * @see         park_transformation_t()
 */
static alphabeta_t clark_transformation_q15(ab_t Input){
	alphabeta_t Output;
	int32_t x,a,b;

	Output.alpha = Input.a;
	a = (int32_t)Input.a;
	b = (int32_t)Input.b;

	x = (b << 1) + a;
	x = (x >> 2);			// rightshift 15 bits: split in two shifts
	x *= DIV_1_SQRT3_Q15;
	x = (x >> 13);
	Output.beta = CLAMP_INT32_TO_INT16(x);

	return (Output);
}


/**
 * @brief       Computes the Park transformation (Clarke -> dq) in Q15 format.
 *
 * @details     Transforms stator currents from the stationary αβ-frame into the rotating dq-frame
 *              using the given electrical angle. Assumes Q15 fixed-point representation for all inputs.
 *
 * @param       I           Stator current vector in αβ-frame (Q15).
 * @param       el_theta    Electrical angle containing sine and cosine values in Q15.
 *
 * @return      Transformed current vector in dq-frame (Q15).
 *
 * @note        Input sine and cosine must already be computed and in Q15 format.
 * @warning     Function assumes all inputs are in valid Q15 range (−1.0 … +0.9999).
 * @see         clarke_transformation_t()
 */
static dq_t park_transformation_q15(alphabeta_t I, angle_t el_theta){

	dq_t Output;

	int32_t x;
	x		= ((int32_t)el_theta.cos * (int32_t)I.alpha) + ((int32_t)el_theta.sin * (int32_t)I.beta);
	x		= (x >> 15);
	Output.d = CLAMP_INT32_TO_INT16(x);

	x		= (-(int32_t)el_theta.sin * (int32_t)I.alpha) + ((int32_t)el_theta.cos * (int32_t)I.beta);
	x		= (x >> 15);
	Output.q = CLAMP_INT32_TO_INT16(x);

	return (Output);

}


/**
 * @brief       Computes the inverse Park transformation (dq -> αβ) in Q15 format.
 *
 * @details     Converts voltage or current vectors from the rotating dq-frame back into the stationary αβ-frame.
 *              Assumes Q15 fixed-point representation for all inputs.
 *
 * @param       V           Input vector in dq-frame (Q15).
 * @param       elec_theta  Electrical angle with sine and cosine in Q15.
 *
 * @return      Transformed vector in αβ-frame (Q15).
 *
 * @note        Input angle components must be precomputed and in valid Q15 range (−1.0 … +0.9999).
 * @see         park_transformation_t()
 */
static alphabeta_t inverse_park_transformation_q15(dq_t V, angle_t elec_theta){

	alphabeta_t Output;
	int32_t x;

//	Output.alpha = (elec_theta.cos * V.d) - (elec_theta.sin * V.q);
//	Output.beta = (elec_theta.sin * V.d) + (elec_theta.cos * V.q);

	x 			= (((int32_t)elec_theta.cos * (int32_t)V.d) - ((int32_t) elec_theta.sin * V.q));
	x			= (x >> 15);
	Output.alpha = CLAMP_INT32_TO_INT16(x);

	x			= ((int32_t)elec_theta.sin * (int32_t)V.d) + ((int32_t)elec_theta.cos * (int32_t)V.q);
	x			= (x >> 15);
	Output.beta = CLAMP_INT32_TO_INT16(x);

	return (Output);
}




/**
 * @brief       Computes the inverse Clarke transformation (αβ -> abc) in Q15 format.
 *
 * @details     Transforms a vector from the stationary αβ-frame back into the three-phase system (a, b, c).
 *              Assumes a balanced system. Uses Q15 fixed-point arithmetic and a pre-scaled √3 constant.
 *
 * @param       v           Input vector in αβ-frame (Q15).
 *
 * @return      Reconstructed three-phase vector in abc-frame (Q15).
 *
 * @note        Constant SQRT3_Q14 ≈ √3 in Q14 format. Scaling is handled internally.
 * @see         clark_transformation_q15()
 */
static abc_16t inverse_clark_transformation_q15(alphabeta_t v){

	abc_16t Output;
	int32_t x;

//	Output.a 	= v.alpha;
//	Output.b = (-v.alpha + (SQRT3 * v.beta))/2;
//	Output.c = -(v.alpha + (SQRT3 * v.beta))/2;

	Output.a 	= v.alpha;

	x 			= ((-(int32_t)v.alpha + ((SQRT3_Q14 * v.beta) >> 14)) >> 1);
	Output.b 	= CLAMP_INT32_TO_INT16(x);

	x 			= -(((int32_t)v.alpha + ((SQRT3_Q14 * v.beta) >> 14)) >> 1);
	Output.c 	= CLAMP_INT32_TO_INT16(x);

	return (Output);
}


void generate_openloop(int16_t speed_rpm, FOC_HandleTypeDef *pHandle){

	int32_t angle_count;
	uint32_t angle = pHandle->theta_openloop;

	angle_count = (int16_t)(speed_rpm * PMSM_POLEPAIR)/60;
	angle_count = (angle_count << 16);
	angle_count = (angle_count / FOC_FREQUENCY);
	angle = angle + CLAMP_INT32_TO_INT16((int32_t)angle_count);

	pHandle->theta_openloop = angle;

}

/**
 * @brief       Computes the control error between reference and actual dq vectors.
 *
 * @details     Subtracts the measured dq values from the reference to obtain the control error.
 *              Operates component-wise in Q15 fixed-point format.
 *
 * @param       ref     Reference dq vector (Q15).
 * @param       val     Measured/actual dq vector (Q15).
 *
 * @return      Control error vector in dq-frame (Q15).
 *
 * @note        This is a simple component-wise subtraction:
 *              \f$ error.d = ref.d - val.d \f$,
 *              \f$ error.q = ref.q - val.q \f$
 */
static dq_t error_fct(dq_t ref, dq_t val){
	dq_t Output;
	int32_t x;
	x 			= (int32_t)ref.d - val.d;
	Output.d 	= CLAMP_INT32_TO_INT16(x);
	x 			= (int32_t)ref.q - val.q;
	Output.q 	= CLAMP_INT32_TO_INT16(x);

	return (Output);
}




static dq_t iir_current_dq_filter_q15(dq_t i){
	static dq_t i_filter;
	static int16_t iir_a_dq = CLAMP_INT32_TO_INT16((int32_t)((float)(PI_MULTIPLY_2 * (1.0f/(float)FOC_FREQUENCY) * 3000.0f)/(PI_MULTIPLY_2 * (1.0f/(float)FOC_FREQUENCY) * 3000 + 1.0f)*Q15));
	i_filter.d = (((int32_t)iir_a_dq * (int32_t)i.d) >> 15) + ((((int32_t)Q15 - (int32_t)iir_a_dq) * (int32_t)i_filter.d) >> 15);
	i_filter.q = (((int32_t)iir_a_dq * (int32_t)i.q) >> 15) + ((((int32_t)Q15 - (int32_t)iir_a_dq) * (int32_t)i_filter.q) >> 15);
	return (i_filter);
}



#define MEDIAN_WIN 8 

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


static void median_filter_d_t(int16_t *data){
    static int16_t buffer[2][MEDIAN_WIN] = {{0}};
    static uint8_t idx = 0;
    idx = (idx+1)%MEDIAN_WIN;

    for(uint8_t i=0; i<2; i++){
        buffer[i][idx] = data[i]; // neuen Wert speichern

        // Kopie fürs Sortieren machen
        int16_t temp[MEDIAN_WIN];
        for(uint8_t k=0; k<MEDIAN_WIN; k++) temp[k]=buffer[i][k];

        sort_int16(temp, MEDIAN_WIN);

        // Median auswählen und zurückschreiben
        data[i] = temp[MEDIAN_WIN/2];
    }
}

static void median_filter_q_t(int16_t *data){
    static int16_t buffer[2][MEDIAN_WIN] = {{0}};
    static uint8_t idx = 0;
    idx = (idx+1)%MEDIAN_WIN;

    for(uint8_t i=0; i<2; i++){
        buffer[i][idx] = data[i]; // neuen Wert speichern

        // Kopie fürs Sortieren machen
        int16_t temp[MEDIAN_WIN];
        for(uint8_t k=0; k<MEDIAN_WIN; k++) temp[k]=buffer[i][k];

        sort_int16(temp, MEDIAN_WIN);

        // Median auswählen und zurückschreiben
        data[i] = temp[MEDIAN_WIN/2];
    }
}
