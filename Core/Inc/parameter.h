/*
 * parameter.h
 *
 *  Created on: Nov 22, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_PARAMETER_H_
#define INC_PARAMETER_H_
#include <stdint.h>


// ################## SYSTEM PARAMETER ##################

#define MAX_CURRENT 	8.0 //8.0	// A
#define MAX_VOLTAGE		12.0f 	// V
//#define MAX_SPEED		7000 // RPM
#define MAX_SPEED		6000 // RPM
//#define FOC_FREQUENCY	10000	// Hz
//#define TIM1_COUNTER	8500
#define FOC_FREQUENCY	20000	// Hz
#define TIM1_COUNTER	(4250-1)
#define FOC_TS			1/FOC_FREQUENCY

#define DIVISION_M		20			// For speed control: 10 = the speed control run 10 times slower (1000Hz)
#define M_FREQUENCY		FOC_FREQUENCY/DIVISION_M
#define M_TS			(float)(1.0/(float)M_FREQUENCY)

#define DIVISION_L		1000		// For Voltage measurement: 1000 => 10 Hz
#define L_FREQUENCY		FOC_FREQUENCY/DIVISION_L

//#define CURRENT_MEAS_MAX_CURRENT	12.0f; // Ampere
//#define CURRENT_MEAS_DIV	5461	// 2^16/12


#define PMSM_RS_OHM		0.14017f		// Ohm
#define PMSM_LS 		0.00008456f	// H
#define PMSM_J_MOTOR	0.0000000069f	// Kg m^2
#define PMSM_J_ROTOR	0.000000057f	// Kg m^2
#define PMSM_J_M_R		0.0000064952	// Parameter estimate
#define PMSM_SYSTEM_J	0.000057633
#define PMSM_J			PMSM_SYSTEM_J
#define PMSM_KT 		((3.0f/2.0f)*((float)PMSM_POLEPAIR * PMSM_LAMBDA))	// Nm/A
#define PMSM_DIV_KT 	738
// #define PMSM_POLEPAIR 	7
#define PMSM_POLEPAIR 	6	// TEST
#define PMSM_LAMBDA		0.000129f
#define PMSM_KE			((3.0f/2.0f)*(float)PMSM_POLEPAIR*PMSM_LAMBDA) 	//Vrms/RPM

#define PMSM_B			0.078985970113088f

// Other

#define PMSM_PHI		(PMSM_KE * 60)/(PMSM_POLEPAIR);//

#define MODULATION_INDEX_SPACE_VEC		0.866f // 0.866 for SVM
#define MODULATION_INDEX_SINUS			0.785f // 0.707 for

// ################## CONTROL PARAMETER ##################
/*
 * Pi Controll Parameter
 * 	current control:
 * 		Pole-zero cancellation: Kp/Ki = Ls/Rs
 *
 * 		-> Kp = Ls * w_c ;
 * 		-> Ki = Rs * w_c * Ts
 *
 * 	source:st.com "UM1052 User manual"
 */

// Motor 1



// Motor 2
#define BANDWIDTH_CURRENT 	5000 // rad
//#define BANDWIDTH_SPEED		500	// rad
//#define BANDWIDTH_SPEED		40 //400	// rad
#define BANDWIDTH_SPEED		80 //400	// rad


//#define CUTOFFF_FREQU_DIV	65//18 //30 //100 // = w_c/w_s
#define CUTOFFF_FREQU_DIV	700//18 //30 //100 // = w_c/w_s



// ################ CURRENT CONTROL ####################
#define KP_CURRENT 			(PMSM_LS * BANDWIDTH_CURRENT)
#define KI_CURRENT			(PMSM_RS_OHM * BANDWIDTH_CURRENT * FOC_TS)

// ################ SPEED CONTROL #######################

//#define PI_IP_ALPHA			0.75f //0.85f
#define PI_IP_ALPHA			0.95f //0.85f
//#define PI_IP_ALPHA			0.45f
#define MAXIMUM_RATE_M		(10*(float)MAX_SPEED)/(float)(M_FREQUENCY) // 0.1 sec to become max speed

//#define KP_SPEED 			((PMSM_SYSTEM_J * BANDWIDTH_SPEED)/(PMSM_KE * (float)PMSM_DIV_KT))
#define KP_SPEED 			(float)((PMSM_J_M_R * (float)BANDWIDTH_SPEED)/((float)PMSM_KE))
#define KI_SPEED			(((KP_SPEED ) * BANDWIDTH_SPEED * DIVISION_M)/(CUTOFFF_FREQU_DIV * FOC_FREQUENCY))


// ################ SAFETY VALUES #####################

#define OVER_TEMPERATURE_VALUE		100 // °C
#define OVER_VOLTAGE_VALUE			25 // V

// ############### OTHER ###############################

#define GOTOSTART_Q_VOLTAGE	2.0f // V


#define OK			1
#define NOT_OK		0

#define ON			1
#define OFF			0

#define HIGH		1
#define LOW			0


#define OPENLOOP_VOLTAGE	(float)2.5

// ############ ENCODER PARAMETER ######################
#define ENCODER_PULS_PER_REVOLUTION	4096
// #define ENCODER_PULS_PER_REVOLUTION	4000

/* 
 *	also the right Timer Period must be set!
 *	in CubeMX under Timer 4 settings, or directly in main.c 
 *	inside of MX_TIM4_Init() function:

*/
/*
static void MX_TIM4_Init(void)
{
...
  htim4.Init.Period = 63999; <- (4000 * 16)-1
  OR
  htim4.Init.Period = 65535; <- (4096 * 16)-1
...
*/

// ############## typedef structs ######################

typedef struct{
	uint8_t EN_GATE;
	uint8_t M_PWM;
	uint8_t nFAULT;
	uint8_t DC_CAL;
	uint8_t OC_ADJ;
	uint8_t M_OC;
	uint8_t nOCTW;
}ESC_HandleTypeDef;

typedef enum{
	FOC_OPENLOOP,
	FOC_CLOSELOOP,
	FOC_CURRENT_CONTROL,
	FOC_OBSERVER
} FOC_Mode;

typedef enum {
	q18 = 18,
	q17 = 17,
	q16 = 16,
    q15 = 15,
    q14 = 14,
    q13 = 13,
    q12 = 12,
	Qerror = -1
} q_format_t;

typedef struct {
	float a;
	float b;
}ab_f;

typedef struct {
	int16_t a;
	int16_t b;
}ab_t;


typedef struct {
	float a;
	float b;
	float c;
}abc_f;

typedef struct {
	int16_t a;
	int16_t b;
	int16_t c;
}abc_16t;

typedef struct {
	uint16_t a;
	uint16_t b;
	uint16_t c;
}abc_u16t;

/**
 * @brief StructDescription
 * @note  Optionaler Hinweis zur Verwendung
 * @see   ReferenzOderModulname
 */
typedef struct
{
	int32_t a; /**< Variable a */
	int32_t b; /**< Variable b */
	int32_t c; /**< Variable c */
} abc_32t;



typedef struct {
	int16_t alpha;
	int16_t beta;
}alphabeta_t;



typedef struct {
	int16_t d;
	int16_t q;
}dq_t;

typedef struct {
	int32_t d;
	int32_t q;
}dq_32t;




typedef struct{
	int16_t sin;
	int16_t cos;
}angle_t;


typedef struct {
	uint32_t Sa;
	uint32_t Sb;
	uint32_t Sc;

}svm_output;


typedef struct{
	uint16_t number;
	float toruqe;
}command;


typedef struct {
	uint8_t state;
	uint8_t old_state;
	int16_t speed_command;
}data;



typedef struct{

	abc_16t v_abc_t;

	int32_t speed_ref;
	int32_t speed;
	int32_t speed_rad;

	uint16_t theta;
	uint16_t theta_openloop;
	angle_t elec_theta_q15;
	FOC_Mode foc_mode;

	int16_t source_voltage;			/* Measuring Source Voltage */
	int16_t temperature;			/* Measuring Temperature */

	dq_t I_ref_q15;
	ab_t I_ab_q15;
	alphabeta_t I_alph_bet_q15;
	dq_t I_dq_q15;
	dq_t V_dq_q15;
	alphabeta_t V_alph_bet_q15;
	abc_16t V_abc_q15;

	int16_t Iq_meas_q15;

}FOC_HandleTypeDef;



typedef struct {
    int16_t     value;
    q_format_t q;
} fixed16_t;

typedef struct {
	fixed16_t Kp;
	fixed16_t Ki;
	dq_32t I_buffer_q20;
	dq_t windup;
} PID_Controller;
typedef struct {
	float Rs;
	float Ls;
	float J;
	float B;
	float Ke;
	float bandwidth_current;
	float bandwidth_speed;
	float cutoff_freq_div;
	float max_current;
	float max_voltage;
	float max_speed;
}motor_parameters_f;
typedef struct {
	motor_parameters_f motor_params;
    PID_Controller current;
    PID_Controller speed;
	int16_t alpha;
	uint8_t new_parameter_flag;
} Control_Loops;




typedef struct{
	float y;
	float u;
}differential_values;

typedef struct{
	int32_t y;
	int32_t u;
}uyt_32;



//typedef struct{
//	float u_k_1;
//}backward_diff;


#endif /* INC_PARAMETER_H_ */
