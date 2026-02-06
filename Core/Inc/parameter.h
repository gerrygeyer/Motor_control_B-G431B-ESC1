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

#define SIGMOID		0
#define SIGNUM		1

#define FOC_CLOSELOOP 	0
#define FOC_OPENLOOP 	1
#define FOC_OBSERVER	2

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
	uint8_t EN_GATE;    // Enable gate driver signal
	uint8_t M_PWM;      // PWM mode control signal
	uint8_t nFAULT;     // Fault flag (active low)
	uint8_t DC_CAL;     // DC calibration signal
	uint8_t OC_ADJ;     // Overcurrent adjustment signal
	uint8_t M_OC;       // Overcurrent mode signal
	uint8_t nOCTW;      // Overcurrent and thermal warning flag (active low)
}ESC_HandleTypeDef;



typedef struct {
	float a;    // Phase a component (floating point)
	float b;    // Phase b component (floating point)
}ab_f;

typedef struct {
	int16_t a;    // Phase a component (fixed point)
	int16_t b;    // Phase b component (fixed point)
}ab_t;


typedef struct {
	float a;    // Phase a component (floating point)
	float b;    // Phase b component (floating point)
	float c;    // Phase c component (floating point)
}abc_f;

typedef struct {
	int16_t a;    // Phase a component (16-bit fixed point)
	int16_t b;    // Phase b component (16-bit fixed point)
	int16_t c;    // Phase c component (16-bit fixed point)
}abc_16t;

typedef struct {
	uint16_t a;    // Phase a component (unsigned 16-bit)
	uint16_t b;    // Phase b component (unsigned 16-bit)
	uint16_t c;    // Phase c component (unsigned 16-bit)
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
	int16_t alpha;    // Alpha axis component in stationary reference frame
	int16_t beta;     // Beta axis component in stationary reference frame
}alphabeta_t;



typedef struct {
	int16_t d;    // D-axis component in rotating reference frame (16-bit)
	int16_t q;    // Q-axis component in rotating reference frame (16-bit)
}dq_t;

typedef struct {
	int32_t d;    // D-axis component in rotating reference frame (32-bit)
	int32_t q;    // Q-axis component in rotating reference frame (32-bit)
}dq_32t;




typedef struct{
	int16_t sin;    // Sine value of the angle (fixed point)
	int16_t cos;    // Cosine value of the angle (fixed point)
}angle_t;


typedef struct {
	uint32_t Sa;    // Space vector modulation duty cycle for phase a
	uint32_t Sb;    // Space vector modulation duty cycle for phase b
	uint32_t Sc;    // Space vector modulation duty cycle for phase c

}svm_output;


typedef struct{
	uint16_t number;    // Command number/identifier
	float toruqe;       // Torque command value [Nm]
}command;


typedef struct {
	uint8_t state;              // Current state of the system
	uint8_t old_state;          // Previous state for state change detection
	int16_t speed_command;      // Speed command reference value [RPM]
}data;

typedef struct{

	abc_16t v_abc_t;                    // Three-phase voltage values (16-bit fixed point)

	int32_t speed_ref;                  // Speed reference setpoint [RPM]
	int32_t speed;                      // Actual motor speed [RPM]
	int32_t speed_rad;                  // Motor speed in radians per second [rad/s]

	uint16_t theta;                     // Electrical rotor angle [0-65535]
	uint16_t theta_openloop;            // Open-loop electrical angle [0-65535]
	angle_t elec_theta_q15;             // Electrical angle sine/cosine values (Q15 format)
	uint8_t foc_mode;                   // FOC operation mode (closeloop/openloop/observer)

	int16_t source_voltage;             // DC bus voltage measurement [V]
	int16_t temperature;                // System temperature [°C]

	abc_16t V_q15;                      // Three-phase voltage commands (Q15 format)
	int16_t Iq_value;                   // Q-axis current value [A]
	int32_t Iq_mean;                    // Mean Q-axis current for filtering [A]

	dq_t I_ref_q15;                     // D-Q reference currents (Q15 format)
	ab_t I_ab_q15;                      // Two-phase currents (Q15 format)
	alphabeta_t I_alph_bet_q15;         // Alpha-beta currents (Q15 format)
	dq_t I_dq_q15;                      // D-Q measured currents (Q15 format)
	dq_t V_dq_q15;                      // D-Q voltage commands (Q15 format)
	alphabeta_t V_alph_bet_q15;         // Alpha-beta voltage commands (Q15 format)
	abc_16t V_abc_q15;                  // Three-phase voltage commands (Q15 format)

	int16_t Iq_meas_q15;                // Measured Q-axis current (Q15 format)

}FOC_HandleTypeDef;

typedef enum {
	q18 = 18,       // Q18 fixed-point format (18 fractional bits)
	q17 = 17,       // Q17 fixed-point format (17 fractional bits)
	q16 = 16,       // Q16 fixed-point format (16 fractional bits)
    q15 = 15,       // Q15 fixed-point format (15 fractional bits)
    q14 = 14,       // Q14 fixed-point format (14 fractional bits)
    q13 = 13,       // Q13 fixed-point format (13 fractional bits)
    q12 = 12,       // Q12 fixed-point format (12 fractional bits)
	Qerror = -1     // Error indicator for invalid Q format
} q_format_t;

typedef struct {
    int16_t     value;    // Fixed-point value (16-bit)
    q_format_t q;         // Q format specification (number of fractional bits)
} fixed16_t;

typedef struct {
	fixed16_t Kp;               // Proportional gain (fixed-point)
	fixed16_t Ki;               // Integral gain (fixed-point)
	dq_32t I_buffer_q20;        // Integral buffer for D-Q axes (Q20 format)
	dq_t windup;                // Anti-windup limits for integral terms
} PID_Controller;
typedef struct {
	float Rs;                   // Stator resistance [Ohm]
	float Ls;                   // Stator inductance [H]
	float J;                    // Rotor inertia [kg*m^2]
	float B;                    // Viscous friction coefficient [Nm*s/rad]
	float Ke;                   // Back-EMF constant [V/rad/s]
	float bandwidth_current;    // Current controller bandwidth [rad/s]
	float bandwidth_speed;      // Speed controller bandwidth [rad/s]
	float cutoff_freq_div;      // Cutoff frequency divider (dimensionless)
	float max_current;          // Maximum current limit [A]
	float max_voltage;          // Maximum voltage limit [V]
	float max_speed;            // Maximum speed limit [RPM]
}motor_parameters_f;
typedef struct {
	motor_parameters_f motor_params;    // Motor physical parameters
    PID_Controller current;             // Current control loop PI controller
    PID_Controller speed;               // Speed control loop PI controller
	int16_t alpha;                      // Low-pass filter coefficient (fixed point)
} Control_Loops;




typedef struct{
	float y;    // Output value of the differential equation
	float u;    // Input value of the differential equation
}differential_values;

typedef struct{
	int32_t y;    // Output value (32-bit fixed point)
	int32_t u;    // Input value (32-bit fixed point)
}uyt_32;



//typedef struct{
//	float u_k_1;
//}backward_diff;


#endif /* INC_PARAMETER_H_ */
