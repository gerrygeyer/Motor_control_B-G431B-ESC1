/*
 * task.c
 *
 *  Created on: Dec 24, 2024
 *      Author: gerrygeyer
 */


#include <settings.h>
#include <current_measurement.h>
#include <communication.h>
#include <observer.h>
#include <stdbool.h>
#include <foc_math.h>
#include <stdio.h>
#include <stdint.h>
#include<task.h>



extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

uint8_t ENGINE_MODE;
int32_t speed_reference;
int32_t position_reference;
uint8_t middlespeed_task_counter;
uint16_t lowspeed_task_counter;

FOC_HandleTypeDef foc_values;
observer observer_values;
data master_command;

uint16_t observer_counter, wait_counter, automatic_counter, openloop_counter;
uint16_t safety_counter; // protection of triggering init state all the time (burning motor)

uint32_t test_function_counter;
uint8_t test_function_flag;

float speed_rate_memory;
uint8_t system_state, old_system_state;
float open_loop_speed_rpm;

int8_t state_encoder;

float Iq_set;
uint8_t closed_loop_token;
bool init_safty_flag = false;


enum {TIMER_SLEEP, TIMER_RUNNING, TIMER_FINISH, TIMER_START} timer_state;
enum {STOP_COMMAND, RUN_COMMAND, WAIT_COMMAND} software_command;

static void timer_function(uint16_t time_ms, uint8_t start, uint8_t reset){
	static uint32_t timer_count = 0;
	static uint32_t timer_target_value = 0;
	static uint8_t start_prev = 0;


	if(reset == 1){
		timer_state = TIMER_SLEEP;
		timer_count = 0;
		timer_target_value = 0;
		start_prev = 0;
		return;
	}

	if((start == 1) && (start_prev == 0)) timer_state = TIMER_START;
	start_prev = start;

	switch (timer_state){

	case (TIMER_START):
		timer_count = 0;
		time_ms = CLAMP(time_ms,0,2000);
		timer_target_value = ((uint32_t)FOC_FREQUENCY * time_ms) / 1000;
		timer_state = TIMER_RUNNING;
	break;

	case (TIMER_RUNNING):
		timer_count++;
		if(timer_count >= timer_target_value) timer_state = TIMER_FINISH;
	break;

	case (TIMER_FINISH):
		start_prev = 0;
		// Timer successful: go to the next stage
	break;

	case (TIMER_SLEEP):
		start_prev = 0;
		// Timer wait for command but not go to the next stage
	break;

	default:
		// do nothing
	break;
	}
}

static void PWM_All_Start(void);
static void PWM_All_Stop(void);
static void SAFETY_system_state_observer(void);

void init_task(void){

	timer_state = TIMER_SLEEP;



	closed_loop_token = 0;
	test_function_counter = 0;
	test_function_flag = 0;
	safety_counter = 0;

	init_foc(&foc_values);
	init_control_functions();
	init_SVM();
	init_current_measurement();
	init_encoder();
	init_communication();
	init_math();

	ENGINE_MODE = RUN_ENGINE;
	foc_values.foc_mode = FOC_CLOSELOOP;
	observer_values.state = OFF;


	/*
	 * SYSTEM STATE
	 *
	 * 	OFFSET_CALIBRATION:
	 * 		Motor
	 * 	GOTO_START_POSITION:
 	 * 		set V1 (sector 1) for short time to 1
 	 * 	RUN_CLOSED_LOOP:
 	 * 		run closed loop with encoder or observer
 	 * 	WAIT:
 	 * 		wait. set output to 0 (V8 is activatet)
 	 * 	OPENLOOP:
 	 * 		start open loop, after time it go automatically to CLOSED_LOOP_OBSERVER
 	 * 	...
 	 *
	 */

	if(DEBUG_MODE == ON){
		system_state = RUN_CLOSED_LOOP;
	}else{
		system_state = GOTO_START_POSITION;
	}

	// set init values

	middlespeed_task_counter 	= 0;
	lowspeed_task_counter		= 0;

	speed_rate_memory 			= 0.0f;
	speed_reference 			= -500;
	position_reference 			= 0;
	state_encoder 				= 0;
	wait_counter 				= 0;
	openloop_counter			= 0;

	observer_counter 			= 0;
	Iq_set 						= 0.0;


}

/*
 * time_management():	this function is triggered by the TIM1 (currently running at 20kHz)
 * 						and manages the functions in task.h.
 */
void time_management(void){


	calc_speed_and_position();
	execute_current_measurement(); // 9.15us


	engine_mode_observer(); // look for START/STOP interrupt
	switch_state_observer();
	highspeed_task();		// run the System (middle speed part are included)
	get_information(&master_command);

	if(AUTOMATIC_SWITCH_OFF){
		middlespeed_task((int16_t)(master_command.speed_command));
	}else{
		middlespeed_task((int16_t)(speed_reference));
	}
	lowspeed_task();

	SAFETY_system_state_observer(); //
}

/*
 * engine_mode_observer():	look for START/STOP interrupt comes from the button (with the engine_mode_start_stopp function).
 */
void engine_mode_observer(void){
	if(ENGINE_MODE == STOPP_ENGINE){
		system_state = STOPP;
		PWM_All_Stop();
	}
	if(ENGINE_MODE == START_ENGINE){
		system_state = RUN_CLOSED_LOOP;
		PWM_All_Start();
	}


}

/*
 * switch_state_observer(): look for switching state. necessary for reset counter
 */
void switch_state_observer(void){
	if(system_state != old_system_state){
	// ####### trigger these section after the system state changed #########

		openloop_counter = 0;
		open_loop_speed_rpm = 0.0f;

	// ##############################################################
	}
	old_system_state = system_state;
}

/*
 * engine_mode_start_stopp():	This function is triggered by the interrupt when the button is pressed
 */
void engine_mode_start_stopp(void){
	if(ENGINE_MODE == STOPP_ENGINE){
		ENGINE_MODE = START_ENGINE;
	}else{
		ENGINE_MODE = STOPP_ENGINE;
	}
}

void engine_start_function(void){

	ENGINE_MODE = START_ENGINE;
}

void engine_stopp_function(void){
	ENGINE_MODE = STOPP_ENGINE;
}

void engine_init_function(void){
	system_state = GOTO_START_POSITION;
}

/*
 * highspeed_task():	executes corresponding outputs depending on system_state.
 */
uint8_t highspeed_task(void){

	svm_output pwm;

	switch (system_state){

		case STOPP:
			foc_values.foc_mode 	= FOC_CLOSELOOP;
			observer_values.state 	= OFF;

			clear_foc(&foc_values);
			reset_pi_integrator();

			speed_rate_memory = 0;
			foc_values.speed_rad = 0;
			automatic_counter  = 0;


			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;

		break;


		/*
		 * GOTO_START_POSITION: move the Rotor to the q-Axis, then set the angle to zero
		 */
		case GOTO_START_POSITION:

			if(init_safty_flag == false){
				PWM_All_Start();
			}else{
				PWM_All_Stop();
				break;
			}

			foc_values.foc_mode 	= FOC_CLOSELOOP;
			observer_values.state 	= OFF;

			dq_t V;
			observer_values.theta = 0;
			execute_FOC(&foc_values, Iq_set);

			safety_counter++;
			if(safety_counter > 15000){ // trigger init-process too often. protect motor
				Error_Handler();
			}

			// here: activate 200 cycles the goto_position function, then wait 4800 cycles before go to WAIT
			if(wait_counter < 5000){
				if(wait_counter < 4000){
					V.d = (GOTOSTART_Q_VOLTAGE / (float)MAX_VOLTAGE); // *2.0f
					V.q = 0.0f;
					pwm = goto_position(V);
					set_encoder_to_zero(&foc_values);
					TIM1->CCR1 = pwm.Sa;
					TIM1->CCR2 = pwm.Sb;
					TIM1->CCR3 = pwm.Sc;

				}
				V.d = 4.0f;
				V.q = 0.0f;
				pwm = goto_position(V);
				TIM1->CCR1 = pwm.Sa;
				TIM1->CCR2 = pwm.Sb;
				TIM1->CCR3 = pwm.Sc;
				wait_counter += 1;
			}else{
				init_safty_flag = true;
				wait_counter = 0;
				if(DEBUG_MODE == ENABLE){
					system_state = WAIT;
				}else{
					system_state = STOPP;
				}
			}

		break;

		/*
		 * WAIT: set Output to zero
		 */

		case WAIT:
			observer_values.state 	= OFF;
			if (wait_counter <4000){
				wait_counter += 1;

				set_output_to_zero();
				TIM1->CCR1 = 0;
				TIM1->CCR2 = 0;
				TIM1->CCR3 = 0;

			}else{

				wait_counter = 0;
				reset_pi_integrator();
				foc_values.speed_rad = 0;
				ENGINE_MODE = RUN_ENGINE;
			}

		break;

		/*
		 * RUN_CLOSED_LOOP:	Start the closed loop. get the angle from the encoder with get_el_angle().
		 * 					if position control is enabled, trigger lowspeed (position control PI) which
		 * 					gives the output for middlespeed (speed control PI) which gives the output (Iq_set)
		 * 					for current control (execute_FOC()). if position control is disabled, then
		 * 					speed_reference is the input for middlespeed_task. Generate the output commands
		 * 					for TIM1 using execute_SVM_matlab().
		 */

		case RUN_CLOSED_LOOP:


			foc_values.foc_mode 	= FOC_CLOSELOOP;
//			observer_values.state 	= OFF;
			observer_values.state = ON;
			get_el_angle(&foc_values);

			safety_counter = 0;


//			state_observer_speed(&foc_values, FOC_FREQUENCY);

//			state_observer_disturbance(&foc_values, FOC_FREQUENCY);

			execute_FOC(&foc_values, Iq_set);
			pwm = get_PWM_OUTPUT_Q15(foc_values.V_q15);

			TIM1->CCR1 = pwm.Sa;
			TIM1->CCR2 = pwm.Sb;
			TIM1->CCR3 = pwm.Sc;
//			Collect_And_Print_Data(foc_values.i_a,foc_values.i_b, foc_values.v_circle.d, foc_values.v_circle.q);

		break;

		/*
		 * OPENLOOP:	Run in openloop
		 */
		case OPENLOOP:

			if (closed_loop_token == 1){
				foc_values.foc_mode 		= FOC_OBSERVER;
			}else{
			foc_values.foc_mode 		= FOC_OPENLOOP;
			}
			observer_values.state 		= ON;
			get_el_angle(&foc_values); // need for debug

			// ############## RAMP FUNCTION #################
			if(speed_reference > 0){
				if((int16_t)open_loop_speed_rpm < speed_reference){
					open_loop_speed_rpm += (OPENLOOP_RAMP * (float)openloop_counter);
				}else{
					open_loop_speed_rpm = (float)speed_reference;
				}
			}else{
				if((int16_t)open_loop_speed_rpm > speed_reference){
					open_loop_speed_rpm -= (OPENLOOP_RAMP * (float)openloop_counter);
				}else{
					open_loop_speed_rpm = (float)speed_reference;
				}
			}
			// ###############################################

			generate_openloop((int16_t)open_loop_speed_rpm, &foc_values);
			//sliding_mode_observer(&foc_values,&observer_values);
			execute_FOC(&foc_values, Iq_set);
			pwm = execute_SVM_matlab(foc_values.v_abc);


			TIM1->CCR1 =pwm.Sa;
			TIM1->CCR2 =pwm.Sb;
			TIM1->CCR3 =pwm.Sc;

//			clear_control_parameter();

			if(openloop_counter< FOC_FREQUENCY) openloop_counter += 1;
		break;

		// default state == STOPP
		default:

			clear_foc(&foc_values);
			reset_pi_integrator();

			speed_rate_memory = 0;
			foc_values.speed_rad = 0;
			automatic_counter  = 0;


			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;


		break;



	}

	return 1;
}


/*
 * middlespeed_task: runs at a lower speed than the highspeed_task. executes the speed controller.
 */
float middlespeed_task(int16_t speed_ref){

	if(middlespeed_task_counter < (DIVISION_M-1)){
			middlespeed_task_counter += 1;

	}else{
		middlespeed_task_counter = 0;
	// ####### START MIDDLESPEED ########

		speed_calculation(&foc_values);
		foc_values.Iq_set_q15 = pi_speed_q15(foc_values.speed,speed_ref,&foc_values);
		speed_reference = speed_ref;
		automatic_switch_off();
		data_log_speed_current_500Hz(&foc_values, system_state);


// ############# END MIDDLESPEED #########################
	}

		return 0;
	}


void lowspeed_task(void){
	if (lowspeed_task_counter < (DIVISION_L-1)){
		lowspeed_task_counter += 1;
	}else{
		lowspeed_task_counter = 0;
		// ####### START LOW SPEED ########
		/*
		 *  Low speed time measurement returns 5 us
		 */


//		state_observer_speed(&foc_values,0.0f, L_FREQUENCY);
//		Collect_And_Send_Data(0xAA,0,0,0); // DEBUG
		// reed voltage
		foc_values.source_voltage = get_voltage_value();
		execute_voltage_measurement();


		// ####### END LOW SPEED ########
	}
}

void test_function_speed(void){
	switch (test_function_flag){

	case 0:
	// nothing
	break;
	default:
		int16_t state = test_function_counter/4500;
		switch (state){

		case 0:
			speed_reference = 500;
			break;
		case 1:
			speed_reference = 1500;
		break;
		case 2:
			speed_reference = 2500;
		break;
		case 3:
			speed_reference = 3000;
		break;
		case 4:
			speed_reference = 3500;
		break;
		case 5:
			speed_reference = 4000;
		break;
		case 6:
			speed_reference = 3000;
		break;
		case 7:
			speed_reference = 1500;
		break;
		default:
			test_function_flag = 0;
			test_function_counter = 0;
		}

		test_function_counter += 1;
	break;

	}

}


static void PWM_All_Stop(void)
{
	if(MOTOR_DECOUPLE_CURRENT == OFF) return; // can be activate in settings.h
    // Hauptkanäle stoppen
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    // Komplementärkanäle stoppen
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

static void PWM_All_Start(void)
{
	if(MOTOR_DECOUPLE_CURRENT == OFF) return;
    // Hauptkanäle starten
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // Komplementärkanäle starten
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

// we can do the init process only we wait 2 sec in other states
static void SAFETY_system_state_observer(void){
	static uint16_t safty_counter = 0;
	if(system_state != GOTO_START_POSITION){
		safty_counter++;
	}

	if(safty_counter > (2*FOC_FREQUENCY)){ // 2s ; remember maximum are 2^16 (65536)
		safty_counter = 0;
		init_safty_flag = false;
	}

}


