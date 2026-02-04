/*
 * svm.h
 *
 *  Created on: Jul 4, 2024
 *      Author: Gerry Geyer
 */

#include <foc_math.h>
#include <foc.h>

#ifndef INC_SVM_H_
#define INC_SVM_H_

#define FS_SWICHING_FREQUENCY 	10000.0f // Hz ; switching frequency fs > 10 * f_{bw\_current} <- w_{bw\_current} = 6000 rad
#define TS_SVM 					1/FS_SWICHING_FREQUENCY
#define CLOCK_COUNTER			TIM1_COUNTER
#define VDC_SVM					MAX_VOLTAGE

#define SVM_FACTOR 				(SQRT3 * 2)/3// / VDC_SVM

#define sin_pi_3_m0 			0.0f
#define sin_pi_3_m1				0.8660254038f // sqrt(3)/2
#define sin_pi_3_m2				0.8660254038f
#define sin_pi_3_m3				0.0f
#define sin_pi_3_m4				-0.8660254038f
#define sin_pi_3_m5				-0.8660254038f
#define sin_pi_3_m6				0.0f

#define cos_pi_3_m0				1.0f
#define cos_pi_3_m1				0.5f
#define cos_pi_3_m2				-0.5f
#define cos_pi_3_m3				-1.0f
#define cos_pi_3_m4				-0.5f
#define cos_pi_3_m5				0.5f
#define cos_pi_3_m6				1.0f



extern struct parameter_svm svm;

typedef struct {
	float t0;
	float t1;
	float t2;

}svm_time;



typedef struct {
	float Sa;
	float Sb;
	float Sc;

}svm_output_f;


void init_SVM(void);

void generate_svm_output(void);
void set_output_to_zero(void);

/**
 * @brief       Converts 3-phase Q15 voltages to PWM duty cycles.
 *
 * @details     Shifts Q15 voltage values from signed range (−1 … +0.9999) into unsigned (0 … 1),
 *              then scales them to the timer resolution defined by `TIM1_COUNTER`.
 *              Final values are clamped to `(TIM1_COUNTER - 1)` to avoid overflow.
 *
 * @param       v       3-phase voltage vector in Q15 format.
 *
 * @return      Corresponding PWM compare values for each phase (0 … TIM1_COUNTER−1).
 *
 * @note        Assumes Q15 input range. Timer output is right-shifted by 15 to preserve resolution.
 * @see         abc_16t, svm_output
 */
svm_output get_PWM_OUTPUT_Q15(abc_16t v);
#endif /* INC_SVM_H_ */
