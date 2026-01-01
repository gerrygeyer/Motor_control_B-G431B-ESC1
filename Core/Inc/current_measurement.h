/*
 * current_measurment.h
 *
 *  Created on: Jul 12, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_CURRENT_MEASUREMENT_H_
#define INC_CURRENT_MEASUREMENT_H_

#include <foc.h>

/*
 * ADC1:
 * 		CH3 			<- OAMP1
 * 		CH12 			<- OAMP3
 * 		CH1				<- VBUS
 * ADC2:
 * 		CH3				<- OAMP2
 * 		ChannelVopamp3 	<- OAMP3
 *
 */


#define SHUNT_MEASRUEMENT_DIRECTION -1

#define ADC_OFFSET_A 	2025 //0x82F  // Offset of 4096/2
#define ADC_OFFSET_B 	2025 //0x82F  // Offset of 4096/2
#define ADC_VOLTAGE		3.3f 	// V

#define ADC_MAX_COUNT 4096.0f


#define LP_CUTOFF_FREQ 150

#define OFFSET_CAL_COUNT FOC_FREQUENCY



#define COUNT_TO_AMP_f 			0.02822876f	// float i = (float)(COUNT_TO_AMP_f * ADC_output_value_without_offset)

/*
 * COUNT_TO_AMP_t include two multiplications:
 * 		1. mulitplicate with 0.02822876f
 * 		2. multiplicate with 2^15
 *
 * 		-> 0.02822876 * 2^15 = 925
 */
#define COUNT_TO_AMP_t			925 //  = 0.02822876 * 2^15
#define MAX_CURRENT_SCALE		32 // 2^5 -> good choice for bitshift

#define DIV_MAX_CURRENT_Q15		5 // 2^5


/*
 * normal case is:
 * 		1. divide to i_max
 * 		2. multiplicate with 2^15
 *
 * 		-> for efficient calculation we can store the parameter int_to_current = 2^15/i_max
 *
 * 		here the trick: choose max current (if possible) as a 2^n number. we take 32A = 2^5
 * 		now we can take a simple bitshift instead of multiplication:
 *
 * 			2^15 / 2^5 = 2^10
 *
 * so every time you multiplicate current by itself you have to devide by this constant:
 * 	Example: i_16t =  (2^15/MAX_CURRENT_SCALE) * i;
 * 			->	x = i_16t * i_16t;
 * 				x >>= CURRENT_15_DEV;
 *
 * 			-> Result: (2^15/MAX_CURRENT_SCALE) * i^2
 * 			Important for resolution of the values
 */
#define CURRENT_15_DEV				10




// Over current protection (Software)
#define MAX_OVER_CURRENT_VALUE		15

/*
 * so the maximum current are (15/32) * 2^15
 */
#define MAX_CURRENT_VALUE_INT		15360


#define MAX_Voltage_Battery			32 // good choice (2^5) wen dont use this, we make bitshift 5
//#define Battery_Voltage_Factor_Q7	15293 // (18/187) * (2^12/3.3) * Q7
#define BAT_VOLT32_Q25				8777 // (3,3/(2^12)) * (187/18) * (2^20)
//#define VOLT_TO_VALUE	(float)119.475
//#define VALUE_TO_VOLT	(float)0.008369954427


typedef struct {
	uint16_t a;
	uint16_t b;
}ab16_t;

typedef struct {
	int32_t a;
	int32_t b;
}abi32_t;

typedef struct{
	uint32_t a;
	uint32_t b;
}ab32_t;

typedef struct{
	uint16_t oamp1;
	uint16_t oamp3;
	uint16_t vbus;
	uint16_t temp;
}adc1_ch;

typedef struct{
	uint16_t oamp2;
	uint16_t oamp3;
}adc2_ch;




extern struct u16_current_measurement u16_adc_offset_value;
/*
 * Initialize the the values to zero and calculate the constants
 */
void init_current_measurement(void);

/*
 *  After DMA has finished, read the current measurement from the ADC and remove
 *  the DC component by substract lowpassfilter (cut. freq around 1 Hz)
 */
void execute_current_measurement(void);

/*
 * returns the current values ia ib in Q15 format
 */
ab_t get_realCurrentQ15(void);
/*
 * execute the ADC in regular mode to measured the battery voltage
 */
void execute_voltage_measurement(void);

/*
 * return the adc voltage
 */
int16_t get_voltage_value(void);

#endif /* INC_CURRENT_MEASUREMENT_H_ */
