/*
 * communication.h
 *
 *  Created on: Nov 1, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include <stdint.h>


// communication
#define MOTOR_STOPP 		0x00
#define MOTOR_START 		0xAA
#define MOTOR_INIT			0xDD
#define MOTOR_NO_RESPONSE	0x11

#define HUNDERT_COUNTER (FOC_FREQUENCY/100)

void get_information(data* pHandle);
void automatic_switch_off(void);
void init_communication(void);

void Collect_And_Send_Data(int16_t signal1, int16_t signal2, int16_t signal3, int16_t signal4);
void Collect_And_Print_Data(float Data1, float Data2, float Data3, float Data4);
void data_log_speed_current_500Hz(FOC_HandleTypeDef *pHandle_foc, uint8_t system_state);
#endif /* INC_COMMUNICATION_H_ */
