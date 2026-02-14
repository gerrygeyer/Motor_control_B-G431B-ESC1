/*
 * communication.c
 *
 *  Created on: Nov 1, 2024
 *      Author: Gerry Geyer
 */


#include "communication.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "settings.h"
#include <stdbool.h>
#include <stdlib.h>
#include "motor_safety.h"
#include "motor_types.h"

#define BUFFER_SIZE 500
#define DATA_POINTS 4
#define BUFFER_LENGHT 2

extern UART_HandleTypeDef huart2;
extern Motor g_motor;

HAL_StatusTypeDef status_send;

data communication;
uint8_t RxData[4];
uint16_t com_counter;
uint16_t cap_counter;
uint8_t hundert_counter;
uint16_t fivehundert_counter;
uint8_t cap_flag;

uint8_t half_speed_flag;
uint8_t half_speed_counter;


int16_t txBuffer_1[BUFFER_SIZE * DATA_POINTS];
int16_t txBuffer_2[BUFFER_SIZE * DATA_POINTS];
uint8_t buffer_flag;

int16_t txBufferTest[1];

float DataBuffer1[BUFFER_LENGHT*500];
float DataBuffer2[BUFFER_LENGHT*500];
float DataBuffer3[BUFFER_LENGHT*500];
float DataBuffer4[BUFFER_LENGHT*500];

float Data_out1;
float Data_out2;
float Data_out3;
float Data_out4;

volatile uint16_t sampleCounter = 0;

int16_t send_mean_Iq;






void init_communication(void){
	HAL_UART_Init(&huart2);;

 	RxData[0] = 0x00;
 	RxData[1] = 0x00;
 	RxData[2] = 0x00;
 	RxData[3] = 0x00;

 	communication.old_state		= MOTOR_STOPP;
 	communication.state 		= MOTOR_STOPP;
 	communication.speed_command	= 0;


	if(HAL_UART_Receive_DMA(&huart2, &RxData[0], sizeof(RxData)) != HAL_OK){
		 Error_Handler();
	}
//	HAL_UART_Receive_IT(&huart2, &RxData[0], sizeof(RxData));
 	com_counter = 0;

 	buffer_flag = LOW;

 	cap_counter = 0;
 	hundert_counter = 0;
 	fivehundert_counter = 0;
 	cap_flag = LOW;
 	half_speed_flag = LOW;
 	half_speed_counter = 0;


}
/*
 * the function is triggered when data is received
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
    if (huart->Instance == USART2) {
        if(RxData[0] == 0xFF){
        	// the data were received correctly
        	communication.state = (uint8_t)RxData[1];
        	communication.speed_command = (int16_t)(RxData[3] | (RxData[2]<<8));
        	if(abs(communication.speed_command) < MIN_RPM_COMMAND_FROM_MASTER){
        		g_motor.stop_request_flag = true;
				// engine_stopp_function();
        	}

        	// let's check if there is a new order
        	if(communication.state != communication.old_state){
        		// if yes, then which one?
				switch (communication.state){
					case MOTOR_STOPP:
						g_motor.stop_request_flag = true;
						// engine_stopp_function();
					break;
					case MOTOR_START:
						g_motor.start_request_flag = true;
						// engine_start_function();
					break;
					case MOTOR_INIT:
//						engine_init_function();
					break;
					default:
						// do nothing
					break;
				}

        	}
        	communication.old_state = communication.state;
        	// we have connection
        	recive_motor_command();
        	// limit the input for safty
        	communication.speed_command = (communication.speed_command > MAX_SPEED)? MAX_SPEED: communication.speed_command;
        	communication.speed_command = (communication.speed_command< -MAX_SPEED)? -MAX_SPEED: communication.speed_command;
			g_motor.recive_command_flag = true; 
			g_motor.speed_ref = communication.speed_command;
        }else{
        	// the data was received incorrectly, delete the memory
        	RxData[0] = 0x00;
        	RxData[1] = 0x00;
        	RxData[2] = 0x00;
        	RxData[3] = 0x00;
        }
    }
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
}
int16_t get_speed_command(void){
	return communication.speed_command;
}


void data_log_speed_current_500Hz(FOC_HandleTypeDef *pHandle_foc, uint8_t system_state){
	static uint16_t send_fault_counter = 0;
	static uint8_t send_value_buffer[10] = {0,0,0,0,0,0,0,0,0,0};

		const uint8_t header1 = 0xF9;
		const uint8_t header2 = 0xF7;
		const uint8_t footer = 0x53;


		int16_t iq_meas 	= pHandle_foc->Iq_meas_q15;
		int16_t speed		= pHandle_foc->speed;
		uint16_t battery	= pHandle_foc->source_voltage;
		uint8_t state		= system_state;

		send_value_buffer[0] = (header1);
		send_value_buffer[1] = (header2);
		send_value_buffer[2] = (state);
		send_value_buffer[3] = iq_meas;
		send_value_buffer[4] = (iq_meas >> 8);
		send_value_buffer[5] = (speed);
		send_value_buffer[6] = (speed >> 8);
		send_value_buffer[7] = (battery);
		send_value_buffer[8] = (battery >> 8);
		send_value_buffer[9] = (footer);
//		HAL_UART_Transmit_IT(&huart2, (uint8_t*)send_value_buffer, 9);
		if (huart2.gState == HAL_UART_STATE_READY){
			status_send = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)send_value_buffer, 10);
			if(status_send != HAL_OK){
				send_fault_counter++;
			}
		}else{
			send_fault_counter++;
		}

	if (send_fault_counter > 100){
		send_fault_counter = 0;
	}
//}
}


// ######## DEBUGGING FUNCTION ########

void Collect_And_Print_Data(float Data1, float Data2, float Data3, float Data4){


	if(half_speed_counter < 6){
		half_speed_flag = HIGH;
	}else{
		half_speed_flag = LOW;
		half_speed_counter = 0;
	}
	half_speed_counter +=1;


	switch (cap_flag){

	case (HIGH):
		if (half_speed_flag == HIGH){
			break;
		}
		if(cap_counter<(BUFFER_LENGHT*500)){
		DataBuffer1[cap_counter] = Data1;
		DataBuffer2[cap_counter] = Data2;
		DataBuffer3[cap_counter] = Data3;
		DataBuffer4[cap_counter] = Data4;
		cap_counter +=1;
		}else{
			if(hundert_counter < HUNDERT_COUNTER){
				hundert_counter +=1;
			}else{
				if(fivehundert_counter < (BUFFER_LENGHT*500)){
					Data_out1 = DataBuffer1[fivehundert_counter];
					Data_out2 = DataBuffer2[fivehundert_counter];
					Data_out3 = DataBuffer3[fivehundert_counter];
					Data_out4 = DataBuffer4[fivehundert_counter];
					fivehundert_counter +=1;
				}else{
					fivehundert_counter = 0;
					cap_flag = LOW;
					cap_counter = 0;
				}
				hundert_counter = 0;
			}

		}
	break;

	default:
		// nothing
	break;

	}
}


