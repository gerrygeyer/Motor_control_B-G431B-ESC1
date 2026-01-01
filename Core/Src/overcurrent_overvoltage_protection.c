/*
 * overcurrent_overvoltage_protection.c
 *
 *  Created on: Sep 26, 2024
 *      Author: Gerry Geyer
 */
#include<main.h>
#include<task.h>

void init_protection(void){

}

void overcurrent_occurs(void){
	engine_stopp_function();
	Error_Handler();
}
