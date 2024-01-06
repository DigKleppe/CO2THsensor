/*
 * PID.h
 *
 *  Created on: Apr 18, 2022
 *      Author: dig
 */

#ifndef MAIN_INCLUDE_PID_H_
#define MAIN_INCLUDE_PID_H_

#include "driver/gpio.h"

#define COOLING_PIN 	GPIO_NUM_2
#define HEATING_PIN 	GPIO_NUM_15

void updatePID ( float temperature);
extern float PIDsetting; // for cgi

typedef enum { THERMOSTATOFF, HEATING_ON, COOLING_ON } thermostatStatus_t;

extern thermostatStatus_t thermostatStatus;

#endif /* MAIN_INCLUDE_PID_H_ */
