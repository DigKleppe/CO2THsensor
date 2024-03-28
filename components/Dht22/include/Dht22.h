/*
 * Sha22.h
 *
 *  Created on: Oct 11, 2020
 *      Author: dig
 */

#ifndef COMPONENTS_DHT22_SHA22_H_
#define COMPONENTS_DHT22_SHA22_H_

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>


class Dht22 {

public:
	typedef struct {
		float temperature;
		float humidity;
	}dht22Data_t;

	Dht22(gpio_num_t shaPin);
	virtual ~Dht22();
	esp_err_t getData( dht22Data_t * pDhtData);

private:
	typedef union {
		uint64_t shr;
		uint8_t bytes[8];
	} dhtLowlevelData_t;

	dhtLowlevelData_t dhtLowLevelData;
	gpio_num_t pin;
	inline int16_t convertData(uint8_t msb, uint8_t lsb);

};

#endif /* COMPONENTS_DHT22_SHA22_H_ */
