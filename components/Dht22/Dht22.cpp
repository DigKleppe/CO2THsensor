/*
 * Sha22.cpp
 *
 *  Created on: Oct 11, 2020
 *      Author: dig
 */

#include "Dht22.h"
#include "esp_timer.h"

#define ESP_INTR_FLAG_DEFAULT 0

static uint64_t *pDhtShr; // global


static int64_t lastTime;
uint16_t tmdat[45];
int pntr;

// measures time of between every negative edge
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	int64_t time = esp_timer_get_time();
	uint16_t diff = time-lastTime;
	if ( diff < 60) // not valid
		return;

	if ( pntr< 45)
		tmdat[pntr++] = diff;

	bool bit = false;
	if (diff > 100) {   // 70-80 us low, 125us high
		bit = true;
	}
	lastTime = time;
	*pDhtShr <<= 1;
	*pDhtShr += bit;
}


Dht22::Dht22(gpio_num_t shaPin) {
	pin = shaPin;
	pDhtShr = &dhtLowLevelData.shr; // to inform isr, cannot access my class members directly
	gpio_config_t io_conf = {
			.pin_bit_mask = 1ULL << pin,
			.mode = GPIO_MODE_OUTPUT_OD,
			.pull_up_en = GPIO_PULLUP_ENABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_NEGEDGE,
	};
	gpio_config(&io_conf);
	gpio_set_level(pin, 1);
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	gpio_isr_handler_add(pin, gpio_isr_handler, (void*) pin);
}




Dht22::~Dht22() {
	// TODO Auto-generated destructor stub
}


inline int16_t Dht22::convertData(uint8_t msb, uint8_t lsb)
{
	int16_t data;
	data = msb & 0x7F;
	data <<= 8;
	data |= lsb;
	if (msb & BIT(7)) {
		data = 0 - data;       // convert it to negative
	}
	return data;
}


esp_err_t Dht22::getData( dht22Data_t * pDhtData){
	int16_t iTemperature, iHumidity;

	gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD); // command dht to ouput data
	gpio_set_level(pin, 0);
	vTaskDelay(2/portTICK_PERIOD_MS);
	gpio_set_level(pin, 1);
	lastTime = esp_timer_get_time();
	dhtLowLevelData.shr = 0;
	pntr =0;
	gpio_set_direction(pin, GPIO_MODE_INPUT);

	vTaskDelay(5/portTICK_PERIOD_MS);  // wait until dht is ready sending data
	if(dhtLowLevelData.shr == 0 )
		return ESP_ERR_NOT_FOUND; // no response

	uint8_t cs = dhtLowLevelData.bytes[1] + dhtLowLevelData.bytes[2] + dhtLowLevelData.bytes[3] + dhtLowLevelData.bytes[4];


	if (dhtLowLevelData.bytes[0] != cs ) {
		printf(" Checksum failed %d %d ", cs, dhtLowLevelData.bytes[0]);
		return ESP_ERR_INVALID_CRC;
	}
	iTemperature = convertData(dhtLowLevelData.bytes[2],  dhtLowLevelData.bytes[1]);
	iHumidity = convertData( dhtLowLevelData.bytes[4],  dhtLowLevelData.bytes[3]);
	pDhtData->temperature = (float) iTemperature/10.0;
	pDhtData->humidity = (float) iHumidity/10.0;
	return ESP_OK;
}
