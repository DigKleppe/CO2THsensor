/*
 * sensor.h
 *
 *  Created on: Jan 1, 2018
 *      Author: dig
 */

#ifndef MAIN_SENSOR_H_
#define MAIN_SENSOR_H_


#define DAYSAMPLESINTERVAL	5			// 5 minutes
#define SAMPLESPERHOUR		(3600/30)	// every 30 seconds
#define SAMPLESPERDAY		24*60/DAYSAMPLESINTERVAL
#define SAMPLESPERMONTH		30	   		// every day

#include "averager.h"

typedef struct {
	float measNr;
	float temperature;
	float humidity;
	float CO2value;
} measValues_t;

extern measValues_t measValues;


extern Averager temperatureHourBuffer;
extern Averager humidityHourBuffer;
extern Averager CO2HourBuffer;

extern Averager temperatureDayBuffer;
extern Averager humidityDayBuffer;
extern Averager CO2DayBuffer;


//
//extern uint16_t temperatureDayBuffer[SAMPLESPERDAY];
//extern uint16_t temperatureHourBuffer[SAMPLESPERHOUR];
//extern uint16_t CO2DayBuffer[SAMPLESPERDAY];
//extern uint16_t CO2HourBuffer[SAMPLESPERHOUR];
//
//
//extern int temperatureHourBufValues;
//extern int temperatureHourBufWrtIndex;
//extern int temperatureDayBufValues;
//extern int temperatureDayBufWrtIndex;
//
//extern int CO2HourBufValues;
//extern int CO2HourBufWrtIndex;
//extern int CO2DayBufValues;
//extern int CO2DayBufWrtIndex;


void pulseTimerTask(void *parameters);
void sensorTask(void *pvParameter);

typedef struct {
	float seqNr;
	float momentaryWatts;
	float minuteTotalWh;
	float hourTotalWh;
	float dayTotalkWh;
	float weekTotalkWh;
}powers_t;

extern powers_t powers;

#endif /* MAIN_SENSOR_H_ */
