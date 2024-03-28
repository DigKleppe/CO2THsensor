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

void pulseTimerTask(void *parameters);
void sensorTask(void *pvParameter);

int getInfoValuesScript (char *pBuffer, int count);
int getCalValuesScript (char *pBuffer, int count);
int saveSettingsScript (char *pBuffer, int count);
int cancelSettingsScript (char *pBuffer, int count);
int calibrateRespScript(char *pBuffer, int count);
int getSensorNameScript (char *pBuffer, int count);
void parseCGIWriteData(char *buf, int received);



#endif /* MAIN_SENSOR_H_ */
