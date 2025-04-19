/*
 * settings.h
 *
 *  Created on: Nov 30, 2017
 *      Author: dig
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <stdbool.h>
#include "esp_system.h"
#include <time.h>
#include <sys/time.h>

typedef enum { FLT, STR, INT , DESCR , CALVAL} varType_t;
#define MAX_STRLEN 16

#define USERSETTINGS_CHECKSTR 	"CO2TH"


typedef struct {
	char moduleName[MAX_STRLEN+1];
	char spiffsVersion[16]; // holding current version
	float temperatureOffset;
	float RHOffset;
	char checkstr[MAX_STRLEN+1];
}userSettings_t;


typedef struct {
	varType_t varType;
	int size;
	void * pValue;
	int minValue;
	int maxValue;
} settingsDescr_t;


//typedef enum {SETTINGS_CHANNEL,SETTINGS_SSID,SETTINGS_IP,SETTINGS_APSSID,SETTINGS_PREC,
//	SETTINGS_AVG,SETTINGS_SPEED } settingsID_t;

extern settingsDescr_t settingsDescr[];

extern "C" {
	esp_err_t saveSettings( void);
	esp_err_t loadSettings( void);
}

//esp_err_t saveUserSettings( void);
//esp_err_t loadUserSettings( void);

//esp_err_t saveCalibrationSettings( void);
//esp_err_t loadCalibrationSettings( void);

extern userSettings_t 			userSettings;


#endif /* SETTINGS_H_ */
