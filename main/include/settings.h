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
	char moduleName[MAX_STRLEN+1];  // not used anymore
	char checkstr[MAX_STRLEN+1];
}userSettings_t;

typedef struct {
	varType_t varType;
	int size;
	void * pValue;
	int minValue;
	int maxValue;
} settingsDescr_t;

extern settingsDescr_t settingsDescr[];

extern "C" {
	esp_err_t saveSettings( void);
	esp_err_t loadSettings( void);
}
extern userSettings_t 			userSettings;


#endif /* SETTINGS_H_ */
