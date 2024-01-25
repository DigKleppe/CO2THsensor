/*
 * getNewBinaryVersion.h
 *
 *  Created on: Jan 8, 2024
 *      Author: dig
 */

#ifndef COMPONENTS_OTA_INCLUDE_GETNEWBINARYVERSION_H_
#define COMPONENTS_OTA_INCLUDE_GETNEWBINARYVERSION_H_

#define BINARY_INFO_FILENAME "firmWareVersion.txt"

#define MAX_STORAGEVERSIONSIZE 16

void getNewVersionTask(void *pvParameter);
esp_err_t esp_err_t getNewVersion (char * infoFileName , char * newVersion);

extern volatile bool getNewFirmwareVersionTaskFinished;

typedef enum {UPDATE_RDY, UPDATE_BUSY, UPDATE_ERROR } updateStatus_t;
extern volatile updateStatus_t updateStatus;



#endif /* COMPONENTS_OTA_INCLUDE_GETNEWBINARYVERSION_H_ */
