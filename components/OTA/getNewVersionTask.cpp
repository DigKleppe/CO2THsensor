/*
 * getNewFirmWareVersionTask.cpp
 *
 *  Created on: Jan 8, 2024
 *      Author: dig
 */

#include <string.h>
#include "errno.h"
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_http_client.h"

#include "wifiConnect.h"
#include "settings.h"
#include "httpsRequest.h"
#include "include/getNewVersion.h"

#include <getNewVersion.h>
#include "esp_app_format.h"
#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "errno.h"

static const char *TAG = "getNewFirmWareVersionTask";

#define BUFFSIZE 1024
#define UPDATETIMEOUT (CONFIG_OTA_RECV_TIMEOUT / portTICK_PERIOD_MS)

volatile bool getNewVersionTaskFinished;

volatile updateStatus_t updateStatus;

typedef struct {
	char *infoFileName;
	char *dest;
} versionInfoParam_t;

void getNewVersionTask(void *pvParameter) {
	esp_err_t err;
	char updateURL[96];

	httpsMssg_t mssg;
	bool rdy = false;
	httpsRegParams_t httpsRegParams;
	versionInfoParam_t *versionInfoParam = (versionInfoParam_t*) pvParameter;
	getNewFirmwareVersionTaskFinished = false;

	httpsRegParams.httpsServer = wifiSettings.upgradeServer;

	strcpy(updateURL, wifiSettings.upgradeURL);
	strcat(updateURL, "//");
	strcat(updateURL, versionInfoParam->infoFileName);
	httpsRegParams.httpsURL = updateURL;
	httpsRegParams.destbuffer = versionInfoParam->dest;
	httpsRegParams.maxChars = MAX_STORAGEVERSIONSIZE - 1;

	int block = 0;

	int data_read;

	ESP_LOGI(TAG, "Starting getNewFirmWareVersionTask");

	xTaskCreate(&httpsGetRequestTask, "httpsGetRequestTask", 2 * 8192, (void*) &httpsRegParams, 5, NULL); // todo stack minimize
	vTaskDelay(100 / portTICK_PERIOD_MS); // wait for messagebox to create and connection to make
	xQueueSend(httpsReqRdyMssgBox, &mssg, 0);
	if (xQueueReceive(httpsReqMssgBox, (void*) &mssg, UPDATETIMEOUT)) {
		if (mssg.len <= 0) {
			ESP_LOGE(TAG, "error reading info file firmware version: %s", BINARY_INFO_FILENAME);
			httpsRegParams.destbuffer[0] = -1;
		} else {
			if (mssg.len < MAX_STORAGEVERSIONSIZE) {
				httpsRegParams.destbuffer[mssg.len] = 0;
				ESP_LOGI(TAG, "New firmware version: %s", httpsRegParams.destbuffer);
			} else {
				ESP_LOGE(TAG, "read firmware version too long: %s", BINARY_INFO_FILENAME);
				httpsRegParams.destbuffer[0] = -1;
			}
		}
	}

	while (mssg.len > 0) { // wait for httpsGetRequestTask to finish
		xQueueSend(httpsReqRdyMssgBox, &mssg, 0);
		xQueueReceive(httpsReqMssgBox, (void*) &mssg, UPDATETIMEOUT); // wait for httpsGetRequestTask to end
	};

	getNewFirmwareVersionTaskFinished = true;
	(void) vTaskDelete(NULL);
}

esp_err_t getNewVersion(char *infoFileName, char *newVersion) {

	getNewVersionTaskFinished = false;

	versionInfoParam_t versionInfoParam;
	versionInfoParam.infoFileName = infoFileName;
	versionInfoParam.dest = newVersion;

	xTaskCreate(&getNewVersionTask, "getNewVersionTask", 8192, (void*) &versionInfoParam, 5, NULL);

	while (!getNewVersionTaskFinished)
		vTaskDelay(100 / portTICK_PERIOD_MS);
	if ((int8_t) newVersion[0] != -1)
		return ESP_OK;
	else
		return ESP_FAIL;
}

void updateTask(void *pvParameter) {
	bool doUpdate;
	char newVersion[MAX_STORAGEVERSIONSIZE];

	const esp_partition_t *update_partition = NULL;
	const esp_partition_t *configured = esp_ota_get_boot_partition();
	const esp_partition_t *running = esp_ota_get_running_partition();

	if (configured != running) {
		ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08"PRIx32", but running from offset 0x%08"PRIx32, configured->address, running->address);
		ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
	}
	ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08"PRIx32")", running->type, running->subtype, running->address);

	while (1) {
		doUpdate = false;
		getNewVersion(BINARY_INFO_FILENAME, newVersion);
		if (newVersion[0] != 0) {
			if (strcmp(newVersion, wifiSettings.firmwareVersion) != 0) {
				ESP_LOGI(TAG, "New firmware version available: %s", newVersion);
				doUpdate = true;
			} else
				ESP_LOGI(TAG, "Firmware up to date: %s", newVersion);
		} else
			ESP_LOGI(TAG, "Reading New firmware info failed");

		if (doUpdate) {
			ESP_LOGI(TAG, "Updating firmware to version: %s", newVersion);
			xTaskCreate(&updateFirmwareTask, "updateFirmwareTask", 2 * 8192, NULL, 5, &otaTaskh);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			while (updateStatus == UPDATE_BUSY)
				vTaskDelay(100 / portTICK_PERIOD_MS);

			if (updateStatus == UPDATE_RDY) {
				ESP_LOGI(TAG, "Prepare to restart system!");
				vTaskDelay(100 / portTICK_PERIOD_MS);
				esp_restart();
			} else
				ESP_LOGI(TAG, "Update firmware failed!");
		}

		doUpdate = false;
		getNewVersion(SPIFFS_INFO_FILENAME, newVersion);
		if (newVersion[0] != 0) {
			if (strcmp(newVersion, wifiSettings.firmwareVersion) != 0) {
				ESP_LOGI(TAG, "New SPIFFS version available: %s", newVersion);
				doUpdate = true;
			} else
				ESP_LOGI(TAG, "SPIFFS  up to date: %s", newVersion);
		} else
			ESP_LOGI(TAG, "Reading New SPIFFS info failed");

		if (doUpdate) {
			ESP_LOGI(TAG, "Updating SPIFFS to version: %s", newVersion);
			xTaskCreate(&updateSpiffsTask, "updateSpiffsTask", 2 * 8192, NULL, 5, &otaTaskh);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			while (updateStatus == UPDATE_BUSY)
				vTaskDelay(100 / portTICK_PERIOD_MS);

			if (updateStatus == UPDATE_RDY) {
				ESP_LOGI(TAG, "SPIFFS flashed OK", newVersion);
			} else
				ESP_LOGI(TAG, "Update SPIFFS failed!");
		}
		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
}

