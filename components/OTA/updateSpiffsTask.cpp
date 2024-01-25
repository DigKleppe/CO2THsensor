/*
 * updateSpiffsTask.cpp
 *
 *  Created on: May 24, 2023
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
#include "esp_image_format.h"
#include "updateSpiffsTask.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "wifiConnect.h"
#include "settings.h"
#include "httpsRequest.h"

static const char *TAG = "updateSPIFFSTask";

#define BUFFSIZE 1024
#define UPDATETIMEOUT 5000

static uint8_t ota_write_data[BUFFSIZE + 1] = { 0 };
volatile bool spiffsUpdateFinised;

static void exitSpiffsTask(void) {
	ESP_LOGE(TAG, "Exiting Spiffs updatetask ");

	spiffsUpdateFinised = true;
	(void) vTaskDelete(NULL);
}

void updateSpiffsTask(void *pvParameter) {
	esp_err_t err;
	size_t binary_file_length = 0;
	char updateURL[96];
	char newStorageVersion[MAX_STORAGEVERSIONSIZE];
	httpsMssg_t mssg;
	bool rdy = false;

	httpsRegParams_t httpsRegParams;
	bool doUpdate;

	ESP_LOGI(TAG, "Starting updateSpiffsTask");

	const esp_partition_t *spiffsPartition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);

	ESP_LOGI(TAG, "SPIFFS partition type %d subtype %d (offset 0x%08"PRIx32")", spiffsPartition->type, spiffsPartition->subtype, spiffsPartition->address);

	int block = 0;
	spiffsUpdateFinised = false;
	int data_read;

	memset(newStorageVersion, 0, sizeof(newStorageVersion));
	while (1) {
		err = ESP_OK;
		doUpdate = false;
		do { // check for new spiffs version

			getNewVersion(SPIFFS_INFO_FILENAME, newStorageVersion);

			if (newStorageVersion[0] != 0) {
				if (strcmp(newStorageVersion, userSettings.spiffsVersion) != 0)
					doUpdate = true;
			}
			if ()
				//vTaskDelay (CONFIG_CHECK_FIRMWARWE_UPDATE_INTERVAL * 60 * 60 * 1000 /portTICK_PERIOD_MS );
				vTaskDelay(10000 / portTICK_PERIOD_MS);

		} while (!doUpdate);

		if (!doUpdate) {
			httpsRegParams.httpsServer = wifiSettings.upgradeServer;
			strcpy(updateURL, wifiSettings.upgradeURL);
			strcat(updateURL, "//");
			strcat(updateURL, CONFIG_SPIFFS_UPGRADE_FILENAME);

			httpsRegParams.httpsURL = updateURL;

			httpsRegParams.destbuffer = ota_write_data;
			httpsRegParams.maxChars = sizeof(ota_write_data - 1);

			ESP_LOGI(TAG, "updating storage");

			xTaskCreate(&httpsGetRequestTask, "httpsGetRequestTask", 8192, (void*) &httpsRegParams, 5, NULL);

			int binary_file_length = 0;
			/*deal with all receive packet*/
			bool started = false;
			while (!rdy) {
				xQueueSend(httpsReqRdyMssgBox, &mssg, 0);
				if (xQueueReceive(httpsReqMssgBox, (void*) &mssg, ( UPDATETIMEOUT / portTICK_PERIOD_MS))) {
					data_read = mssg.len;
					block++;
					if (data_read < 0) {
						ESP_LOGE(TAG, "Error reading");
						exitSpiffsTask();
					}
					if (data_read == 0) {
						ESP_LOGI(TAG, "Ready written %d bytes %d mssgs", binary_file_length, block);
						rdy = true;
					} else {
						if (!started) {
							started = true;
							if (data_read > 0) {
								err = esp_partition_erase_range(spiffsPartition, 0, spiffsPartition->size);
								if (err != ESP_OK) {
									ESP_LOGE(TAG, "spiffs partition erase failed: (%s)", esp_err_to_name(err));
									exitSpiffsTask();
								}
								ESP_LOGI(TAG, "writing spiffs partition started");
							} else {
								ESP_LOGE(TAG, "Error reading");
								exitSpiffsTask();
							}
						}
						err = esp_partition_write(spiffsPartition, binary_file_length, (const void*) ota_write_data, data_read);
						if (err != ESP_OK) {
							ESP_LOGE(TAG, "Error ota write (%s)", esp_err_to_name(err));
							exitSpiffsTask();
						}
						binary_file_length += data_read;
//					ESP_LOGD(TAG, "Written image length %d", binary_file_length);
					}
				}
				strcpy((char*) pvParameter, newStorageVersion);
			}
		}
		if (!rdy) {
			do {
				xQueueSend(httpsReqRdyMssgBox, &mssg, 0);
				xQueueReceive(httpsReqMssgBox, (void*) &mssg, (UPDATETIMEOUT / portTICK_PERIOD_MS)); // wait for httpsGetRequestTask to end
			} while (mssg.len > 0);
		}

		exitSpiffsTask();
	}

