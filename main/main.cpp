///* OTA example
//
//   This example code is in the Public Domain (or CC0 licensed, at your option.)
//
//   Unless required by applicable law or agreed to in writing, this
//   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
//   CONDITIONS OF ANY KIND, either express or implied.
//*/
#include <string.h>
#include "errno.h"
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_image_format.h"


#include "string.h"

#include "nvs.h"
#include "nvs_flash.h"
//#include "protocol_examples_common.h"
//#include <sys/socket.h>
//#if CONFIG_EXAMPLE_CONNECT_WIFI
//#include "esp_wifi.h"
//#endif
#include "driver/gpio.h"
#include "wifiConnect.h"
#include "settings.h"
#include "updateSpiffsTask.h"


esp_err_t init_spiffs(void);
TaskHandle_t connectTaskh;


#define BLINK_GPIO	GPIO_NUM_4
static const char *TAG = "mainSPIFFSOTA";


static void blinkTask(void *pvParameter) {
	ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
	gpio_reset_pin (BLINK_GPIO);
	/* Set the GPIO as a push/pull output */
	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

	while (1) {
		gpio_set_level(BLINK_GPIO, 1);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		gpio_set_level(BLINK_GPIO, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

// ensure after reset back to factory app for OTA
static void setBootPartitionToFactory(void) {
	esp_image_metadata_t metaData;
	esp_err_t err;

	const esp_partition_t *factPart = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, "factory");
	if (factPart != NULL) {
		esp_partition_pos_t factPartPos;
		factPartPos.offset = factPart->address;
		factPartPos.size = factPart->size;

		esp_image_verify(ESP_IMAGE_VERIFY, &factPartPos, &metaData);

		if (metaData.image.magic == ESP_IMAGE_HEADER_MAGIC) {
			ESP_LOGI(TAG, "Setting bootpartition to OTA factory");

			err = esp_ota_set_boot_partition(factPart);
			if (err != ESP_OK) {
				ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
			}
		}
	}
}

extern "C" void app_main(void) {
	TaskHandle_t otaTaskh;
	esp_err_t err;
	char newStorageVersion[MAX_STORAGEVERSIONSIZE] = {};

	ESP_LOGI(TAG, "OTA Klp_main start");

	setBootPartitionToFactory();

	xTaskCreate(&blinkTask, "blink", 8192, NULL, 5, NULL);
	ESP_ERROR_CHECK(init_spiffs());

	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);

	loadSettings();
	if (strcmp(wifiSettings.upgradeFileName ,CONFIG_FIRMWARE_UPGRADE_FILENAME)!= 0) {
		strcpy(wifiSettings.upgradeFileName, CONFIG_FIRMWARE_UPGRADE_FILENAME);
		saveSettings();  // set filename for OTA via factory firmware
	}

	wifiConnect();

	do {
		vTaskDelay(1000);
	} while ( connectStatus != IP_RECEIVED);

	while (1) {
		vTaskDelay(10000);
		newStorageVersion[0] = 0;
		spiffsUpdateFinised = true;
		xTaskCreate(&updateSpiffsTask, "updateSpiffsTask", 8192, (void *)newStorageVersion, 5, &otaTaskh);
		while( !spiffsUpdateFinised)
			vTaskDelay(1000);

		if ( newStorageVersion[0]) {
			strcpy( userSettings.spiffsVersion ,newStorageVersion );
			saveSettings();
		}
	}
}

