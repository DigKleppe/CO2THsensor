#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "wifiConnect.h"
#include "settings.h"
#include "guiTask.h"
#include "clockTask.h"

#include <esp_err.h>

esp_err_t init_spiffs(void);

void sensorTask(void *pvParameter);

#define BLINK_GPIO	GPIO_NUM_4
static const char *TAG = "main";

// static void blinkTask(void *pvParameter) {

// 	gpio_reset_pin(BLINK_GPIO);
// 	/* Set the GPIO as a push/pull output */
// 	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

// 	while (1) {
// 		gpio_set_level(BLINK_GPIO, 1);
// 		vTaskDelay(1000 / portTICK_PERIOD_MS);
// 		gpio_set_level(BLINK_GPIO, 0);
// 		if (connectStatus == IP_RECEIVED)
// 			vTaskDelay(5000 / portTICK_PERIOD_MS);
// 		else
// 			vTaskDelay(1000 / portTICK_PERIOD_MS);
// 	}
// }

extern "C" void app_main(void) {
	esp_err_t err;

	int OldconnectStatus = -1;

	displayMssg_t recDdisplayMssg;
	char line[50];

	ESP_LOGI(TAG, "started\n\n");

	gpio_set_drive_capability((gpio_num_t) CONFIG_TFT_SCLK, GPIO_DRIVE_CAP_3);

	ESP_ERROR_CHECK(init_spiffs());
	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ESP_ERROR_CHECK(nvs_flash_init());
	}
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	loadSettings();

	xTaskCreate(&guiTask, "guiTask", 1024 * 4, NULL, 0, NULL);
	do {
		vTaskDelay(10);
	} while (!displayReady);

 //strcpy(wifiSettings.SSID, "asasa"); // test

	wifiConnect();

	recDdisplayMssg.str1 = line;
	recDdisplayMssg.showTime = 0;

	xTaskCreate(sensorTask, "sensorTask", 1024 * 5, (void*) 0, 10, NULL);

	do {
		vTaskDelay(100);
	} while (connectStatus != IP_RECEIVED);
	
	xTaskCreate(clockTask, "clock", 4 * 1024, NULL, 0, NULL);

	while (1) {
		if (OldconnectStatus != connectStatus) {
			switch (connectStatus)
			{
			case CONNECTING:
				recDdisplayMssg.line = 5;
				sprintf(line, "Verbinden met");
				xQueueSend(displayMssgBox, &recDdisplayMssg, 0);
				xQueueReceive(displayReadyMssgBox, &recDdisplayMssg, portMAX_DELAY);
				recDdisplayMssg.line = 6;
				sprintf(line, "%s", wifiSettings.SSID);
				xQueueSend(displayMssgBox, &recDdisplayMssg, 0);
				xQueueReceive(displayReadyMssgBox, &recDdisplayMssg, portMAX_DELAY);
				break;
			case CONNECTED:	
				recDdisplayMssg.line = 5;
				sprintf(line, "Verbonden met");
				xQueueSend(displayMssgBox, &recDdisplayMssg, 0);
				xQueueReceive(displayReadyMssgBox, &recDdisplayMssg, portMAX_DELAY);
				recDdisplayMssg.line = 6;
				sprintf(line, "%s", wifiSettings.SSID);
				xQueueSend(displayMssgBox, &recDdisplayMssg, 0);
				xQueueReceive(displayReadyMssgBox, &recDdisplayMssg, portMAX_DELAY);
				break;

			case WPS_ACTIVE:
				recDdisplayMssg.line = 5;
				sprintf(line, "WPS actief");
				xQueueSend(displayMssgBox, &recDdisplayMssg, 0);
				xQueueReceive(displayReadyMssgBox, &recDdisplayMssg, portMAX_DELAY);
				recDdisplayMssg.line = 6;
				sprintf(line, "  ..........");
				xQueueSend(displayMssgBox, &recDdisplayMssg, 0);
				xQueueReceive(displayReadyMssgBox, &recDdisplayMssg, portMAX_DELAY);
				break;

			case IP_RECEIVED:
				recDdisplayMssg.line = 5;
				snprintf(line, sizeof(line), "%s %s", wifiSettings.SSID , userSettings.moduleName);
				xQueueSend(displayMssgBox, &recDdisplayMssg, 500/portTICK_PERIOD_MS);
				xQueueReceive(displayReadyMssgBox, &recDdisplayMssg, portMAX_DELAY);
				recDdisplayMssg.line = 6;
				sprintf(line, "%s", myIpAddress);
				xQueueSend(displayMssgBox, &recDdisplayMssg, 0);
				xQueueReceive(displayReadyMssgBox, &recDdisplayMssg, portMAX_DELAY);
				break;

				default:
				break;
			}
			OldconnectStatus = connectStatus;
		}
		vTaskDelay(1000);
	}
}

