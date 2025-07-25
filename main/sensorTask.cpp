/*
 * sensor.c
 *
 *  Created on: Jan 1, 2018
 *      Author: dig
 */

// #define SIMULATE
// #define FAST

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "settings.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "averager.h"
#include "cgiScripts.h"
#include "guiTask.h"
#include "sensorTask.h"
#include "sht4x.h"
#include "udpClient.h"
#include "wifiConnect.h"
#include "clockTask.h"

static const char *TAG = "sensorTask";

#define TIMER_DIVIDER 16							 //  Hardware timer clock divider 80Mhz
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // convert counter value to seconds
#define TIMERIDX 0									 // timer 0 group 0 used

#define SAMPLEPERIOD 5 // seconds

Averager temperatureAverager(60 / SAMPLEPERIOD);
Averager RHaverager(60 / SAMPLEPERIOD);
Averager CO2Averager(60 / SAMPLEPERIOD);

time_t now;

#define I2C_EXAMPLE_MASTER_SCL_IO 19		/*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO 18		/*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1			/*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ 100000	/*!< I2C master clock frequency */

#define CO2_SENSOR_ADDR 0x68		 /*!< slave address for BH1750 sensor */
#define WRITE_BIT I2C_MASTER_WRITE	 /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ	 /*!< I2C master read */
#define ACK_CHECK_EN 0x1			 /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0			 /*!< I2C master will not check ack from slave */
#define ACK_VAL (i2c_ack_type_t)0x0	 /*!< I2C ack value */
#define NACK_VAL (i2c_ack_type_t)0x1 /*!< I2C nack value */

#define DHT_GPIO GPIO_NUM_25

static esp_err_t readC02(i2c_port_t i2c_num, int *pValue);
static void i2c_master_init();

#define LOGINTERVAL 60 // seconds
// #define MEASUREINTERVAL 1 //
#define MAXLOGVALUES (24 * 60)
#define UDPTXPORT 5050
#define UDPCALTXPORT 5051

typedef struct {
	int32_t timeStamp;
	float temperature;
	float hum;
	int32_t co2;
} log_t;

static log_t tLog[MAXLOGVALUES];
static log_t lastVal;
static int timeStamp = 1;

static int logTxIdx;
static int logRxIdx;

extern int scriptState;
static float lastTemperature;
static float lastRH;
static bool enableAutCal;

static esp_err_t readC02(i2c_port_t i2c_num, int *pValue) {
	esp_err_t ret = -1;
	bool ok = false;
	i2c_cmd_handle_t cmd;
	uint8_t buffer[4];
	int value;

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, 0, ACK_CHECK_DIS);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	vTaskDelay(10 / portTICK_PERIOD_MS);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, CO2_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x22, ACK_CHECK_EN); // read RAM 2 databytes
	i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); // RAM address MSB
	i2c_master_write_byte(cmd, 0x08, ACK_CHECK_EN); // RAM address LSB
	i2c_master_write_byte(cmd, 0x2A, ACK_CHECK_EN); // checksum
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	if (ret == ESP_OK) {
		vTaskDelay(50 / portTICK_PERIOD_MS);
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, CO2_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
		i2c_master_read_byte(cmd, &buffer[0], ACK_VAL);
		i2c_master_read_byte(cmd, &buffer[1], ACK_VAL);
		i2c_master_read_byte(cmd, &buffer[2], ACK_VAL);
		i2c_master_read_byte(cmd, &buffer[3], NACK_VAL);
		i2c_master_stop(cmd);
		ret = i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
	} else
		printf("\n CO2 err 1 ");
	if (ret == ESP_OK)
		ok = true;
	//	}
	if (!ok)
		return ret;

	value = 0;
	value |= buffer[1] & 0xFF;
	value = value << 8;
	value |= buffer[2] & 0xFF;

	uint8_t sum = 0;						 // Checksum Byte
	sum = buffer[0] + buffer[1] + buffer[2]; // Byte addition utilizes overflow

	if (sum == buffer[3]) {
		*pValue = value;
		return ESP_OK;
	}
	printf("\nCO2 Checksum error\n");
	return ESP_ERR_INVALID_CRC;
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init() {
	i2c_port_t i2c_master_port = I2C_MASTER_NUM;
	i2c_config_t conf;
	memset(&conf, 0, sizeof(conf));
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;

	i2c_param_config(i2c_master_port, &conf);
	i2c_driver_install(i2c_master_port, conf.mode, I2C_EXAMPLE_MASTER_RX_BUF_DISABLE, I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}
// sends averaged values at 10, 20 , 30 abs seconds ,  11,21,31 for module 1 etc
void updTransmitTask(void *pvParameter) {

	time_t now;
	struct tm timeinfo;
	char str[80];
	bool isSend = false;

	vTaskDelay( 5000/ portTICK_PERIOD_MS); // wait to start for samples to collect

	while (1) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
		time(&now);
		localtime_r(&now, &timeinfo);
		if ((timeinfo.tm_sec % 10) == 0) {
			if (!isSend) {
				isSend = true;
				sprintf(str, "S0,%d,%2.2f,%3.1f,%d\n\r",(int) (CO2Averager.average()/ 1000.0), temperatureAverager.average() / 1000.0,
						RHaverager.average() / 1000.0, getRssi());
				UDPsendMssg(UDPTXPORT, str, strlen(str));
				ESP_LOGI(TAG, "UDP send %s %d", str, timeinfo.tm_sec);
				if (enableAutCal) {
					vTaskDelay(10);
					UDPsendMssg(UDPCALTXPORT, str, strlen(str));
				}
			}
		} else
			isSend = false;
	}
}

void sensorTask(void *pvParameter) {
	uint32_t xLastWakeTime;
	struct tm timeinfo;
#ifdef SIMULATE
	const uint32_t xPeriod = (500 / portTICK_PERIOD_MS);
	int n;
#else
	const uint32_t xPeriod = (SAMPLEPERIOD * 1000 / portTICK_PERIOD_MS);
#endif
	log_t tempVal;
	int32_t iTemperature, iHumidity; // temperary

	localtime_r(&now, &timeinfo);

	float temperature = 0, humidity = 0;
	int CO2value = 0;
	int tempCO2value = 0;
	esp_err_t err = 0;
	int I2Ctimeout = 10;

	bool minutePassed = false;
	bool SHTpresent = false;

	displayMssg_t recDdisplayMssg;
	char line[33];
	recDdisplayMssg.str1 = line;

	i2c_master_init();
	sensirion_i2c_init(I2C_MASTER_NUM);

	uint32_t serial = 0;
	int retries = 3;
	do {
		sht4x_read_serial(&serial);
		retries--;
	} while ((serial == 0) && (retries > 0));

	if (retries) {
		printf("\nSHT45 serial: %ld\n\r", serial);
		SHTpresent = true;
	} else
		printf("\nSHT45 not found!");
	
	xTaskCreate(updTransmitTask, "udptx", 4 * 1024, NULL, 0, NULL);

	xLastWakeTime = xTaskGetTickCount();

#ifdef SIMULATE
	for (n = 0; n < SAMPLESPERDAY; n++) {
		temperature = 150 / 2 * (1 + sin(n / 10));
		humidity = 150 / 2 * (1 + sin(n / 3));
		CO2value = 250 / 2 * (1 + sin(n / 3));
		temperatureHourBuffer.write((uint16_t)temperature);
		humidityHourBuffer.write((uint16_t)humidity);
		CO2HourBuffer.write((uint16_t)CO2value);
	}
	n = 0;
#endif

	while (1) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
		time(&now);
		localtime_r(&now, &timeinfo);

#ifdef SIMULATE
		err = ESP_OK;
		temperature = 150 / 2 * (1 + sin(n / 10));
		humidity = 150 / 2 * (1 + sin(n / 3));
		CO2value = 450 / 2 * (1 + sin(n / 3));
		n++;
#else
		if (SHTpresent) {
			err = sht4x_measure_blocking_read(&iTemperature, &iHumidity);

			if (err == ESP_OK) {
				temperature = iTemperature / 1000.0f;
				humidity = iHumidity / 1000.0f;
				lastTemperature = temperature;
				lastRH = humidity;
			}
#endif
		RHaverager.write((uint16_t)(1000.0 * humidity)); // use last values for graph
		temperatureAverager.write((uint16_t)(1000.0 * temperature));
	}
	else {
		temperature = 999;
		humidity = 999;
		err = ESP_OK;
	}

	if (err == ESP_OK) {
		I2Ctimeout = 10;

		recDdisplayMssg.line = 1;
		recDdisplayMssg.showTime = 0;
		sprintf(line, "T  :%2.1f", temperature - userSettings.temperatureOffset);
		xQueueSend(displayMssgBox, &recDdisplayMssg, 0);
		xQueueReceive(displayReadyMssgBox, &recDdisplayMssg, portMAX_DELAY);

		recDdisplayMssg.line = 2;
		sprintf(line, "RH :%2.1f ", humidity - userSettings.RHOffset);
		xQueueSend(displayMssgBox, &recDdisplayMssg, 0);
		xQueueReceive(displayReadyMssgBox, &recDdisplayMssg, portMAX_DELAY);

		printf("\n temp: %2.2f , hum: %2.1f ", temperature, humidity);
	} else {
		//	measValues.temperature = 9999;
		//	measValues.humidity = 9999;
		printf("\n error reading SHT: %x ", err);
	}
#ifndef SIMULATE
	err = readC02(I2C_NUM_1, &tempCO2value);
	if (err == ESP_OK)
		CO2value = tempCO2value;
#endif
	if (CO2value > 300)
		CO2Averager.write(CO2value * 1000);

	if (err == ESP_OK) {
		I2Ctimeout = 10;
		recDdisplayMssg.line = 0;
		sprintf(line, "CO2:%d", CO2value);
		xQueueSend(displayMssgBox, &recDdisplayMssg, 0);
		xQueueReceive(displayReadyMssgBox, &recDdisplayMssg, portMAX_DELAY);
	} else {
		printf(" Error reading CO2 ");
		// measValues.CO2value = 9999;
	}

#ifdef FAST
	if (1) {
		minutePassed = false;
#else
		if (timeinfo.tm_sec < 10) {
#endif

		if (!minutePassed) {
			minutePassed = true;
			tempVal.co2 = CO2Averager.average() / 1000.0;
			tempVal.temperature = temperatureAverager.average() / 1000.0;
			tempVal.hum = RHaverager.average() / 1000.0;
			tempVal.timeStamp = timeStamp++;
			lastVal = tempVal;
			tLog[logTxIdx] = tempVal;

			logTxIdx++;
			if (logTxIdx >= MAXLOGVALUES)
				logTxIdx = 0;
		}
	} else
		minutePassed = false;
	I2Ctimeout--;
}
}

// called from CGI

int getRTMeasValuesScript(char *pBuffer, int count) {
	int len = 0;

	switch (scriptState) {
	case 0:
		scriptState++;
		len += sprintf(pBuffer + len, "%ld,", lastVal.timeStamp);
		len += sprintf(pBuffer + len, "%3.2f,", lastVal.temperature - userSettings.temperatureOffset);
		len += sprintf(pBuffer + len, "%3.2f,", lastVal.hum - userSettings.RHOffset);
		len += sprintf(pBuffer + len, "%ld,", lastVal.co2);
		return len;
		break;
	default:
		break;
	}
	return 0;
}

// these functions only work for one user!

int getNewMeasValuesScript(char *pBuffer, int count) {

	int left, len = 0;
	if (logRxIdx != (logTxIdx)) { // something to send?
		do {
			len += sprintf(pBuffer + len, "%ld,", tLog[logRxIdx].timeStamp);
			len += sprintf(pBuffer + len, "%3.2f,", tLog[logRxIdx].temperature - userSettings.temperatureOffset);
			len += sprintf(pBuffer + len, "%3.2f,", tLog[logRxIdx].hum - userSettings.RHOffset);
			len += sprintf(pBuffer + len, "%ld,", tLog[logRxIdx].co2);
			logRxIdx++;
			if (logRxIdx > MAXLOGVALUES)
				logRxIdx = 0;
			left = count - len;

		} while ((logRxIdx != logTxIdx) && (left > 40));
	}
	return len;
}
// reads all avaiable data from log
// issued as first request.

int getLogScript(char *pBuffer, int count) {
	static int oldTimeStamp = 0;
	static int logsToSend = 0;
	int left, len = 0;
	int n;
	if (scriptState == 0) { // find oldest value in cyclic logbuffer
		logRxIdx = 0;
		oldTimeStamp = 0;
		for (n = 0; n < MAXLOGVALUES; n++) {
			if (tLog[logRxIdx].timeStamp < oldTimeStamp)
				break;
			else {
				oldTimeStamp = tLog[logRxIdx++].timeStamp;
			}
		}
		if (tLog[logRxIdx].timeStamp == 0) { // then log not full
			// not written yet?
			logRxIdx = 0;
			logsToSend = n;
		} else
			logsToSend = MAXLOGVALUES;
		scriptState++;
	}
	if (scriptState == 1) { // send complete buffer
		if (logsToSend) {
			do {
				len += sprintf(pBuffer + len, "%ld,", tLog[logRxIdx].timeStamp);
				len += sprintf(pBuffer + len, "%3.2f,", tLog[logRxIdx].temperature - userSettings.temperatureOffset);
				len += sprintf(pBuffer + len, "%3.2f,", tLog[logRxIdx].hum - userSettings.RHOffset);
				len += sprintf(pBuffer + len, "%ld\n", tLog[logRxIdx].co2);
				logRxIdx++;
				if (logRxIdx >= MAXLOGVALUES)
					logRxIdx = 0;
				left = count - len;
				logsToSend--;

			} while (logsToSend && (left > 40));
		}
	}
	return len;
}

int getSensorNameScript(char *pBuffer, int count) {
	int len = 0;
	switch (scriptState) {
	case 0:
		scriptState++;
		len += sprintf(pBuffer + len, "Actueel,Nieuw\n");
		len += sprintf(pBuffer + len, "%s\n", userSettings.moduleName);
		return len;
		break;
	default:
		break;
	}
	return 0;
}

int getInfoValuesScript(char *pBuffer, int count) {
	int len = 0;
	switch (scriptState) {
	case 0:
		scriptState++;
		len += sprintf(pBuffer + len, "%s\n", "Meting,Actueel,Offset");
		len += sprintf(pBuffer + len, "Temperatuur ,%3.2f,%3.2f\n", lastTemperature - userSettings.temperatureOffset,
					   userSettings.temperatureOffset);															   // send values and offset
		len += sprintf(pBuffer + len, "RH ,%3.1f,%3.2f\n", lastRH - userSettings.RHOffset, userSettings.RHOffset); // send values and offset

		return len;
		break;
	default:
		break;
	}
	return 0;
}

int getCalValuesScript(char *pBuffer, int count) {
	int len = 0;
	switch (scriptState) {
	case 0:
		scriptState++;
		len += sprintf(pBuffer + len, "%s\n", "Meting,Referentie,Stel in,Herstel");
		len += sprintf(pBuffer + len, "%s\n", "Temperatuur\n RH");
		return len;
		break;
	default:
		break;
	}
	return 0;
}

int saveSettingsScript(char *pBuffer, int count) {
	saveSettings();
	return 0;
}

int cancelSettingsScript(char *pBuffer, int count) {
	loadSettings();
	return 0;
}

int enableAutCalScript(char *pBuffer, int count) {
	switch (scriptState) {
	case 0:
		scriptState++;
		strcpy(pBuffer, "OK");
		enableAutCal = true;
		return 3;
		break;
	default:
		break;
	}
	return 0;
}
int disableAutCalScript(char *pBuffer, int count) {
	switch (scriptState) {
	case 0:
		scriptState++;
		strcpy(pBuffer, "OK");
		enableAutCal = false;
		return 3;
		break;
	default:
		break;
	}
	return 0;
}

int clearLogScript(char *pBuffer, int count) {
	if (scriptState == 0) { // find oldest value in cyclic logbuffer
		logRxIdx = 0;
		logTxIdx = 0;
		memset(&tLog, 0, sizeof(tLog));
		strcpy(pBuffer, "OK");
		scriptState++;
		return 3;
	}
	return 0;
}

// "setCal:Temperatuur=23"

void parseCGIWriteData(char *buf, int received) {
	float ref;
	bool save = false;

	if (sscanf(buf, "setCal:Temperatuur=%f", &ref) == 1) {
		userSettings.temperatureOffset = lastVal.temperature - ref;
		ESP_LOGI(TAG, "temperatureOffset set to %f", userSettings.temperatureOffset);
		save = true;
	}
	if (sscanf(buf, "setCal:RH=%f", &ref) == 1) {
		userSettings.RHOffset = lastVal.hum - ref;
		ESP_LOGI(TAG, "RHOffset set to %f", userSettings.RHOffset);
		save = true;
	}

	if (sscanf(buf, "setName:moduleName=%s", userSettings.moduleName) == 1) {
		ESP_LOGI(TAG, "Hostname set to %s", userSettings.moduleName);
		save = true;
	}

	if (save)
		saveSettings();
}
