/*
 * sensor.c
 *
 *  Created on: Jan 1, 2018
 *      Author: dig
 */

//#define SIMULATE
//#define FAST
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "settings.h"
#include "driver/timer.h"
#include "driver/i2c.h"

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <string.h>

#include "sensorTask.h"
#include "Dht22.h"
#include "udpClient.h"

#define TIMER_DIVIDER         16  //  Hardware timer clock divider 80Mhz
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMERIDX			  0  // timer 0 group 0 used

#define SAMPLEPERIOD					   10 // seconds

Averager temperatureHourBuffer(60/SAMPLEPERIOD);  // todo make minute
Averager humidityHourBuffer(60/SAMPLEPERIOD);
Averager CO2HourBuffer(60/SAMPLEPERIOD);
//Averager temperatureDayBuffer(SAMPLESPERDAY);
//Averager humidityDayBuffer(SAMPLESPERDAY);
//Averager CO2DayBuffer(SAMPLESPERDAY);

time_t now;
struct tm timeinfo;



#define I2C_EXAMPLE_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define CO2_SENSOR_ADDR    		           0x68             /*!< slave address for BH1750 sensor */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                           (i2c_ack_type_t) 0x0             /*!< I2C ack value */
#define NACK_VAL                          (i2c_ack_type_t) 0x1              /*!< I2C nack value */

#define DHT_GPIO							GPIO_NUM_25

static esp_err_t readC02(i2c_port_t i2c_num, int *pValue);
static void i2c_master_init();

measValues_t measValues;


#define LOGINTERVAL			 	60  // seconds
#define MEASUREINTERVAL			1	//
#define MAXLOGVALUES			(24*60)

#define UDPTXPORT				5001

typedef struct {
	int32_t timeStamp;
	float temperature;
	float hum;
	int32_t co2;
	float refTemperature;
} log_t;

static log_t tLog[ MAXLOGVALUES];
static log_t lastVal;
static int timeStamp =0;

static int logTxIdx;
static int logRxIdx;

extern int scriptState;

//
//
//void initBuffers( void) {
//	initBuffer (&temperatureHourBuffer,SAMPLESPERHOUR);
//	initBuffer (&humidityHourBuffer,SAMPLESPERHOUR);
//	initBuffer (&CO2HourBuffer,SAMPLESPERHOUR);
//
//	initBuffer (&temperatureDayBuffer,SAMPLESPERDAY);
//	initBuffer (&humidityDayBuffer,SAMPLESPERDAY);
//	initBuffer (&CO2DayBuffer,SAMPLESPERDAY);
//}

static esp_err_t readC02(i2c_port_t i2c_num, int *pValue) {
	esp_err_t ret = -1;
	bool ok = false;
	i2c_cmd_handle_t cmd;
	uint8_t buffer[4];
	int value;
//
//	i2c_config_t conf;
//	conf.mode = I2C_MODE_MASTER;
//	conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
//	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//	conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
//	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//	conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
//	conf.clk_flags = 0;
//	i2c_param_config(I2C_MASTER_NUM, &conf);
////	ret= i2c_set_pin(I2C_MASTER_NUM, I2C_EXAMPLE_MASTER_SDA_IO,I2C_EXAMPLE_MASTER_SCL_IO,true,true, I2C_MODE_MASTER); // ???

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, 0, ACK_CHECK_DIS);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	vTaskDelay(10 / portTICK_PERIOD_MS);

	//for (int retry = 0; (retry < 3) && (ok == false); retry++) {
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

	uint8_t sum = 0; //Checksum Byte
	sum = buffer[0] + buffer[1] + buffer[2]; //Byte addition utilizes overflow

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
	i2c_port_t  i2c_master_port = I2C_MASTER_NUM;
	i2c_config_t conf;
	memset( &conf, 0, sizeof( conf));
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;

	i2c_param_config(i2c_master_port, &conf);
	i2c_driver_install(i2c_master_port, conf.mode,
	I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
	I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}

void sensorTask(void *pvParameter) {
	uint32_t xLastWakeTime;
#ifdef SIMULATE
	const portTickType xPeriod = (500 / portTICK_PERIOD_MS);
#else
	const uint32_t xPeriod = (5000 / portTICK_PERIOD_MS);
#endif
	char str[20];
	log_t tempVal;
	Dht22 dht22(DHT_GPIO);
	Dht22::dht22Data_t dhtData;

	float temperature = 0, humidity = 0;
	int CO2value = 0;
	int tempCO2value = 0;
	esp_err_t err = 0;
	int I2Ctimeout = 10;
//	int dayMinutesPrescaler = DAYSAMPLESINTERVAL;

	bool minutePassed = false;
	i2c_master_init();
//	int n = 1;

	xLastWakeTime = xTaskGetTickCount();

//	while (1)
//		vTaskDelayUntil(&xLastWakeTime, xPeriod);

#ifdef SIMULATE
	for ( n = 0; n < SAMPLESPERDAY ; n ++ ) {
		temperature = 150 / 2 * (1 + sin(n / 10));
		humidity = 150 / 2 * (1 + sin(n / 3));
		CO2value = 150 / 2 * (1 + sin(n / 3));
		temperatureDayBuffer.write( (uint16_t) temperature);
		humidityDayBuffer.write( (uint16_t) humidity);
		CO2DayBuffer.write( (uint16_t) CO2value);
	}
	n = 0;
#endif

	while (1) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);

#ifdef SIMULATE
		err = ESP_OK;
		temperature = 150 / 2 * (1 + sin(n / 10));
		humidity = 150 / 2 * (1 + sin(n / 3));
		CO2value = 150 / 2 * (1 + sin(n / 3));
		n++;
#else

		err = dht22.getData(&dhtData);
		if (err == ESP_OK) {
			temperature = dhtData.temperature;
			humidity = dhtData.humidity;
		}
#endif
		humidityHourBuffer.write((uint16_t) (100.0 * humidity));  // use last values for graph
		temperatureHourBuffer.write((uint16_t) (100.0 * temperature));

		if (err == ESP_OK) {
			I2Ctimeout = 10;
			measValues.temperature = temperature;
			measValues.humidity = humidity;
			printf("\n temp: %2.2f , hum: %2.1f ", temperature, humidity);
		} else {
			measValues.temperature = 9999;
			measValues.humidity = 9999;
			printf("\n error reading DHT: %x ", err);
		}
#ifndef SIMULATE
		err = readC02(I2C_NUM_1, &tempCO2value);
		if ( err == ESP_OK)
			CO2value = tempCO2value;
#endif
		if  ( CO2value > 300)
			CO2HourBuffer.write(CO2value);

		if (err == ESP_OK) {
			I2Ctimeout = 10;
			printf(" CO2: %d ppm", CO2value);
			measValues.CO2value = (float) CO2value;
			sprintf( str, "2:%d",CO2value);
			UDPsendMssg(UDPTXPORT, str , strlen(str));
		} else {
			printf(" Error reading CO2 ");
			measValues.CO2value = 9999;
		}
		measValues.measNr += 1;
		time(&now);
		localtime_r(&now, &timeinfo);
#ifdef FAST
		if(1){
			minutePassed = false;
			dayMinutesPrescaler = 1;
#else
		if (timeinfo.tm_sec < 10) {
#endif

			if (!minutePassed) {
				minutePassed = true;
		//		dayMinutesPrescaler--;

				tempVal.co2  = CO2HourBuffer.average();
				tempVal.temperature  = temperatureHourBuffer.average()/100.0;
				tempVal.hum  = humidityHourBuffer.average()/100.0;
				tempVal.timeStamp = timeStamp++;

				lastVal = tempVal;

				if (abs ( tempVal.temperature - lastVal.temperature) >= 4.0) { // spike error
					tempVal.temperature = lastVal.temperature;
					tempVal.hum = lastVal.hum;
				}

				tLog[logTxIdx] = tempVal;

				logTxIdx++;
				if (logTxIdx >= MAXLOGVALUES)
					logTxIdx = 0;


//				if (dayMinutesPrescaler == 0) {
//					dayMinutesPrescaler = DAYSAMPLESINTERVAL;
//					//			float av = 12.34;
//					float av = temperatureHourBuffer.average();
//					temperatureDayBuffer.write((uint16_t) av);
//					//			printf( "\n ** av temp: %f", av/ 100);
//					av = humidityHourBuffer.average();
//					humidityDayBuffer.write((uint16_t) av);
//					//		printf( " av hum: %f", av/ 100);
//					av = CO2HourBuffer.average();
//					CO2DayBuffer.write((uint16_t) av);
//
//					heap_caps_check_integrity_all(true);
//
//				}
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
		len += sprintf(pBuffer + len, "%3.2f,", lastVal.temperature);
		len += sprintf(pBuffer + len, "%3.2f,", lastVal.hum);
		len += sprintf(pBuffer + len, "%ld,", lastVal.co2);
		len += sprintf(pBuffer + len, "%3.3f\n", lastVal.refTemperature);
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
	if (logRxIdx != (logTxIdx)) {  // something to send?
		do {
			len += sprintf(pBuffer + len, "%ld,", tLog[logRxIdx].timeStamp);
			len += sprintf(pBuffer + len, "%3.2f,", tLog[logRxIdx].temperature);
			len += sprintf(pBuffer + len, "%3.2f,", tLog[logRxIdx].hum);
			len += sprintf(pBuffer + len, "%ld,", tLog[logRxIdx].co2);
			len += sprintf(pBuffer + len, "%3.3f\n", tLog[logRxIdx].refTemperature);
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
				len += sprintf(pBuffer + len, "%3.2f,", tLog[logRxIdx].temperature);
				len += sprintf(pBuffer + len, "%3.2f,", tLog[logRxIdx].hum);
				len += sprintf(pBuffer + len, "%ld,", tLog[logRxIdx].co2);
				len += sprintf(pBuffer + len, "%3.3f\n", tLog[logRxIdx].refTemperature);
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


