/*
 * temp_humidity.c
 *
 *  Created on: 06.04.2017
 *      Author: seven
 */


#include "esp/gpio.h"
#include "esp/uart.h"
#include "espressif/esp_common.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "temp_humidity.h"
#include "dht/dht.h"


#define DHT_PIN              13
#define DHT_SENSOR_TYPE      DHT_TYPE_DHT22


#define DHT_NUM_SAMPLES             10
#define DHT_DELAY_QUERY_PAUSE       2100
#define DHT_DELAY_INTERVAL			110000




void task_tempHumidity(void *pvParameters)
{
	tempHumidityResult_t result;
    int16_t temperature = 0;
    int16_t humidity = 0;
	uint8_t sampleCount;

    printf("task TH started\r\n");

    QueueHandle_t *queue = (QueueHandle_t *)pvParameters;


    gpio_set_pullup(DHT_PIN, false, false);

    while(1)
    {
    	result.humidity = 0.0;
    	result.temperature = 0.0;
    	sampleCount = 0;

    	while(sampleCount < DHT_NUM_SAMPLES)
    	{
    		if(dht_read_data(DHT_SENSOR_TYPE, DHT_PIN, &humidity, &temperature))
    		{
    	    	result.humidity += humidity;
    	    	result.temperature += temperature;
    	    	sampleCount++;
    		}

    		vTaskDelay(DHT_DELAY_QUERY_PAUSE / portTICK_PERIOD_MS);
    	}

    	result.humidity /= (10.0 * DHT_NUM_SAMPLES);
    	result.temperature /= (10.0 * DHT_NUM_SAMPLES);

    	xQueueSend(*queue, &result, 0);

        vTaskDelay(DHT_DELAY_INTERVAL / portTICK_PERIOD_MS);
    }
}
