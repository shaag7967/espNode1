/* Very basic example that just demonstrates we can run at all!
 */
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "sds.h"
#include "temp_humidity.h"
#include "ssid_config.h"
#include "wifi.h"
#include "mqtt.h"


static QueueHandle_t sdsQueue;
static QueueHandle_t temp_humidityQueue;
static QueueHandle_t mqttPublishQueue;


void task_main(void *pvParameters);



void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    wifi_init();

    sdsQueue = xQueueCreate(3, sizeof(sdsResult_t));
    temp_humidityQueue = xQueueCreate(3, sizeof(tempHumidityResult_t));
    mqttPublishQueue = xQueueCreate(3, sizeof(mqttPublishPacket_t));

    xTaskCreate(task_main, "task_main", 256, NULL, 1, NULL);
    xTaskCreate(task_wifi, "task_wifi",  256, NULL, 5, NULL);
    xTaskCreate(task_sds, "task_sds", 256, &sdsQueue, 2, NULL);
    xTaskCreate(task_tempHumidity, "task_th", 256, &temp_humidityQueue, 3, NULL);
    xTaskCreate(task_mqtt, "task_mqtt", 1024, &mqttPublishQueue, 4, NULL);
}

void task_main(void *pvParameters)
{
    printf("task MAIN started\r\n");

    sdsResult_t sdsResult;
    tempHumidityResult_t tempHumidityResult;

    mqttPublishPacket_t packet;


    while(1)
    {
    	if(xQueueReceive(sdsQueue, &sdsResult, 0) == pdTRUE)
    	{
            packet.topic = TOPIC_AIR_QUALITY;
            packet.data.airQuality.pm2_5 = sdsResult.value_pm2_5;
            packet.data.airQuality.pm10 = sdsResult.value_pm10;

            xQueueSend(mqttPublishQueue, &packet, 0);

    		printf("PM2.5: %.01f PM10: %.01f\r\n", sdsResult.value_pm2_5, sdsResult.value_pm10);
    	}

    	if(xQueueReceive(temp_humidityQueue, &tempHumidityResult, 0) == pdTRUE)
    	{
    	    packet.topic = TOPIC_TEMP_HUMIDITY;
    	    packet.data.temp_humidity.humidity = tempHumidityResult.humidity;
    	    packet.data.temp_humidity.temperature = tempHumidityResult.temperature;

    	    xQueueSend(mqttPublishQueue, &packet, 0);

    		printf("Humidity: %.01f Temp.: %.01fC\r\n", tempHumidityResult.humidity, tempHumidityResult.temperature);
    	}

    	vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
