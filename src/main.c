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


struct sdk_station_config wifiConfig = {
	.ssid = WIFI_SSID,
	.password = WIFI_PASS,
};

static QueueHandle_t sdsQueue;
static QueueHandle_t temp_humidityQueue;


void task_main(void *pvParameters);



void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    sdsQueue = xQueueCreate(3, sizeof(sdsResult_t));
    temp_humidityQueue = xQueueCreate(3, sizeof(tempHumidityResult_t));

    xTaskCreate(task_sds, "task_sds", 256, &sdsQueue, 2, NULL);
    xTaskCreate(task_tempHumidity, "task_th", 256, &temp_humidityQueue, 2, NULL);
    xTaskCreate(task_main, "task_main", 256, NULL, 1, NULL);
}

void task_main(void *pvParameters)
{
    printf("task MAIN started\r\n");


    sdsResult_t sdsResult;
    tempHumidityResult_t tempHumidityResult;

    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&wifiConfig);


    while(1)
    {
    	if(xQueueReceive(sdsQueue, &sdsResult, 0) == pdTRUE)
    	{
    		printf("PM2.5: %.01f PM10: %.01f\r\n", sdsResult.value_pm2_5, sdsResult.value_pm10);
    	}

    	if(xQueueReceive(temp_humidityQueue, &tempHumidityResult, 0) == pdTRUE)
    	{
    		printf("Humidity: %.01f Temp.: %.01fC\r\n", tempHumidityResult.humidity, tempHumidityResult.temperature);
    	}

    	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
