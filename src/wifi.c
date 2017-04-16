/*
 * wifi.c
 *
 *  Created on: 06.04.2017
 *      Author: seven
 */

#include "espressif/esp_common.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "wifi.h"

#include <ssid_config.h>

#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>

struct sdk_station_config wifiConfig = { WIFI_SSID, WIFI_PASS };

void wifi_init(void)
{
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&wifiConfig);
}

void task_wifi(void *pvParameters)
{
    uint8_t lastStatus = 255;
    uint8_t status = STATION_IDLE;

    while (1)
    {
        status = sdk_wifi_station_get_connect_status();

        if (lastStatus != status)
        {
            switch (status)
            {
            case STATION_IDLE:
                printf("Wifi: idle\r\n");
                break;
            case STATION_CONNECTING:
                printf("Wifi: connecting\r\n");
                break;
            case STATION_WRONG_PASSWORD:
                printf("Wifi: wrong password\r\n");
                break;
            case STATION_NO_AP_FOUND:
                printf("Wifi: access point not found\r\n");
                break;
            case STATION_CONNECT_FAIL:
                printf("Wifi: failed to connect\r\n");
                break;
            case STATION_GOT_IP:
                printf("Wifi: connected\r\n");
                // todo send signal to maintask
                break;
            default:
                break;
            }

            lastStatus = status;
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
