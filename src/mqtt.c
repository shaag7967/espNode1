/*
 * mqtt.c
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
#include "mqtt.h"

#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>

#define MQTT_HOST ("phone")
#define MQTT_PORT 1883

#define MQTT_USER "openhabian"
#define MQTT_PASS "*******"

#define TOPIC_STR_TEMPERATURE   "sensor/%s/temperature"
#define TOPIC_STR_HUMIDITY      "sensor/%s/humidity"
#define TOPIC_STR_DUST_PM2_5    "sensor/%s/dustPM2.5"
#define TOPIC_STR_DUST_PM10     "sensor/%s/dustPM10"


//static void topic_received(mqtt_message_data_t *md)
//{
//    int i;
//    mqtt_message_t *message = md->message;
//    printf("Received: ");
//    for (i = 0; i < md->topic->lenstring.len; ++i)
//        printf("%c", md->topic->lenstring.data[i]);
//
//    printf(" = ");
//    for (i = 0; i < (int) message->payloadlen; ++i)
//        printf("%c", ((char *) (message->payload))[i]);
//
//    printf("\r\n");
//}

static const char * get_my_id(void)
{
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *) my_id))
        return NULL;
    for (i = 5; i >= 0; --i)
    {
        x = my_id[i] & 0x0F;
        if (x > 9)
            x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9)
            x += 7;
        my_id[i * 2] = x + '0';
    }

    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

void task_mqtt(void *pvParameters)
{
    int ret = 0;
    struct mqtt_network network;
    mqtt_client_t client = mqtt_client_default;
    char mqtt_client_id[20];
    uint8_t mqtt_buf[100];
    uint8_t mqtt_readbuf[100];
    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;
    char* topicBuffer[50];

    QueueHandle_t *queue = (QueueHandle_t *) pvParameters;

    mqtt_network_new(&network);
    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "ESP-");
    strcat(mqtt_client_id, get_my_id());

    printf("MQTT client ID: %s\r\n", mqtt_client_id);

    while (1)
    {
//        xSemaphoreTake(wifi_alive, portMAX_DELAY);
        printf("%s: started\n\r", __func__);
        printf("%s: (Re)connecting to MQTT server %s ... ", __func__, MQTT_HOST);

        ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT);
        if (ret)
        {
            printf("error: %d\n\r", ret);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }
        printf("done\n\r");

        mqtt_client_new(&client, &network, 5000, mqtt_buf, 100, mqtt_readbuf, 100);

        data.willFlag = 0;
        data.MQTTVersion = 3;
        data.clientID.cstring = mqtt_client_id;
        data.username.cstring = MQTT_USER;
        data.password.cstring = MQTT_PASS;
        data.keepAliveInterval = 10;
        data.cleansession = 1;

        printf("Send MQTT connect ... ");

        ret = mqtt_connect(&client, &data);

        if (ret)
        {
            printf("error: %d\n\r", ret);
            mqtt_network_disconnect(&network);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        printf("done\r\n");

//      mqtt_subscribe(&client, "/esptopic", MQTT_QOS1, topic_received);


        while (1)
        {
            mqttPublishPacket_t packet;

            if (xQueueReceive(*queue, (void *)&packet, 0) == pdTRUE)
            {
                char payloadBuffer[200];
                int len;

                mqtt_message_t message;
                message.dup = 0;
                message.qos = MQTT_QOS0;
                message.retained = 0;

                len = snprintf(payloadBuffer, sizeof(payloadBuffer), "%.01f", packet.value);
                message.payload = payloadBuffer;
                message.payloadlen = len;

                switch(packet.topic)
                {
                case TOPIC_TEMPERATURE:
                    snprintf(topicBuffer, sizeof(topicBuffer), TOPIC_STR_TEMPERATURE, mqtt_client_id);
                    break;
                case TOPIC_HUMIDITY:
                    snprintf(topicBuffer, sizeof(topicBuffer), TOPIC_STR_HUMIDITY, mqtt_client_id);
                    break;
                case TOPIC_DUST_PM2_5:
                    snprintf(topicBuffer, sizeof(topicBuffer), TOPIC_STR_DUST_PM2_5, mqtt_client_id);
                    break;
                case TOPIC_DUST_PM10:
                    snprintf(topicBuffer, sizeof(topicBuffer), TOPIC_STR_DUST_PM10, mqtt_client_id);
                    break;
                default:
                    continue;
                }

                ret = mqtt_publish(&client, topicBuffer, &message);

                if(ret != MQTT_SUCCESS)
                    printf("error while publishing message: %d\n", ret);
            }

            ret = mqtt_yield(&client, 1000);
            if (ret == MQTT_DISCONNECTED)
                break;
        }

        printf("Connection dropped, request restart\n\r");

        mqtt_network_disconnect(&network);

        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}
