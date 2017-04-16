/*
 * mqtt.h
 *
 *  Created on: 06.04.2017
 *      Author: seven
 */

#ifndef MQTT_H_
#define MQTT_H_


#define PUB_MSG_LEN     16



typedef enum mqttTopic_e
{
    TOPIC_TEMP_HUMIDITY,
    TOPIC_AIR_QUALITY
} mqttTopic_e;

typedef struct mqttPublishPacket_s
{
    mqttTopic_e topic;
    union
    {
        struct
        {
            float temperature;
            float humidity;
        } temp_humidity;
        struct
        {
            float pm2_5;
            float pm10;
        } airQuality;
    } data;
} mqttPublishPacket_t;



void task_mqtt(void *pvParameters);



#endif /* MQTT_H_ */
