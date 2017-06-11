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
    TOPIC_TEMPERATURE,
    TOPIC_HUMIDITY,
    TOPIC_DUST_PM2_5,
    TOPIC_DUST_PM10
} mqttTopic_e;

typedef struct mqttPublishPacket_s
{
    mqttTopic_e topic;
    float value;
} mqttPublishPacket_t;



void task_mqtt(void *pvParameters);



#endif /* MQTT_H_ */
