/*
 * temp_humidity.h
 *
 *  Created on: 06.04.2017
 *      Author: seven
 */

#ifndef TEMP_HUMIDITY_H_
#define TEMP_HUMIDITY_H_


typedef struct tempHumidityResult_s
{
	float temperature;
	float humidity;
} tempHumidityResult_t;



void task_tempHumidity(void *pvParameters);


#endif /* TEMP_HUMIDITY_H_ */
