/*
 * sds.h
 *
 *  Created on: 06.04.2017
 *      Author: seven
 */

#ifndef SDS_H_
#define SDS_H_


typedef struct sdsResult_s
{
	float value_pm2_5;
	float value_pm10;
} sdsResult_t;



void task_sds(void *pvParameters);


#endif /* SDS_H_ */
