/*
 * sds.c
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
#include "softwareUart.h"

#include "sds.h"


#define SDS_UART_BAUDRATE     9600
#define SDS_UART_RX_PIN       5
#define SDS_UART_TX_PIN       4
#define SDS_RX_TIMEOUT_MS     250

typedef enum sdsError_e
{
    SDS_ERR_OK = 0,
    SDS_ERR_TIMEOUT,
    SDS_ERR_INVALID_CMD,
    SDS_ERR_INVALID_ANSWER,
    SDS_ERR_INVALID_CHECKSUM,
    SDS_ERR_FAILED
} sdsError_t;



const uint8_t CMD_SDS_SET_STATE_SLEEP[] =         {0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB};
const uint8_t CMD_SDS_SET_STATE_MEASURING[] =         {0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB};
const uint8_t CMD_SDS_SET_REPORTING_MODE_ACTIVE[] = {0xAA, 0xB4, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x01, 0xAB};
const uint8_t CMD_SDS_SET_REPORTING_MODE_QUERY[] =     {0xAA, 0xB4, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x02, 0xAB};
const uint8_t CMD_SDS_SET_WORKING_CONTINUOUS[] =     {0xAA, 0xB4, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x07, 0xAB};

const uint8_t CMD_SDS_GET_REPORTING_MODE[] =     {0xAA, 0xB4, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xAB};
const uint8_t CMD_SDS_GET_DATA[] =                 {0xAA, 0xB4, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x02, 0xAB};
const uint8_t CMD_SDS_GET_STATE[] =             {0xAA, 0xB4, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x04, 0xAB};
const uint8_t CMD_SDS_GET_FW_VERSION[] =         {0xAA, 0xB4, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB};
const uint8_t CMD_SDS_GET_WORKING_PERIOD[] =     {0xAA, 0xB4, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB};

#define SDS_NUM_SAMPLES             60
#define SDS_DELAY_INIT              2000
#define SDS_DELAY_SENSOR_START_TIME 30000 // should be 30 seconds
#define SDS_DELAY_QUERY_PAUSE       2000
#define SDS_DELAY_INTERVAL          210000

typedef enum commProtocolState_e
{
    SDS_HEAD,
    SDS_CMD,
    SDS_DATA_1,
    SDS_DATA_2,
    SDS_DATA_3,
    SDS_DATA_4,
    SDS_DATA_5,
    SDS_DATA_6,
    SDS_CHECK,
    SDS_TAIL
} commProtocolState_t;

typedef struct sdsAnswer_s
{
    sdsError_t error;
    uint8_t command;
    union
    {
        uint8_t raw[6];
        struct
        {
            uint8_t byte1;
            uint8_t byte2;
            uint8_t byte3;
            uint8_t byte4;
            uint8_t deviceId_high;
            uint8_t deviceId_low;
        } normalCmd;
        struct
        {
            uint16_t pm2_5;
            uint16_t pm10;
            uint8_t deviceId_high;
            uint8_t deviceId_low;
        } queryCmd;
    } data;
} sdsAnswer_t;

typedef struct sdsFwVersion_s
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
} sdsFwVersion_t;


static sdsError_t sds_initializeSensor(softwareUart_t* uart);
static sdsError_t sds_querySensorData(softwareUart_t* uart, uint16_t* value_pm2_5, uint16_t* value_pm10);
static void sds_startSensor(softwareUart_t* uart);
static sdsError_t sds_stopSensor(softwareUart_t* uart);

static void sds_write(softwareUart_t* uart, const uint8_t* cmd, uint8_t cmdLen);
static sdsAnswer_t sds_read(softwareUart_t* uart, uint32_t timeoutInMsecs);
static void sds_printAnswer(sdsAnswer_t* answer);


sdsFwVersion_t fwVersion = {0, 0, 0};


void task_sds(void *pvParameters)
{
    sdsError_t err;
    sdsResult_t result = {0.0, 0.0};
    uint16_t tempVal_pm2_5 = 0;
    uint16_t tempVal_pm10 = 0;
    uint8_t sampleCount;


    printf("task SDS started\r\n");

    QueueHandle_t *queue = (QueueHandle_t *)pvParameters;

    softwareUart_t* uart = softwareUart_open(SDS_UART_BAUDRATE, SDS_UART_RX_PIN, SDS_UART_TX_PIN);
    if(uart == NULL)
        return;

    vTaskDelay(SDS_DELAY_INIT / portTICK_PERIOD_MS);

    if((err = sds_initializeSensor(uart)) != SDS_ERR_OK)
        printf("SDS init failed %d\r\n", err);

    while(1)
    {
        sampleCount = 0;
        result.value_pm2_5 = 0.0;
        result.value_pm10 = 0.0;

        sds_startSensor(uart);

        vTaskDelay(SDS_DELAY_SENSOR_START_TIME / portTICK_PERIOD_MS);

        while(sampleCount < SDS_NUM_SAMPLES)
        {
            if(sds_querySensorData(uart, &tempVal_pm2_5, &tempVal_pm10) == SDS_ERR_OK)
            {
                result.value_pm2_5 += tempVal_pm2_5;
                result.value_pm10 += tempVal_pm10;
                sampleCount++;
            }
            else
                printf("sds_querySensorData failed\r\n");

            vTaskDelay(SDS_DELAY_QUERY_PAUSE / portTICK_PERIOD_MS);
        }

        result.value_pm2_5 /= (10.0 * SDS_NUM_SAMPLES);
        result.value_pm10 /= (10.0 * SDS_NUM_SAMPLES);

        xQueueSend(*queue, &result, 0);

        if(sds_stopSensor(uart) != SDS_ERR_OK)
            printf("sds_stopSensor failed\r\n");

        vTaskDelay(SDS_DELAY_INTERVAL / portTICK_PERIOD_MS);
    }
}


static void sds_write(softwareUart_t* uart, const uint8_t* cmd, uint8_t cmdLen)
{
    softwareUart_dumpRxBuffer(uart);

    for(int i = 0; i < cmdLen; i++)
        softwareUart_put(uart, cmd[i]);
}

static sdsAnswer_t sds_read(softwareUart_t* uart, uint32_t timeoutInMsecs)
{
    uint8_t reading = 1;
    uint8_t rxByte;
    sdsAnswer_t answer;
    uint16_t checksum = 0;
    commProtocolState_t protState = SDS_HEAD;
    uint32_t endTime = sdk_system_get_time() + (timeoutInMsecs * 1000);

    memset(&answer, 0, sizeof(sdsAnswer_t));
    answer.error = SDS_ERR_OK;

    while(reading)
    {
        if(sdk_system_get_time() > endTime)
        {
            answer.error = SDS_ERR_TIMEOUT;
            break;
        }

        if(softwareUart_bytesAvailable(uart) == 0)
            continue;

        rxByte = softwareUart_read(uart);

        switch(protState)
        {
        case SDS_HEAD:
            if(rxByte == 0xAA)
                protState++;
            break;
        case SDS_CMD:
            answer.command = rxByte;

            if(answer.command == 0xC0 || answer.command == 0xC5)
                protState++;
            else
            {
                answer.error = SDS_ERR_INVALID_CMD;
                reading = 0;
            }
            break;
        case SDS_DATA_1:
            answer.data.raw[0] = rxByte;
            checksum = rxByte;
            protState++;
            break;
        case SDS_DATA_2:
            answer.data.raw[1] = rxByte;
            checksum += rxByte;
            protState++;
            break;
        case SDS_DATA_3:
            answer.data.raw[2] = rxByte;
            checksum += rxByte;
            protState++;
            break;
        case SDS_DATA_4:
            answer.data.raw[3] = rxByte;
            checksum += rxByte;
            protState++;
            break;
        case SDS_DATA_5:
            answer.data.raw[4] = rxByte;
            checksum += rxByte;
            protState++;
            break;
        case SDS_DATA_6:
            answer.data.raw[5] = rxByte;
            checksum += rxByte;
            protState++;
            break;
        case SDS_CHECK:
            if(rxByte == (uint8_t)(checksum))
                protState++;
            else
            {
                answer.error = SDS_ERR_INVALID_CHECKSUM;
                reading = 0;
            }
            break;
        case SDS_TAIL:
            if(rxByte != 0xAB)
                answer.error = SDS_ERR_INVALID_ANSWER;
            reading = 0;
            break;
        }
    }

    return answer;
}

static sdsError_t sds_initializeSensor(softwareUart_t* uart)
{
    sdsAnswer_t answer;

    // start sensor
    sds_startSensor(uart);
    vTaskDelay(500 / portTICK_PERIOD_MS); // wait till all potential rx bytes are received

    // get firmware version and id
    sds_write(uart, CMD_SDS_GET_FW_VERSION, sizeof(CMD_SDS_GET_FW_VERSION));
    answer = sds_read(uart, SDS_RX_TIMEOUT_MS);

    if(answer.error != SDS_ERR_OK)
    {
        printf("CMD_SDS_GET_FW_VERSION failed\r\n");
        return answer.error;
    }

    fwVersion.year = answer.data.normalCmd.byte2;
    fwVersion.month = answer.data.normalCmd.byte3;
    fwVersion.day = answer.data.normalCmd.byte4;

//    sds_printAnswer(&answer);

    // get reporting mode
    sds_write(uart, CMD_SDS_GET_REPORTING_MODE, sizeof(CMD_SDS_GET_REPORTING_MODE));
    answer = sds_read(uart, SDS_RX_TIMEOUT_MS);

    if(answer.error != SDS_ERR_OK)
    {
        printf("CMD_SDS_GET_REPORTING_MODE failed\r\n");
        return answer.error;
    }

    if(answer.data.normalCmd.byte3 == 0x00) // sensor in active mode
    {
        // stop reporting of data
        sds_write(uart, CMD_SDS_SET_REPORTING_MODE_QUERY, sizeof(CMD_SDS_SET_REPORTING_MODE_QUERY));
        answer = sds_read(uart, SDS_RX_TIMEOUT_MS);

        if(answer.error != SDS_ERR_OK)
        {
            printf("CMD_SDS_SET_REPORTING_MODE_QUERY failed\r\n");
            return answer.error;
        }
    }

    // get working period
    sds_write(uart, CMD_SDS_GET_WORKING_PERIOD, sizeof(CMD_SDS_GET_WORKING_PERIOD));
    answer = sds_read(uart, SDS_RX_TIMEOUT_MS);

    if(answer.error != SDS_ERR_OK)
    {
        printf("CMD_SDS_GET_WORKING_PERIOD failed\r\n");
        return answer.error;
    }

    if(answer.data.normalCmd.byte3 != 0x00) // not continuous
    {
        // set working mode to continuous
        sds_write(uart, CMD_SDS_SET_WORKING_CONTINUOUS, sizeof(CMD_SDS_SET_WORKING_CONTINUOUS));
        answer = sds_read(uart, SDS_RX_TIMEOUT_MS);

        if(answer.error != SDS_ERR_OK)
        {
            printf("CMD_SDS_SET_WORKING_CONTINUOUS failed\r\n");
            return answer.error;
        }
    }

    return SDS_ERR_OK;
}

static void sds_startSensor(softwareUart_t* uart)
{
    // When sensor is in sleep mode, every command sent to it puts the sensor back in
    // measuring mode. So a special command for wakeing it up would not be neccessary.
    // Normaly the sensor processes a command and ~100ms later it sends an answer back.
    // In sleep mode the sensor sends an answer back immediatelly after receiving the
    // first byte (sensor doesn't wait for the last command byte, but it has to be a valid
    // and complete command...).
    // Because we are using softwareserial we cannot send and receive at the same time
    // (the rx irq would destroy the tx timing). This means we cannot read and check
    // the answer. To prevent an rx irq between the tx bytes we disable the rx interrupt.
    //
    // In case the sensor is already measuring we get a normal answer after ~100ms. These
    // bytes will be unused and dumped on next write (see function sds_write).

    softwareUart_disableRx(uart);

    sds_write(uart, CMD_SDS_SET_STATE_MEASURING, sizeof(CMD_SDS_SET_STATE_MEASURING));

    softwareUart_enableRx(uart);
}

static sdsError_t sds_stopSensor(softwareUart_t* uart)
{
    sdsAnswer_t answer;

    // if sensor is already in sleep mode, you will wake sensor up
    // and not get a proper answer (see function sds_startSensor)

    sds_write(uart, CMD_SDS_SET_STATE_SLEEP, sizeof(CMD_SDS_SET_STATE_SLEEP));
    answer = sds_read(uart, SDS_RX_TIMEOUT_MS);

    if(answer.error != SDS_ERR_OK)
    {
        printf("error: %d\r\n", answer.error);
        return answer.error;
    }

    return SDS_ERR_OK;
}

static sdsError_t sds_querySensorData(softwareUart_t* uart, uint16_t* value_pm2_5, uint16_t* value_pm10)
{
    sdsAnswer_t answer;

    sds_write(uart, CMD_SDS_GET_DATA, sizeof(CMD_SDS_GET_DATA));
    answer = sds_read(uart, SDS_RX_TIMEOUT_MS*2);

    if(answer.error != SDS_ERR_OK)
    {
        printf("error: %d\r\n", answer.error);
        return answer.error;
    }

    *value_pm2_5 = answer.data.queryCmd.pm2_5;
    *value_pm10 = answer.data.queryCmd.pm10;

    return SDS_ERR_OK;
}

static void sds_printAnswer(sdsAnswer_t* answer)
{
    printf("\r\ncommand: %02X\r\n", answer->command);
    printf("error: %d\r\n", answer->error);
    printf("raw: %02X %02X %02X %02X %02X %02X\r\n",
            answer->data.raw[0],
            answer->data.raw[1],
            answer->data.raw[2],
            answer->data.raw[3],
            answer->data.raw[4],
            answer->data.raw[5]);

    if(answer->error == SDS_ERR_OK)
    {
        printf("Device-ID: 0x%02X%02X\r\n", answer->data.normalCmd.deviceId_high, answer->data.normalCmd.deviceId_low);

        switch(answer->data.normalCmd.byte1)
        {
        case 2:
            printf("Reporting mode: %s\r\n", (answer->data.normalCmd.byte3)?"query":"active");
            break;
        case 6:
            printf("State: %s\r\n", (answer->data.normalCmd.byte3)?"work":"sleep");
            break;
        case 7:
            printf("FW-Version: 20%02d-%02d-%02d\r\n",
                    answer->data.normalCmd.byte2, answer->data.normalCmd.byte3, answer->data.normalCmd.byte4);
            break;
        case 8:
            printf("Period: %d\r\n", answer->data.normalCmd.byte3); // 0 = continuous
            break;
        }
    }
}
