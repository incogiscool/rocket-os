#pragma once

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "sx1262.h"

#define CALLSIGN_ID 0x88

#define TRANSMISSION_TASK_STACK_SIZE 4096
#define TRANSMISSION_TASK_PRIORITY 1


/* Two types of telemetry packets - if telem fails we still have the gps since its most priority */
typedef enum {
    TELEMETRY_PACKET_TYPE_SENSOR = 0x01,
    TELEMETRY_PACKET_TYPE_GPS = 0x02,
} telemetry_packet_type_t;

/* Telemetry packet structure - packed so that we can decode the struct on ground station */
typedef struct __attribute__((packed)) {
    uint8_t callsign_id;     /* Unique ID for this device - let's keep this const in a config */
    uint16_t seq;
    telemetry_packet_type_t packet_type;    /* Type of the packet*/
    uint32_t timestamp_ms;
    int16_t altitude;      /* Altitude from sensor fusion of barometer and accelerometer */
    uint16_t heading;       /* Magnetometer heading (compass?) */
    int8_t temperature;
    int16_t speed;         /* Speed from accelerometer and barometer changes sensor fusion */
    int16_t acceleration;  /* Acceleration vertically from the accelerometer (change this depending on how the accelerometer is mounted) */
} __attribute__((packed)) sensor_telemetry_packet_t;


typedef struct __attribute__((packed)) {
    uint8_t callsign_id;     /* Unique ID for this device - let's keep this const in a config */
    uint16_t seq;
    telemetry_packet_type_t packet_type;    /* Type of the packet*/
    uint32_t timestamp_ms;
    int32_t latitude;
    int32_t longitude;
    int16_t altitude;
    uint8_t satellites_tracked;
    uint8_t fix_quality;   /* GPS fix quality - High four is fix quality and low four bits satellite count */
} __attribute__((packed)) gps_telemetry_packet_t;


typedef struct {
    QueueHandle_t xSensorQueue;
    QueueHandle_t xGPSQueue;
    sx1262_t *radio;
} transmission_task_params_t;


void vTransmissionTask(void *pvParameters);