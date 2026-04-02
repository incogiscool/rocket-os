#pragma once

#include "transmission.h"

#define SENSOR_TELEMETRY_STACK_SIZE ( sizeof( sensor_telemetry_packet_t ) * 10 ) /* Enough for 10 packets in the queue */

void vSensorTelemetryTask(void *pvParameters);