#pragma once

#include "transmission.h"

#define GPS_TELEMETRY_STACK_SIZE ( sizeof( gps_telemetry_packet_t ) * 10 ) /* Enough for 10 packets in the queue */

void vGPSTelemetryTask(void *pvParameters);