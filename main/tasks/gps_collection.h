#pragma once

#include "transmission.h"

#define GPS_COLLECTION_TASK_STACK_SIZE 4096
#define GPS_TELEMETRY_QUEUE_SIZE 10
#define GPS_COLLECTION_TASK_PRIORITY 1

void vGPSCollectionTask(void *pvParameters);