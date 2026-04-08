#pragma once

#include "transmission.h"

/* Note we may need to increase this */
#define SENSOR_COLLECTION_TASK_STACK_SIZE 4096 /* Words, not bytes. 1 word is 4 bits in 32 bit system like esp32. */
#define SENSOR_TELEMETRY_QUEUE_SIZE 10
#define SENSOR_COLLECTION_TASK_PRIORITY 1

void vSensorCollectionTask(void *pvParameters);