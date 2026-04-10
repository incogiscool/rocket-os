#include "transmission.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"

static const char *TAG = "TRANSMISSION";

void vTransmissionTask(void *pvParameters) {

    transmission_task_params_t *params = (transmission_task_params_t *)pvParameters;
    QueueHandle_t xSensorQueue = params->xSensorQueue;
    QueueHandle_t xGPSQueue = params->xGPSQueue;

    for (;;) {

            /* Check for sensor telemetry packets */
            sensor_telemetry_packet_t sensor_packet;
            if (xQueueReceive(xSensorQueue, &sensor_packet, 0) == pdPASS) {
                ESP_LOGI(TAG, "Received sensor packet: seq=%d alt=%d temp=%d accel_z=%d",
                        sensor_packet.seq, sensor_packet.altitude, sensor_packet.temperature, sensor_packet.acceleration);
            }

            /* Check for GPS telemetry packets */
            gps_telemetry_packet_t gps_packet;
            if (xQueueReceive(xGPSQueue, &gps_packet, 0) == pdPASS) {
                ESP_LOGI(TAG, "Received GPS packet: seq=%d lat=%ld lon=%ld alt=%d",
                        gps_packet.seq, gps_packet.latitude, gps_packet.longitude, gps_packet.altitude);
            }

            vTaskDelay(pdMS_TO_TICKS(50));
    }


    vTaskDelete(NULL);

};