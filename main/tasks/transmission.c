#include "transmission.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"

static const char *TAG = "TRANSMISSION";

static esp_err_t transmit_packet(sx1262_t *radio, void *packet, uint8_t len) {
    /* SX1262 packet length must be re-set per-packet because sensor and GPS
       payloads differ in size. preamble=12, explicit header, CRC on, IQ normal —
       must match ground-station RadioLib config. */
    esp_err_t res = sx1262_set_packet_params(radio, 12, 0x00, len, 0x01, 0x00);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "set_packet_params failed: %d", res);
        return res;
    }

    res = sx1262_transmit(radio, (uint8_t *)packet, len);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "sx1262_transmit failed: %d", res);
    }
    return res;
}

void vTransmissionTask(void *pvParameters) {

    transmission_task_params_t *params = (transmission_task_params_t *)pvParameters;
    QueueHandle_t xSensorQueue = params->xSensorQueue;
    QueueHandle_t xGPSQueue = params->xGPSQueue;
    sx1262_t *radio = params->radio;

    for (;;) {

            /* Check for sensor telemetry packets */
            sensor_telemetry_packet_t sensor_packet;
            if (xQueueReceive(xSensorQueue, &sensor_packet, 0) == pdPASS) {
                ESP_LOGI(TAG, "TX sensor: seq=%d alt=%d temp=%d accel_z=%d",
                        sensor_packet.seq, sensor_packet.altitude, sensor_packet.temperature, sensor_packet.acceleration);
                transmit_packet(radio, &sensor_packet, sizeof(sensor_packet));
            }

            /* Check for GPS telemetry packets */
            gps_telemetry_packet_t gps_packet;
            if (xQueueReceive(xGPSQueue, &gps_packet, 0) == pdPASS) {
                ESP_LOGI(TAG, "TX gps: seq=%d lat=%ld lon=%ld alt=%d",
                        gps_packet.seq, gps_packet.latitude, gps_packet.longitude, gps_packet.altitude);
                transmit_packet(radio, &gps_packet, sizeof(gps_packet));
            }

            vTaskDelay(pdMS_TO_TICKS(50));
    }


    vTaskDelete(NULL);

};
