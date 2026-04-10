#include "gps_collection.h"
#include "transmission.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "gps/neo6m.h"
#include "esp_log.h"

static const char *TAG = "GPS_COLLECTION";

void vGPSCollectionTask(void *pvParameters) {
    QueueHandle_t xGPSQueue = (QueueHandle_t)pvParameters;

    /* NEO-6M Setup */
    neo6m_init();
    ESP_LOGI(TAG, "NEO-6M init OK");

    uint16_t seq = 0;

    for (;;) {
        nmea_gps_data_t gps = neo6m_read_parsed();

        if (isnan(gps.latitude) || isnan(gps.longitude)) {
            ESP_LOGW(TAG, "Invalid GPS data received, skipping transmission");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        gps_telemetry_packet_t packet = {
            .callsign_id = CALLSIGN_ID,
            .seq = seq++,
            .packet_type = TELEMETRY_PACKET_TYPE_GPS,
            .timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
            .altitude = (int16_t)gps.altitude,
            .latitude = (int32_t)(gps.latitude * 1000000),   /* Degrees scaled to 6 decimal places */
            .longitude = (int32_t)(gps.longitude * 1000000), /* Degrees scaled to 6 decimal places */
            .satellites_tracked = gps.satellites_tracked,
            .fix_quality = gps.fix_quality,
        };

        xQueueSendToBack(xGPSQueue, &packet, pdMS_TO_TICKS(250));

        vTaskDelay(pdMS_TO_TICKS(1000)); /* GPS updates at 1Hz */
    };

    vTaskDelete(NULL);
}
