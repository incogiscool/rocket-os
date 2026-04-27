#include "sx1262.h"
#include "itg3205.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "parse/nmea.h"
#include "gps/neo6m.h"
#include <string.h>

#define BUILT_IN_LED_PIN 2
#define RF_FREQ_HZ 915000000 /* 915Mhz */


static const char *TAG = "MAIN";

// void app_main(void) {
//     sx1262_t radio;

//     esp_err_t ret = sx1262_init(&radio);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Radio init failed");
//         return;
//     }

//     // Set to standby
//     sx1262_set_standby(&radio, SX1262_STDBY_RC);

//     // Set to LoRa mode
//     sx1262_set_packet_type(&radio, SX1262_PACKET_TYPE_LORA);

//     // Set frequency to 915 MHz
//     sx1262_set_rf_frequency(&radio, RF_FREQ_HZ);

//     ESP_LOGI(TAG, "Radio configured!");

//     gpio_reset_pin(BUILT_IN_LED_PIN);
//     /* Set the GPIO as a push/pull output */
//     gpio_set_direction(BUILT_IN_LED_PIN, GPIO_MODE_OUTPUT);
    

//     gpio_set_level(BUILT_IN_LED_PIN, 1);
// }




// #include "sx1262.h"
// #include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// static const char *TAG = "RX";

// void app_main(void) {
//     sx1262_t radio;

//     esp_err_t ret = sx1262_init(&radio);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Radio init failed");
//         return;
//     }

//     sx1262_set_standby(&radio, SX1262_STDBY_RC);
//     sx1262_set_packet_type(&radio, SX1262_PACKET_TYPE_LORA);
//     sx1262_set_rf_frequency(&radio, RF_FREQ_HZ);
//     sx1262_set_mod_params_lora(&radio, 7, 0x04, 0x01, 0x00);
//     sx1262_set_packet_params(&radio, 12, 0x00, 64, 0x01, 0x00);
//     sx1262_set_dio_irq_params(&radio);

//     ESP_LOGI(TAG, "Receiver ready!");

//     while (1) {
//         uint8_t data[256];
//         uint8_t len = 0;

//         esp_err_t res = sx1262_receive(&radio, data, &len);

//         if (res == ESP_OK) {
//             data[len] = '\0'; /* Null terminate so we can print as string */
//             ESP_LOGI(TAG, "Received: %s", (char *)data);
//         } else if (res == ESP_ERR_TIMEOUT) {
//             ESP_LOGW(TAG, "No packet, listening again...");
//         } else {
//             ESP_LOGE(TAG, "RX error: %d", res);
//         }
//     }
// }

// #include "sx1262.h"
// #include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// static const char *TAG = "TX";

// void app_main(void) {
//     sx1262_t radio;

//     esp_err_t ret = sx1262_init(&radio);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Radio init failed");
//         return;
//     }



//     char msg[] = "Hello!";
//     int len = sizeof(msg) / sizeof(char);
    

//     /* TODO: Add RSSI to response */
//     sx1262_set_standby(&radio, SX1262_STDBY_RC);
//     sx1262_set_packet_type(&radio, SX1262_PACKET_TYPE_LORA);
//     sx1262_set_rf_frequency(&radio, RF_FREQ_HZ);
//     sx1262_set_mod_params_lora(&radio, 7, 0x04, 0x01, 0x00);
//     sx1262_set_packet_params(&radio, 12, 0x00, sizeof(msg), 0x01, 0x00);
//     sx1262_set_dio_irq_params(&radio);

//     ESP_LOGI(TAG, "Transmitter ready!");


    


//     while (1) {

//         ESP_LOGI(TAG, "Sending: %s", msg);

//         esp_err_t res = sx1262_transmit(&radio, (uint8_t *)msg, len);
//         if (res == ESP_OK) {
//             ESP_LOGI(TAG, "Sent!");
//         } else {
//             ESP_LOGE(TAG, "TX failed: %d", res);
//         }

//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }


// #include "sx1262.h"
// #include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// static const char *TAG = "TX";




// void app_main(void) {
//     uint8_t data[NEO6M_UART_BUF_SIZE];

//     neo6m_init();

//     while (1) {
//         int len = neo6m_read(data);

//         if (len > 0) {
//             ESP_LOGI(TAG, "Length: %d", len);

//             // Split into individual NMEA sentences by newline
//             char *line = strtok((char *)data, "\r\n");
            
//             while (line != NULL) {
//                 if (line[0] == '$') {
//                     parseNmea(line);
//                 }
//                 line = strtok(NULL, "\r\n");
//             }
//         }
//     }
// }



#include "bme280_port.h"
#include "adxl345_port.h"
#include "gpio/i2c.h"
#include "lib/BME280_SensorAPI/bme280.h"
#include "tasks/transmission.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/collection.h"
#include "tasks/gps_collection.h"
#include "tasks/gps_collection.h"
#include "tasks/transmission.h"

// void app_main(void) {
//     /* I2C + Sensor Init */
//     i2c_master_bus_handle_t bus_handle;
//     i2c_master_dev_handle_t bme280_dev_handle;
//     i2c_master_dev_handle_t adxl345_dev_handle;

//     ESP_ERROR_CHECK(i2c_master_init(&bus_handle));

//     /* BME280 Init */
//     struct bme280_settings settings;
//     struct bme280_dev device;
//     bme280_intf_ctx_t intf_ctx;
//     ESP_ERROR_CHECK(bme280_full_init(&bus_handle, &bme280_dev_handle, &intf_ctx, &device));

//     bme280_get_sensor_settings(&settings, &device);
//     bme280_set_settings(&settings);
//     bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &device);
//     bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &device);
//     ESP_LOGI(TAG, "BME280 init OK");

//     /* ADXL345 Init */
//     adxl345_handle_t adxl345_handle;
//     ESP_ERROR_CHECK(adxl345_full_init(&bus_handle, &adxl345_dev_handle, &adxl345_handle));
//     adxl345_set_rate(&adxl345_handle, ADXL345_RATE_100);
//     adxl345_set_range(&adxl345_handle, ADXL345_RANGE_16G);
//     adxl345_set_measure(&adxl345_handle, ADXL345_BOOL_TRUE);
//     ESP_LOGI(TAG, "ADXL345 init OK");

//     /* NEO-6M Init */
//     neo6m_init();
//     ESP_LOGI(TAG, "NEO-6M init OK");

//     /* Radio Init */
//     sx1262_t radio;
//     esp_err_t ret = sx1262_init(&radio);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Radio init failed");
//         return;
//     }

//     ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_set_standby(&radio, SX1262_STDBY_RC));
//     ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_set_packet_type(&radio, SX1262_PACKET_TYPE_LORA));
//     ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_set_rf_frequency(&radio, RF_FREQ_HZ));
//     ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_set_mod_params_lora(&radio, 7, 0x04, 0x01, 0x00));
//     ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_set_packet_params(&radio, 12, 0x00, sizeof(sensor_telemetry_packet_t), 0x01, 0x00));
//     ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_set_dio_irq_params(&radio));
//     ESP_LOGI(TAG, "Radio init OK, starting sensor TX test...");

//     uint16_t seq = 0;

//     while (1) {
//         /* BME280 Read */
//         uint32_t period = 100000;
//         bme280_cal_meas_delay(&period, &settings);
//         struct bme280_data bme280_output;
//         int8_t rslt = bme280_get_sensor_data(BME280_ALL, &bme280_output, &device);
//         if (rslt != BME280_OK) {
//             ESP_LOGE(TAG, "BME280 read failed");
//         }

//         /* ADXL345 Read */
//         int16_t accel_raw[3];
//         float accel_g[3];
//         uint16_t accel_len = 1;
//         uint8_t accel_rslt = adxl345_read(&adxl345_handle, (int16_t (*)[3])&accel_raw, (float (*)[3])&accel_g, &accel_len);
//         if (accel_rslt != 0) {
//             ESP_LOGE(TAG, "ADXL345 read failed");
//         }

//         /* Build packet */
//         sensor_telemetry_packet_t packet = {
//             .callsign_id = CALLSIGN_ID,
//             .seq = seq++,
//             .packet_type = TELEMETRY_PACKET_TYPE_SENSOR,
//             .timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
//             .altitude = (int16_t)bme280_output_to_altitude(&bme280_output),
//             .heading = 0,
//             .temperature = (int8_t)bme280_output.temperature,
//             .speed = 0,
//             .acceleration = accel_raw[2], /* Z-axis as vertical */
//         };

//         ESP_LOGI(TAG, "Pressure: %.2f Pa, Temperature: %.2f C, Humidity: %.2f %%",
//                  bme280_output.pressure, bme280_output.temperature, bme280_output.humidity);
//         ESP_LOGI(TAG, "Accel X: %.3f g, Y: %.3f g, Z: %.3f g", accel_g[0], accel_g[1], accel_g[2]);

//         /* NEO-6M Read */
//         nmea_gps_data_t gps = neo6m_read_parsed();
//         ESP_LOGI(TAG, "GPS valid: %s, Lat: %.6f, Lon: %.6f, Alt: %.1f m, Sats: %d, Speed: %.1f kph",
//                  gps.valid ? "yes" : "no",
//                  gps.latitude, gps.longitude, gps.altitude,
//                  gps.satellites_tracked, gps.speed_kph);

//         ESP_LOGI(TAG, "TX seq=%d alt=%d temp=%d accel_z=%d",
//                  packet.seq, packet.altitude, packet.temperature, packet.acceleration);

//         esp_err_t res = sx1262_transmit(&radio, (uint8_t *)&packet, sizeof(packet));
//         if (res == ESP_OK) {
//             ESP_LOGI(TAG, "Sent!");
//         } else {
//             ESP_LOGE(TAG, "TX failed: %d", res);
//         }

//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }




void app_main(void) {

    /* Radio init — must match ground-station RadioLib config:
       LoRa, 915 MHz, SF=7, BW=125 kHz (0x04), CR=4/5 (0x01), LDRO off,
       preamble=12, explicit header, CRC on, IQ normal. */
    static sx1262_t radio;
    esp_err_t ret = sx1262_init(&radio);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Radio init failed: %d", ret);
        return;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_set_standby(&radio, SX1262_STDBY_RC));
    ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_set_packet_type(&radio, SX1262_PACKET_TYPE_LORA));
    ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_set_rf_frequency(&radio, RF_FREQ_HZ));
    ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_set_mod_params_lora(&radio, 7, 0x04, 0x01, 0x00));
    ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_set_tx_params(&radio, 22, 0x04));
    ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_set_dio_irq_params(&radio));
    ESP_LOGI(TAG, "Radio configured for TX");

    /* We will seperate tasks for sensor collection and gps collection so that if general sensor collection fails we still transmit gps. */
    TaskHandle_t sensor_collection_task_handle = NULL;
    TaskHandle_t gps_collection_task_handle = NULL;
    TaskHandle_t transmission_task_handle = NULL;

    QueueHandle_t xSensorTelemetryQueue = xQueueCreate(SENSOR_TELEMETRY_QUEUE_SIZE, sizeof(sensor_telemetry_packet_t));
    QueueHandle_t xGPSQueue = xQueueCreate(GPS_TELEMETRY_QUEUE_SIZE, sizeof(gps_telemetry_packet_t));

    static transmission_task_params_t tx_params;
    tx_params.xSensorQueue = xSensorTelemetryQueue;
    tx_params.xGPSQueue = xGPSQueue;
    tx_params.radio = &radio;

    xTaskCreate(vSensorCollectionTask, "SENSOR_COLLECTION", SENSOR_COLLECTION_TASK_STACK_SIZE, (void *) xSensorTelemetryQueue, SENSOR_COLLECTION_TASK_PRIORITY, &sensor_collection_task_handle);
    xTaskCreate(vGPSCollectionTask, "GPS_COLLECTION", GPS_COLLECTION_TASK_STACK_SIZE, (void *) xGPSQueue, GPS_COLLECTION_TASK_PRIORITY, &gps_collection_task_handle);
    xTaskCreate(vTransmissionTask, "TRANSMISSION", TRANSMISSION_TASK_STACK_SIZE, (void *) &tx_params, TRANSMISSION_TASK_PRIORITY, &transmission_task_handle);

}