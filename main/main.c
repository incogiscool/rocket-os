#include "sx1262.h"
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
#include "gpio/i2c.h"
#include "lib/BME280_SensorAPI/bme280.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"




void app_main(void) {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t bme280_dev_handle;
    struct bme280_settings settings;
    struct bme280_dev *device;
    uint32_t period = 100000;

    ESP_ERROR_CHECK(i2c_master_init(&bus_handle));

    bme280_intf_ctx_t intf_ctx = {
        .dev_handle = bme280_dev_handle,
    };

    ESP_ERROR_CHECK(bme280_full_init(&bus_handle, &bme280_dev_handle, &intf_ctx, &device));

    int8_t rslt;

    rslt = bme280_get_sensor_settings(&settings, &device);
    // bme280_error_codes_print_result("bme280_get_sensor_settings", rslt);

    /* Configuring the over-sampling rate, filter coefficient and standby time */
    /* Overwrite the desired settings */
    settings.filter = BME280_FILTER_COEFF_2;

    /* Over-sampling rate for humidity, temperature and pressure */
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;

    /* Setting the standby time */
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &device);
    // bme280_error_codes_print_result("bme280_set_sensor_settings", rslt);

    /* Always set the power mode after setting the configuration */
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &device);
    // bme280_error_codes_print_result("bme280_set_power_mode", rslt);

    while (1) {

        /* Calculate measurement time in microseconds */
        rslt = bme280_cal_meas_delay(&period, &settings);
        // bme280_error_codes_print_result("bme280_cal_meas_delay", rslt);

        printf("\nPressure calculation (Data displayed are compensated values)\n");
        printf("Measurement time : %lu us\n\n", (long unsigned int)period);

        struct bme280_data comp_data;

        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &device);

        if (rslt != BME280_OK) {
            ESP_LOGE(TAG, "Something went wrong with getting sensor data.");
        }

        

        ESP_LOGI(TAG, "Pressure: %f\nTemperature: %f\nHumidity: %f\n", comp_data.pressure, comp_data.temperature, comp_data.humidity);

    
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    

    // rslt = get_pressure(period, &device);
    // bme280_error_codes_print_result("get_pressure", rslt);

}
