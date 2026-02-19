
#include "sx1262.h"
#include "esp_log.h"

#define BUILT_IN_LED_PIN 2
#define RF_FREQ_HZ 915000000 /* 915Mhz */

// static const char *TAG = "MAIN";

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

#include "sx1262.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TX";

void app_main(void) {
    sx1262_t radio;

    esp_err_t ret = sx1262_init(&radio);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Radio init failed");
        return;
    }

    sx1262_set_standby(&radio, SX1262_STDBY_RC);
    sx1262_set_packet_type(&radio, SX1262_PACKET_TYPE_LORA);
    sx1262_set_rf_frequency(&radio, RF_FREQ_HZ);
    sx1262_set_mod_params_lora(&radio, 7, 0x04, 0x01, 0x00);
    sx1262_set_packet_params(&radio, 12, 0x00, 64, 0x01, 0x00);
    sx1262_set_dio_irq_params(&radio);

    ESP_LOGI(TAG, "Transmitter ready!");

    int counter = 0;

    while (1) {
        char msg[64];
        int len = snprintf(msg, sizeof(msg), "Hello #%d", counter);

        ESP_LOGI(TAG, "Sending: %s", msg);

        esp_err_t res = sx1262_transmit(&radio, (uint8_t *)msg, len);
        if (res == ESP_OK) {
            ESP_LOGI(TAG, "Sent!");
        } else {
            ESP_LOGE(TAG, "TX failed: %d", res);
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}