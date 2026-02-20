#include "sx1262.h"
#include "esp_log.h"
#include "driver/uart.h"

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


#include "sx1262.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TX";

#define GPS_RX 26 /* GPS TX -> Pin 26 */
#define GPS_TX 27 /* GPS RX -> Pin 27 */
#define GPS_RTS  (UART_PIN_NO_CHANGE)
#define GPS_CTS  (UART_PIN_NO_CHANGE)
#define BAUD_RATE 9600
#define BUF_SIZE (1024)


void app_main(void) {
    
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, GPS_TX, GPS_RX, GPS_RTS, GPS_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

        // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "DATA: \n%s", (char *) data);
        }

        ESP_LOGI(TAG, "Length: %d", len);
        // Write data back to the UART
        // uart_write_bytes(UART_NUM_1, (const char *) data, len);
    }

    

}