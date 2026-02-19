
#include "sx1262.h"
#include "esp_log.h"

#define BUILT_IN_LED_PIN 2
#define RF_FREQ_HZ 915000000 /* 915Mhz */

static const char *TAG = "MAIN";

void app_main(void) {
    sx1262_t radio;

    esp_err_t ret = sx1262_init(&radio);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Radio init failed");
        return;
    }

    // Set to standby
    sx1262_set_standby(&radio, SX1262_STDBY_RC);

    // Set to LoRa mode
    sx1262_set_packet_type(&radio, SX1262_PACKET_TYPE_LORA);

    // Set frequency to 915 MHz
    sx1262_set_rf_frequency(&radio, RF_FREQ_HZ);

    ESP_LOGI(TAG, "Radio configured!");

    gpio_reset_pin(BUILT_IN_LED_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BUILT_IN_LED_PIN, GPIO_MODE_OUTPUT);
    

    gpio_set_level(BUILT_IN_LED_PIN, 1);
}