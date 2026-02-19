#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"


#define SPI_CLK_PIN 18
#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 23
#define SPI_LORA_CS_PIN 2
#define LORA_NSRT 21
#define LORA_BUSY 16
#define LORA_DO1 17

#define BUILT_IN_LED_PIN 2

void app_main(void)
{

    gpio_reset_pin(BUILT_IN_LED_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BUILT_IN_LED_PIN, GPIO_MODE_OUTPUT);
    

    for (;;) {
        gpio_set_level(BUILT_IN_LED_PIN, 1);
    }
}