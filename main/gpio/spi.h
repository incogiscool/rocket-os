/* Same as ifndef */
#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define SPI_CLK_PIN 18
#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 23

#define SPI_CLK_SPEED 1000000 /* For now define 1Mhz, possibly go up to 16 later on */

esp_err_t spi_init(spi_device_handle_t *handle);
esp_err_t spi_write (spi_device_handle_t handle, uint8_t *data, size_t length);
esp_err_t spi_read(spi_device_handle_t handle, uint8_t *tx_data, uint8_t *rx_data, size_t length);
