#include "spi.h"
#include "sx1262.h"

/* Function returns the error state, and then puts the device handle(s), in this case the sx1262 device, inside of the handle pointer passed into the function. */
esp_err_t spi_init(spi_device_handle_t *handle) {
    spi_bus_config_t bus_config = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_CLK_PIN,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1
    };

    esp_err_t res = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);

    if (res != ESP_OK) return res;


    /* Initializing all of the SPI devices on the bus */


    spi_device_interface_config_t sx1262_device_config = {
        .clock_speed_hz = SPI_CLK_SPEED,
        .mode = 0, /* SPI mode 0 (CPOL=0, CPHA=0) */
        .spics_io_num = SX1262_SPI_CS_PIN,
        .queue_size = 1
    };

    return spi_bus_add_device(SPI2_HOST, &sx1262_device_config, handle);
}

esp_err_t spi_write(spi_device_handle_t handle, uint8_t *data, size_t length) {
    spi_transaction_t transaction = {
        .length = length * 8, /* We will pass in length as bytes, but the transaction takes length in bits */
        .tx_buffer = data
    };

    return spi_device_transmit(handle, &transaction);
}

esp_err_t spi_read(spi_device_handle_t handle, uint8_t *tx_data, uint8_t *rx_data, size_t length) {
    spi_transaction_t transaction = {
        .length = length * 8, /* We will pass in length as bytes, but the transaction takes length in bits */
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    return spi_device_transmit(handle, &transaction);
}