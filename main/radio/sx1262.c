#include <string.h>
#include "sx1262.h"
#include "spi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char *TAG = "SX1262 MODULE";

void sx1262_wait_busy(void) {
    /* While the chip is in BUSY mode, time out the thread. */
    while (gpio_get_level(SX1262_BUSY) == 1) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* is the device pointer even needed? */
esp_err_t sx1262_reset(sx1262_t *device) {
    gpio_set_level(SX1262_NRST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(SX1262_NRST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    sx1262_wait_busy();

    ESP_LOGI(TAG, "Chip Resetted.");

    return ESP_OK;
}

esp_err_t sx1262_init(sx1262_t *device) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SX1262_NRST), /* 1ULL is a 1 as a 64-bit unsigned int (Unsigned Long Long). 
                                                  Each bit represents a GPIO pin, we put 1 to the pin we want.
                                                  In this case we bit shift by the pin number of NSRT
                                                  (ie. if NSRT is pin 21, we make the 21st bit 1).
                                                */
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0
    };

    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << SX1262_BUSY) | (1ULL << SX1262_DI01); /* | (OR) operator combines them, setting both bits to 1. Here we're selecting the BUSY and Interup pins. */
    io_conf.mode = GPIO_MODE_INPUT; /* Then we set those pins to input mode. */

    gpio_config(&io_conf);


    /* Initialize SPI */

    esp_err_t res = spi_init(&device->device_handle);

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "SPI init failed");
    }

    sx1262_reset(device);

    uint8_t status;
    sx1262_get_status(device, &status);

    /* In the future, put the status in an enum or something so we can read it as text instead of hex */
    ESP_LOGI(TAG, "Status: 0x%02X", status);

    return ESP_OK;
}

esp_err_t sx1262_get_status(sx1262_t *device, uint8_t *status) {
    sx1262_wait_busy();

    uint8_t tx[] = { SX1262_OP_GET_STATUS, 0x00};
    uint8_t rx[2] = { 0 };

    esp_err_t res = spi_read(device->device_handle, tx, rx, 2);

    *status = rx[1];

    return res;

    /* To extract data from the returned status byte:
       uint8_t chip_mode = (status >> 4) & 0x07;      // shift right 4, keep 3 bits
       uint8_t cmd_status = (status >> 1) & 0x07;      // shift right 1, keep 3 bits
       
       
       Status is bit 1-3 so we shift by 1 to compensate for the first bit
       First bit is garbage, bits 4-6 are the chip mode, so we shift by 4       
    */
}

esp_err_t sx1262_set_standby(sx1262_t *device, uint8_t mode) {
    sx1262_wait_busy();

    uint8_t cmd[] = { SX1262_OP_SET_STANDBY, mode };

    return spi_write(device->device_handle, cmd, sizeof(cmd));
}

esp_err_t sx1262_set_packet_type(sx1262_t *device, uint8_t type) {
    sx1262_wait_busy();

    uint8_t cmd[] = { SX1262_OP_SET_PACKET_TYPE, type };

    return spi_write(device->device_handle, cmd, sizeof(cmd));
}

esp_err_t sx1262_set_rf_frequency(sx1262_t *device, uint32_t freq_hz) {
    /* Formula from data sheet, convert hz to register freq val */
    uint32_t rf_freq = (uint32_t)((double)freq_hz / (double)SX1262_CRYSTAL_FREQUENCY * (double)(1 << 25));

    sx1262_wait_busy();

    /* Chopping up the 32bit freq value into 4 seperate bytes, sending the MSB first (big endian). Masking to 0xFF so that it's only 8 bits (1 byte) */
    uint8_t cmd[] = {
        SX1262_OP_SET_RF_FREQUENCY,
        (rf_freq >> 24) & 0xFF,
        (rf_freq >> 16) & 0xFF,
        (rf_freq >> 8) & 0xFF,
        rf_freq & 0xFF,
    };

    return spi_write(device->device_handle, cmd, sizeof(cmd));
}

/* TODO: Ramp time is a hex value based on table 13-41. Put this in an enum */
esp_err_t sx1262_set_tx_params(sx1262_t *device, int8_t power, uint8_t ramp_time) {
    sx1262_wait_busy();

    uint8_t cmd[] = { SX1262_OP_SET_TX_PARAMS, power, ramp_time };

    return spi_write(device->device_handle, cmd, sizeof(cmd));
}

/* Spreading factor, bandwidth, coding rate, low data rate optimization. TODO: Put these in an enum like the last function. */
esp_err_t sx1262_set_mod_params_lora(sx1262_t *device, uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro) {
    sx1262_wait_busy();

    uint8_t cmd[] = { SX1262_OP_SET_MOD_PARAMS, sf, bw, cr, ldro };

    return spi_write(device->device_handle, cmd, sizeof(cmd));
}

esp_err_t sx1262_transmit(sx1262_t *device, uint8_t *data, uint8_t len) {
    sx1262_wait_busy();

    /* Command to write to the transmission buffer */
    uint8_t write_cmd[2 + len];
    write_cmd[0] = SX1262_OP_WRITE_BUFFER;
    write_cmd[1] = 0x00;

    for (int i = 0; i < len; i++) {
        write_cmd[i + 2] = data[i];
    }

    esp_err_t write_res = spi_write(device->device_handle, write_cmd, sizeof(write_cmd));

    if (write_res != ESP_OK) {
        ESP_LOGE(TAG, "Failed writing to transmission buffer.");
        return write_res;
    }

    uint32_t timeout = SX1262_TX_RX_TIMEOUT / 15.625;

    /* Set the device to transmit mode */
    uint8_t tx_cmd[] = {
        SX1262_OP_SET_TX,
        (timeout >> 16) & 0xFF,
        (timeout >> 8) & 0xFF,
        timeout & 0xFF
    };

    sx1262_wait_busy();

    esp_err_t set_tx_res = spi_write(device->device_handle, tx_cmd, sizeof(tx_cmd));

    if (set_tx_res != ESP_OK) {
        ESP_LOGE(TAG, "Failed setting module to transmission mode.");
        return set_tx_res;
    }

    /* Wait for DIO1 to fire (TX_DONE or TIMEOUT) */
    while (gpio_get_level(SX1262_DI01) == 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* Check IRQ status */
    sx1262_wait_busy();

    uint8_t irq_cmd[] = { SX1262_OP_GET_IRQ_STATUS, 0x00, 0x00, 0x00 };
    uint8_t irq_rx[4] = { 0 };

    spi_read(device->device_handle, irq_cmd, irq_rx, 4);

    uint16_t irq_status = (irq_rx[2] << 8) | irq_rx[3];

    /* Clear IRQ flags */
    sx1262_wait_busy();

    uint8_t clear_cmd[] = {
        SX1262_OP_CLEAR_IRQ_STATUS,
        (irq_status >> 8) & 0xFF,
        irq_status & 0xFF
    };

    spi_write(device->device_handle, clear_cmd, sizeof(clear_cmd));

    /* Check for timeout (bit 9) */
    if (irq_status & (1 << 9)) {
        ESP_LOGE(TAG, "TX timeout.");

        return ESP_ERR_TIMEOUT;
    }

    /* Check TX_DONE (bit 0) */
    if (irq_status & (1 << 0)) {
        ESP_LOGI(TAG, "Transmission complete.");

        return ESP_OK;
    }

    ESP_LOGE(TAG, "Unknown TX IRQ status: 0x%04X", irq_status);

    return ESP_FAIL;
}

esp_err_t sx1262_receive(sx1262_t *device, uint8_t *data, uint8_t *len) {
    sx1262_wait_busy();

    uint32_t timeout = SX1262_TX_RX_TIMEOUT / 15.625;

    uint8_t rx_cmd[] = {
        SX1262_OP_SET_RX,
        (timeout >> 16) & 0xFF,
        (timeout >> 8) & 0xFF,
        timeout & 0xFF
    };

    esp_err_t res = spi_write(device->device_handle, rx_cmd, sizeof(rx_cmd));

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed setting module to receive mode.");

        return res;
    }

    /* Wait for DIO1 to fire */
    while (gpio_get_level(SX1262_DI01) == 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* Check what happened */
    sx1262_wait_busy();

    uint8_t irq_cmd[] = { SX1262_OP_GET_IRQ_STATUS, 0x00, 0x00, 0x00 };
    uint8_t irq_rx[4] = { 0 };

    spi_read(device->device_handle, irq_cmd, irq_rx, 4);

    uint16_t irq_status = (irq_rx[2] << 8) | irq_rx[3];

    /* Clear IRQ flags */
    sx1262_wait_busy();

    uint8_t clear_cmd[] = {
        SX1262_OP_CLEAR_IRQ_STATUS,
        (irq_status >> 8) & 0xFF,
        irq_status & 0xFF
    };

    spi_write(device->device_handle, clear_cmd, sizeof(clear_cmd));

    /* Check for timeout */
    if (irq_status & (1 << 9)) {
        ESP_LOGW(TAG, "RX timeout - no packet received.");
        *len = 0;

        return ESP_ERR_TIMEOUT;
    }

    /* Check for CRC error */
    if (irq_status & (1 << 6)) {
        ESP_LOGE(TAG, "CRC error - packet corrupted.");
        *len = 0;

        return ESP_FAIL;
    }

    /* Get payload length and buffer offset */
    sx1262_wait_busy();

    uint8_t buf_status_cmd[] = { SX1262_OP_GET_RX_BUFFER_STATUS, 0x00, 0x00, 0x00 };
    uint8_t buf_status_rx[4] = { 0 };

    spi_read(device->device_handle, buf_status_cmd, buf_status_rx, 4);

    uint8_t payload_len = buf_status_rx[2];
    uint8_t buf_offset = buf_status_rx[3];

    /* Read the payload from the buffer */
    sx1262_wait_busy();

    uint8_t read_cmd[3 + payload_len];
    uint8_t read_rx[3 + payload_len];

    read_cmd[0] = SX1262_OP_READ_BUFFER;
    read_cmd[1] = buf_offset;
    read_cmd[2] = 0x00;

    for (int i = 3; i < 3 + payload_len; i++) {
        read_cmd[i] = 0x00;
    }

    memset(read_rx, 0, sizeof(read_rx));

    spi_read(device->device_handle, read_cmd, read_rx, sizeof(read_cmd));

    /* Copy payload to output */
    memcpy(data, read_rx + 3, payload_len);
    *len = payload_len;

    ESP_LOGI(TAG, "Received %d bytes.", payload_len);

    return ESP_OK;
}

/* its 2:50 AM im tired this is straight copy paste from the big claude */
esp_err_t sx1262_set_packet_params(sx1262_t *device, uint16_t preamble_len, uint8_t header_type, uint8_t payload_len, uint8_t crc, uint8_t invert_iq) {
    sx1262_wait_busy();

    uint8_t cmd[] = {
        SX1262_OP_SET_PACKET_PARAMS,
        (preamble_len >> 8) & 0xFF,
        preamble_len & 0xFF,
        header_type,
        payload_len,
        crc,
        invert_iq
    };

    return spi_write(device->device_handle, cmd, sizeof(cmd));
}

/* its 3:01 AM im tired this is straight copy paste from the big claude */
esp_err_t sx1262_set_dio_irq_params(sx1262_t *device) {
    sx1262_wait_busy();

    /* IRQ mask: enable TX_DONE (bit 0), RX_DONE (bit 1), TIMEOUT (bit 9), CRC_ERROR (bit 6) */
    uint16_t irq_mask = (1 << 0) | (1 << 1) | (1 << 6) | (1 << 9);

    /* Map all enabled IRQs to DIO1 */
    uint16_t dio1_mask = irq_mask;
    uint16_t dio2_mask = 0x0000;
    uint16_t dio3_mask = 0x0000;

    uint8_t cmd[] = {
        SX1262_OP_SET_DIO_IRQ_PARAMS,
        (irq_mask >> 8) & 0xFF,
        irq_mask & 0xFF,
        (dio1_mask >> 8) & 0xFF,
        dio1_mask & 0xFF,
        (dio2_mask >> 8) & 0xFF,
        dio2_mask & 0xFF,
        (dio3_mask >> 8) & 0xFF,
        dio3_mask & 0xFF,
    };

    return spi_write(device->device_handle, cmd, sizeof(cmd));
}