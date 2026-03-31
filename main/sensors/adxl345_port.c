#include "adxl345_port.h"
#include "freertos/FreeRTOS.h"
#include <string.h>
#include <stdarg.h>
#include "esp_log.h"

static const char *TAG = "ADXL345_PORT";

/* Static device handle so LibDriver callbacks can access it (no intf_ptr in their signature) */
static i2c_master_dev_handle_t s_adxl345_dev_handle;


esp_err_t adxl345_i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *adxl345_handle) {
    i2c_device_config_t dev_config = {
        .device_address = ADXL345_I2C_ADDR,
        .dev_addr_length = I2C_ADDR_BIT_7,
        .scl_speed_hz = I2C_SCL_SPEED_HZ
    };

    return i2c_master_bus_add_device(*bus_handle, &dev_config, adxl345_handle);
}


uint8_t adxl345_iic_init_cb(void) {
    /* I2C bus is already initialized in main, nothing to do */
    return 0;
}


uint8_t adxl345_iic_deinit_cb(void) {
    return 0;
}


uint8_t adxl345_iic_read_cb(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    esp_err_t ret = i2c_master_transmit_receive(
        s_adxl345_dev_handle,
        &reg, 1,
        buf, len,
        pdMS_TO_TICKS(10000)
    );

    return (ret == ESP_OK) ? 0 : 1;
}


uint8_t adxl345_iic_write_cb(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t write_buf[len + 1];
    write_buf[0] = reg;
    memcpy(write_buf + 1, buf, len);

    esp_err_t ret = i2c_master_transmit(
        s_adxl345_dev_handle,
        write_buf, len + 1,
        pdMS_TO_TICKS(10000)
    );

    return (ret == ESP_OK) ? 0 : 1;
}


void adxl345_delay_ms_cb(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}


void adxl345_debug_print_cb(const char *const fmt, ...) {
    va_list args;
    va_start(args, fmt);
    esp_log_writev(ESP_LOG_INFO, TAG, fmt, args);
    va_end(args);
}


void adxl345_receive_cb(uint8_t type) {
    (void)type;
}


uint8_t adxl345_spi_init_cb(void) { return 0; }
uint8_t adxl345_spi_deinit_cb(void) { return 0; }
uint8_t adxl345_spi_read_cb(uint8_t reg, uint8_t *buf, uint16_t len) { return 0; }
uint8_t adxl345_spi_write_cb(uint8_t reg, uint8_t *buf, uint16_t len) { return 0; }


esp_err_t adxl345_full_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *adxl345_dev_handle, adxl345_handle_t *handle) {

    ESP_ERROR_CHECK(adxl345_i2c_init(bus_handle, adxl345_dev_handle));
    s_adxl345_dev_handle = *adxl345_dev_handle;

    /* Zero out the handle and link all callbacks */
    DRIVER_ADXL345_LINK_INIT(handle, adxl345_handle_t);
    DRIVER_ADXL345_LINK_IIC_INIT(handle, adxl345_iic_init_cb);
    DRIVER_ADXL345_LINK_IIC_DEINIT(handle, adxl345_iic_deinit_cb);
    DRIVER_ADXL345_LINK_IIC_READ(handle, adxl345_iic_read_cb);
    DRIVER_ADXL345_LINK_IIC_WRITE(handle, adxl345_iic_write_cb);
    DRIVER_ADXL345_LINK_SPI_INIT(handle, adxl345_spi_init_cb);
    DRIVER_ADXL345_LINK_SPI_DEINIT(handle, adxl345_spi_deinit_cb);
    DRIVER_ADXL345_LINK_SPI_READ(handle, adxl345_spi_read_cb);
    DRIVER_ADXL345_LINK_SPI_WRITE(handle, adxl345_spi_write_cb);
    DRIVER_ADXL345_LINK_DELAY_MS(handle, adxl345_delay_ms_cb);
    DRIVER_ADXL345_LINK_DEBUG_PRINT(handle, adxl345_debug_print_cb);
    DRIVER_ADXL345_LINK_RECEIVE_CALLBACK(handle, adxl345_receive_cb);

    /* Set to I2C interface with ADDR pin to GND */
    adxl345_set_interface(handle, ADXL345_INTERFACE_IIC);
    adxl345_set_addr_pin(handle, ADXL345_ADDRESS_ALT_0);

    uint8_t rslt = adxl345_init(handle);

    return (rslt == 0) ? ESP_OK : ESP_FAIL;
}
