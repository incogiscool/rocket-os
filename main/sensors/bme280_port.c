#include "bme280_port.h"
#include "freertos/FreeRTOS.h"
#include <rom/ets_sys.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "BME280_PORT";

esp_err_t bme280_i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *bme280_handle) {
    i2c_device_config_t dev_config = {
        .device_address = BME280_I2C_ADDR,
        .dev_addr_length = I2C_ADDR_BIT_7,
        .scl_speed_hz = I2C_SCL_SPEED_HZ
    };


    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, bme280_handle));

    return ESP_OK;
}


BME280_INTF_RET_TYPE user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    bme280_intf_ctx_t *ctx = (bme280_intf_ctx_t *)intf_ptr;

    esp_err_t ret = i2c_master_transmit_receive(
        ctx->dev_handle, /* Device handle */
        &reg_addr, 1, /* Transmitting the register we want to read from (size is 1 byte) */
        reg_data, len, /* Read buffer and exected size of the read buffer */
        pdMS_TO_TICKS(10000) /* Timeout */
    );

    return (ret == ESP_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}


BME280_INTF_RET_TYPE user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    bme280_intf_ctx_t *ctx = (bme280_intf_ctx_t *)intf_ptr;

    /* Building a single buffer [reg_addr, data0, data1, ...]
       Sensor is expecting the register addr first then the data bytes
    */

    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    memcpy(buf + 1, reg_data, len); /* Copying the register data to the rest of the register buffer after index 0 */

    esp_err_t ret = i2c_master_transmit(
        ctx->dev_handle,
        buf, len + 1,
        pdMS_TO_TICKS(10000)
    );


    return (ret == ESP_OK) ? BME280_OK : BME280_E_COMM_FAIL;
}


void user_delay_us(uint32_t period_us, void *intf_ptr)
{
    if (period_us >= 1000) {
        vTaskDelay(pdMS_TO_TICKS(period_us / 1000));
    } else {
        ets_delay_us(period_us);  // busy-wait for sub-millisecond delays
    }
}