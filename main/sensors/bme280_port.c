#include "bme280_port.h"
#include "freertos/FreeRTOS.h"
#include <rom/ets_sys.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "BME280_PORT";

esp_err_t bme280_i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *bme280_handle) {
    i2c_device_config_t dev_config = {
        .device_address = BME280_I2C_ADDR,
        .dev_addr_length = I2C_ADDR_BIT_7,
        .scl_speed_hz = I2C_SCL_SPEED_HZ
    };

    return i2c_master_bus_add_device(*bus_handle, &dev_config, bme280_handle);
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

esp_err_t bme280_full_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *bme280_dev_handle, bme280_intf_ctx_t *intf_ctx_ptr, struct bme280_dev *device_struct) {

    ESP_ERROR_CHECK(bme280_i2c_init(bus_handle, bme280_dev_handle));
    intf_ctx_ptr->dev_handle = *bme280_dev_handle;

    struct bme280_dev device = {
        .intf = BME280_I2C_INTF,
        .intf_ptr = intf_ctx_ptr,
        .read = user_i2c_read,
        .write = user_i2c_write,
        .delay_us = user_delay_us
    };
    
    int8_t rslt = bme280_init(&device);

    *device_struct = device;

    return (rslt == BME280_OK) ? ESP_OK : ESP_FAIL;
}

void bme280_set_settings(struct bme280_settings *settings) {
    settings->filter = BME280_FILTER_COEFF_2;
    settings->osr_h = BME280_OVERSAMPLING_4X;
    settings->osr_p = BME280_OVERSAMPLING_4X;
    settings->osr_t = BME280_OVERSAMPLING_4X;
    settings->standby_time = BME280_STANDBY_TIME_0_5_MS;
}

int32_t bme280_output_to_altitude(struct bme280_data *data) {
    /* h = (T0 / L) * (1 - (P / P0) ^ (R*L / g*M)) */
    float ratio = (float)data->pressure / BME280_SEA_LEVEL_PRESSURE_PA;
    float altitude = (BME280_SEA_LEVEL_TEMP_K / BME280_TEMP_LAPSE_RATE) * (1.0f - powf(ratio, BME280_HYPSOMETRIC_EXP));

    return (int32_t)altitude;
}