#include "hmc5883l_port.h"
#include "freertos/FreeRTOS.h"
#include <string.h>
#include <stdarg.h>
#include "esp_log.h"

static const char *TAG = "HMC5883L_PORT";

/* Static device handle so LibDriver callbacks can access it (no intf_ptr in their signature) */
static i2c_master_dev_handle_t s_hmc5883l_dev_handle;


esp_err_t hmc5883l_i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *hmc5883l_handle) {
    i2c_device_config_t dev_config = {
        .device_address = HMC5883L_I2C_ADDR,
        .dev_addr_length = I2C_ADDR_BIT_7,
        .scl_speed_hz = I2C_SCL_SPEED_HZ
    };

    return i2c_master_bus_add_device(*bus_handle, &dev_config, hmc5883l_handle);
}


uint8_t hmc5883l_iic_init_cb(void) {
    /* I2C bus is already initialized in main, nothing to do */
    return 0;
}


uint8_t hmc5883l_iic_deinit_cb(void) {
    return 0;
}


uint8_t hmc5883l_iic_read_cb(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    esp_err_t ret = i2c_master_transmit_receive(
        s_hmc5883l_dev_handle,
        &reg, 1,
        buf, len,
        pdMS_TO_TICKS(10000)
    );

    return (ret == ESP_OK) ? 0 : 1;
}


uint8_t hmc5883l_iic_write_cb(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t write_buf[len + 1];
    write_buf[0] = reg;
    memcpy(write_buf + 1, buf, len);

    esp_err_t ret = i2c_master_transmit(
        s_hmc5883l_dev_handle,
        write_buf, len + 1,
        pdMS_TO_TICKS(10000)
    );

    return (ret == ESP_OK) ? 0 : 1;
}


void hmc5883l_delay_ms_cb(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}


void hmc5883l_debug_print_cb(const char *const fmt, ...) {
    va_list args;
    va_start(args, fmt);
    esp_log_writev(ESP_LOG_INFO, TAG, fmt, args);
    va_end(args);
}


esp_err_t hmc5883l_full_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *hmc5883l_dev_handle, hmc5883l_intf_ctx_t *intf_ctx_ptr, hmc5883l_handle_t *handle) {

    ESP_ERROR_CHECK(hmc5883l_i2c_init(bus_handle, hmc5883l_dev_handle));
    intf_ctx_ptr->dev_handle = *hmc5883l_dev_handle;
    s_hmc5883l_dev_handle = *hmc5883l_dev_handle;

    /* Zero out the handle and link all callbacks */
    DRIVER_HMC5883L_LINK_INIT(handle, hmc5883l_handle_t);
    DRIVER_HMC5883L_LINK_IIC_INIT(handle, hmc5883l_iic_init_cb);
    DRIVER_HMC5883L_LINK_IIC_DEINIT(handle, hmc5883l_iic_deinit_cb);
    DRIVER_HMC5883L_LINK_IIC_READ(handle, hmc5883l_iic_read_cb);
    DRIVER_HMC5883L_LINK_IIC_WRITE(handle, hmc5883l_iic_write_cb);
    DRIVER_HMC5883L_LINK_DELAY_MS(handle, hmc5883l_delay_ms_cb);
    DRIVER_HMC5883L_LINK_DEBUG_PRINT(handle, hmc5883l_debug_print_cb);

    uint8_t rslt = hmc5883l_init(handle);

    return (rslt == 0) ? ESP_OK : ESP_FAIL;
}
