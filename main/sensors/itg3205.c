#include "gpio/i2c.h"
#include "itg3205.h"

esp_err_t itg3205_i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *itg3205_handle) {

    i2c_device_config_t dev_config = {
            .device_address = ITG3205_ADDR,
            .dev_addr_length = I2C_ADDR_BIT_7,
            .scl_speed_hz = I2C_SCL_SPEED_HZ
    };

    return i2c_master_bus_add_device(*bus_handle, &dev_config, itg3205_handle);
}

esp_err_t itg3205_read_reg(i2c_master_dev_handle_t *itg3205_handle, uint8_t reg_addr, uint8_t *data, uint32_t len) {
    return i2c_master_transmit_receive(itg3205_handle, &reg_addr, 1, &data, len, ITG3205_I2C_TIMEOUT_MS);
}

esp_err_t itg3205_write_reg(i2c_master_dev_handle_t *itg3205_handle, uint8_t reg_addr, const uint8_t *data) {
    uint8_t buf[2] = { reg_addr, data };

    return i2c_master_transmit(itg3205_handle, buf, 2, ITG3205_I2C_TIMEOUT_MS);
}

