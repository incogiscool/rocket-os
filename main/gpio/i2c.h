#pragma once

#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "driver/i2c_types.h"

#define I2C_MASTER_SCL_IO 22 /* GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO 23 /* GPIO number for I2C master data */

#define I2C_SCL_SPEED_HZ 100000 /* Set to 100khz */

esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *device_handle);
esp_err_t i2c_master_write_to_slave(uint8_t device_address, uint8_t *data, size_t data_len);
esp_err_t i2c_master_read_from_slave(uint8_t device_address, uint8_t *data, size_t data_len);

