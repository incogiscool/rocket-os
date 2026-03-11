#pragma once

#include "gpio/i2c.h"
#include "lib/BME280_SensorAPI/bme280.h"

#define BME280_I2C_ADDR 0x76


/* Context passed to every callback via intf_ptr */
typedef struct {
    i2c_master_dev_handle_t dev_handle;
} bme280_intf_ctx_t;


/* Init the module on the i2c bus */
esp_err_t bme280_i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *bme280_handle);

/* BME280 Library Callbacks */
BME280_INTF_RET_TYPE user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BME280_INTF_RET_TYPE user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void user_delay_us(uint32_t period_us, void *intf_ptr);