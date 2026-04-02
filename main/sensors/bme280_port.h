#pragma once

#include "gpio/i2c.h"
#include "lib/BME280_SensorAPI/bme280.h"

#define BME280_I2C_ADDR 0x76

/* Hypsometric formula constants */
#define BME280_SEA_LEVEL_PRESSURE_PA  101325.0f  /* Standard sea level pressure (Pa) */
#define BME280_SEA_LEVEL_TEMP_K       288.15f    /* Standard sea level temperature (K) */
#define BME280_TEMP_LAPSE_RATE        0.0065f    /* Temperature lapse rate (K/m) */
#define BME280_HYPSOMETRIC_EXP        0.190263f  /* R*L / (g*M) = 8.31446*0.0065 / (9.80665*0.0289644) */


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

/**
 * [in] bus_handle, reference to the i2c bus handle
 * [out] bme280_dev_handle, reference to the i2c handle for the BME280 device
 * [out] intf_ctx_ptr, interface context for the bme280 - see below for example of this struct
 * [out] device_struct, the complete BME280 driver config struct to be used in other functions
 * 
 * 
 * 
 *  bme280_intf_ctx_t intf_ctx = {
        .dev_handle = bme280_dev_handle,
    };
 */
esp_err_t bme280_full_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *bme280_dev_handle, bme280_intf_ctx_t *intf_ctx, struct bme280_dev *device_struct);

void bme280_set_settings(struct bme280_settings *settings);

int32_t bme280_output_to_altitude(struct bme280_data *data);