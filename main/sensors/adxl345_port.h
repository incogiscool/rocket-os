#pragma once

#include "gpio/i2c.h"
#include "lib/adxl345/src/driver_adxl345.h"

#define ADXL345_I2C_ADDR 0x53 /* 7-bit address (ADDR pin to GND) */


/* Context passed to full_init, holds the ESP-IDF device handle */
typedef struct {
    i2c_master_dev_handle_t dev_handle;
} adxl345_intf_ctx_t;


/* Init the module on the i2c bus */
esp_err_t adxl345_i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *adxl345_handle);

/* LibDriver Callbacks */
uint8_t adxl345_iic_init_cb(void);
uint8_t adxl345_iic_deinit_cb(void);
uint8_t adxl345_iic_read_cb(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t adxl345_iic_write_cb(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
void adxl345_delay_ms_cb(uint32_t ms);
void adxl345_debug_print_cb(const char *const fmt, ...);

/**
 * [in] bus_handle, reference to the i2c bus handle
 * [out] adxl345_dev_handle, reference to the i2c handle for the ADXL345 device
 * [out] intf_ctx_ptr, interface context for the adxl345
 * [out] handle, the complete ADXL345 driver handle struct to be used in other functions
 */
esp_err_t adxl345_full_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *adxl345_dev_handle, adxl345_intf_ctx_t *intf_ctx, adxl345_handle_t *handle);
