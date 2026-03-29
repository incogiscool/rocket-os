#pragma once

#include "gpio/i2c.h"
#include "lib/hmc5883l/src/driver_hmc5883l.h"

#define HMC5883L_I2C_ADDR 0x1E /* 7-bit address */


/* Context passed to full_init, holds the ESP-IDF device handle */
typedef struct {
    i2c_master_dev_handle_t dev_handle;
} hmc5883l_intf_ctx_t;


/* Init the module on the i2c bus */
esp_err_t hmc5883l_i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *hmc5883l_handle);

/* LibDriver Callbacks */
uint8_t hmc5883l_iic_init_cb(void);
uint8_t hmc5883l_iic_deinit_cb(void);
uint8_t hmc5883l_iic_read_cb(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t hmc5883l_iic_write_cb(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
void hmc5883l_delay_ms_cb(uint32_t ms);
void hmc5883l_debug_print_cb(const char *const fmt, ...);

/**
 * [in] bus_handle, reference to the i2c bus handle
 * [out] hmc5883l_dev_handle, reference to the i2c handle for the HMC5883L device
 * [out] intf_ctx_ptr, interface context for the hmc5883l
 * [out] handle, the complete HMC5883L driver handle struct to be used in other functions
 */
esp_err_t hmc5883l_full_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *hmc5883l_dev_handle, hmc5883l_intf_ctx_t *intf_ctx, hmc5883l_handle_t *handle);
