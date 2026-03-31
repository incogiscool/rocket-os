#pragma once

#include "gpio/i2c.h"

#define ITG3205_ADDR 0x68
#define ITG3205_I2C_TIMEOUT_MS 1000

typedef enum register_map {
    ITG3205_REG_WHOAMI = 0x00,
    ITG3205_REG_SMPLRT_DIV = 0x15,
    ITG3205_REG_DLPF_FS = 0x16,
    ITG3205_REG_INT_CFG = 0x17,
    ITG3205_REG_INT_STATUS = 0x1A,
    ITG3205_REG_TEMP_OUT_H = 0x1B, /* High and Low bytes - i2c only transfers 1 byte (8 bits) at a time so we split the 16 bit value into two 8-bit values 
                                      The High (H) byte contains the most significant 8 bits of the temperature data, while the Low (L) byte contains the least significant 8 bits. To get the full 16-bit temperature value, you would read both bytes and combine them.
                                    */
    ITG3205_REG_TEMP_OUT_L = 0x1C,
    ITG3205_REG_GYRO_X_OUT_H = 0x1D,
    ITG3205_REG_GYRO_X_OUT_L = 0x1E,
    ITG3205_REG_GYRO_Y_OUT_H = 0x1F,
    ITG3205_REG_GYRO_Y_OUT_L = 0x20,
    ITG3205_REG_GYRO_Z_OUT_H = 0x21,
    ITG3205_REG_GYRO_Z_OUT_L = 0x22,
    ITG3205_REG_PWR_MGM = 0x3E
} register_map_t;

typedef struct itg3205_power_config{
    bool reset; /* Reset the device internal registers */
    bool sleep; /* Put the device in sleep mode (low power consumption mode) */
    bool stby_x; /* Put the gyro x in standby mode */
    bool stby_y; /* Put the gyro y in standby mode */
    bool stby_z; /* Put the gyro z in standby mode */
    unsigned int clk_sel : 3; /* 3 bit unsigned int for clock select */
} itg3205_power_config_t;


esp_err_t itg3205_i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *itg3205_handle);
esp_err_t itg3205_read_reg(i2c_master_dev_handle_t *itg3205_handle, register_map_t reg_addr, uint8_t *data, uint32_t len);
esp_err_t itg3205_write_reg(i2c_master_dev_handle_t *itg3205_handle, register_map_t reg_addr, const uint8_t *data);

esp_err_t itg3205_set_power(i2c_master_dev_handle_t *itg3205_handle, itg3205_power_config_t *config);
esp_err_t itg3205_read_temp(i2c_master_dev_handle_t *itg3205_handle, int16_t *raw_temp);
esp_err_t itg3205_read_gyro_x(i2c_master_dev_handle_t *itg3205_handle, int16_t *gyro_x);
esp_err_t itg3205_read_gyro_y(i2c_master_dev_handle_t *itg3205_handle, int16_t *gyro_y);
esp_err_t itg3205_read_gyro_z(i2c_master_dev_handle_t *itg3205_handle, int16_t *gyro_z);
float itg3205_raw_temp_to_c(int16_t raw_temp);
float itg3205_raw_to_dps(int16_t raw);


/* TODO: Add calibration - ZRO (zero rate output) bias values calibration*/