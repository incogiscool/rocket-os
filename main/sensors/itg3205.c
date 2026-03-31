#include "gpio/i2c.h"
#include "itg3205.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

char *TAG = "ITG3205";

esp_err_t itg3205_i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *itg3205_handle) {

    i2c_device_config_t dev_config = {
            .device_address = ITG3205_ADDR,
            .dev_addr_length = I2C_ADDR_BIT_7,
            .scl_speed_hz = I2C_SCL_SPEED_HZ
    };

    return i2c_master_bus_add_device(*bus_handle, &dev_config, itg3205_handle);
}

esp_err_t itg3205_read_reg(i2c_master_dev_handle_t *itg3205_handle, register_map_t reg_addr, uint8_t *data, uint32_t len) {
    return i2c_master_transmit_receive(*itg3205_handle, (uint8_t *) &reg_addr, 1, data, len, ITG3205_I2C_TIMEOUT_MS);
}

esp_err_t itg3205_write_reg(i2c_master_dev_handle_t *itg3205_handle, register_map_t reg_addr, const uint8_t *data) {
    uint8_t buf[2] = { (uint8_t) reg_addr, *data };

    return i2c_master_transmit(*itg3205_handle, buf, 2, ITG3205_I2C_TIMEOUT_MS);
}


esp_err_t itg3205_set_power(i2c_master_dev_handle_t *itg3205_handle, itg3205_power_config_t *config) {
    uint8_t data = (config->reset << 7) |
                   (config->sleep << 6) | 
                   (config->stby_x << 5) | 
                   (config->stby_y << 4) | 
                   (config->stby_z << 3) | 
                   (config->clk_sel);

    esp_err_t res = itg3205_write_reg(itg3205_handle, ITG3205_REG_PWR_MGM, &data);

    vTaskDelay(pdMS_TO_TICKS(20)); /* 20ms delay for chip to reset/config */

    return res;
}


// esp_err_t itg3205_read_temp(i2c_master_dev_handle_t *itg3205_handle, int16_t *data) {
//     uint8_t high;
//     uint8_t low;

//     esp_err_t read_h_status = itg3205_read_reg(itg3205_handle, ITG3205_REG_TEMP_OUT_H, &high, 1);
//     esp_err_t read_l_status = itg3205_read_reg(itg3205_handle, ITG3205_REG_TEMP_OUT_L, &low, 1);

//     if (read_h_status != ESP_OK) {
//         ESP_LOGE(TAG, "Reading high byte failed.");

//         return ESP_FAIL;
//     };

//     if (read_l_status != ESP_OK) {
//         ESP_LOGE(TAG, "Reading low byte failed.");

//         return ESP_FAIL;
//     };

//     *data =  (high << 8) | low;

//     return ESP_OK;
// }


esp_err_t itg3205_read_temp(i2c_master_dev_handle_t *itg3205_handle, int16_t *raw_temp) {
    uint8_t buf[2];

    esp_err_t status = itg3205_read_reg(itg3205_handle, ITG3205_REG_TEMP_OUT_H, buf, 2); /* Having a buff length of 2 instead of 1 tells the chip to also get the low. This is called a burst read. */

    if (status != ESP_OK) {
        ESP_LOGE(TAG, "Reading temperature failed.");

        return ESP_FAIL;
    }

    *raw_temp = (int16_t) ((buf[0] << 8) | buf[1]); /* Shift high bit 8 bits right and then OR (add) with low bits */

    return ESP_OK;
};

float itg3205_raw_temp_to_c(int16_t raw_temp) {
    return (float) 35.0f + ((float) raw_temp + 13200.0f) / 280.0f; /* Analog to digital converter count -> celcius formula */
}

esp_err_t itg3205_read_gyro_x(i2c_master_dev_handle_t *itg3205_handle, int16_t *gyro_x) {
    uint8_t buf[2];

    esp_err_t status = itg3205_read_reg(itg3205_handle, ITG3205_REG_GYRO_X_OUT_H, buf, 2); /* Having a buff length of 2 instead of 1 tells the chip to also get the low. This is called a burst read. */

    if (status != ESP_OK) {
        ESP_LOGE(TAG, "Reading gyro x failed.");

        return ESP_FAIL;
    }

    *gyro_x = (int16_t) ((buf[0] << 8) | buf[1]); /* Shift high bit 8 bits right and then OR (add) with low bits */

    return ESP_OK;
};

esp_err_t itg3205_read_gyro_y(i2c_master_dev_handle_t *itg3205_handle, int16_t *gyro_y) {
    uint8_t buf[2];

    esp_err_t status = itg3205_read_reg(itg3205_handle, ITG3205_REG_GYRO_Y_OUT_H, buf, 2);

    if (status != ESP_OK) {
        ESP_LOGE(TAG, "Reading gyro y failed.");

        return ESP_FAIL;
    }

    *gyro_y = (int16_t) ((buf[0] << 8) | buf[1]);

    return ESP_OK;
};

esp_err_t itg3205_read_gyro_z(i2c_master_dev_handle_t *itg3205_handle, int16_t *gyro_z) {
    uint8_t buf[2];

    esp_err_t status = itg3205_read_reg(itg3205_handle, ITG3205_REG_GYRO_Z_OUT_H, buf, 2);

    if (status != ESP_OK) {
        ESP_LOGE(TAG, "Reading gyro z failed.");

        return ESP_FAIL;
    }

    *gyro_z = (int16_t) ((buf[0] << 8) | buf[1]);

    return ESP_OK;
};

/* Convert raw adc count value to degrees per second */
float itg3205_raw_to_dps(int16_t raw) {
    return (float) raw / 14.375f;
}