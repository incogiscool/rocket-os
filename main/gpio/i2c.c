#include "i2c.h"

esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle) {
    i2c_master_bus_config_t config = {
        .i2c_port = I2C_NUM_0, /* Initalize on i2c port 0 */
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    return i2c_new_master_bus(&config, bus_handle);
}

