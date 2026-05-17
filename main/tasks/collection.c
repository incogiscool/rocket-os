#include "collection.h"
#include "transmission.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stddef.h>
#include "gpio/i2c.h"
#include "sensors/bme280_port.h"
#include "sensors/adxl345_port.h"
#include "sensors/itg3205.h"
#include "esp_log.h"

#define MEASUREMENT_TIME 200

static const char *TAG = "SENSOR_COLLECTION";

void vSensorCollectionTask(void *pvParameters) {
    QueueHandle_t xSensorTelemetryQueue = (QueueHandle_t)pvParameters;


    /* I2C Setup */
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t bme280_device_handle;   /* Barometer */
    i2c_master_dev_handle_t adxl345_device_handle;  /* Accelerometer */
    // i2c_master_dev_handle_t itg3205_device_handle;  /* Gyroscope */


    ESP_ERROR_CHECK(i2c_master_init(&bus_handle));

    /* Barometer Setup */
    struct bme280_settings settings;
    struct bme280_dev device;
    bme280_intf_ctx_t intf_ctx; /* BME280 library interface context */

    ESP_ERROR_CHECK(bme280_full_init(&bus_handle, &bme280_device_handle, &intf_ctx, &device));
    

    bme280_get_sensor_settings(&settings, &device);

    /* Configuring the over-sampling rate, filter coefficient and standby time */
    bme280_set_settings(&settings);

    bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &device);
    bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &device);
    

    /* Accelerometer Setup */
    adxl345_handle_t adxl345_handle;
    ESP_ERROR_CHECK(adxl345_full_init(&bus_handle, &adxl345_device_handle, &adxl345_handle));

    /* Configure ADXL345: +-16G range, 100Hz data rate, start measurement */
    adxl345_set_rate(&adxl345_handle, ADXL345_RATE_100);
    adxl345_set_range(&adxl345_handle, ADXL345_RANGE_16G);
    adxl345_set_measure(&adxl345_handle, ADXL345_BOOL_TRUE);


    /* Gyroscope Setup */
    // itg3205_i2c_init(&bus_handle, &itg3205_device_handle);
    // itg3205_power_config_t init_power_config = {
    //     .reset = 1, /* Reset registers in chip */
    //     .sleep = 0,
    //     .stby_x = 0,
    //     .stby_y = 0,
    //     .stby_z = 0,
    //     .clk_sel = 0x0
    // };

    // itg3205_set_power(&itg3205_device_handle, &init_power_config);

    /* TODO: Calibrate sensors based on boot (and altitude reading) */

    for (;;) {


        /* BME280 (Barometer) Read */
        uint32_t period = 100000; /* Measurement Period */
        bme280_cal_meas_delay(&period, &settings);
        struct bme280_data bme280_output;
        int8_t rslt = bme280_get_sensor_data(BME280_ALL, &bme280_output, &device);

        if (rslt != BME280_OK) {
            ESP_LOGE(TAG, "BME280 read failed.");
        }

        

        /* ADXL345 (Accelerometer) Read */
        int16_t accel_raw[3];
        float accel_g[3];
        uint16_t accel_len = 1; /* Length of 1 means the chip to bypass mode - only take one input per measurement. Maybe we should switch to stream mode (32 measurements) then avg? */
        uint8_t accel_rslt = adxl345_read(&adxl345_handle, (int16_t (*)[3])&accel_raw, (float (*)[3])&accel_g, &accel_len);

        if (accel_rslt != 0) {
            ESP_LOGE(TAG, "ADXL345 read failed.");
        }


        /* TODO: Filter and generate altitude & velocity based on accel and baro. Calculate time so that the LoRa module isn't a bottleneck and the vals fill up the queue. */
        /* Filter and make sure data is accurate and noise free. */
        /* Note we are targeting ~4hz transmission rate so we have 250ms to work with (A LOT) */

        sensor_telemetry_packet_t packet = {
            .acceleration = accel_raw[1] * -1, /* Based on orientation of the module - and multiply by -1 since its upside down. */
            .altitude = bme280_output_to_altitude(&bme280_output),
            .callsign_id = CALLSIGN_ID,
            .packet_type = TELEMETRY_PACKET_TYPE_SENSOR,
            .heading = 0, /* TODO: Magnetometer Heading */
            .seq = 0, /* TODO: Track sequence number */
            .speed = 0, /* TODO: Placeholder until calculated */
            .temperature = bme280_output.temperature,
            .timestamp_ms = 0 /* TODO: Track timestamp */
        };

        /* Send the data to the transmission queue */

        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_TIME));

        
        xQueueSendToBack(xSensorTelemetryQueue, &packet, pdTICKS_TO_MS(MEASUREMENT_TIME));
    };


    
    vTaskDelete( NULL );
}