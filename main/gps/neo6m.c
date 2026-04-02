#include "neo6m.h"
#include "driver/uart.h"
#include "parse/nmea.h"
#include <string.h>

void neo6m_init() {
    uart_config_t uart_config = {
        .baud_rate = NEO6M_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, NEO6M_UART_TX_PIN, NEO6M_UART_RX_PIN, NEO6M_UART_RTS_PIN, NEO6M_UART_CTS_PIN);
    uart_driver_install(UART_NUM_1, NEO6M_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
}

int neo6m_read(uint8_t *buffer) {
    int len = uart_read_bytes(UART_NUM_1, buffer, NEO6M_UART_BUF_SIZE - 1, pdMS_TO_TICKS(100));

    

    if (len > 0) {
        buffer[len] = '\0'; /* Null terminate the string */
        // Process the NMEA sentences in 'data'
    }

    return len;
}

nmea_gps_data_t neo6m_read_parsed() {
    uint8_t buffer[NEO6M_UART_BUF_SIZE];
    nmea_gps_data_t gps = {0};

    int len = neo6m_read(buffer);

    if (len > 0) {
        char *line = strtok((char *)buffer, "\r\n");
        while (line != NULL) {
            if (line[0] == '$') {
                parseNmea(line, &gps);
            }
            line = strtok(NULL, "\r\n");
        }
    }

    return gps;
}

/* NOTE: NMEA sentences are split up by \n, so we need to split up the data by line when parsing and check if starts with $*/

/*
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';
            // Split into individual NMEA sentences by newline
            char *line = strtok((char *)data, "\r\n");
            while (line != NULL) {
                if (line[0] == '$') {
                    parseNmea(line);
                }
                line = strtok(NULL, "\r\n");
            }
        }

        ESP_LOGI(TAG, "Length: %d", len);
        // Write data back to the UART
        // uart_write_bytes(UART_NUM_1, (const char *) data, len);
    }

*/