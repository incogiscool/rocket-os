#pragma once

#include <stdint.h>
#include "parse/nmea.h"

#define NEO6M_UART_RX_PIN 26 /* GPS TX -> Pin 26 */
#define NEO6M_UART_TX_PIN 27 /* GPS RX -> Pin 27 */
#define NEO6M_UART_RTS_PIN  (UART_PIN_NO_CHANGE)
#define NEO6M_UART_CTS_PIN  (UART_PIN_NO_CHANGE)
#define NEO6M_UART_BAUD_RATE 9600
#define NEO6M_UART_BUF_SIZE (1024) /* We can make this smaller? */

void neo6m_init();
int neo6m_read(uint8_t *buffer);
nmea_gps_data_t neo6m_read_parsed();
