#pragma once

#include "spi.h"

/* Chip select pin for the radio module (NSS pin on module, active low) */
#define SX1262_SPI_CS_PIN 2

#define SX1262_NRST 21 /* Reset pin, resets the chip when low, put back to high to bring back. Used to put the chip in a known state */
#define SX1262_BUSY 16 /* Busy pin (can't perform any actions when this is high) */
#define SX1262_DI01 17 /* Digital interrupt pin 1, high when action is done (packet recieved, transmit done, timeout, etc.) */

#define SX1262_CRYSTAL_FREQUENCY 32000000 /* The crystal freqency the chip is running. Chip has many options, but for now will only use 32Mhz */

#define SX1262_TX_RX_TIMEOUT 5000000 /* Set tx/rx timeout, 5s in microseconds */


/* Command Opcodes */
#define SX1262_OP_GET_STATUS           0xC0
#define SX1262_OP_SET_STANDBY          0x80
#define SX1262_OP_SET_PACKET_TYPE      0x8A
#define SX1262_OP_SET_RF_FREQUENCY     0x86
#define SX1262_OP_SET_TX_PARAMS        0x8E
#define SX1262_OP_SET_MOD_PARAMS       0x8B
#define SX1262_OP_SET_PACKET_PARAMS    0x8C
#define SX1262_OP_SET_BUFFER_BASE_ADDR 0x8F
#define SX1262_OP_WRITE_BUFFER         0x0E
#define SX1262_OP_READ_BUFFER          0x1E
#define SX1262_OP_SET_DIO_IRQ_PARAMS   0x08
#define SX1262_OP_GET_IRQ_STATUS       0x12
#define SX1262_OP_CLEAR_IRQ_STATUS     0x02
#define SX1262_OP_SET_TX               0x83
#define SX1262_OP_SET_RX               0x82
#define SX1262_OP_GET_RX_BUFFER_STATUS 0x13

/* Standby modes */
#define SX1262_STDBY_RC    0x00
#define SX1262_STDBY_XOSC  0x01

/* Packet types */
#define SX1262_PACKET_TYPE_GFSK  0x00 /* Use Gaussian frequency shift keying, fast but shorter range */
#define SX1262_PACKET_TYPE_LORA  0x01 /* Use LoRa, slower but much longer range */
/* Not implementing FHSS packet type */

typedef struct {
    spi_device_handle_t device_handle;
} sx1262_t;

esp_err_t sx1262_init(sx1262_t *device);
esp_err_t sx1262_reset(sx1262_t *device);
void sx1262_wait_busy(void);
esp_err_t sx1262_get_status(sx1262_t *device, uint8_t *status);
esp_err_t sx1262_set_standby(sx1262_t *device, uint8_t mode);
esp_err_t sx1262_set_packet_type(sx1262_t *device, uint8_t type);
esp_err_t sx1262_set_rf_frequency(sx1262_t *device, uint32_t freq_hz);
esp_err_t sx1262_set_tx_params(sx1262_t *device, int8_t power, uint8_t ramp_time);
esp_err_t sx1262_set_mod_params_lora(sx1262_t *device, uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro);
esp_err_t sx1262_transmit(sx1262_t *device, uint8_t *data, uint8_t len);
esp_err_t sx1262_receive(sx1262_t *device, uint8_t *data, uint8_t *len);
esp_err_t sx1262_set_packet_params(sx1262_t *device, uint16_t preamble_len, uint8_t header_type, uint8_t payload_len, uint8_t crc, uint8_t invert_iq);
esp_err_t sx1262_set_dio_irq_params(sx1262_t *device);