
#ifndef SPI_H
#define SPI_H

#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <stdint.h>

// SPI defines and utility functions.
//====================================

// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pin assignment is based on pico-serprog with additions from iceprogduino
#define SPI_PORT spi0
#define PIN_SCK  2
#define PIN_MOSI 3
#define PIN_MISO 4
#define PIN_CS   5


// Copied from the spi_flash example, also used in pico-serprog
static inline void cs_select(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop"); // FIXME
}

// Copied from the spi_flash example, also used in pico-serprog
static inline void cs_deselect(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop"); // FIXME
}

// Simultaneously send and recieve an 8-bit value from SPI_PORT
static inline uint8_t spi_transfer(uint8_t send_byte) {
    uint8_t recv_byte = 0;
    spi_write_read_blocking(SPI_PORT, &send_byte,
                            &recv_byte, sizeof(uint8_t));
    return recv_byte;
}

// Simultaneously send and recieve a 16-bit value from SPI_PORT
// NOTE: Values are sent and recieved in big-endian order
static inline uint16_t spi_transfer16(uint16_t send_word) {
    uint8_t send_bytes[2] = {
        (uint8_t)(send_word >> 8), (uint8_t)send_word
    };
    uint8_t recv_bytes[2];
    spi_write_read_blocking(SPI_PORT, send_bytes,
                            recv_bytes, sizeof(uint16_t));
    return ((uint16_t)recv_bytes[0]) << 8 | recv_bytes[1];
}

// Simultaneously send and recieve a 24-bit value from SPI_PORT
// NOTE: Values are sent and recieved in big-endian order
static inline uint32_t spi_transfer24(uint32_t send_word) {
    uint8_t send_bytes[3] = {
        (uint8_t)(send_word >> 16),
        (uint8_t)(send_word >> 8),
        (uint8_t)send_word,
    };
    uint8_t recv_bytes[3];
    spi_write_read_blocking(SPI_PORT, send_bytes,
                            recv_bytes, 3);
    return ((uint32_t)recv_bytes[0]) << 16
         | ((uint32_t)recv_bytes[1]) << 8
         | recv_bytes[2];
}

#endif // SPI_H
