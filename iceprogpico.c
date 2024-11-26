/**********************************************************
 * iceprogpico - version 0.1
 * Copyright (c) 2024 - Ivan Tabashki
 *
 * Based on the following projects:
 *   iceprog by Chris B. at Olimex Ltd.
 *   pico-serprog by Thomas Roth at stacksmashing.net
 *   spi_flash (pico-example) by Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: MIT
 **********************************************************/
#include <pico/error.h>
#include <pico/stdio.h>
#include <pico/stdio_usb.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/time.h>
#include <stdint.h>
#include <stdio.h>

#include "spi.h"
#include "spi_flash.h"
#include "log.h"

// Other pins
//------------
#define PIN_LED PICO_DEFAULT_LED_PIN
#define PIN_CDONE 0 // iCE40 Configuration Done signal
#define PIN_RESET 1 // iCE40 Reset (active-low)

// Serial programming frame constants
//------------------------------------
enum iceprog_frame_consts {
    // Iceprogduino serial programming commands
    FRAME_CMD_READ_ID = 0x9F,
    FRAME_CMD_POWER_UP = 0xAB,
    FRAME_CMD_POWER_DOWN = 0xB9,
    FRAME_CMD_WRITE_ENABLE = 0x06,
    FRAME_CMD_BULK_ERASE = 0xC7,
    FRAME_CMD_SECURE_ERASE = 0xD8,
    FRAME_CMD_PROGRAM_PAGE = 0x02,
    FRAME_CMD_READ_PAGE = 0x03,
    FRAME_CMD_READ_ALL = 0x83,
    FRAME_CMD_ERROR = 0xEE,
    FRAME_CMD_READY = 0x44,
    FRAME_CMD_EMPTY = 0x45,

    // Private debug command
    FRAME_CMD_PRIVATE_DUMP_LOG = 0xF0,

    FRAME_END = 0xC0,
    FRAME_ESC = 0xDB,
    FRAME_ESCAPED_END = 0xDC,
    FRAME_ESCAPED_ESC = 0xDD,
};

// Additional error codes
//------------------------
enum frame_error_codes {
    FRAME_OK = PICO_OK,
    FRAME_ERROR_CHECKSUM = -1000,
    FRAME_ERROR_ESCAPE_SEQUENCE = -1001,
    FRAME_ERROR_BUFFER_2BIG = -1002,
    FRAME_ERROR_BUFFER_2SMALL = -1003,
};

#define MIN_FRAME_SIZE 2    // Excluding terminator
#define MAX_FRAME_SIZE 512
#define MAX_FRAME_PAYLOAD_SIZE (MAX_FRAME_SIZE - 3) // -3 bytes for framing


// Serial Communication Helpers
//==============================

// Rough equivalent of Arduino Serial.readBytesUntil
// Returns:
//   The amount of bytes read (excluding the terminator character),
//   or PICO_ERROR_TIMEOUT if a timeout occurred while attempting to read.
int stdio_read_bytes_until(uint8_t terminator, uint8_t* buf_out, size_t buf_size) {
    const uint32_t timeout_us = 50 * 1000; // 50 ms
    size_t n = 0;

    while (n < buf_size) {
        int result = stdio_getchar_timeout_us(timeout_us);
        if (result < PICO_OK) {
            return result; // Pass thru error code (likely PICO_ERROR_TIMEOUT)
        }
        uint8_t b = (uint8_t)result;
        if (b == terminator) {
            return (int)n; // Found terminator, done reading
        }
        buf_out[n++] = b;
    }

    return (int)n; // Reached capacity of supplied buffer
}

// Decodes a serial frame sent from the PC into a buffer, skipping the FRAME_END
// bytes as well as decoding the following escape sequences:
//   <FRAME_ESC><FRAME_ESCAPED_END>  ->  0xC0 (FRAME_END)
//   <FRAME_ESC><FRAME_ESCAPED_ESC>  ->  0xDB (FRAME_ESC)
// Returns:
//   The size of the decoded frame or FRAME_CHECKSUM_ERROR
int decode_frame(const uint8_t* rx_data, size_t rx_size, uint8_t* dest,
                 size_t dest_size) {
    uint8_t frame_checksum = 0;
    size_t rx_i = 0;
    size_t dst_i = 0;

    if (rx_size < MIN_FRAME_SIZE) {
        return FRAME_ERROR_BUFFER_2SMALL;
    }
    if (rx_data[0] == FRAME_END) {
        rx_i++;
    }

    while (rx_i < rx_size) {
        uint8_t b = rx_data[rx_i++];
        if (b == FRAME_END) {
            break;
        }
        if (b == FRAME_ESC) {
            // Try to read ahead to decode escape sequence
            if (rx_i >= rx_size) {
                LOG_ERROR("ESC sequence underflow, buf len: %lu", rx_size);
                return FRAME_ERROR_ESCAPE_SEQUENCE;
            }
            b = rx_data[rx_i++];
            if (b == FRAME_ESCAPED_END) {
                b = (uint8_t)FRAME_END;
            } else if (b == FRAME_ESCAPED_ESC) {
                b = (uint8_t)FRAME_ESC;
            } else {
                LOG_ERROR("Unrecognized ESC pair: 0x%02X 0x%02X",
                          FRAME_ESC, b);
                return FRAME_ERROR_ESCAPE_SEQUENCE;
            }
        }
        if (dst_i >= dest_size) {
            LOG_ERROR("Decode overflow, rx_size: %lu, dest_size: %lu",
                      rx_size, dest_size);
            return FRAME_ERROR_BUFFER_2BIG;
        }
        dest[dst_i++] = b;
        frame_checksum += b;
    }
    if (frame_checksum != 0xFF) {
        LOG_ERROR("Expected 0xFF checksum, was: 0x%02X", frame_checksum);
        return FRAME_ERROR_CHECKSUM;
    }
    return (int)dst_i;
}

int encode_frame(uint8_t frame_cmd, const uint8_t* payload, size_t payload_size,
                 uint8_t* dest, size_t dest_size) {
    // At minimum destination must be large enough to contain payload +3 bytes
    // for header and terminating FRAME_END. Actual encoded frame size may be
    // larger from escape sequence encoding
    if (dest_size < (MIN_FRAME_SIZE + 1)) {
        return FRAME_ERROR_BUFFER_2SMALL;
    }
    if (payload_size > MAX_FRAME_PAYLOAD_SIZE) {
        LOG_ERROR("Payload length above limit: %lu", payload_size);
        return FRAME_ERROR_BUFFER_2BIG;
    }
    uint8_t frame_checksum = frame_cmd;
    size_t n = 0;
    dest[n++] = FRAME_END;
    dest[n++] = frame_cmd;

    size_t i = 0;
    while (i < payload_size && n < dest_size) {
        uint8_t p = payload[i++];
        frame_checksum += p;

        bool needs_escape = (p == FRAME_END || p == FRAME_ESC);
        if (needs_escape) {
            if ((n+1) >= dest_size) {
                LOG_ERROR("ESC sequence overflow, dest_size: %lu", dest_size);
                return FRAME_ERROR_BUFFER_2BIG;
            }
            uint8_t escaped = 0;
            switch (p)
            {
            case FRAME_END:
                escaped = FRAME_ESCAPED_END;
                break;
            case FRAME_ESC:
                escaped = FRAME_ESCAPED_ESC;
                break;
            default:
                LOG_ERROR("Tried to ESC non-escapable char: 0x%02X", p);
                return FRAME_ERROR_ESCAPE_SEQUENCE;
            }
            dest[n++] = FRAME_ESC;
            dest[n++] = escaped;
        } else {
            dest[n++] = p;
        }
    }
    if ((n + 2) >= dest_size) {
        LOG_ERROR("Dest too small to write FRAME_END, size: %lu", dest_size);
        return FRAME_ERROR_BUFFER_2SMALL;
    }
    dest[n++] = (0xFF - frame_checksum);
    dest[n++] = FRAME_END;

    return (int)n;
}

int recieve_and_decode_frame(uint8_t* dest_data, size_t dest_size) {
    uint8_t rx_data[MAX_FRAME_SIZE];
    int result = stdio_read_bytes_until(FRAME_END, rx_data, sizeof(rx_data));
    if (result < FRAME_OK) {
        LOG_COND_ERROR(result != PICO_ERROR_TIMEOUT,
                       "Failed to recieve frame, result: %d", result);
        return result;
    }
    if (result < MIN_FRAME_SIZE) {
        LOG_ERROR("Recieved frame too small: %d", result);
        return FRAME_ERROR_BUFFER_2SMALL;
    }
    LOG_INFO("Recieved raw frame, len: %d", result);

    return decode_frame(rx_data, (size_t)result, dest_data, dest_size);
}

int encode_and_send_frame(uint8_t frame_cmd, const uint8_t* payload,
                          size_t payload_size) {
    uint8_t frame[MAX_FRAME_SIZE];
    int result = encode_frame(frame_cmd, payload, payload_size,
                              frame, sizeof(frame));
    if (result < FRAME_OK) {
        LOG_ERROR("Failed to encode frame: %d", result);
        return result;
    }
    return stdio_put_string(frame, result, false, false);
}


// Command Handlers
//==================

void handle_cmd_read_id() {
    spi_flash_power_up();
    uint8_t jedec_id[3];
    int result = spi_flash_read_jedec_id(jedec_id, sizeof(jedec_id));
    if (result < 0 || (size_t)result != sizeof(jedec_id)) {
        LOG_ERROR("Unexpected JEDEC ID size: %d", result);
        return; // ID read failed for some reason
    }
    spi_flash_power_down();

    LOG_INFO("Flash returned JEDEC ID: %02X-%02X-%02X",
             jedec_id[0], jedec_id[1], jedec_id[2]);

    result = encode_and_send_frame(FRAME_CMD_READ_ID, jedec_id, sizeof(jedec_id));
    if (result < PICO_OK) {
        LOG_ERROR("Failed to encode/send READ_ID frame: %d", result);
    }
}

void handle_cmd_read_page(const uint16_t page_addr) {
    uint8_t response[SPI_FLASH_PAGE_SIZE + 2];
    response[0] = (uint8_t)(page_addr >> 8);
    response[1] = (uint8_t)page_addr;

    spi_flash_power_up();
    int result = spi_flash_read_page(page_addr, response + 2);
    spi_flash_power_down();

    if (result < PICO_OK) {
        LOG_ERROR("Failed to read flash page 0x%X, result: %d",
                  page_addr, result);
        return;
    }
    bool page_is_zeroed = true;
    for (uint16_t i = 0; i < SPI_FLASH_PAGE_SIZE; i++) {
        if (response[i + 2] != 0) {
            page_is_zeroed = false;
            break;
        }
    }
    if (page_is_zeroed) {
        result = encode_and_send_frame(FRAME_CMD_EMPTY, response, 2);
        if (result < FRAME_OK) {
            LOG_ERROR("Failed to send EMPTY response: %d", result);
            return;
        }
    } else {
        result = encode_and_send_frame(FRAME_CMD_READ_PAGE, response, sizeof(response));
        if (result < FRAME_OK) {
            LOG_ERROR("Failed to send READ_PAGE response: %d", result);
            return;
        }
    }
}

void prog_loop() {
    if (!stdio_usb_connected()) {
        return;
    }

    uint8_t frame_data[MAX_FRAME_SIZE];
    int result = recieve_and_decode_frame(frame_data, sizeof(frame_data));
    if (result < FRAME_OK) {
        LOG_COND_ERROR(result != PICO_ERROR_TIMEOUT,
                       "Failed to recieve/decode frame: %d", result);
        return;
    }
    LOG_INFO("Recieved frame, len: %d, data[0]: 0x%02X", result, frame_data[0]);

    // Hold the iCE40 in reset while processing programmer request
    gpio_put(PIN_RESET, 0);

    const uint8_t frame_cmd = frame_data[0];
    const uint8_t* payload = (frame_data + 1);
    const size_t payload_size = (size_t)result - 1;

    switch(frame_cmd) {
    case FRAME_CMD_PRIVATE_DUMP_LOG:
        log_sink_to_stdio();
        break;
    case FRAME_CMD_READ_ID:
        handle_cmd_read_id();
        break;
    case FRAME_CMD_READ_PAGE: {
            if (payload_size < 2) {
                LOG_ERROR("READ_PAGE payload too small: %d", result);
                break;
            }
            const uint16_t page_addr = ((uint16_t)payload[0]) << 16 | payload[1];
            handle_cmd_read_page(page_addr);
        }
        break;
    // TODO: Implement all other commands
    default:
        LOG_ERROR("Unrecognized command: 0x%02X", frame_cmd);
    }

    // Release the iCE40 from reset
    gpio_put(PIN_RESET, 1);
}


// Setup and Main
//================

void setup_io() {
    stdio_init_all();

    // SPI initialisation, clock at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    // Initialize all other GPIOs
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1);

    gpio_init(PIN_CDONE);
    gpio_set_dir(PIN_CDONE, GPIO_IN);

    // iCE40 Reset is active-low, drive it high inititally
    gpio_init(PIN_RESET);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    gpio_put(PIN_RESET, 1);

    stdio_set_translate_crlf(&stdio_usb, false);

    LOG_INFO("Initialized I/Os");
}

int main() {
    // Metadata for picotool
    bi_decl(bi_program_description("Iceprogduino port for the Raspberry Pi Pico"));
    bi_decl(bi_program_url("https://github.com/tabashki/iceprogpico"));
    bi_decl(bi_1pin_with_name(PIN_RESET, "RESET"));
    bi_decl(bi_1pin_with_name(PIN_CDONE, "CDONE"));
    bi_decl(bi_1pin_with_name(PIN_LED, "LED"));
    bi_decl(bi_1pin_with_name(PIN_SCK, "SCK"));
    bi_decl(bi_1pin_with_name(PIN_MOSI, "MOSI"));
    bi_decl(bi_1pin_with_name(PIN_MISO, "MISO"));
    bi_decl(bi_1pin_with_name(PIN_CS, "CS#"));

    LOG_INFO("Booting up...");
    setup_io();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    LOG_INFO("USB connection established");

    while (true) {
        // TODO: Support bridge mode
        prog_loop();
    }
    return 0;
}
