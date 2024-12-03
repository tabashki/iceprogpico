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
    FRAME_CMD_POWER_UP = 0xAB,      // Seemingly unused
    FRAME_CMD_POWER_DOWN = 0xB9,    // Seemingly unused
    FRAME_CMD_WRITE_ENABLE = 0x06,  // Seemingly unused
    FRAME_CMD_BULK_ERASE = 0xC7,
    FRAME_CMD_SECTOR_ERASE = 0xD8,
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
    FRAME_ERROR_UNIMPLEMENTED = -999,
    FRAME_ERROR_CHECKSUM = -1000,
    FRAME_ERROR_ESCAPE_SEQUENCE = -1001,
    FRAME_ERROR_BUFFER_2BIG = -1002,
    FRAME_ERROR_BUFFER_2SMALL = -1003,
    FRAME_ERROR_VERIFY_FAILED = -1004,
};

#define MIN_FRAME_SIZE 2    // Excluding terminator
#define MAX_FRAME_SIZE 512
#define MAX_FRAME_PAYLOAD_SIZE (MAX_FRAME_SIZE - 3) // -3 bytes for framing

// !!! DANGER ZONE !!!
// Used to enable/disable destructive (write/erase) operations for development
#ifndef WITH_DESTRUCTIVE_CMDS
#define WITH_DESTRUCTIVE_CMDS 1
#endif


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

int handle_cmd_read_id() {
    spi_flash_power_up();
    uint8_t jedec_id[3];
    int result = spi_flash_read_jedec_id(jedec_id, sizeof(jedec_id));
    if (result < PICO_OK) {
        LOG_ERROR("Failed to read JEDEC ID: %d", result);
        return result;
    }
    if ((size_t)result != sizeof(jedec_id)) {
        LOG_ERROR("Unexpected JEDEC ID size: %d", result);
        return FRAME_ERROR_BUFFER_2SMALL;
    }
    spi_flash_power_down();

    LOG_INFO("Flash returned JEDEC ID: %02X-%02X-%02X",
             jedec_id[0], jedec_id[1], jedec_id[2]);

    result = encode_and_send_frame(FRAME_CMD_READ_ID, jedec_id, sizeof(jedec_id));
    LOG_COND_ERROR(result < FRAME_OK,
                    "Failed to encode/send READ_ID frame: %d", result);
    return result;
}

int handle_cmd_read_page(const uint16_t page_addr) {
    uint8_t response[SPI_FLASH_PAGE_SIZE + 2];
    response[0] = (uint8_t)(page_addr >> 8);
    response[1] = (uint8_t)page_addr;

    spi_flash_power_up();
    int result = spi_flash_read_page(page_addr, response + 2);
    spi_flash_power_down();

    if (result < PICO_OK) {
        LOG_ERROR("Failed to read flash page 0x%X, result: %d",
                  page_addr, result);
        return result;
    }
    bool page_is_zeroed = true;
    for (uint16_t i = 0; i < SPI_FLASH_PAGE_SIZE; i++) {
        if (response[i + 2] != 0) {
            page_is_zeroed = false;
            break;
        }
    }
    if (page_is_zeroed) {
        result = encode_and_send_frame(FRAME_CMD_EMPTY,
                                       response, 2);
        LOG_COND_ERROR(result < FRAME_OK,
                       "Failed to send EMPTY response: %d", result);
    } else {
        result = encode_and_send_frame(FRAME_CMD_READ_PAGE,
                                       response, sizeof(response));
        LOG_COND_ERROR(result < FRAME_OK,
                       "Failed to send READ_PAGE response: %d", result);
    }
    return (result < FRAME_OK) ? result : FRAME_OK;
}

int handle_cmd_read_all() {
    // The original iceprogduino firmware AND the PC-side command line tool
    // both have this hardcoded max page count, so we must respec it.
    const uint32_t max_pages = 0x2000;

    uint32_t pages = spi_flash_read_size() / SPI_FLASH_PAGE_SIZE;
    pages = MIN(pages, max_pages);

    int result = FRAME_OK;
    for (uint32_t page_addr = 0; page_addr < pages; page_addr++) {
        result = handle_cmd_read_page(page_addr);
        if (result < FRAME_OK) {
            return result;
        }
    }
    return FRAME_OK;
}

int handle_cmd_bulk_erase() {
#if WITH_DESTRUCTIVE_CMDS
    // TODO: Implement this
    return FRAME_ERROR_UNIMPLEMENTED;
#else
    return FRAME_OK;
#endif // WITH_DESTRUCTIVE_CMDS
}

int handle_cmd_sector_erase() {
#if ENABLE_DESTRUCTIVE_CMDS
    // TODO: Implement this
    return FRAME_ERROR_UNIMPLEMENTED;
#else
    return FRAME_OK;
#endif // WITH_DESTRUCTIVE_CMDS
}

int handle_cmd_program_page(const uint8_t* payload, size_t payload_size) {
#if WITH_DESTRUCTIVE_CMDS
    if (payload_size < (2 + SPI_FLASH_PAGE_SIZE)) {
        LOG_ERROR("PROGRAM_PAGE payload too small: %llu", payload_size);
        return FRAME_ERROR_BUFFER_2SMALL;
    }

    const uint16_t page_addr = ((uint16_t)payload[0]) << 8 | payload[1];

    spi_flash_power_up();
    int result = spi_flash_write_page(page_addr, payload + 2);
    spi_flash_power_down();

    if (result < PICO_OK) {
        LOG_ERROR("Failed to write flash page 0x%X, result: %d",
                  page_addr, result);
        return result;
    }

    uint8_t read_back[SPI_FLASH_PAGE_SIZE];
    spi_flash_power_up();
    result = spi_flash_read_page(page_addr, read_back);
    spi_flash_power_down();

    if (result < PICO_OK) {
        LOG_ERROR("Failed to read back page 0x%X, result: %d",
                  page_addr, result);
        return result;
    }
    for (size_t i = 0; i < sizeof(read_back); i++) {
        const uint8_t src = payload[i+2];
        const uint8_t dest = read_back[i];
        if (src != dest) {
            size_t byte_addr = page_addr * SPI_FLASH_PAGE_SIZE + i;
            LOG_ERROR("Verify failed at 0x%X: expected 0x%02X, was 0x%02X",
                      byte_addr, src, dest);
            return FRAME_ERROR_VERIFY_FAILED;
        }
    }

    result = encode_and_send_frame(FRAME_CMD_READY, NULL, 0);
    return (result < FRAME_OK) ? result : FRAME_OK;
#else
    return encode_and_send_frame(FRAME_CMD_READY, NULL, 0);;
#endif // WITH_DESTRUCTIVE_CMDS
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

    // Turn on status LED during frame handlig
    gpio_put(PIN_LED, 1);
    // Hold the iCE40 in reset while processing programmer request
    gpio_put(PIN_RESET, 0);

    const uint8_t frame_cmd = frame_data[0];
    const uint8_t* payload = (frame_data + 1);
    const size_t payload_size = (size_t)result - 1;

    switch(frame_cmd) {
#if WITH_LOGGING
    case FRAME_CMD_PRIVATE_DUMP_LOG:
        log_sink_to_stdio();
        result = FRAME_OK;
        break;
#endif // WITH_LOGGING

    case FRAME_CMD_READ_ID:
        result = handle_cmd_read_id();
        break;

    case FRAME_CMD_READ_PAGE: {
            if (payload_size < 2) {
                LOG_ERROR("READ_PAGE payload too small: %d", result);
                result = FRAME_ERROR_BUFFER_2SMALL;
                break;
            }
            const uint16_t page_addr = ((uint16_t)payload[0]) << 8 | payload[1];
            result = handle_cmd_read_page(page_addr);
        }
        break;

    case FRAME_CMD_READ_ALL:
        result = handle_cmd_read_all();
        break;

    case FRAME_CMD_SECTOR_ERASE:
        result = handle_cmd_sector_erase();
        break;

    case FRAME_CMD_PROGRAM_PAGE:
        result = handle_cmd_program_page(payload, payload_size);
        break;

    // TOOD: Decide whether to even implement these
    case FRAME_CMD_POWER_UP:
    case FRAME_CMD_POWER_DOWN:
    case FRAME_CMD_WRITE_ENABLE:
        result = FRAME_ERROR_UNIMPLEMENTED;
        break;

    default:
        LOG_WARN("Dropping unknown command: 0x%02X", frame_cmd);
    }

    if (result < FRAME_OK) {
        LOG_ERROR("Command handler for: 0x%02X, returned error: %d",
                  frame_cmd, result);
    }
    // Release the iCE40 from reset
    gpio_put(PIN_RESET, 1);
    // Turn off status LED when finished
    gpio_put(PIN_LED, 0);
}


// Setup and Main
//================

void setup_io() {
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);

    // SPI initialisation, clock at 1MHz
    //-----------------------------------
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    // Initialize all other GPIOs
    //----------------------------
    gpio_init(PIN_CDONE);
    gpio_init(PIN_RESET);
    gpio_init(PIN_LED);

    gpio_set_dir(PIN_CDONE, GPIO_IN);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    gpio_set_dir(PIN_LED,   GPIO_OUT);

    // iCE40 Reset is active-low, drive it high inititally
    gpio_put(PIN_RESET, 1);
    // Set status LED to off initially
    gpio_put(PIN_LED, 0);

    LOG_INFO("Initialized I/Os");
}

#define _STRINGIFY(x) #x
#define BI_DECL_BUILD_MACRO_STATE(m) \
    bi_decl(bi_program_build_attribute(#m "=" _STRINGIFY(m)))

int main() {
    // Metadata for picotool
    bi_decl(bi_program_description("Iceprogduino port for the Raspberry Pi Pico"));
    bi_decl(bi_program_url("https://github.com/tabashki/iceprogpico"));
    BI_DECL_BUILD_MACRO_STATE(WITH_LOGGING);
    BI_DECL_BUILD_MACRO_STATE(WITH_DESTRUCTIVE_CMDS);

    bi_decl(bi_1pin_with_name(PIN_RESET, "RESET"));
    bi_decl(bi_1pin_with_name(PIN_CDONE, "CDONE"));
    bi_decl(bi_1pin_with_name(PIN_LED, "LED"));
    bi_decl(bi_3pins_with_func(PIN_SCK, PIN_MOSI, PIN_MISO, GPIO_FUNC_SPI));
    bi_decl(bi_1pin_with_name(PIN_CS, "SPI CS#"));

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
