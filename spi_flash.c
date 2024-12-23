#include <hardware/spi.h>
#include <pico/error.h>
#include <pico/time.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "spi_flash.h"
#include "spi.h"

// SPI Flash communication library
//=================================
// Based on the Arduino Winbond SPIFlash Library by Prajwal Bhattaram.

enum flash_commands {
    CMD_MANID = 0x90,
    CMD_PAGEPROG = 0x02,
    CMD_READDATA = 0x03,
    CMD_FASTREAD = 0x0B,
    CMD_WRITEDISABLE = 0x04,
    CMD_READSTAT1 = 0x05,
    CMD_READSTAT2 = 0x35,
    CMD_WRITEENABLE = 0x06,
    CMD_SECTORERASE = 0x20,
    CMD_BLOCK32ERASE = 0x52,
    CMD_CHIPERASE = 0x60,
    CMD_SUSPEND = 0x75,
    CMD_ID = 0x90,
    CMD_RESUME = 0x7A,
    CMD_JEDECID = 0x9f,
    CMD_RELEASE = 0xAB,
    CMD_POWERDOWN = 0xB9,
    CMD_BLOCK64ERASE = 0xD8,
};

#define STATUS_BUSY      0x01
#define STATUS_WRTEN     0x02
#define STATUS_SUSPENDED 0x40
#define STATUS_SLEEPING  0xFF
#define DUMMY_BYTE       0xEE

#define WINBOND_MFG_ID 0xEF
#define ZETTA_MFG_ID   0xBA

uint32_t cached_flash_size = 0;

// Internal Functions
//--------------------

static void spi_flash_begin_cmd(uint8_t cmd) {
    cs_select(PIN_CS);
    spi_transfer(cmd);
}

static void spi_flash_end_cmd() {
    cs_deselect(PIN_CS);
    sleep_us(3);
}

static uint8_t spi_flash_read_stat1() {
    spi_flash_begin_cmd(CMD_READSTAT1);
    uint8_t stat1 = spi_transfer(0);
    spi_flash_end_cmd();
    return stat1;
}

static bool spi_flash_busy() {
    return (spi_flash_read_stat1() & STATUS_BUSY) != 0;
}

static bool spi_flash_page_addr_valid(uint16_t page_addr) {
    if (cached_flash_size == 0) {
        int result = spi_flash_read_size();
        if (result < PICO_OK) {
            return false; // Something really bad happened if we get here
        }
    }
    return (page_addr < (cached_flash_size / SPI_FLASH_PAGE_SIZE));
}

static bool spi_flash_write_enable() {
    spi_flash_begin_cmd(CMD_WRITEENABLE);
    spi_flash_end_cmd();

    uint8_t status = spi_flash_read_stat1();
    return (status & STATUS_WRTEN) != 0;
}

// Public Functions
//------------------

bool spi_flash_wait_idle(uint32_t timeout_ms) {
    absolute_time_t end_time = make_timeout_time_ms(timeout_ms);
    uint8_t stat1 = 0;
    do {
        stat1 = spi_flash_read_stat1();
        if (time_reached(end_time)) {
            return false;
        }
    }
    while ((stat1 & STATUS_BUSY) != 0);
    return true;
}

bool spi_flash_power_up() {
    spi_flash_begin_cmd(CMD_RELEASE);
    spi_flash_end_cmd();
    sleep_us(3); // Max release time according to datasheet

    return (spi_flash_read_stat1() != STATUS_SLEEPING);
}

bool spi_flash_power_down() {
    if (spi_flash_busy()) {
        return false;
    }

    spi_flash_begin_cmd(CMD_POWERDOWN);
    spi_flash_end_cmd();
    sleep_us(3); // Max power down time according to datasheet

    uint8_t status = 0;

    // Read the status a few times to ensure that any old value is shifted out
    for (int i = 0; i < 4; i++) {
        status = spi_flash_read_stat1();
    }
    return (status == STATUS_SLEEPING);
}

int spi_flash_read_jedec_id(uint8_t* dest, size_t size) {
    if (size < 3) {
        return PICO_ERROR_BUFFER_TOO_SMALL;
    }
    const uint32_t timeout_ms = 10;
    if (!spi_flash_wait_idle(timeout_ms)) {
        return PICO_ERROR_TIMEOUT;
    }

    size_t n = 0;
    spi_flash_begin_cmd(CMD_JEDECID);
    dest[n++] = spi_transfer(0); // Manufacturer ID;
    dest[n++] = spi_transfer(0); // Memory type
    dest[n++] = spi_transfer(0); // Memory capacity
    spi_flash_end_cmd();

    return (int)n;
}

int spi_flash_read_size() {
    uint8_t id[3];
    int result = spi_flash_read_jedec_id(id, sizeof(id));
    if (result < PICO_OK) {
        return result;
    }
    if (result < 3) {
        return PICO_ERROR_BUFFER_TOO_SMALL;
    }
    uint32_t shift = id[2];
    if (shift < 1 || shift > 31) {
        return PICO_ERROR_INVALID_DATA;
    }
    cached_flash_size = (1u << shift);
    return (int)cached_flash_size;
}

int spi_flash_read_page(uint16_t page_addr, uint8_t *dest_page) {
    if (!spi_flash_page_addr_valid(page_addr)) {
        return PICO_ERROR_INVALID_ADDRESS;
    }

    spi_flash_begin_cmd(CMD_READDATA);
    spi_transfer24(page_addr << 8);

    int result = spi_read_blocking(SPI_PORT, 0,
                                   dest_page, SPI_FLASH_PAGE_SIZE);
    spi_flash_end_cmd();
    return result;
}

int spi_flash_write_page(uint16_t page_addr, const uint8_t* src_page) {
    if (!spi_flash_page_addr_valid(page_addr)) {
        return PICO_ERROR_INVALID_ADDRESS;
    }
    if (!spi_flash_write_enable()) {
        return PICO_ERROR_NOT_PERMITTED;
    }

    spi_flash_begin_cmd(CMD_PAGEPROG);
    spi_transfer24(page_addr << 8);

    int result = spi_write_blocking(SPI_PORT, src_page, SPI_FLASH_PAGE_SIZE);
    spi_flash_end_cmd();
    return result;
}

int spi_flash_erase_block_64k(uint16_t page_addr) {
    if (!spi_flash_page_addr_valid(page_addr)) {
        return PICO_ERROR_INVALID_ADDRESS;
    }
    if (spi_flash_busy()) {
        return PICO_ERROR_RESOURCE_IN_USE;
    }
    if (!spi_flash_write_enable()) {
        return PICO_ERROR_NOT_PERMITTED;
    }

    spi_flash_begin_cmd(CMD_BLOCK64ERASE);
    spi_transfer24(page_addr << 8);
    spi_flash_end_cmd();

    // Typical Block 64K erase time is about 200ms, however it can vary up to
    // 2000ms for Winbond devices, and up to 3000ms for Zetta devices. To handle
    // the edge cases we use an exponential backoff in 200ms steps here
    for (int wait = 1; wait < 6; wait++) {
        const uint32_t timeout_ms = 200 * wait;
        if (spi_flash_wait_idle(timeout_ms)) {
            return PICO_OK;
        }
    }
    return PICO_ERROR_TIMEOUT;
}

int spi_flash_chip_erase() {
    if (spi_flash_busy()) {
        return PICO_ERROR_RESOURCE_IN_USE;
    }
    if (!spi_flash_write_enable()) {
        return PICO_ERROR_NOT_PERMITTED;
    }

    spi_flash_begin_cmd(CMD_CHIPERASE);
    spi_flash_end_cmd();

    // Typical Chip Erase time is 5000ms for Winbond devices, 7000ms for Zetta,
    // and potentially up to 20 seconds. Exponential backoff in -1s steps
    for (int wait = 0; wait < 6; wait++) {
        const uint32_t timeout_ms = 6000 - 1000 * wait;
        if (spi_flash_wait_idle(timeout_ms)) {
            return PICO_OK;
        }
    }
    return PICO_ERROR_TIMEOUT;
}