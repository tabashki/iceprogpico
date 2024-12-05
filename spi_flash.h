
#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include <pico/stdlib.h>

// SPI Flash communication library
//=================================
// Based on the Arduino Winbond SPIFlash Library by Prajwal Bhattaram.

#define SPI_FLASH_PAGE_SIZE 0x100

bool spi_flash_power_up();

bool spi_flash_power_down();

int spi_flash_read_jedec_id(uint8_t* dest, size_t size);

// NOTE: Returns the size of the flash in bytes
int spi_flash_read_size();

// NOTE: Assumes `dest_page` buffer size is least `SPI_FLASH_PAGE_SIZE`
int spi_flash_read_page(uint16_t page_addr, uint8_t* dest_page);

// NOTE: Assumes `src_page` buffer size is least `SPI_FLASH_PAGE_SIZE`
int spi_flash_write_page(uint16_t page_addr, const uint8_t* src_page);

// Erase the whole 64K block that the given page is located in. Each block
// contains 256 pages, so for example:
//      Block 0 -> Page 0-255
//      Block 1 -> Page 256-511
//      ... And so on
int spi_flash_erase_block_64k(uint16_t page_addr);

int spi_flash_chip_erase();

#endif // SPI_FLASH_H
