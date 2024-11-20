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
#include <pico/stdio.h>
#include <pico/stdio_usb.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/time.h>
#include <stdio.h>

// SPI Defines
//-------------
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pin assignment is based on pico-serprog with additions from iceprogduino
#define SPI_PORT spi0
#define PIN_SCK  2
#define PIN_MOSI 3
#define PIN_MISO 4
#define PIN_CS   5

// Other pins
//------------
#define PIN_LED PICO_DEFAULT_LED_PIN
#define PIN_RESET 6
#define PIN_CDONE 7

//
// Copied from the spi_flash example, also used in pico-serprog
//
static inline void cs_select(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop"); // FIXME
}

static inline void cs_deselect(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop"); // FIXME
}

void prog_loop() {
    if (!stdio_usb_connected()) {
        return;
    }
    // TODO:
}

void setup_io() {
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
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
    gpio_put(PIN_LED, 0);

    gpio_init(PIN_CDONE);
    gpio_set_dir(PIN_CDONE, GPIO_IN);

    // iCE40 Reset is active-low, drive it high inititally
    gpio_init(PIN_RESET);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    gpio_put(PIN_RESET, 1);

    stdio_set_translate_crlf(&stdio_usb, false);
}

int main() {
    // Metadata for picotool
    bi_decl(bi_program_description("Iceprogduino port for the Raspberry Pi Pico"));
    bi_decl(bi_program_url("https://github.com/tabashki/iceprogpico"));
    bi_decl(bi_1pin_with_name(PIN_LED, "LED"));
    bi_decl(bi_1pin_with_name(PIN_SCK, "SCK"));
    bi_decl(bi_1pin_with_name(PIN_MISO, "MISO"));
    bi_decl(bi_1pin_with_name(PIN_MOSI, "MOSI"));
    bi_decl(bi_1pin_with_name(PIN_CS, "CS#"));
    bi_decl(bi_1pin_with_name(PIN_RESET, "RESET"));
    bi_decl(bi_1pin_with_name(PIN_CDONE, "CDONE"));

    setup_io();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    puts_raw("USB connection established!\n");

    while (true) {
        // TODO: Support bridge mode
        prog_loop();
    }

    return 0;
}
