// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText 2023 Kristin Ebuengan
// SPDX-FileCopyrightText 2023 Melody Gill
// SPDX-FileCopyrightText 2023 Gabriel Marcano

/* 
* This is an edited file of main.c from https://github.com/gemarcano/redboard_lora_example
* which this repo was forked from
*
* Tests reading time from the RTC to reading/writing/erasing from the flash chip.
*/

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include <string.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include <spi.h>
#include <uart.h>
#include <flash.h>
#include <am1815.h>

/*
* Inializing the MCU and UART are the only things kept from the original file
*/
static struct uart uart;
static struct spi spi;
static struct flash flash;
static struct am1815 rtc;

int main(void)
{
	// Prepare MCU by init-ing clock, cache, and power level operation
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);
	am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
	am_hal_cachectrl_enable();
	am_bsp_low_power_init();
	am_hal_sysctrl_fpu_enable();
	am_hal_sysctrl_fpu_stacking_enable(true);

	// Init UART, registers with SDK printf
	uart_init(&uart, UART_INST0);

	am_hal_interrupt_master_enable();

	// Print the banner.
	am_util_stdio_terminal_clear();
	am_util_stdio_printf("Hello World!\r\n\r\n");

	// Initialize spi and select the CS channel
	spi_init(&spi, 0, 2000000u);
	spi_enable(&spi);

	// Initialize flash
	flash_init(&flash, &spi);

	// Initalize RTC
	am1815_init(&rtc, &spi);

	spi_chip_select(&spi, SPI_CS_0);

	// Test flash functions
	int size = 15;
	uint8_t buffer[size];		// this is 4x bigger than necessary
	// initialize buffer to all -1 (shouldn't be necessary to do this)
	for (int i = 0; i < size; i++) {
		buffer[i] = -1;
	}

	// print the data before write
	flash_read_data(&flash, 0x04, buffer, size);
	char* buf = buffer;
	am_util_stdio_printf("before write:\r\n");
	for (int i = 0; i < size; i++) {
		am_util_stdio_printf("%02X ", (int) buf[i]);
		am_util_delay_ms(10);
	}
	am_util_stdio_printf("\r\n");
	am_util_delay_ms(250);

	// print the flash ID to make sure the CS is connected correctly
	am_util_stdio_printf("flash ID: %02X\r\n", flash_read_id(&flash));

	// print the RTC ID to make sure the CS is connected correctly
	spi_chip_select(&spi, SPI_CS_3);
	am_util_delay_ms(250);
	am_util_stdio_printf("RTC ID: %02X\r\n", am1815_read_register(&rtc, 0x28));

	// write the RTC time to the flash chip
	struct timeval time = am1815_read_time(&rtc);
	// print the seconds from the RTC to make sure the time is correct
	am_util_stdio_printf("secs: %lld\r\n", time.tv_sec);
	uint64_t sec = (uint64_t)time.tv_sec;
	uint8_t* tmp = (uint8_t*)&sec;
	spi_chip_select(&spi, SPI_CS_0);
	am_util_delay_ms(250);

	// print the flash ID to make sure the CS is connected coorrectly
	am_util_stdio_printf("flash ID: %02X\r\n", flash_read_id(&flash));
	flash_page_program(&flash, 0x05, tmp, 8);

	// print flash data after write
	am_util_delay_ms(1000);
	flash_read_data(&flash, 0x04, buffer, size);
	buf = buffer;
	am_util_stdio_printf("after write:\r\n");
	for (int i = 0; i < size; i++) {
		am_util_stdio_printf("%02X ", (int) buf[i]);
		am_util_delay_ms(10);
	}
	am_util_stdio_printf("\r\n");

	// print the seconds again to make sure we are reading correctly
	uint64_t writtenSecs = 0;
	memcpy(&writtenSecs, buffer+1, 8);
	am_util_stdio_printf("writtenSecs: %lld\r\n", writtenSecs);

	// erase data
	am_util_delay_ms(250);
	flash_sector_erase(&flash, 0x05);

	// print flash data after write
	am_util_delay_ms(1000);
	flash_read_data(&flash, 0x04, buffer, size);
	buf = buffer;
	am_util_stdio_printf("after erase:\r\n");
	for (int i = 0; i < size; i++) {
		am_util_stdio_printf("%02X ", (int) buf[i]);
		am_util_delay_ms(10);
	}
	am_util_stdio_printf("\r\n");
	am_util_stdio_printf("done\r\n");
}
