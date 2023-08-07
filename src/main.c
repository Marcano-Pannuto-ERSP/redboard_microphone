// SPDX-License-Identifier: Apache-2.0
// Copyright: Gabriel Marcano, 2023
// Copyright: Kristin Ebuengan, 2023
// Copyright: Melody Gill, 2023
// Copyright: Ambiq Micro, Inc., 2023

/*
 * Takes a constant recording from the microhpone
 * and saves it as a raw file
*/

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

// asimple imports
#include <uart.h>
#include <pdm.h>

// MAIN

static struct uart uart;
static struct pdm pdm;

int
main(void)
{
	// Initialization copied from redboard_sensors
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);
	am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
	am_hal_cachectrl_enable();
	am_bsp_low_power_init();
	am_hal_sysctrl_fpu_enable();
	am_hal_sysctrl_fpu_stacking_enable(true);

	// Init UART, registers with SDK printf
	uart_init(&uart, UART_INST0);
    uart_set_baud_rate(&uart, 921600);

    am_util_stdio_printf("Hello World!\r\n");

    //
    // Turn on the PDM, set it up for our chosen recording settings, and start
    // the first DMA transaction.
    //
    pdm_init(&pdm);
    am_hal_pdm_fifo_flush(pdm.PDMHandle);
    pdm_data_get(&pdm, pdm.g_ui32PDMDataBuffer1);

    am_hal_interrupt_master_enable();
    
    //
    // Loop forever while sleeping.
    //
    bool toggle = true;
    while (1)
    {
        am_hal_uart_tx_flush(uart.handle);

        am_hal_interrupt_master_disable();
        bool ready = isPDMDataReady();
        am_hal_interrupt_master_enable();

        if (ready)
        {
            ready = false;

            if(toggle){
                pdm_data_get(&pdm, pdm.g_ui32PDMDataBuffer2);
                pcm_print(&uart, pdm.g_ui32PDMDataBuffer1);
                toggle = false;
            }
            else{
                pdm_data_get(&pdm, pdm.g_ui32PDMDataBuffer1);
                pcm_print(&uart, pdm.g_ui32PDMDataBuffer2);
                toggle = true;
            }
        }

        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
