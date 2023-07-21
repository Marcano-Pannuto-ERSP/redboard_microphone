// SPDX-License-Identifier: Apache-2.0
// Copyright: Gabriel Marcano, 2023
// Copyright: Kristin Ebuengan, 2023
// Copyright: Melody Gill, 2023

// #include <iostream>
// #include <fstream>
#include <stdint.h>
#include <limits.h>
#include <math.h>

#include <arm_math.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

// asimple imports
#include <uart.h>

#define M_PI  3.141592653589793238462643383279502884

static struct uart uart;

int main()
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

    am_hal_interrupt_master_enable();

    int16_t amplitude = INT16_MAX; // std::numeric_limits<int16_t>::max();
    unsigned int sampling_rate = 8000;
    double frequency = 440;
    unsigned int length_s = 10;
    for (size_t sample = 0; sample < sampling_rate * length_s; ++sample)
    {
        double count = sample % sampling_rate;
        double p = count / sampling_rate * frequency;
        int16_t result = amplitude * sin(p * 2.0 * M_PI);
        for (size_t sent = 0; sent != 2;)
            sent += uart_write(&uart, (uint8_t *)&result + sent, 2 - sent);
    }
    // size_t sent = 0;
    // while(sent != (sampling_rate * length_s * 2)){
    //     sent += uart_write(&uart, (uint8_t*)buffer + sent, (sampling_rate * length_s * 2) - sent);
    // }
    return 0;
}
