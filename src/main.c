#define ARM_MATH_CM4
#include <arm_math.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

// asimple imports
#include <uart.h>

//*****************************************************************************
//
// Example parameters.
//
//*****************************************************************************
#define PDM_FFT_SIZE                4096
#define PDM_FFT_BYTES               (PDM_FFT_SIZE * 2)
#define PRINT_PDM_DATA              0
#define PRINT_FFT_DATA              0

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
volatile bool g_bPDMDataReady = false;
uint32_t g_ui32PDMDataBuffer[PDM_FFT_SIZE];
float g_fPDMTimeDomain[PDM_FFT_SIZE * 2];
float g_fPDMFrequencyDomain[PDM_FFT_SIZE * 2];
float g_fPDMMagnitudes[PDM_FFT_SIZE * 2];
uint32_t g_ui32SampleFreq;

//*****************************************************************************
//
// PDM configuration information.
//
//*****************************************************************************
void *PDMHandle;

am_hal_pdm_config_t g_sPdmConfig =
{
    .eClkDivider = AM_HAL_PDM_MCLKDIV_1,
    .eLeftGain = AM_HAL_PDM_GAIN_0DB,
    .eRightGain = AM_HAL_PDM_GAIN_0DB,
    .ui32DecimationRate = 64,
    .bHighPassEnable = 0,
    .ui32HighPassCutoff = 0xB,
    .ePDMClkSpeed = AM_HAL_PDM_CLK_6MHZ,
    .bInvertI2SBCLK = 0,
    .ePDMClkSource = AM_HAL_PDM_INTERNAL_CLK,
    .bPDMSampleDelay = 0,
    .bDataPacking = 1,
    .ePCMChannels = AM_HAL_PDM_CHANNEL_RIGHT,
    .ui32GainChangeDelay = 1,
    .bI2SEnable = 0,
    .bSoftMute = 0,
    .bLRSwap = 0,
};

//*****************************************************************************
//
// PDM initialization.
//
//*****************************************************************************
void
pdm_init(void)
{
    //
    // Initialize, power-up, and configure the PDM.
    //
    am_hal_pdm_initialize(0, &PDMHandle);
    am_hal_pdm_power_control(PDMHandle, AM_HAL_PDM_POWER_ON, false);
    am_hal_pdm_configure(PDMHandle, &g_sPdmConfig);
    am_hal_pdm_enable(PDMHandle);

    //
    // Configure the necessary pins.
    //
    // am_hal_gpio_pincfg_t sPinCfg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // sPinCfg.uFuncSel = AM_BSP_GPIO_MIC_DATA;
    am_hal_gpio_pinconfig(AM_BSP_GPIO_MIC_DATA, g_AM_BSP_GPIO_MIC_DATA);

    // sPinCfg.uFuncSel = AM_BSP_GPIO_MIC_CLK;
    am_hal_gpio_pinconfig(AM_BSP_GPIO_MIC_CLK, g_AM_BSP_GPIO_MIC_CLK);		//pass ptr to g_AM_.... instead of spincfg

    // am_hal_gpio_state_write(14, AM_HAL_GPIO_OUTPUT_CLEAR);
    // am_hal_gpio_pinconfig(14, g_AM_HAL_GPIO_OUTPUT);

    //
    // Configure and enable PDM interrupts (set up to trigger on DMA
    // completion).
    //
    am_hal_pdm_interrupt_enable(PDMHandle, (AM_HAL_PDM_INT_DERR
                                            | AM_HAL_PDM_INT_DCMP
                                            | AM_HAL_PDM_INT_UNDFL
                                            | AM_HAL_PDM_INT_OVF));

    NVIC_EnableIRQ(PDM_IRQn);
}

//*****************************************************************************
//
// Print PDM configuration data.
//
//*****************************************************************************
void
pdm_config_print(void)
{
    uint32_t ui32PDMClk;
    uint32_t ui32MClkDiv;
    float fFrequencyUnits;

    //
    // Read the config structure to figure out what our internal clock is set
    // to.
    //
    switch (g_sPdmConfig.eClkDivider)
    {
        case AM_HAL_PDM_MCLKDIV_4: ui32MClkDiv = 4; break;
        case AM_HAL_PDM_MCLKDIV_3: ui32MClkDiv = 3; break;
        case AM_HAL_PDM_MCLKDIV_2: ui32MClkDiv = 2; break;
        case AM_HAL_PDM_MCLKDIV_1: ui32MClkDiv = 1; break;

        default:
            ui32MClkDiv = 0;
    }

    switch (g_sPdmConfig.ePDMClkSpeed)
    {
        case AM_HAL_PDM_CLK_12MHZ:  ui32PDMClk = 12000000; break;
        case AM_HAL_PDM_CLK_6MHZ:   ui32PDMClk =  6000000; break;
        case AM_HAL_PDM_CLK_3MHZ:   ui32PDMClk =  3000000; break;
        case AM_HAL_PDM_CLK_1_5MHZ: ui32PDMClk =  1500000; break;
        case AM_HAL_PDM_CLK_750KHZ: ui32PDMClk =   750000; break;
        case AM_HAL_PDM_CLK_375KHZ: ui32PDMClk =   375000; break;
        case AM_HAL_PDM_CLK_187KHZ: ui32PDMClk =   187000; break;

        default:
            ui32PDMClk = 0;
    }

    //
    // Record the effective sample frequency. We'll need it later to print the
    // loudest frequency from the sample.
    //
    g_ui32SampleFreq = (ui32PDMClk /
                        (ui32MClkDiv * 2 * g_sPdmConfig.ui32DecimationRate));

    fFrequencyUnits = (float) g_ui32SampleFreq / (float) PDM_FFT_SIZE;

    am_util_stdio_printf("Settings:\r\n");
    am_util_stdio_printf("PDM Clock (Hz):         %12d\r\n", ui32PDMClk);
    am_util_stdio_printf("Decimation Rate:        %12d\r\n", g_sPdmConfig.ui32DecimationRate);
    am_util_stdio_printf("Effective Sample Freq.: %12d\r\n", g_ui32SampleFreq);
    am_util_stdio_printf("FFT Length:             %12d\r\n\n", PDM_FFT_SIZE);
    am_util_stdio_printf("FFT Resolution: %15.3f Hz\r\n", fFrequencyUnits);
}

//*****************************************************************************
//
// Start a transaction to get some number of bytes from the PDM interface.
//
//*****************************************************************************
void
pdm_data_get(void)
{
    //
    // Configure DMA and target address.
    //
    am_hal_pdm_transfer_t sTransfer;
    sTransfer.ui32TargetAddr = (uint32_t ) g_ui32PDMDataBuffer;
    sTransfer.ui32TotalCount = PDM_FFT_BYTES;

    //
    // Start the data transfer.
    //
    am_hal_pdm_enable(PDMHandle);
    am_util_delay_ms(100);
    am_hal_pdm_fifo_flush(PDMHandle);
    am_hal_pdm_dma_start(PDMHandle, &sTransfer);
}

//*****************************************************************************
//
// PDM interrupt handler.
//
//*****************************************************************************
void
am_pdm0_isr(void)
{
    uint32_t ui32Status;

    //
    // Read the interrupt status.
    //
    am_hal_pdm_interrupt_status_get(PDMHandle, &ui32Status, true);
    am_hal_pdm_interrupt_clear(PDMHandle, ui32Status);

    //
    // Once our DMA transaction completes, we will disable the PDM and send a
    // flag back down to the main routine. Disabling the PDM is only necessary
    // because this example only implemented a single buffer for storing FFT
    // data. More complex programs could use a system of multiple buffers to
    // allow the CPU to run the FFT in one buffer while the DMA pulls PCM data
    // into another buffer.
    //
    if (ui32Status & AM_HAL_PDM_INT_DCMP)
    {
        am_hal_pdm_disable(PDMHandle);
        g_bPDMDataReady = true;
    }
}

//*****************************************************************************
//
// Analyze and print frequency data.
//
//*****************************************************************************
void
pcm_fft_print(void)
{
    float fMaxValue;
    uint32_t ui32MaxIndex;
    int16_t *pi16PDMData = (int16_t *) g_ui32PDMDataBuffer;
    uint32_t ui32LoudestFrequency;

    //
    // Convert the PDM samples to floats, and arrange them in the format
    // required by the FFT function.
    //
    for (uint32_t i = 0; i < PDM_FFT_SIZE; i++)
    {
        if (PRINT_PDM_DATA)
        {
            am_util_stdio_printf("%d\r\n", pi16PDMData[i]);
        }

        g_fPDMTimeDomain[2 * i] = pi16PDMData[i] / 1.0;
        g_fPDMTimeDomain[2 * i + 1] = 0.0;
    }

    if (PRINT_PDM_DATA)
    {
        am_util_stdio_printf("END\r\n");
    }
	/*

    //
    // Perform the FFT.
    //
    arm_cfft_radix4_instance_f32 S;
    arm_cfft_radix4_init_f32(&S, PDM_FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&S, g_fPDMTimeDomain);
    arm_cmplx_mag_f32(g_fPDMTimeDomain, g_fPDMMagnitudes, PDM_FFT_SIZE);

    if (PRINT_FFT_DATA)
    {
        for (uint32_t i = 0; i < PDM_FFT_SIZE / 2; i++)
        {
            am_util_stdio_printf("%f\r\n", g_fPDMMagnitudes[i]);
        }

        am_util_stdio_printf("END\r\n");
    }

    //
    // Find the frequency bin with the largest magnitude.
    //
    arm_max_f32(g_fPDMMagnitudes, PDM_FFT_SIZE / 2, &fMaxValue, &ui32MaxIndex);

    ui32LoudestFrequency = (g_ui32SampleFreq * ui32MaxIndex) / PDM_FFT_SIZE;

    if (PRINT_FFT_DATA)
    {
        am_util_stdio_printf("Loudest frequency bin: %d\r\n", ui32MaxIndex);
    }
//    am_util_stdio_printf("Loudest frequency: %d         \r", ui32LoudestFrequency);
    am_util_stdio_printf("Loudest frequency: %d\r\n", ui32LoudestFrequency);
	*/
}


// MAIN

static struct uart uart;

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

	am_hal_interrupt_master_enable();

	am_util_stdio_printf("Hello World!\r\n");

	

    //
    // Initialize the printf interface for ITM output
    // do we need this??
    // am_bsp_itm_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("PDM FFT example.\r\n");

    //
    // Turn on the PDM, set it up for our chosen recording settings, and start
    // the first DMA transaction.
    //
    pdm_init();
    pdm_config_print();
    am_hal_pdm_fifo_flush(PDMHandle);
    pdm_data_get();

    //
    // Loop forever while sleeping.
    //
    while (1)
    {
        am_hal_interrupt_master_disable();

        if (g_bPDMDataReady)
        {
            g_bPDMDataReady = false;

            pcm_fft_print();

            while (PRINT_PDM_DATA || PRINT_FFT_DATA);

            //
            // Start converting the next set of PCM samples.
            //
            pdm_data_get();
        }

        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

        am_hal_interrupt_master_enable();
    }
	
}