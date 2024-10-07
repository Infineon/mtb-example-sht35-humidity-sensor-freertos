/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the XENSIV Sensor Shield: SHT35 
*              Humidity Sensor interfacing using I2C for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "GUI.h"
#include "mtb_st7735s.h"
#include "mtb_sht3x.h"
#include "cy_retarget_io.h"
#include "Generated/Resource.h"
#include "ID_SCREEN_00.h"

/******************************************************************************
* Macros
*******************************************************************************/
#define SPI_SEL                    (CYBSP_D9)
#define TFT_TASK_STACK_SIZE        (1024*10)
#define TFT_TASK_PRIORITY          (configMAX_PRIORITIES - 3)

#define SHT35_TASK_STACK_SIZE      (1024*2)
#define SHT35_TASK_PRIORITY        (configMAX_PRIORITIES - 5)

/* SPI baud rate in Hz */
#define SPI_FREQ_HZ                (1000000UL)
/* SPI transfer bits per frame */
#define BITS_PER_FRAME             (8)

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* This enables RTOS aware debugging. */
volatile int uxTopUsedPriority;

/* The pins are defined by the st7735s library. If the display is being used
 *  on different hardware the mappings will be different. */
const mtb_st7735s_pins_t tft_pins =
{
        .dc       = CYBSP_A0,
        .rst      = CYBSP_D2
};

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void tft_task(void *arg);
void sht35_task(void *arg);

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
*********************************************************************************
* Summary:
*  System entrance point. This function initializes BSP, retarget IO, sets up 
*  the TFT and SHT35 tasks, and then starts the RTOS scheduler.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* This enables RTOS aware debugging in OpenOCD */
    uxTopUsedPriority = configMAX_PRIORITIES - 1 ;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the retarget-io */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                  CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* To avoid compiler warning */
    (void)result;

    /* Enable global interrupts */
    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("***********************************************************\r\n");
    printf("         SHT35: Sensirion Humidity Sensor Example          \r\n");
    printf("***********************************************************\r\n\n");

    /* Create the TFT and SHT35 tasks */
    xTaskCreate(tft_task, "tftTask", TFT_TASK_STACK_SIZE,
                    NULL,  TFT_TASK_PRIORITY,  NULL);

    xTaskCreate(sht35_task, "sht35Task", SHT35_TASK_STACK_SIZE,
                        NULL,  SHT35_TASK_PRIORITY,  NULL);

    /* Start the FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* Should never get here. */
    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: void sht35_task(void *arg)
*********************************************************************************
* Summary:
* This function interfaces SHT35 sensor and reads the Humidity and Temperature
*
* Parameters:
*  arg: task argument (unused)
*
* Return:
*  None
*
*******************************************************************************/
void sht35_task(void *arg)
{
    cy_rslt_t result;
    cyhal_i2c_t kit_i2c;

    /* I2C configuration settings */
    static const cyhal_i2c_cfg_t kit_i2c_cfg =
    {
        .is_slave        = false,
        .address         = 0,
        .frequencyhal_hz = 400000
    };

    char temp_string[10] = " ";
    char hum_string[10] = " ";

    struct sensor_data_t data;

    /* Initialize the I2C peripheral */
    result =  cyhal_i2c_init(&kit_i2c, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    /* I2C init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure the I2C peripheral */
    result =  cyhal_i2c_configure(&kit_i2c, &kit_i2c_cfg);
    /* I2C config failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initializes and configure SHT35 sensor */
    mtb_sht3x_init(&kit_i2c, MTB_SHT35_ADDRESS_DEFAULT);

    /* Starts the periodic measurement mode of SHT35 sensor */
    mtb_sht3x_start_periodic_measurement(&kit_i2c, REPEAT_MEDIUM, MPS_ONE_PER_SEC);

    for(;;)
    {
        /* Reads the SHT35 sensor data and stores in sensor_data_t structure */
        data = mtb_sht3x_read(&kit_i2c);

        /* Updates the sensor data on the display and serial terminal */
        sprintf(hum_string, "%.2lf", data.humidity);
        APPW_SetText(ID_SCREEN_00, ID_TEXT_01, hum_string);
        printf("Humidity : %.2lf %%RH\r\n", data.humidity);

        sprintf(temp_string, "%.2lf", data.temperature);
        APPW_SetText(ID_SCREEN_00, ID_TEXT_02, temp_string);
        printf("Temperature : %.2lf degC\r\n\n", data.temperature);
    }

}

/*******************************************************************************
* Function Name: void tft_task(void *arg)
*********************************************************************************
* Summary: The Following functions are performed in this task
*           1. Initializes the EmWin display engine
*           2. Displays the SHT35 humidity sensor data
*
* Parameters:
*  arg: task argument (unused)
*
* Return:
*  None
*
*******************************************************************************/
void tft_task(void *arg)
{
    cy_rslt_t result;
    cyhal_spi_t mSPI;

    /* Initialize SPI_SEL pin to select the SPI CS for display */
    result = cyhal_gpio_init(SPI_SEL, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the SPI peripheral */
    result = cyhal_spi_init(&mSPI, CYBSP_SPI_MOSI, CYBSP_SPI_MISO, CYBSP_SPI_CLK,
                                CYBSP_SPI_CS, NULL,BITS_PER_FRAME,
                                CYHAL_SPI_MODE_00_MSB, false);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Set the SPI baud rate */
    result = cyhal_spi_set_frequency(&mSPI, SPI_FREQ_HZ);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the display controller */
    result = mtb_st7735s_init_spi(&mSPI, &tft_pins);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Calling the Appwizard application entry point*/
    MainTask();
}

/* [] END OF FILE */
