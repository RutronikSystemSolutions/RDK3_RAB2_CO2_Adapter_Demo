/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK3_RAB2_CO2_Adapter_Demo
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-12-21
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "xensiv_pasco2_mtb.h"
#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"

#define CYBSP_I2C_SCL ARDU_SCL
#define CYBSP_I2C_SDA ARDU_SDA
#define CYBSP_DEBUG_UART_TX KITPROG_TX
#define CYBSP_DEBUG_UART_RX KITPROG_RX

/* Wait time for PASCO2 ready (milliseconds) */
#define WAIT_PASCO2_RDY_MS          	(2000)

/* The CO2 concentration value acquired by the sensor depends on the external atmospheric pressure.
   To compensate for this effect, pressure values can be acquired from a pressure sensor such as an
   Infineon XENSIV&trade; DPS3xx. (https://github.com/Infineon/sensor-xensiv-dps3xx) */
#define DEFAULT_PRESSURE_REF_HPA        (0x3F7)     /* Default atmospheric pressure to compensate for (hPa) */

/*I2C Peripheral Global Objects*/
cyhal_i2c_t I2C_scb3;
cyhal_i2c_cfg_t I2C_cfg;

/*PASCO2 Sensor Global Structure*/
xensiv_pasco2_t xensiv_pasco2;

int main(void)
{
    cy_rslt_t result;
    uint16_t pasco2ppm;
    uint16_t serial_0;
    uint16_t serial_1;
    uint16_t serial_2;
    int16_t scd_err = 0;
    uint16_t scd_co2;
    int32_t scd_temperature;
    int32_t scd_humidity;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }
    printf("\x1b[2J\x1b[;H");

    /*Initialize LEDs*/
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    /*I2C Peripheral Configuration*/
    I2C_cfg.is_slave = false;
    I2C_cfg.address = 0;
    I2C_cfg.frequencyhal_hz = 100000UL;
    result = cyhal_i2c_init(&I2C_scb3, ARDU_SDA, ARDU_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
    	printf("I2C peripheral initialization error.\r\n");
    	CY_ASSERT(0);
    }
    result = cyhal_i2c_configure(&I2C_scb3, &I2C_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
    	printf("I2C peripheral configuration error.\r\n");
    	CY_ASSERT(0);
    }
    printf("I2C peripheral ready.\r\n");
    CyDelay(WAIT_PASCO2_RDY_MS);

    /* Initialize PAS CO2 sensor with default parameter values */
    result = xensiv_pasco2_mtb_init_i2c(&xensiv_pasco2, &I2C_scb3);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("PAS CO2 device initialization error.\r\n");
        CY_ASSERT(0);
    }
    printf("PAS CO2 device initialized successfully.\r\n");

    /* Clean up potential SCD40 states */
    scd4x_wake_up();
    scd4x_stop_periodic_measurement();
    scd4x_reinit();

    /* Read the serial number of the SCD4x sensor attached*/
    scd_err = scd4x_get_serial_number(&serial_0, &serial_1, &serial_2);
    if (scd_err)
    {
        printf("Error reading SCD4X serial number.\r\n");
    }
    else
    {
        printf("SCD4X sensor is online, the serial number is: " );
        printf("0x%X", serial_0);
        printf("%X", serial_1);
        printf("%X\r\n", serial_2);
    }

    /* Start periodic measurement of the sensor*/
    scd_err = scd4x_start_periodic_measurement();
    if (!scd_err)
    {
        printf("SCD4X periodic measurement started.\r\n");
    }
    else
    {
        printf("Could not start the periodic measurement of the SCD4X.\r\n");
        CY_ASSERT(0);
    }

    for (;;)
    {
    	/*Read the data from PAS CO2*/
        result = xensiv_pasco2_mtb_read(&xensiv_pasco2, DEFAULT_PRESSURE_REF_HPA, &pasco2ppm);
        if (result == CY_RSLT_SUCCESS)
        {
            printf("Infineon PAS CO2 %d ppm.\r\n", pasco2ppm);
        }
        cyhal_system_delay_ms(10);

        /*Read the data from SDC4X*/
        scd_err = scd4x_read_measurement(&scd_co2, &scd_temperature, &scd_humidity);
        if (!scd_err)
        {
            printf("Sensirion SDC4X CO2: %u ppm ", scd_co2);
            printf("Temperature: %.2f\xB0 C ", (float)scd_temperature/1000);
            printf("Humidity: %.2f%% RH\r\n", (float)scd_humidity/1000);
            sensirion_i2c_hal_sleep_usec(10000000);
        }
    }
}

/* [] END OF FILE */
