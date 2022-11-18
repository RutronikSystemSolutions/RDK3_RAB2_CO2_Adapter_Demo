
/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for RDK2 Rutronik development kit Application
*              for ModusToolbox.
*
*
*  Created on: 2022-08-24
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Authors: IUS, GDR
* Related Document: See README.md
*
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

#include "scd4x/scd4x_i2c.h"
#include "scd4x/sensirion_common.h"
#include "scd4x/sensirion_i2c_hal.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "xensiv_pasco2_mtb.h"


#define CYBSP_I2C_SCL ARDU_SCL
#define CYBSP_I2C_SDA ARDU_SDA

#define CYBSP_DEBUG_UART_TX KITPROG_TX
#define CYBSP_DEBUG_UART_RX KITPROG_RX

static cyhal_i2c_t cyhal_i2c;

#define I2C_MASTER_FREQUENCY (100000U)

#define WAIT_SENSOR_RDY_MS          (5000)      /* Wait time for sensor ready (milliseconds) */

#define DEFAULT_PRESSURE_REF_HPA    (0x3F7)     /* Default atmospheric pressure to compensate for (hPa) */

/*******************************************************************************
* Global Variables
*******************************************************************************/
static xensiv_pasco2_t xensiv_pasco2;
uint16_t ppm, pas_co2;
cy_rslt_t result = CY_RSLT_SUCCESS;

int i2c_init();

int main(void)
{
	 int16_t error = 0;

	    /* Initialize the device and board peripherals */
	    result = cybsp_init();
	    CY_ASSERT(result == CY_RSLT_SUCCESS);

	    __enable_irq();

	    /* Initialize retarget-io to use the debug UART port. */
	     result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
	     CY_ASSERT(result == CY_RSLT_SUCCESS);

	     i2c_init();

		 printf("\x1b[2J\x1b[;H");
	     printf("Powered by RUTRONIK\r\n");
	     printf("RUT CO2 BOARD TEST EXAMPLE \r\n");
	     printf("Measuring interval: 10 seconds \r\n");


	     /* Initialize PAS CO2 sensor with default parameter values */
	         result = xensiv_pasco2_mtb_init_i2c(&xensiv_pasco2, &cyhal_i2c);
	         if (result != CY_RSLT_SUCCESS)
	         {
	             printf("PAS CO2 device initialization error \r\n");
	             CY_ASSERT(0);
	         }
	         else {
		        printf("PAS CO2 sensor is online \r\n");
	         }

	    /* Initialize PAS CO2 sensor */

	     cyhal_system_delay_ms(WAIT_SENSOR_RDY_MS);

	    // Clean up potential SCD40 states
	    scd4x_wake_up();
	    scd4x_stop_periodic_measurement();
	    scd4x_reinit();

	    uint16_t serial_0;
	    uint16_t serial_1;
	    uint16_t serial_2;
	    error = scd4x_get_serial_number(&serial_0, &serial_1, &serial_2);
	    if (error) {
	        printf("Error executing scd4x_get_serial_number(): %i\r\n", error);
	    } else {
	        printf("SCD4X sensor is online \r\n");
	    }

	    // Start Measurement

	    error = scd4x_start_periodic_measurement();
	    if (error) {
	        printf("Error executing scd4x_start_periodic_measurement(): %i\r\n",
	               error);
	    }

	    printf("Start first meausrement in 5 sec. ...\r\n");

	    uint16_t co2;
	    int32_t temperature;
	    int32_t humidity, count=0;
	    for (;;) {
	            // Read Measurement

	        cyhal_system_delay_ms(WAIT_SENSOR_RDY_MS);
            printf("\x1b[2J\x1b[;H"); // clear terminal
            count++;
            printf("     COUNT: %d \r\n", (int)count);

            result = xensiv_pasco2_mtb_read(&xensiv_pasco2, DEFAULT_PRESSURE_REF_HPA, &ppm);
            if (result == CY_RSLT_SUCCESS)
            {
            	pas_co2=ppm;
            }

            	printf("    PAS CO2: %u ppm\r\n", pas_co2);


	            error = scd4x_read_measurement(&co2, &temperature, &humidity);
	            if (error) {
	                printf("Error executing scd4x_read_measurement(): %i \r\n", error);
	            } else if (co2 == 0) {
	                printf("Invalid sample detected, skipping.\r\n");
	            } else {

	            	//float h=(humidity/1000);
	                printf("  SDC4X CO2: %u ppm\r\n", co2);
	                printf("Temperature: %.2f °C\r\n", (float)temperature/1000);
	                printf("   Humidity: %.2f RH\r\n", (float)humidity/1000);
		            sensirion_i2c_hal_sleep_usec(10000000);
	            }
	        }



    return 0;
}
int i2c_init()
{

	 cyhal_i2c_cfg_t i2c_master_config = {CYHAL_I2C_MODE_MASTER,
	                                         0,
	                                         I2C_MASTER_FREQUENCY};

	    result = cyhal_i2c_init(&cyhal_i2c, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
	    CY_ASSERT(result == CY_RSLT_SUCCESS);

	    result = cyhal_i2c_configure(&cyhal_i2c, &i2c_master_config);
	    CY_ASSERT(result == CY_RSLT_SUCCESS);
	    return (int)result;
}
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint16_t count) {

	return (int8_t)cyhal_i2c_master_read(&cyhal_i2c,(uint16_t)address, data, count, 100, false);

}


int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data,
                               uint16_t count) {
	return (int8_t)cyhal_i2c_master_write(&cyhal_i2c,(uint16_t)address, data, count, 100, false);
}

