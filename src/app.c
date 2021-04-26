/**
 * @file app.c
 * @author Travis H
 * @date April 25th, 2021
 *
 */


//***********************************************************************************
// Include files
//***********************************************************************************
#include "app.h"

//***********************************************************************************
// function
//***********************************************************************************


/***************************************************************************//**
 * @brief
 *   calls peripheral init functions.
 *
 * @details
 *   calls peripheral functions to initialize
 * 	 needed in the main while loop in main.c
 *
 * @note
 *
 *
 ******************************************************************************/
void app_peripheral_setup(void){
	cmu_open();
	gpio_open();
	buttons_setup();
	slider_setup();
}

/***************************************************************************//**
 * @brief
 *   initializes letimer0
 *
 * @details
 * 	 initializes the letimer0 module.
 *
 * @note
 *   Only called once
 *
 ******************************************************************************/
void app_letimer0_open(void){

}
