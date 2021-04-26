/*
 * buttons.c
 *
 *  Created on: Feb 19th, 2021
 *      Author: Travis H
 */


#include "buttons.h"

/***************************************************************************//**
 * @brief
 *   initializes the buttons.
 *
 * @details
 * 	 initializes the buttons.
 *
 * @note
 *   Only called once
 *
 ******************************************************************************/
void buttons_setup() {
	// Configure PB0
	GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPull, 1);
	GPIO_IntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, true, true, true);

	// Configure PB1
	GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInputPull, 1);
	GPIO_IntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, true, true, true);

	// Clear interrupts on even
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);

	// Clear interrupts on odd
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/***************************************************************************//**
 * @brief
 *   polls pushbutton 0.
 *
 * @details
 * 	 It obtains the value of pushbutton 0.
 *
 * @note
 *   called each time a pushbutton value is required.
 *
 * @param[in] status
 *   Pointer to global variable that stores pushbutton 0. Passed by reference.
 *
 ******************************************************************************/
void poll_PB0(bool *status) {
	// get value of pushbutton 0
	unsigned int val_pb0 = GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN);
	// update
	if (val_pb0) {
		*status = false;
	}
	else {
		*status = true;
	}
}

/***************************************************************************//**
 * @brief
 *   polls pushbutton 1.
 *
 * @details
 * 	 obtains the value of pushbutton 1.
 *
 * @note
 *   called each time a pushbutton value is required.
 *
 * @param[in] status
 *   Pointer to global variable that stores pushbutton 1. Passed by reference.
 *
 ******************************************************************************/
void poll_PB1(bool *status) {
	// get value of PB1
	unsigned int val_pb1 = GPIO_PinInGet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN);
	// update
	if (val_pb1) {
		*status = false;
	}
	else {
		*status = true;
	}
}
