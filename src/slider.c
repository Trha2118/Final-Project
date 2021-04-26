/*
 * slider.c
 *
 *  Created on: Apr 25th, 2021
 *      Author: Travis H
 */


//***********************************************************************************
// Include files
//***********************************************************************************

#include "slider.h"
#include "capsense.h"


/***************************************************************************//**
 * @brief
 *   initialize CAPSENSE slider
 *
 * @details
 * 	 initializes CAPSENSE
 *
 * @note
 *   called once
 *
 ******************************************************************************/

void slider_setup() {
	CAPSENSE_Init();
}


/***************************************************************************//**
 * @brief
 *   gets CAPSENSE slider value.
 *
 * @note
 *   called every time CAPSENSE slider value is desired.
 *
 * @param[in] position
 *   Pointer to slider position value.
 *
 ******************************************************************************/

void slider_position(uint8_t *position) {
	uint8_t pos_cnt = 0;
	CAPSENSE_Sense();
	*position = NONE;
	if (CAPSENSE_getPressed(0)) {
		*position = FAR_LEFT;
		pos_cnt++;
	}
	if (CAPSENSE_getPressed(1)) {
		*position = LEFT;
		pos_cnt++;
	}
	if (CAPSENSE_getPressed(2)) {
		*position = RIGHT;
		pos_cnt++;
	}
	if (CAPSENSE_getPressed(3)) {
		*position = FAR_RIGHT;
		pos_cnt++;
	}
	if (pos_cnt > 1) {
		*position = NONE;
	}
	return;
}

