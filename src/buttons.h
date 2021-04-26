/*
 * buttons.h
 *
 *  Created on: Feb 19th, 2021
 *      Author: Travis Hardesty
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

#include  <stdint.h>
#include  <kernel/include/os.h>
#include  <kernel/include/os_trace.h>
#include "gpio.h"

//***********************************************************************************
// Include files
//***********************************************************************************
#include "bspconfig.h"
#include "em_gpio.h"
#include "bsp.h"


//***********************************************************************************
// function prototypes
//***********************************************************************************
void buttons_setup(void);
void poll_PB0(bool *status);
void poll_PB1(bool *status);




#endif /* BUTTONS_H_ */
