//***********************************************************************************
// Include files
//***********************************************************************************
#include "em_gpio.h"
#include "bspconfig.h"

//***********************************************************************************
// defined files
//***********************************************************************************

// LED 0 pin is
#define	LED0_port		gpioPortF
#define LED0_pin		4u
#define LED0_default	false 	// Default false (0) = off, true (1) = on
// LED 1 pin is
#define LED1_port		gpioPortF
#define LED1_pin		5u
#define LED1_default	false	// Default false (0) = off, true (1) = on



//***********************************************************************************
// global variables
//***********************************************************************************



//***********************************************************************************
// function prototypes
//***********************************************************************************
void gpio_open(void);


