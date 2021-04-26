/*
 * slider.h
 *
 *  Created on: Apr 25th, 2021
 *      Author: Travis H
 */

#ifndef SRC_HEADER_FILES_SLIDER_H_
#define SRC_HEADER_FILES_SLIDER_H_

#include <stdint.h>


enum slider_position {
	NONE,
	FAR_LEFT,
	LEFT,
	RIGHT,
	FAR_RIGHT,
};

void slider_setup(void);
void slider_position(uint8_t *position);



#endif /* SRC_HEADER_FILES_SLIDER_H_ */
