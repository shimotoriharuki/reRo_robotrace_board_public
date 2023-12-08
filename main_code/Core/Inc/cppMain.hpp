/*
 * cppMain.hpp
 *
 *  Created on: Jun 19, 2023
 *      Author: SHIMOTORI Haruki
 */

#ifndef INC_CPPMAIN_HPP_
#define INC_CPPMAIN_HPP_

#include "stm32f4xx_hal.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif


void cppInit(void);
void cppExit(uint16_t);
void cppFlip1ms(void);
void cppFlip100ns(void);
void cppFlip10ms(void);
void cppLoop(void);


#ifdef __cplusplus
};
#endif




#endif /* INC_CPPMAIN_HPP_ */
