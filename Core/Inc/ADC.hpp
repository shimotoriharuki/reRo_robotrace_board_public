/*
 * ADC.hpp
 *
 *  Created on: Jun 25, 2023
 *      Author: SHIMOTORI Haruki
 */

#ifndef INC_ADC_HPP_
#define INC_ADC_HPP_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "globalDefine.h"

class ADConverter{
private:
public:
	uint16_t analog_val_[ADC_DATA_SIZE];

	ADConverter();
	void start();

};




#endif /* INC_ADC_HPP_ */
