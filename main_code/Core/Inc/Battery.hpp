/*
 * Battery.hpp
 *
 *  Created on: Jun 24, 2023
 *      Author: SHIMOTORI Haruki
 */

#ifndef INC_BATTERY_HPP_
#define INC_BATTERY_HPP_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "globalDefine.h"
#include "ADC.hpp"

#define BATTERY_ADC_INDEX 12

class Battery{
private:
	float voltage_;
	ADConverter *adc_;
public:
	Battery(ADConverter *);
	float getVoltage();

};




#endif /* INC_BATTERY_HPP_ */
