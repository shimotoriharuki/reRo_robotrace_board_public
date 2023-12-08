/*
 * SideSensor.hpp
 *
 *  Created on: Jul 14, 2023
 *      Author: SHIMOTORI Haruki
 */

#ifndef INC_SIDESENSOR_HPP_
#define INC_SIDESENSOR_HPP_

#include "main.h"

class SideSensor{
private:
	bool state_;
	GPIO_TypeDef *gpio_type_;
	uint16_t gpio_num_;
	uint16_t timer_;
	bool raise_fall_flag_; //true: raise, false: fall

public:
	SideSensor(GPIO_TypeDef *, uint16_t);
	void externalInterrupt();
	void updateState();
	bool getState();

};




#endif /* INC_SIDESENSOR_HPP_ */
