/*
 * UI.hpp
 *
 *  Created on: Jun 21, 2023
 *      Author: SHIMOTORI Haruki
 */

#ifndef INC_UI_HPP_
#define INC_UI_HPP_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "Encoder.hpp"
#include "globalDefine.h"

#define TURNED_THRESHOLD 1000

class Switch{
private:
	GPIO_TypeDef *gpio_type_;
	uint16_t gpio_num_;

public:
	Switch(GPIO_TypeDef *, uint16_t);
	bool getStatus();
};

class LED{
private:

public:
	LED();
	void set(int8_t);

};

class WheelDial{
private:
	bool is_left_cw_turned_, is_right_cw_turned_;
	bool is_left_ccw_turned_, is_right_ccw_turned_;

	Encoder *encoder_;

public:
	WheelDial(Encoder *);
	void flip();
	bool isCW(bool);
	bool isCCW(bool);
};




#endif /* INC_UI_HPP_ */
