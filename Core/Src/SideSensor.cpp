/*
 * SideSensor.cpp
 *
 *  Created on: Jul 14, 2023
 *      Author: SHIMOTORI Haruki
 */


#include "SideSensor.hpp"

SideSensor::SideSensor(GPIO_TypeDef *gpio_type, uint16_t gpio_num) : state_(false), gpio_type_(gpio_type), gpio_num_(gpio_num), timer_(0), raise_fall_flag_(true){}

void SideSensor::externalInterrupt()
{
	if(HAL_GPIO_ReadPin(gpio_type_, gpio_num_) == GPIO_PIN_RESET){ // たち下がり　White
		timer_ = 0;
		raise_fall_flag_ = false;
	}
	else{	//立ち上がり Black
		timer_ = 0;
		raise_fall_flag_ = true;
	}
}

void SideSensor::updateState()
{
	timer_++;
	if(timer_ >= 10000) timer_ = 10000;

	if(raise_fall_flag_ == true && timer_ >= 4){
		state_ = false;
		timer_ = 0;
	}
	else if(raise_fall_flag_ == false && timer_ >= 4){
		state_ = true;
		timer_ = 0;
	}
}

bool SideSensor::getState()
{
	return state_;
}
