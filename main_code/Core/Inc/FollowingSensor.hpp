/*
 * FollowingSensor.hpp
 *
 *  Created on: Jun 21, 2023
 *      Author: under
 */

#ifndef INC_FOLLOWINGSENSOR_HPP_
#define INC_FOLLOWINGSENSOR_HPP_

#include "stm32f4xx_hal.h"
#include "UI.hpp"
#include "ADC.hpp"

#define SENSOR_DATA_SIZE 12

class FollowingSensor{

private:
	float store_vals_[10][SENSOR_DATA_SIZE];
	float sensor_coefficient_[SENSOR_DATA_SIZE];
	float offset_values_[SENSOR_DATA_SIZE];

	Switch *push_switch_;
	ADConverter *adc_;

public:

	float sensor[SENSOR_DATA_SIZE];

	FollowingSensor(ADConverter *);
	void init();
	void storeValues();
	void updateValues();
	void calibration();
	void printSensorValues();
	bool isAllSensorBlack();

};



#endif /* INC_FOLLOWINGSENSOR_HPP_ */
