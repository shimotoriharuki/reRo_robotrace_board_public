/*
 * Battery.cpp
 *
 *  Created on: Jun 24, 2023
 *      Author: SHIMOTORI Haruki
 */

#include "Battery.hpp"

float mon_voltage;

Battery::Battery(ADConverter *adc) : voltage_(0)
{
	adc_ = adc;
}


float Battery::getVoltage()
{
	voltage_ = float(MAX_BATTERY_VOLTAGE) * (float(adc_->analog_val_[BATTERY_ADC_INDEX]) / MAX_BATTERY_VOLTAGE_AD_VALUE);
	mon_voltage = voltage_;

	return voltage_;
}



