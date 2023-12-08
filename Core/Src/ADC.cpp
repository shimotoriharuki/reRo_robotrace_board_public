/*
 * ADC.cpp
 *
 *  Created on: Jun 25, 2023
 *      Author: SHIMOTORI Haruki
 */

#include "ADC.hpp"

ADConverter::ADConverter()
{
	for(auto &av : analog_val_){
		av = 0;
	}
}

void ADConverter::start()
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) analog_val_, ADC_DATA_SIZE);
}


