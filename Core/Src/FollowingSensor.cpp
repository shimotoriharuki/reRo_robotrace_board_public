/*
 * FollowingSensor.cpp
 *
 *  Created on: Jun 21, 2023
 *      Author: under
 */

#include <FollowingSensor.hpp>
#include "main.h"

#define MAX_SENSOR_VALUE 1000

float mon_vals[SENSOR_DATA_SIZE];

FollowingSensor::FollowingSensor(ADConverter *adc)
{
	adc_ = adc;
	push_switch_ = new Switch(GPIOA, GPIO_PIN_12);

	for(auto &s : sensor){
		s = 0;
	}

	for(auto &m : offset_values_){
		m = 0;
	}
	for(auto &s : sensor_coefficient_){
		s = 1;
	}

}

void FollowingSensor::init()
{
}

void FollowingSensor::storeValues()
{
	static uint8_t cnt = 0;

	for(int i = 0; i < SENSOR_DATA_SIZE; i++){
		store_vals_[cnt][i] = sensor_coefficient_[i] * (adc_->analog_val_[i] - offset_values_[i]) ;
	}

	cnt++;
	if(cnt >= 10) cnt = 0;


}
void FollowingSensor::updateValues()
{
	float temp_val[10];

	for(uint8_t ad_cnt = 0; ad_cnt < SENSOR_DATA_SIZE; ad_cnt++){
		for(uint8_t store_cnt = 0; store_cnt < 10; store_cnt++){
			temp_val[store_cnt] = store_vals_[store_cnt][ad_cnt];
		}

		// sort
		for(uint8_t i = 0; i < 10; i++){
			for (uint8_t j = i+1; j < 10; j++) {
				if(temp_val[i] < temp_val[j]){
					float tmp = temp_val[j];
					temp_val[j] = temp_val[i];
					temp_val[i] = tmp;
				}
			}
		}

		sensor[ad_cnt] = temp_val[5];

		if(sensor[ad_cnt] <= 0) sensor[ad_cnt] = 0;

		mon_vals[ad_cnt] = sensor[ad_cnt];


	}

}

void FollowingSensor::calibration()
{
	HAL_Delay(100);

	float max_values[SENSOR_DATA_SIZE];
	float min_values[SENSOR_DATA_SIZE];

	//初期化
	for(uint16_t i = 0; i < SENSOR_DATA_SIZE; i++){
		max_values[i] = sensor[i];
		min_values[i] = sensor[i];
	}

	while(push_switch_->getStatus() == false){
		for(uint16_t i = 0; i < SENSOR_DATA_SIZE; i++){
			if(max_values[i] < sensor[i]){
				max_values[i] = sensor[i];
			}
			else if(min_values[i] > sensor[i]){
				min_values[i] = sensor[i];
			}
		}
	}


	for(uint16_t i = 0; i < SENSOR_DATA_SIZE; i++){
		sensor_coefficient_[i] = MAX_SENSOR_VALUE / (max_values[i] - min_values[i]);
	}
	for(uint16_t i = 0; i < SENSOR_DATA_SIZE; i++){
		offset_values_[i] = min_values[i];
	}

	HAL_Delay(1000);

}

void FollowingSensor::printSensorValues()
{
	//printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", sensor[0], sensor[1], sensor[2], sensor[3], sensor[4], sensor[5], sensor[6], sensor[7], sensor[8], sensor[9], sensor[10], sensor[11], sensor[12], sensor[13]);
}

bool FollowingSensor::isAllSensorBlack()
{
	uint16_t out_cnt = 0;
	static uint16_t cnt = 0;
	static bool flag = false;

	/*
	for(const auto & s : sensor){
		if(s >= 550) out_cnt++;
	}
	*/
	for(uint16_t i = 1; i <= 10; i++){
		if(sensor[i] >= 750) out_cnt++;
	}

	if(out_cnt >= 10){
		cnt++;
	}
	else{
		cnt = 0;
	}

	if(cnt >= 800){
		flag = true;
	}
	else flag = false;

	if(cnt >= 10000) cnt = 10000;

	return flag;

}


