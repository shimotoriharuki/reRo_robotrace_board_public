/*
 * IMU.cpp
 *
 *  Created on: Jun 21, 2023
 *      Author: SHIMOTORI Haruki
 */


#include "IMU.hpp"
#include "MPU6500.h"
#include "globalDefine.h"

float mon_zg, mon_omega, mon_theta;

IMU::IMU() : xa_(0), ya_(0), za_(0), xg_(0), yg_(0), zg_(0), omega_(0), offset_(0), theta_(0), theta_10mm_(0)
{

}

uint8_t IMU::init()
{
	uint8_t who_i_am;
	who_i_am = IMU_init();

	return who_i_am;

}

void IMU::updateValues()
{
	read_gyro_data();
	//read_accel_data();

	static int16_t pre_zg;
	zg_ = int((R_IMU)*(zg) + (1.0 - (R_IMU))* (pre_zg)); // lowpath filter

	pre_zg = zg_;
	mon_zg= zg_;

	float corrected_zg = float(zg_) - offset_;
	omega_ = -(corrected_zg / 16.4) * PI / 180;
	mon_omega = omega_;

	theta_ += omega_ * DELTA_T;
	theta_10mm_ += omega_ * DELTA_T;
	mon_theta = theta_;
}

float IMU::getOmega()
{
	return omega_;
}

void IMU::calibration()
{
	int16_t num = 2000;
	float zg_vals[num];
	for(uint16_t i = 0; i < num; i++){
		zg_vals[i] = float(zg_);
		HAL_Delay(1);
	}

	float sum;
	for(const auto &v : zg_vals){
		sum += v;
	}

	offset_ = sum / num;
}

float IMU::getOffsetVal()
{
	return offset_;
}

float IMU::getTheta()
{
	return theta_;
}

void IMU::correctionTheta(float theta)
{
	theta_ = theta;
}

void IMU::clearTheta()
{
	theta_ = 0;
}
float IMU::getTheta10mm()
{
	return theta_10mm_;
}

void IMU::clearTheta10mm()
{
	theta_10mm_ = 0;
}
