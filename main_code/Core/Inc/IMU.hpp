/*
 * IMU.hpp
 *
 *  Created on: Jun 21, 2023
 *      Author: SHIMOTORI Haruki
 */

#ifndef INC_IMU_HPP_
#define INC_IMU_HPP_

#include "stm32f4xx_hal.h"

#define R_IMU 0.1 //0.03 Lowpath filter constant. The smaller it is, the more effective/

class IMU{
private:
	int16_t xa_, ya_, za_, xg_, yg_, zg_;
	float omega_;
	float offset_;
	float theta_;
	float theta_10mm_;

public:
	IMU();
	uint8_t init();
	void updateValues();
	float getOmega();
	void calibration();

	float getOffsetVal();

	float getTheta();

	void correctionTheta(float);
	void clearTheta();

	float getTheta10mm();
	void clearTheta10mm();

};




#endif /* INC_IMU_HPP_ */
