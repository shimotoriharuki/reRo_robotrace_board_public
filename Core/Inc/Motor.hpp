/*
 * Motor.hpp
 *
 *  Created on: Jun 19, 2023
 *      Author: SHIMOTORI Haruki
 */

#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "globalDefine.h"

#define MAX_TIM1_COUNTER_PERIOD 2249
#define MAX_FAN_TIM_COUNTER_PERIOD 1799
#define MAX_TIM2_COUNTER_PERIOD_TEMPORARY 899 //899

#define MAX_DRIVE_MOTOR_DUTY 1000
#define MAX_FAN_MOTOR_DUTY 1000

#define CELL_VOLTAGE_ADJUSTMENT_CONSTANUT (4.2 / 4.2)
#define CELL_VOLTAGE_COUNTER_PERIOD (CELL_VOLTAGE_ADJUSTMENT_CONSTANUT * MAX_FAN_TIM_COUNTER_PERIOD)


class DriveMotor{

private:
	float duty_l_, duty_r_;
	uint16_t duty2CounterPeriod(float);

public:
	DriveMotor();
	void init();
	void controlFlip();
	void setDuty(float, float); //Duty: -1000~1000
};

class FanMotor{

private:
	float duty_;
	TIM_HandleTypeDef *tim_type;
	uint16_t channel;
	uint16_t duty2CounterPeriod(float);

public:
	FanMotor(TIM_HandleTypeDef *, uint16_t);
	void init();
	void controlFlip();
	void setDuty(float); //Duty: 0~1000
};

/*
class TemporaryFanMotor{

private:
	float duty_;
	uint16_t duty2CounterPeriod(float);

public:
	TemporaryFanMotor();
	void init();
	void controlFlip();
	void setDuty(float); //Duty: 0~1000
};
*/

#endif /* INC_MOTOR_HPP_ */
