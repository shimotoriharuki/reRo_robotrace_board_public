/*
 * Motor.cpp
 *
 *  Created on: Jun 19, 2023
 *      Author: SHIMOTORI Haruki
 */

#include "Motor.hpp"

int16_t mon_rev_l, mon_rev_r;
float mon_counter_period;

//*************************************//
//************DriveMotor***************//
//*************************************//

//---private---//
uint16_t DriveMotor::duty2CounterPeriod(float duty)
{
	//setMotorDuryで指定した0~1000をタイマーのカウンターピリオド0~MAX_TIM1_COUNTER_PERIODに変換
	return (int)(MAX_TIM1_COUNTER_PERIOD * duty / MAX_DRIVE_MOTOR_DUTY);
}

//---public---//
DriveMotor::DriveMotor() : duty_l_(0), duty_r_(0)
{
}

void DriveMotor::init()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //PWM start
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MAX_TIM1_COUNTER_PERIOD);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MAX_TIM1_COUNTER_PERIOD);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MAX_TIM1_COUNTER_PERIOD);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MAX_TIM1_COUNTER_PERIOD);

	HAL_Delay(10);
}
void DriveMotor::controlFlip()
{
	int16_t counter_period_l, counter_period_r;
	int16_t reverse_counter_period_l, reverse_counter_period_r;

	counter_period_l = duty2CounterPeriod(duty_l_);
	counter_period_r = duty2CounterPeriod(duty_r_);

	if(duty_l_ >= 0){
		// motor2
		reverse_counter_period_l = MAX_TIM1_COUNTER_PERIOD - counter_period_l;

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MAX_TIM1_COUNTER_PERIOD);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, reverse_counter_period_l);
	}
	else{
		// motor2
		reverse_counter_period_l = MAX_TIM1_COUNTER_PERIOD - (-counter_period_l);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, reverse_counter_period_l);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MAX_TIM1_COUNTER_PERIOD);
	}

	if(duty_r_ >= 0){
		// motor1
		reverse_counter_period_r = MAX_TIM1_COUNTER_PERIOD - counter_period_r;

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, reverse_counter_period_r);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MAX_TIM1_COUNTER_PERIOD);
	}
	else{
		//motor1
		reverse_counter_period_r = MAX_TIM1_COUNTER_PERIOD - (-counter_period_r);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MAX_TIM1_COUNTER_PERIOD);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, reverse_counter_period_r);
	}

	mon_rev_l = reverse_counter_period_l;
	mon_rev_r = reverse_counter_period_r;


}

void DriveMotor::setDuty(float l, float r)
{
	if(l >= MAX_DRIVE_MOTOR_DUTY) l = MAX_DRIVE_MOTOR_DUTY;
	else if(l <= -MAX_DRIVE_MOTOR_DUTY) l = -MAX_DRIVE_MOTOR_DUTY;

	if(r >= MAX_DRIVE_MOTOR_DUTY) r = MAX_DRIVE_MOTOR_DUTY;
	else if(r <= -MAX_DRIVE_MOTOR_DUTY) r = -MAX_DRIVE_MOTOR_DUTY;

	duty_l_ = l;
	duty_r_ = r;
}

//*************************************//
//************FanMotor***************//
//*************************************//

//---private---//
uint16_t FanMotor::duty2CounterPeriod(float duty)
{
	//setMotorDuryで指定した0~1000をタイマーのカウンターピリオド0~cell_voltage_counter_periodに変換
	return (int)(CELL_VOLTAGE_COUNTER_PERIOD * duty / MAX_FAN_MOTOR_DUTY);	//本当のDutyマックスにするとモータが壊れるので，Dutyマックスを指定しても最大4.2Vを超えないようにしている
}

//---public---//
FanMotor::FanMotor(TIM_HandleTypeDef *tim_type1, uint16_t channel1) : duty_(0),
		tim_type(tim_type1), channel(channel1) {}

void FanMotor::init()
{
	/*
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //IN1
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //IN3

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0); //IN1
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 300); //IN2
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MAX_FAN_TIM_COUNTER_PERIOD); //IN3
	*/

	HAL_TIM_PWM_Start(tim_type, channel); //IN1 or 3

	//__HAL_TIM_SET_COMPARE(tim_type1_, channel1_, 0); //IN1
	//__HAL_TIM_SET_COMPARE(tim_type2_, channel2_, 250); //IN2
}

void FanMotor::controlFlip()
{
	int16_t counter_period;

	counter_period = duty2CounterPeriod(duty_);
	mon_counter_period = counter_period;

	__HAL_TIM_SET_COMPARE(tim_type, channel, counter_period);

}

void FanMotor::setDuty(float duty)
{
	if(duty >= MAX_FAN_MOTOR_DUTY) duty = MAX_FAN_MOTOR_DUTY;
	else if(duty <= 0) duty = 0;

	duty_ = duty;

}

//*************************************//
//*********TemporaryFanMotor***********//
//*************************************//

/*
//---private---//
uint16_t TemporaryFanMotor::duty2CounterPeriod(float duty)
{
	//setMotorDuryで指定した0~1000をタイマーのカウンターピリオド0~cell_voltage_counter_periodに変換
	return (int)(MAX_TIM2_COUNTER_PERIOD_TEMPORARY * duty / MAX_FAN_MOTOR_DUTY); //電源が1Sのとき用
}

//---public---//
TemporaryFanMotor::TemporaryFanMotor() : duty_(0){}

void TemporaryFanMotor::init()
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

void TemporaryFanMotor::controlFlip()
{
	int16_t counter_period;

	counter_period = duty2CounterPeriod(duty_);
	mon_counter_period = counter_period;

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, counter_period);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, counter_period);


}

void TemporaryFanMotor::setDuty(float duty)
{
	if(duty >= MAX_FAN_MOTOR_DUTY) duty = MAX_FAN_MOTOR_DUTY;
	else if(duty <= 0) duty = 0;

	duty_ = duty;

}
*/
