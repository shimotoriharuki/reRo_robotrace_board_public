/*
 * VelocityControl.hpp
 *
 *  Created on: Jun 21, 2023
 *      Author: SHIMOTORI Haruki
 */

#ifndef INC_VELOCITYCONTROL_HPP_
#define INC_VELOCITYCONTROL_HPP_

#include "Motor.hpp"
#include "Encoder.hpp"
#include "IMU.hpp"
#include "globalDefine.h"

#define I_TERM_LIMIT 1000

class VelocityControl
{

private:
	float target_velocity_, target_omega_;
	float current_velocity_;
	float current_omega_;
	float t_kp_, t_kd_, t_ki_;
	float r_kp_, r_kd_, r_ki_;
	bool excution_flag_;
	bool i_reset_flag_;
	float rotation_ratio_;
	float translation_ratio_;
	bool flag_angular_velocity_pid_control_; // trueのときは角速度もPIDで速度制御する．falseのとき角速度はLineFollowingで計算されたものを使うようになる
	DriveMotor *motor_;
	Encoder *encoder_;
	IMU *imu_;

	float calcVelocity();
	void pid();
	void pidTranslationOnly();

public:
	VelocityControl(DriveMotor *, Encoder *, IMU *);
	void setTargetVelocity(float, float);
	void setTargetTranslationVelocityOnly(float, float);

	void setTranslationGain(float, float, float);
	void setRotationGain(float, float, float);

	void flip();

	void enableAngularVelocityPIDControl();
	void disableAngularVelocityPIDControl();
	void start();
	void stop();

	float getCurrentVelocity();
	float getTranslationRatio();
	float getRotationRatio();

	void setClearFlagOfVelocityControlI();

};

#endif /* INC_VELOCITYCONTROL_HPP_ */
