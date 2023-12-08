/*
 * VelocityControl.cpp
 *
 *  Created on: Jun 21, 2023
 *      Author: SHIMOTORI Haruki
 */


#include "VelocityControl.hpp"
#include <math.h>

float mon_current_velocity;

VelocityControl::VelocityControl(DriveMotor *motor, Encoder *encoder, IMU *imu) :
target_velocity_(0), target_omega_(0), current_velocity_(0), current_omega_(0), t_kp_(0), t_kd_(0), t_ki_(0),
	r_kp_(0), r_kd_(0), r_ki_(0), excution_flag_(false), i_reset_flag_(false), rotation_ratio_(0), translation_ratio_(0), flag_angular_velocity_pid_control_(false)
{
	motor_ = motor;
	encoder_ = encoder;
	imu_ = imu;

}

// ---------private ---------//

float VelocityControl::calcVelocity()
{
	float enc_l, enc_r;
	encoder_->getCnt(enc_l, enc_r);
	float enc_cnt = (enc_l + enc_r) / 2;

	current_velocity_ = DISTANCE_PER_CNT * enc_cnt;
	mon_current_velocity = current_velocity_;

	return current_velocity_;
}


void VelocityControl::pid()
{
	float static t_pre_diff, r_pre_diff;
	float t_diff = target_velocity_ - current_velocity_;
	float r_diff = target_omega_- imu_->getOmega();

	float t_p, t_d, r_p, r_d;
	static float t_i, r_i;

	if(i_reset_flag_ == true){
		t_i = r_i = 0;
		i_reset_flag_ = false;
	}

	t_p = t_kp_ * t_diff;
	t_i += t_ki_ * t_diff * DELTA_T;
	t_d = t_kd_ * (t_diff - t_pre_diff) / DELTA_T;

	r_p = r_kp_ * r_diff;
	r_i += r_ki_ * r_diff * DELTA_T;
	r_d = r_kd_ * (r_diff - r_pre_diff) / DELTA_T;

	if(t_i >= I_TERM_LIMIT) t_i = I_TERM_LIMIT;
	else if(t_i <= -I_TERM_LIMIT) t_i = -I_TERM_LIMIT;

	if(r_i >= I_TERM_LIMIT) r_i = I_TERM_LIMIT;
	else if(r_i <= -I_TERM_LIMIT) r_i = -I_TERM_LIMIT;

	float t_left_ratio, t_right_ratio, r_left_ratio, r_right_ratio;

	t_left_ratio = t_right_ratio =  t_p + t_d + t_i;

	r_left_ratio = r_p + r_d + r_i;
	r_right_ratio = -(r_p + r_d + r_i);

	motor_->setDuty(t_left_ratio + r_left_ratio, t_right_ratio + r_right_ratio);

	t_pre_diff = t_diff;
	r_pre_diff = r_diff;
}

void VelocityControl::pidTranslationOnly()
{
	float static t_pre_diff;
	float t_diff = target_velocity_ - current_velocity_;

	float t_p, t_d ;
	static float t_i;

	if(i_reset_flag_ == true){
		t_i = 0;
		i_reset_flag_ = false;
	}

	t_p = t_kp_ * t_diff;
	t_i += t_ki_ * t_diff * DELTA_T;
	t_d = t_kd_ * (t_diff - t_pre_diff) / DELTA_T;

	if(t_i >= I_TERM_LIMIT) t_i = I_TERM_LIMIT;
	else if(t_i <= -I_TERM_LIMIT) t_i = -I_TERM_LIMIT;

	translation_ratio_ =  t_p + t_d + t_i;

	float limit = 800;
	if(translation_ratio_ >= limit) translation_ratio_= limit;
	else if(translation_ratio_ <= -limit) translation_ratio_ = -limit;

	float exceeded = 0;
	if(translation_ratio_ + rotation_ratio_ >= 1000){
		exceeded = (translation_ratio_ + rotation_ratio_) - 1000;
	}
	else if(translation_ratio_ - rotation_ratio_ <= -1000){
		exceeded = -1000 - (translation_ratio_ - rotation_ratio_) ;
	}

	translation_ratio_ -= exceeded;
	rotation_ratio_ += exceeded;

	motor_->setDuty(translation_ratio_ + rotation_ratio_, translation_ratio_ - rotation_ratio_);

	t_pre_diff = t_diff;
}

// --------public -----------//

void VelocityControl::setTargetVelocity(float velocity, float omega)
{
	target_velocity_ = velocity;
	target_omega_= omega;
}

void VelocityControl::setTargetTranslationVelocityOnly(float velocity, float rotation_ratio)
{
	target_velocity_ = velocity;
	rotation_ratio_ = rotation_ratio;
}

void VelocityControl::setTranslationGain(float kp, float ki, float kd)
{
	t_kp_ = kp;
	t_ki_ = ki;
	t_kd_ = kd;
}

void VelocityControl::setRotationGain(float kp, float ki, float kd)
{
	r_kp_ = kp;
	r_ki_ = ki;
	r_kd_ = kd;
}

void VelocityControl::flip()
{
    calcVelocity();

	if(excution_flag_ == true){
		if(flag_angular_velocity_pid_control_ == true)	pid();
		else pidTranslationOnly();
	}


}

void VelocityControl::enableAngularVelocityPIDControl()
{
	flag_angular_velocity_pid_control_ = true;
}

void VelocityControl::disableAngularVelocityPIDControl()
{
	flag_angular_velocity_pid_control_ = false;
}

void VelocityControl::start()
{
	excution_flag_ = true;
	i_reset_flag_ = true;
}

void VelocityControl::stop()
{
	excution_flag_ = false;
	motor_->setDuty(0, 0);

}

float VelocityControl::getCurrentVelocity()
{
	return current_velocity_;
}

float VelocityControl::getTranslationRatio()
{
	return translation_ratio_;
}

float VelocityControl::getRotationRatio()
{
	return rotation_ratio_;
}

void VelocityControl::setClearFlagOfVelocityControlI()
{
	i_reset_flag_ = true;
}
