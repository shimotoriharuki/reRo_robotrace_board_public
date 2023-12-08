/*
 * LineFollowing.hpp
 *
 *  Created on: Jun 21, 2023
 *      Author: SHIMOTORI Haruki
 */

#ifndef INC_LINEFOLLOWING_HPP_
#define INC_LINEFOLLOWING_HPP_

#include "FollowingSensor.hpp"
#include "VelocityControl.hpp"

class LineFollowing
{
private:
	bool i_reset_flag_;
	float kp_, ki_, kd_;
	float target_velocity_;
	float rotation_ratio_;
	bool processing_flag_;

	FollowingSensor *following_sensor_;
	VelocityControl *velocity_control_;

	float calcError();

public:
	LineFollowing(VelocityControl *, FollowingSensor *);

	void setTargetVelocity(float);
	float getTargetVelocity();
	void pidFlip();

	void setGain(float, float, float);
	void start();
	void stop();
	void emergencyStop();
};



#endif /* INC_LINEFOLLOWING_HPP_ */
