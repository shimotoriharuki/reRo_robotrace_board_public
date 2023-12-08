/*
 * RunningStateController.hpp
 *
 *  Created on: Jun 25, 2023
 *      Author: SHIMOTORI Haruki
 */

#ifndef INC_RUNNINGSTATECONTROLLER_HPP_
#define INC_RUNNINGSTATECONTROLLER_HPP_

#include "Motor.hpp"
#include "LineFollowing.hpp"
#include "FollowingSensor.hpp"
#include "SideSensor.hpp"
#include "VelocityControl.hpp"
#include "Encoder.hpp"
#include "IMU.hpp"
#include "UI.hpp"
#include "sdCard.hpp"
#include "Logger.hpp"

#define COURSE_STORAGE_SIZE 4000
#define GOAL_LINE_JUDGE_DISTANCE 70 //mm
#define SIDE_LINE_JUDGE_DISTANCE 70 //mm

class RunningStateController{
private:

	DriveMotor *drive_motor_;
	FanMotor *fan_motor_;
	LineFollowing *line_following_;
	FollowingSensor *following_sensor_;
	SideSensor *side_sensor_l_;
	SideSensor *side_sensor_r_;
	VelocityControl *velocity_control_;
	Encoder *encoder_;
	IMU *imu_;
	WheelDial *wheel_dial_;
	sdCard *sd_card_;
	Logger *searching_run_logger_distance_;
	Logger *searching_run_logger_theta_;
	Logger *searching_run_logger_side_;
	Logger *searching_run_logger_cross_;

	Logger *acc_dec_run_logger_distance_;
	Logger *acc_dec_run_logger_theta_;
	Logger *acc_dec_run_logger_side_;
	Logger *acc_dec_run_logger_cross_;

	Logger *current_velocity_logger_;
	Logger *target_velocity_logger_;

	Switch *push_switch_;
	LED led_;

	bool break_flag_;

	float velocity_table_[COURSE_STORAGE_SIZE];

	//uint8_t mon_is_crossline_;
	uint16_t velocity_table_idx_;
	uint16_t mode_;

	float ref_distance_;

	uint8_t start_goal_line_cnt_;
	bool logging_flag_;
	bool velocity_update_flag_;

	bool cross_line_ignore_flag_;
	bool side_line_ignore_flag_;
	bool goal_judge_flag_ = false;
	bool side_line_judge_flag_ = false;
	bool continuous_cnt_reset_flag_ = false;
	bool continuous_curve_flag_ = false;
	bool running_flag_ = false;

	uint16_t cross_line_idx_;
	uint16_t side_line_idx_;

	uint16_t correction_check_cnt_cross_, correction_check_cnt_side_;
	uint16_t continuous_curve_check_cnt_;

	float min_velocity_, max_velocity_;
	float acceleration_, deceleration_;
	float straight_radius_;

	float linear_function(float a, int16_t r, int16_t r_shift, float b);

public:

	RunningStateController(DriveMotor *, FanMotor *, LineFollowing *, FollowingSensor *, SideSensor *, SideSensor *, VelocityControl *, Encoder *, IMU *, WheelDial *, sdCard *);
	void timerCountUp();

	bool isCrossLine();
	bool isContinuousCurvature();

	void setRunMode(uint16_t);
	bool isTargetDistance(float);

	void loop();
	void flip();
	void flip10ms();
	void init();

	void storeLog();
	void startLogging();
	void stopLogging();
	void saveLog();

	void startVelocityUpdate();
	void stopVelocityUpdate();
	void createVelocityTable();
	float radius2Velocity(float);
	void decelerateProcessing(const float, const float *);
	void accelerateProcessing(const float, const float *);
	void addDecelerationDistanceMergin(float *, int16_t);
	void addAccelerationDistanceMergin(float *, int16_t);
	void shiftVelocityTable(float *, int16_t);
	void updateTargetVelocity();
	float getVelocityTableValue(uint16_t);

	void correctionTotalDistanceFromCrossLine();
	void correctionTotalDistanceFromSideLine();

	void setMaxVelocity(float);
	void setMinVelocity(float);
	void setAccDec(float, float);
	void setStraightRadius(float);

	void loadSearchinRunLog();
};





#endif /* INC_RUNNINGSTATECONTROLLER_HPP_ */
