/*
 * RunningStateController.cpp
 *
 *  Created on: Jun 25, 2023
 *      Author: SHIMOTORI Haruki
 */

#include <globalDefine.h>
#include <RunningStateController.hpp>
#include <stm32f446xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <sys/_stdint.h>
#include <cmath>

bool mon_flag;
bool mon_status;
float mon_judge_distance;

float mon_th_10mm;
bool emergerncy_flag;

float RunningStateController::linear_function(float a, int16_t r, int16_t r_shift, float b)
{
    return a * (r - r_shift) + b;
}

RunningStateController::RunningStateController(DriveMotor *drive_motor, FanMotor *fan_motor, LineFollowing *line_following, FollowingSensor *following_sensor,
		SideSensor *side_sensor_l, SideSensor *side_sensor_r, VelocityControl *velocity_control, Encoder *encoder, IMU *imu, WheelDial *wheel_dial, sdCard *sd_card) : break_flag_(false),
				velocity_table_idx_(0), start_goal_line_cnt_(0), logging_flag_(false), velocity_update_flag_(false), cross_line_ignore_flag_(false), goal_judge_flag_(false),
				side_line_judge_flag_(false), continuous_cnt_reset_flag_(false), continuous_curve_flag_(false), running_flag_(false), cross_line_idx_(0), side_line_idx_(0), correction_check_cnt_cross_(0),
				correction_check_cnt_side_(0), continuous_curve_check_cnt_(0), min_velocity_(0), max_velocity_(0), acceleration_(0), deceleration_(0), straight_radius_(0)
{
	drive_motor_ = drive_motor;
	fan_motor_ = fan_motor;
	line_following_ = line_following;
	following_sensor_ = following_sensor;
	side_sensor_l_ = side_sensor_l;
	side_sensor_r_ = side_sensor_r;
	velocity_control_ = velocity_control;
	encoder_ = encoder;
	imu_ = imu;
	wheel_dial_ = wheel_dial;
	sd_card_ = sd_card;
	searching_run_logger_distance_ = new Logger(sd_card_, COURSE_STORAGE_SIZE);
	searching_run_logger_theta_ = new Logger(sd_card_, COURSE_STORAGE_SIZE);
	searching_run_logger_side_= new Logger(sd_card_, 100);
	searching_run_logger_cross_= new Logger(sd_card_, 100);

	acc_dec_run_logger_distance_ = new Logger(sd_card_, COURSE_STORAGE_SIZE);
	acc_dec_run_logger_theta_ = new Logger(sd_card_, COURSE_STORAGE_SIZE);
	acc_dec_run_logger_side_= new Logger(sd_card_, 100);
	acc_dec_run_logger_cross_= new Logger(sd_card_, 100);

	current_velocity_logger_ = new Logger(sd_card_, 1500);
	target_velocity_logger_ = new Logger(sd_card_, 1500);

	push_switch_ = new Switch(GPIOA, GPIO_PIN_12);

	for(uint16_t i = 0; i < COURSE_STORAGE_SIZE; i++){
		velocity_table_[i] = 0;
	}
}

void RunningStateController::timerCountUp()
{

}


bool RunningStateController::isCrossLine()
{
	static uint16_t cnt = 0;
	//float sensor_edge_val_l = (sensor[0] + sensor[1]) / 2;
	//float sensor_edge_val_r = (sensor[10] + sensor[11]) / 2;
	float sensor_edge_val_l = (following_sensor_->sensor[2] + following_sensor_->sensor[3]) / 2;
	float sensor_edge_val_r = (following_sensor_->sensor[9] + following_sensor_->sensor[10]) / 2;
	static bool flag = false;

	if(sensor_edge_val_l < 700 && sensor_edge_val_r < 700){
		cnt++;
	}
	else{
		cnt = 0;
	}

	if(cnt >= 3){
		flag = true;
	}
	else{
		flag = false;
	}

	return flag;
}

bool RunningStateController::isContinuousCurvature()
{
	static float pre_theta;
	static float continuous_cnt;
	bool continuous_flag = false;
	float diff_theta = abs(pre_theta - imu_->getTheta10mm());

	if(continuous_cnt_reset_flag_ == true){
		continuous_cnt_reset_flag_ = false;
		continuous_cnt = 0;
	}

	//if(diff_theta <= 0.005) continuous_cnt++;
	//if(diff_theta <= 0.010) continuous_cnt++;
	if(diff_theta <= 0.020) continuous_cnt++;
	else continuous_cnt = 0;

	if(continuous_cnt >= 40) continuous_flag = true;

	if(continuous_cnt >= 1000) continuous_cnt = 1000;

	pre_theta = imu_->getTheta10mm();

	return continuous_flag;
}

void RunningStateController::setRunMode(uint16_t num)
{
	mode_ = num;
}

bool RunningStateController::isTargetDistance(float target)
{
	bool ret = false;
	if(encoder_->getDistance10mm() >= target){
		ret = true;
	}
	return ret;
}

void RunningStateController::loop()
{
	break_flag_ = false; //もう一度この関数に戻ってきたとき用にfalseにする

	//走行開始直前の初期化
	init();

	//スタートゴール判定処理
	uint16_t pattern = 0;
	while(break_flag_ == false){
		switch(pattern){
		case 0:
			if(side_sensor_r_->getState() == true){ //最初のスタートマーカを読んだ
				start_goal_line_cnt_++;

				startLogging(); //ログ取り開始

				if(mode_ != 1) startVelocityUpdate(); //探索以外は速度アップデートを開始

				encoder_->clearGoalJudgeDistance();
				encoder_->clearSideLineJudgeDistance();
				pattern = 5;
			}

			break;

		case 5:
			if(side_sensor_r_->getState() == false){ //最初のスタートマーカを過ぎた
				pattern = 10;
				encoder_->clearGoalJudgeDistance();
			}
			break;

		case 10:
			//--- Goal marker Check ---//
			if(side_sensor_l_->getState() == true || side_line_ignore_flag_ == true){ //左マーカを検知したら即座にgoal_judge_flagを下ろして一定距離はゴール判定を行わないようにする
				goal_judge_flag_ = false;
				encoder_->clearGoalJudgeDistance();
			}

			if(goal_judge_flag_ == false && side_sensor_r_->getState() == true && encoder_->getGoalJudgeDistance() >= GOAL_LINE_JUDGE_DISTANCE){ //右マーカを検知かつ左マーカを読んでから一定距離以上進んでたらgoal_judge_flagを上げる
				goal_judge_flag_ = true;
				encoder_->clearGoalJudgeDistance();
			}
			else if(goal_judge_flag_ == true && encoder_->getGoalJudgeDistance() >= GOAL_LINE_JUDGE_DISTANCE){ //goal_judge_flagは上がったまま一定距離進んだらゴールマーカーを読んだ判定
				start_goal_line_cnt_++;
				goal_judge_flag_ = false;
				encoder_->clearGoalJudgeDistance();
			}

			if(start_goal_line_cnt_ >= 2){ //ゴールした
				stopLogging();
				stopVelocityUpdate();
				encoder_->clearGoalAreaDistance();
				pattern = 20;
			}

			break;

		case 20:
			if(encoder_->getGoalAreaDistance() >= 100){
				line_following_->stop(); //ゴールエリアで止まる処理
				fan_motor_->setDuty(0);
				emergerncy_flag = false;

				pattern = 30;
			}

			break;

		case 30: //後始末
			running_flag_ = false;

			saveLog();

			while(push_switch_->getStatus() == false){
				if(emergerncy_flag == true){
					led_.set(0x7);
					HAL_Delay(100);
					led_.set(0x00);
					HAL_Delay(100);
				}
				else{
					led_.set(0x07);
				}
			}

			HAL_Delay(1000);

			break_flag_ = true;

			break;
		}


		//緊急停止処理
		if(following_sensor_->isAllSensorBlack() == true){
			stopLogging();

			line_following_->emergencyStop();
			fan_motor_->setDuty(0);
			emergerncy_flag = true;

			pattern = 30;
		}

	}
}
void RunningStateController::flip()
{
	if(running_flag_ == true){
		updateTargetVelocity();

		if(isTargetDistance(10) == true){ //10mmごとの処理
			storeLog();

			if(isContinuousCurvature() == true){ //同じ曲率を走っているか判定
				continuous_curve_flag_ = true;
			}

			encoder_->clearDistance10mm();
			imu_->clearTheta10mm();
		}

		//--- Cross Line Process ---//
		if(isCrossLine() == true && cross_line_ignore_flag_ == false){ //Cross line detect
			cross_line_ignore_flag_ = true;
			continuous_curve_flag_ = true; //クロスラインということは直線なはずなのでフラグを立てる
			side_line_ignore_flag_ = true;
			encoder_->clearCrossLineJudgeDistance();
			encoder_->clearSideLineJudgeDistance();

			if(mode_ == 1){
				correction_check_cnt_cross_ = 0;
				searching_run_logger_cross_->storeLogs(encoder_->getTotalDistance());
			}
			else{
				correctionTotalDistanceFromCrossLine();
				acc_dec_run_logger_cross_->storeLogs(encoder_->getTotalDistance());
			}
		}
		else if(cross_line_ignore_flag_ == true && encoder_->getCrossLineJudgeDistance() >= 80){ //クロスライン読んでから一定距離立ったらフラグを下げる
			cross_line_ignore_flag_ = false;
			side_line_ignore_flag_ = false;
		}

		//--- Side marker Process---//
		if(side_sensor_r_->getState() == true){ //右マーカを読んだら即座にフラグを下げる
			side_line_judge_flag_ = false;
			encoder_->clearSideLineJudgeDistance();
		}
		if(side_line_judge_flag_ == false && side_sensor_l_->getState() == true && encoder_->getSideLineJudgeDistance() >= SIDE_LINE_JUDGE_DISTANCE){
			side_line_judge_flag_ = true;
			encoder_->clearSideLineJudgeDistance();
		}
		else if(side_line_judge_flag_ == true && encoder_->getSideLineJudgeDistance() >= SIDE_LINE_JUDGE_DISTANCE){ //サイドライン読んだ判定
			encoder_->clearSideLineJudgeDistance();
			side_line_judge_flag_ = false;

			if(continuous_curve_flag_ == true){ //サイドライン読んだときにしばらく一定の曲率を曲がっていたら
				continuous_curve_flag_ = false;
				continuous_cnt_reset_flag_ = true;

				if(mode_ == 1){
					correction_check_cnt_side_ = 0;
					searching_run_logger_side_->storeLogs(encoder_->getTotalDistance()); //補正用のサイドマーカーの位置を記録
				}
				else{
					correctionTotalDistanceFromSideLine();
					acc_dec_run_logger_side_->storeLogs(encoder_->getTotalDistance()); //補正用のサイドマーカーの位置を記録
				}
			}
		}


		// Debug LED //
		correction_check_cnt_cross_++;
		if(correction_check_cnt_cross_ >= 10000) correction_check_cnt_cross_ = 10000;

		correction_check_cnt_side_++;
		if(correction_check_cnt_side_ >= 10000) correction_check_cnt_side_ = 10000;

		continuous_curve_check_cnt_++;
		if(continuous_curve_check_cnt_ >= 10000) continuous_curve_check_cnt_ = 10000;

		static uint8_t led_state;
		if(correction_check_cnt_cross_ <= 150) led_.set(led_state | 0x01);
		else led_.set(led_state & ~0x01);
		if(correction_check_cnt_side_ <= 150) led_.set(led_state | 0x02);
		else led_.set(led_state & ~0x02);
		if(continuous_curve_flag_ == true) led_.set(led_state | 0x04);
		else led_.set(led_state & ~0x04);
	}


}

void RunningStateController::flip10ms()
{
	if(logging_flag_ == true){
		current_velocity_logger_->storeLogs(velocity_control_->getCurrentVelocity());
		target_velocity_logger_ ->storeLogs(line_following_->getTargetVelocity());
	}
}

void RunningStateController::init()
{
	if(mode_ != 1){
		createVelocityTable();
	}

	encoder_->clearCrossLineJudgeDistance();
	encoder_->clearSideLineJudgeDistance();
	encoder_->clearTotalDistance();

	start_goal_line_cnt_ = 0;
	cross_line_ignore_flag_ = false;
	side_line_ignore_flag_ = false;
	goal_judge_flag_ = false;
	side_line_judge_flag_ = false;
	continuous_cnt_reset_flag_ = true;
	continuous_curve_flag_ = false;
	running_flag_ = true;

	fan_motor_->setDuty(SUCTION_DUTY);
	HAL_Delay(2000);

	line_following_->setTargetVelocity(min_velocity_);
	line_following_->start();
}

void RunningStateController::storeLog()
{

	/*if(logging_flag_ == true){
		searching_run_logger_distance_->storeLogs(encoder_->getDistance10mm());
		searching_run_logger_theta_->storeLogs(imu_->getTheta10mm());
	}
	else if(velocity_update_flag_ == true){
		//saveDebug(getTargetVelocity());
		//saveDebug(getCurrentVelocity());
	}
	*/
	if(logging_flag_ == true){
		if(mode_ == 1){
			searching_run_logger_distance_->storeLogs(encoder_->getDistance10mm());
			searching_run_logger_theta_->storeLogs(imu_->getTheta10mm());

		}
		else{
			acc_dec_run_logger_distance_->storeLogs(encoder_->getDistance10mm());
			acc_dec_run_logger_theta_->storeLogs(imu_->getTheta10mm());
		}
	}
}

void RunningStateController::startLogging()
{
	encoder_->clearDistance10mm();
	imu_->clearTheta10mm();
	encoder_->clearTotalDistance();
	logging_flag_ = true;

	if(mode_ == 1){ //探索のときは探索用のログを取る
		searching_run_logger_distance_->clearLogs();
		searching_run_logger_theta_->clearLogs();
		searching_run_logger_cross_->clearLogs();
		searching_run_logger_side_->clearLogs();

		searching_run_logger_distance_->start();
		searching_run_logger_theta_->start();
		searching_run_logger_cross_->start();
		searching_run_logger_side_->start();
	}
	else{ //探索以外はデバッグ用のログを取る
		acc_dec_run_logger_distance_->clearLogs();
		acc_dec_run_logger_theta_->clearLogs();
		acc_dec_run_logger_cross_->clearLogs();
		acc_dec_run_logger_side_->clearLogs();

		acc_dec_run_logger_distance_->start();
		acc_dec_run_logger_theta_->start();
		acc_dec_run_logger_cross_->start();
		acc_dec_run_logger_side_->start();
	}

	current_velocity_logger_->clearLogs();
	target_velocity_logger_->clearLogs();

	current_velocity_logger_->start(); //速度追従のログはいつでも取る
	target_velocity_logger_->start();
}

void RunningStateController::stopLogging()
{
	logging_flag_ = false;

	if(mode_ == 1){
		searching_run_logger_distance_->stop();
		searching_run_logger_theta_->stop();
		searching_run_logger_cross_->stop();
		searching_run_logger_side_->stop();
	}
	else{
		acc_dec_run_logger_distance_->stop();
		acc_dec_run_logger_theta_->stop();
		acc_dec_run_logger_cross_->stop();
		acc_dec_run_logger_side_->stop();
	}

	current_velocity_logger_->stop();
	target_velocity_logger_->stop();
}

void RunningStateController::saveLog()
{
	if(mode_ == 1){ //First running
		searching_run_logger_distance_->saveLogs("run_1", "distance");
		searching_run_logger_theta_->saveLogs("run_1", "theta");
		searching_run_logger_cross_->saveLogs("run_1", "cross");
		searching_run_logger_side_->saveLogs("run_1", "side");

		current_velocity_logger_->saveLogs("run_1", "current_velocity");
		target_velocity_logger_->saveLogs("run_1", "target_velocity");
	}
	else if(mode_ == 2){
		acc_dec_run_logger_distance_->saveLogs("run_2", "distance");
		acc_dec_run_logger_theta_->saveLogs("run_2", "theta");
		acc_dec_run_logger_cross_->saveLogs("run_2", "cross");
		acc_dec_run_logger_side_->saveLogs("run_2", "side");

		current_velocity_logger_->saveLogs("run_2", "current_velocity");
		target_velocity_logger_->saveLogs("run_2", "target_velocity");
	}
	else if(mode_ == 3){
		acc_dec_run_logger_distance_->saveLogs("run_3", "distance");
		acc_dec_run_logger_theta_->saveLogs("run_3", "theta");
		acc_dec_run_logger_cross_->saveLogs("run_3", "cross");
		acc_dec_run_logger_side_->saveLogs("run_3", "side");

		current_velocity_logger_->saveLogs("run_3", "current_velocity");
		target_velocity_logger_->saveLogs("run_3", "target_velocity");
	}
	else if(mode_ == 4){
		acc_dec_run_logger_distance_->saveLogs("run_4", "distance");
		acc_dec_run_logger_theta_->saveLogs("run_4", "theta");
		acc_dec_run_logger_cross_->saveLogs("run_4", "cross");
		acc_dec_run_logger_side_->saveLogs("run_4", "side");

		current_velocity_logger_->saveLogs("run_4", "current_velocity");
		target_velocity_logger_->saveLogs("run_4", "target_velocity");
	}
	else if(mode_ == 5){
		acc_dec_run_logger_distance_->saveLogs("run_5", "distance");
		acc_dec_run_logger_theta_->saveLogs("run_5", "theta");
		acc_dec_run_logger_cross_->saveLogs("run_5", "cross");
		acc_dec_run_logger_side_->saveLogs("run_5", "side");

		current_velocity_logger_->saveLogs("run_5", "current_velocity");
		target_velocity_logger_->saveLogs("run_5", "target_velocity");
	}
}

void RunningStateController::startVelocityUpdate(){
	encoder_->clearDistance10mm();
	encoder_->clearTotalDistance();
	velocity_table_idx_ = 0;
	ref_distance_ = 0;
	velocity_update_flag_ = true;

	cross_line_idx_ = 0;
	side_line_idx_ = 0;
}

void RunningStateController::stopVelocityUpdate()
{
	velocity_update_flag_ = false;
}

void RunningStateController::createVelocityTable(){
	const float *p_distance, *p_theta;
	p_distance = searching_run_logger_distance_->getLogsPointer();
	p_theta = searching_run_logger_theta_->getLogsPointer();
	float temp_distance, temp_theta;

	uint16_t log_size = searching_run_logger_distance_->getLogsSize();

	uint16_t crossline_idx = 0;
	float total_distance = 0;
	for(uint16_t i = 0; i < log_size; i++){
		temp_distance = p_distance[i];
		temp_theta = p_theta[i];

		if(temp_theta == 0) temp_theta = 0.00001;
		float radius = fabs(temp_distance / temp_theta);
		if(radius >= straight_radius_) radius = straight_radius_;
		velocity_table_[i] = radius2Velocity(radius);

		//Forced maximum speed on the crossline
		total_distance += temp_distance;
		float crossline_distance = searching_run_logger_cross_->getLogData(crossline_idx);
		if(crossline_distance + 60 >= total_distance && total_distance >= crossline_distance - 60){
			velocity_table_[i] = max_velocity_;
		}

		if(total_distance >= crossline_distance + 60){
			crossline_idx++;
		}
	}
	for(uint16_t i = log_size; i < COURSE_STORAGE_SIZE; i++){
		velocity_table_[i] = max_velocity_;
		//velocity_table_[i] = 3.5;
	}


	addDecelerationDistanceMergin(velocity_table_, 20); //10
	addAccelerationDistanceMergin(velocity_table_, 25); //15
	//shiftVelocityTable(velocity_table_, 2); //5

	velocity_table_[0] = min_velocity_;

	decelerateProcessing(deceleration_, p_distance);
	accelerateProcessing(acceleration_, p_distance);

}

float RunningStateController::radius2Velocity(float radius){
	float velocity;

	if(mode_ == 2){
		velocity = 1e-3 * radius * radius * ((max_velocity_ - min_velocity_) / straight_radius_) + min_velocity_; // quadratic function
	}
	else if(mode_ == 3){
		//velocity = 1*radius * ((max_velocity_ - min_velocity_) / straight_radius_) + min_velocity_; // linear function
		/*
		float adjust_x = 25;
		float adjust_y = 0;
		float gain = 0.45;
		float facter = straight_radius_/adjust_x;
		velocity = (1 / (1 + exp(-(gain/facter) * radius + (adjust_x / 2) * gain))) * (max_velocity_ - min_velocity_) + min_velocity_ + adjust_y; // sigmoid function
		*/
		//9.90675990675985e-12	-2.17560217560216e-08	1.83041958041957e-05	-0.00195843045843044	2.51958041958042
		velocity = 9.90675990675985e-12 * pow(radius, 4) + -2.17560217560216e-08 * pow(radius, 3) + 1.83041958041957e-05 * pow(radius, 2) + -0.00195843045843044 * radius + 2.51958041958042;
	}
	else if(mode_ == 4){
		//velocity = 1*radius * ((max_velocity_ - min_velocity_) / straight_radius_) + min_velocity_; // linear function
		//条件別1次関数
		uint16_t r1 = 400;
		uint16_t r2 = 800;

		float ratio1 = 0.4;
		float ratio2 = 0.9;

		float a = (max_velocity_ - min_velocity_) / straight_radius_;

		float v1 = linear_function(a, r1, 0, min_velocity_);
		float vv1 = (v1-min_velocity_) * ratio1 + min_velocity_;
		float a1 = (vv1 - min_velocity_) / r1;

		float v2 = linear_function(a, r2, 0, min_velocity_);
		float vv2 = (v2-min_velocity_) * ratio2 + min_velocity_;
		float a2 = (vv2 - vv1) / (r2 - r1);

		float v3 = max_velocity_;
		float vv3 = v3;
		float a3 = (vv3 - vv2) / (straight_radius_ - r2);

		if (radius < r1){
			velocity = linear_function(a1, radius, 0, min_velocity_);
		}
		else if (radius < r2){
			velocity = linear_function(a2, radius, r1, vv1);
		}
		else{
			velocity = linear_function(a3, radius, r2, vv2);
		}
		//velocity = 1.13636363636363e-11 * pow(radius, 4) - 3.26534576534575e-08 * pow(radius, 3) + 2.94551282051281e-05 * pow(radius, 2) - 0.00420726495726492 * radius + 2.55244755244755;
		//velocity = -4.07925407925417e-12 * pow(radius, 4) + 1.63170163170184e-09 * pow(radius, 3) + 5.68181818181801e-06 * pow(radius, 2) + 0.000832167832167887 * radius + 2.44965034965034;

		//velocity = 1*radius * ((max_velocity_ - min_velocity_) / straight_radius_) + min_velocity_; // linear function
		/*
		float adjust_x = 25;
		float adjust_y = 0;
		float gain = 0.45;
		float facter = straight_radius_/adjust_x;
		velocity = (1 / (1 + exp(-(gain/facter) * radius + (adjust_x / 2) * gain))) * (max_velocity_ - min_velocity_) + min_velocity_ + adjust_y; // sigmoid function
		*/
	}
	else if(mode_ == 5){
		//velocity = 1*radius * ((max_velocity_ - min_velocity_) / straight_radius_) + min_velocity_; // linear function

		float adjust_x = 25;
		float adjust_y = 0;
		float gain = 0.30;
		float facter = straight_radius_/adjust_x;
		velocity = (1 / (1 + exp(-(gain/facter) * radius + (adjust_x / 2) * gain))) * (max_velocity_ - min_velocity_) + min_velocity_ + adjust_y; // sigmoid function
	}

	return velocity;
}

void RunningStateController::decelerateProcessing(const float am, const float *p_distance){
	uint16_t log_size = searching_run_logger_distance_->getLogsSize();
	for(uint16_t i = log_size - 1; i >= 1; i--){
		float v_diff = velocity_table_[i-1] - velocity_table_[i];

		if(v_diff > 0){
			float t = p_distance[i]*1e-3 / v_diff;
			float a = v_diff / t;
			if(a > am){
				velocity_table_[i-1] = velocity_table_[i] + am * p_distance[i]*1e-3;
			}
		}
	}
}

void RunningStateController::accelerateProcessing(const float am, const float *p_distance){
	uint16_t log_size = searching_run_logger_distance_->getLogsSize();
	for(uint16_t i = 0; i <= log_size - 1; i++){
		float v_diff = velocity_table_[i+1] - velocity_table_[i];

		if(v_diff > 0){
			float t = p_distance[i]*1e-3 / v_diff;
			float a = v_diff / t;
			if(a > am){
				velocity_table_[i+1] = velocity_table_[i] + am * p_distance[i]*1e-3;
			}
		}
	}
}

void RunningStateController::addDecelerationDistanceMergin(float *table, int16_t mergin_size)
{
	//setRGB('R');
	uint16_t idx = mergin_size;
	float pre_target_velocity = table[idx];

	while(idx <= COURSE_STORAGE_SIZE - 1){
		if(pre_target_velocity > table[idx]){
			float low_velocity = table[idx];
			for(uint16_t i = idx - mergin_size; i < idx; i++){
				table[i] = low_velocity;
			}
			pre_target_velocity = table[idx];
		}

		pre_target_velocity = table[idx];

		idx++;
	}
	//setRGB('r');

}

void RunningStateController::addAccelerationDistanceMergin(float *table, int16_t mergin_size)
{
	uint16_t idx = 0;
	float pre_target_velocity = table[idx];

	while(idx <= COURSE_STORAGE_SIZE - 1 - mergin_size){
		if(pre_target_velocity < table[idx]){
			float low_velocity = pre_target_velocity;
			for(uint16_t i = idx; i < idx + mergin_size; i++){
				table[i] = low_velocity;
			}
			idx += mergin_size;
			pre_target_velocity = table[idx];
		}

		pre_target_velocity = table[idx];

		idx++;
	}
	//setRGB('r');

}

void RunningStateController::shiftVelocityTable(float *table, int16_t shitf_size)
{
	for(uint16_t i = shitf_size; i < COURSE_STORAGE_SIZE; i++){
		table[i - shitf_size] = table[i];
	}

	for(uint16_t i = COURSE_STORAGE_SIZE - 1 - shitf_size; i < COURSE_STORAGE_SIZE; i++){
		table[i] = max_velocity_;
	}


}

void RunningStateController::updateTargetVelocity(){
	static float pre_target_velocity;

	if(velocity_update_flag_ == true){
		if(encoder_->getTotalDistance() >= ref_distance_){
			ref_distance_ += searching_run_logger_distance_->getLogData(velocity_table_idx_);
			velocity_table_idx_++;
		}
		if(velocity_table_idx_ >= searching_run_logger_distance_->getLogsSize()){
			velocity_table_idx_ = searching_run_logger_distance_->getLogsSize() - 1;
		}

		line_following_->setTargetVelocity(velocity_table_[velocity_table_idx_]);

		/*
		float temp_distance = searching_run_logger_distance_->getLogData(velocity_table_idx_);
		float temp_theta = searching_run_logger_theta_ ->getLogData(velocity_table_idx_);
		float radius = fabs(temp_distance / temp_theta);
		if(radius >= straight_radius_) radius = straight_radius_;

		if(radius >= straight_radius_){ //直線のとき吸引弱める
			fan_motor_->setDuty(int(SUCTION_DUTY * 0.5));
		}
		else{
			fan_motor_->setDuty(SUCTION_DUTY);
		}
		*/

		if(pre_target_velocity > velocity_table_[velocity_table_idx_]){
			//velocity_control_->setClearFlagOfVelocityControlI(); //速度制御のI項をリセット
		}

		pre_target_velocity = velocity_table_[velocity_table_idx_];
	}
}

void RunningStateController::correctionTotalDistanceFromCrossLine()
{
	while(cross_line_idx_ <= searching_run_logger_cross_->getLogsSize()){
		float temp_crossline_distance = searching_run_logger_cross_->getLogData(cross_line_idx_);
		float diff = abs(temp_crossline_distance - encoder_->getTotalDistance());
		if(diff <= 250){
			correction_check_cnt_cross_ = 0;
			encoder_->setTotalDistance(temp_crossline_distance);
			cross_line_idx_++;
			break;
		}
		cross_line_idx_++;

		if(cross_line_idx_ >= searching_run_logger_cross_->getLogsSize()){
			cross_line_idx_ = searching_run_logger_cross_->getLogsSize() - 1;
			break;
		}
	}
}

void RunningStateController::correctionTotalDistanceFromSideLine()
{
	while(side_line_idx_ <= searching_run_logger_side_->getLogsSize()){
		float temp_sideline_distance = searching_run_logger_side_->getLogData(side_line_idx_);
		float diff = abs(temp_sideline_distance - encoder_->getTotalDistance());
		//if(diff <= 700){
		if(diff <= 250){
			correction_check_cnt_side_ = 0;
			encoder_->setTotalDistance(temp_sideline_distance);
			side_line_idx_++;
			break;
		}
		side_line_idx_++;

		if(side_line_idx_ >= searching_run_logger_side_->getLogsSize()){
			side_line_idx_ = searching_run_logger_side_->getLogsSize() - 1;
			break;
		}
	}
}

void RunningStateController::setMaxVelocity(float max_vel)
{
	max_velocity_ = max_vel;
}

void RunningStateController::setMinVelocity(float min_vel)
{
	min_velocity_ = min_vel;
}
void RunningStateController::setAccDec(float acc, float dec)
{
	acceleration_ = acc;
	deceleration_ = dec;
}

void RunningStateController::setStraightRadius(float radius)
{
	straight_radius_ = radius;
}

void RunningStateController::loadSearchinRunLog()
{
	searching_run_logger_distance_->importLatestLogs("run_1", "distance");
	searching_run_logger_theta_->importLatestLogs("run_1", "theta");
	searching_run_logger_side_->importLatestLogs("run_1", "side");
	searching_run_logger_cross_->importLatestLogs("run_1", "cross");

}



