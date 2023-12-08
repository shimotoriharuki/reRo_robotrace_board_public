/*
 * IdleStateController.cpp
 *
 *  Created on: Jun 25, 2023
 *      Author: SHIMOTORI Haruki
 */
#include "UI.hpp"

#include "IdleStateController.hpp"

int8_t mon_state, mon_param_state;
//-------------------------//
//---------private---------//
//-------------------------//
void IdleStateController::parameterAdjustmentMode()
{
	bool break_flag = false;

	while(break_flag == false){
		//メニュー切り替えの制御
		if(wheel_dial_->isCW(RIGHT) == true) parameter_state_++;
		else if(wheel_dial_->isCCW(RIGHT) == true) parameter_state_--;

		if(parameter_state_ > 7) parameter_state_ = 0;
		else if(parameter_state_ < 0) parameter_state_ = 7;

		mon_param_state = parameter_state_;
		led_.set(parameter_state_);

		//条件分岐
		switch(parameter_state_){
		case 0: //速度制御確認
			if(push_switch_->getStatus() == true){
				HAL_Delay(500);

				fan_motor_->setDuty(SUCTION_DUTY);
				HAL_Delay(1000);

				velocity_control_->enableAngularVelocityPIDControl();
				velocity_control_->setTargetVelocity(0.0, 0);
				velocity_control_->start();

				HAL_Delay(3000);

				fan_motor_->setDuty(0);
				velocity_control_->stop();

				HAL_Delay(500);
			}
			break;

		case 1: //ライン追従制御確認
			if(push_switch_->getStatus() == true){
				HAL_Delay(500);

				fan_motor_->setDuty(SUCTION_DUTY);
				HAL_Delay(1000);

				line_following_->setTargetVelocity(0.0);
				line_following_->start();

				HAL_Delay(3000);

				fan_motor_->setDuty(0);
				line_following_->stop();

				HAL_Delay(500);
			}

			break;

		case 2: //ラインセンサキャリブレーション
			break;
		case 3:
			HAL_Delay(100);
			break;

		case 4: //吸引ファンテスト
			if(push_switch_->getStatus() == true){
				HAL_Delay(500);

				fan_motor_->setDuty(SUCTION_DUTY);

				while(push_switch_->getStatus() == false){
					HAL_Delay(10);
				}
				HAL_Delay(500);

				fan_motor_->setDuty(0);
			}

			HAL_Delay(100);
			break;

		case 5: //モータテスト
			if(push_switch_->getStatus() == true){
				HAL_Delay(500);
				drive_motor_->setDuty(300, 300);

				HAL_Delay(2000);

				drive_motor_->setDuty(0, 0);
			}
			break;

		case 6:
			if(push_switch_->getStatus() == true){
				HAL_Delay(500);
				logger_->start();

				logger_->storeLogs(0);
				logger_->storeLogs(1);
				logger_->storeLogs(2);
				logger_->storeLogs(3);

				logger_->stop();

				logger_->saveLogs("TEST", "test");
			}
			break;

		case 7:
			if(push_switch_->getStatus() == true){
				HAL_Delay(500);
				break_flag = true;
			}
			HAL_Delay(100);
			break;

		}
	}

}

//-------------------------//
//---------public----------//
//-------------------------//
IdleStateController::IdleStateController(DriveMotor *drive_motor, FanMotor *fan_motor, LineFollowing *line_following, FollowingSensor *following_sensor,
		VelocityControl *velocity_control, Encoder *encoder, IMU *imu, WheelDial *wheel_dial, sdCard *sd_card, RunningStateController *running_state_controller) :
		break_flag_(false), state_(0), parameter_state_(0)
{
	drive_motor_ = drive_motor;
	fan_motor_ = fan_motor;
	line_following_ = line_following;
	following_sensor_ = following_sensor;
	velocity_control_ = velocity_control;
	encoder_ = encoder;
	imu_ = imu;
	wheel_dial_ = wheel_dial;
	sd_card_ = sd_card;
	running_state_controller_ = running_state_controller;

	logger_ = new Logger(sd_card, 10);
	push_switch_ = new Switch(GPIOA, GPIO_PIN_12);

}

void IdleStateController::loop()
{
	break_flag_ = false; //もう一度この関数に帰ってきたときのためにfalseにしておく

	while(break_flag_ == false){

		//メニュー切り替えの制御
		if(wheel_dial_->isCW(RIGHT) == true) state_++;
		else if(wheel_dial_->isCCW(RIGHT) == true) state_--;

		if(state_ > 7) state_ = 0;
		else if(state_ < 0) state_ = 7;

		mon_state = state_;
		led_.set(state_);

		//条件分岐
		switch(state_){
		case 0:
			if(push_switch_->getStatus() == true){
				HAL_Delay(500);

				running_state_controller_->setRunMode(1);
				running_state_controller_->setMinVelocity(2.5);
				running_state_controller_->setAccDec(1.0, 1.0);
				running_state_controller_->setStraightRadius(1000);

				running_state_controller_->loop(); //走行状態ループ．

			}
			break;

		case 1:
			if(push_switch_->getStatus() == true){
				HAL_Delay(500);

				running_state_controller_->setRunMode(2);
				running_state_controller_->setMinVelocity(2.0);
				running_state_controller_->setMaxVelocity(6.0);
				running_state_controller_->setAccDec(2.0, 2.0);
				running_state_controller_->setStraightRadius(1000);

				running_state_controller_->loop(); //走行状態ループ．

			}
			break;

		case 2:
			if(push_switch_->getStatus() == true){
				HAL_Delay(500);

				running_state_controller_->setRunMode(3);
				running_state_controller_->setMinVelocity(2.5);
				running_state_controller_->setMaxVelocity(7.0);
				running_state_controller_->setAccDec(8.0, 6.0);
				running_state_controller_->setStraightRadius(1000);

				running_state_controller_->loop(); //走行状態ループ．
			}
			break;

		case 3:
			if(push_switch_->getStatus() == true){
				HAL_Delay(500);

				running_state_controller_->setRunMode(4);
				running_state_controller_->setMinVelocity(2.5);
				running_state_controller_->setMaxVelocity(7.0);
				running_state_controller_->setAccDec(10.0, 6.0);
				running_state_controller_->setStraightRadius(1000);

				running_state_controller_->loop(); //走行状態ループ．
			}
			break;

		case 4:
			if(push_switch_->getStatus() == true){
				HAL_Delay(500);

				running_state_controller_->setRunMode(5);
				running_state_controller_->setMinVelocity(2.5);
				running_state_controller_->setMaxVelocity(7.0);
				running_state_controller_->setAccDec(10.0, 10.0); //8.0, 8.0
				running_state_controller_->setStraightRadius(1000);

				running_state_controller_->loop(); //走行状態ループ．
			}
			break;

		case 5:
			HAL_Delay(100);
			break;

		case 6: //最新の記録したコースをロード
			if(push_switch_->getStatus() == true){
				HAL_Delay(500);
				running_state_controller_->loadSearchinRunLog();
			}

			break;

		case 7:
			HAL_Delay(100);
			if(push_switch_->getStatus() == true){
				HAL_Delay(500);
				parameterAdjustmentMode();
			}

			break;

		}
	}

}


