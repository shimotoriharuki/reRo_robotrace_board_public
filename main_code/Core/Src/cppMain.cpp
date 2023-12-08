/*
 * cppMain.cpp
 *
 *  Created on: Jun 19, 2023
 *      Author: SHIMOTORI Haruki
 */

#include "cppMain.hpp"
#include "Motor.hpp"
#include "Encoder.hpp"
#include "IMU.hpp"
#include "VelocityControl.hpp"
#include "UI.hpp"
#include "FollowingSensor.hpp"
#include "LineFollowing.hpp"
#include "Logger.hpp"
#include "Battery.hpp"
#include "ADC.hpp"
#include "IdleStateController.hpp"
#include "RunningStateController.hpp"
#include "SideSensor.hpp"

DriveMotor drive_motor;
FanMotor fan_motor(&htim3, TIM_CHANNEL_3);
//TemporaryFanMotor temp_fan_motor;

Encoder encoder;
IMU imu;
VelocityControl velocity_control(&drive_motor, &encoder, &imu);

ADConverter adc;
Switch push_switch_l(GPIOA, GPIO_PIN_12);
Switch push_switch_r(GPIOC, GPIO_PIN_9);
LED led;
WheelDial wheel_dial(&encoder);
FollowingSensor following_sensor(&adc);
SideSensor side_sensor_l(GPIOC, GPIO_PIN_13);
SideSensor side_sensor_r(GPIOC, GPIO_PIN_14);
LineFollowing line_following(&velocity_control, &following_sensor);

sdCard sd_card;
Logger logger(&sd_card, 10);
Battery battery(&adc);

RunningStateController running_state_controller(&drive_motor, &fan_motor, &line_following, &following_sensor, &side_sensor_l, &side_sensor_r, &velocity_control, &encoder, &imu, &wheel_dial, &sd_card);
IdleStateController idle_state_controller(&drive_motor, &fan_motor, &line_following, &following_sensor, &velocity_control, &encoder, &imu, &wheel_dial, &sd_card, &running_state_controller);

bool mon_side_l, mon_side_r;

void cppInit(void)
{
	//アクチュエータ初期化
	drive_motor.init();
	//fan_motor.init();
	fan_motor.init();

	//センサー初期化
	adc.start();
	imu.init();
	encoder.init();

	//その他初期化
	bool mount_check = sd_card.init();
	if(mount_check == true){
		//logger.saveLogs("TEST", "test");
	}


	//タイマー割り込み開始
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim10);

	//キャリブレーションなど
	following_sensor.calibration();
	HAL_Delay(500);

	led.set(0x01);
	imu.calibration();
	led.set(0x00);


	//ゲインの設定
	line_following.setGain(0.23, 0, 0.0045); //吸引ありゲイン /0.20, 0, 0.0045
	velocity_control.setTranslationGain(1000, 12000, 0); //吸引ありゲイン 1200, 12000, 0
	velocity_control.setRotationGain(0, 0, 0);

}

void cppExit(uint16_t gpio_pin)
{
	if(gpio_pin == GPIO_PIN_14){ //Right
		side_sensor_r.externalInterrupt();
	}
	else if(gpio_pin == GPIO_PIN_13){ //Left
		side_sensor_l.externalInterrupt();
	}
}

void cppFlip1ms(void)
{
	//センサー値を更新
	imu.updateValues();
	following_sensor.updateValues();
	encoder.update();
	side_sensor_l.updateState();
	side_sensor_r.updateState();

	mon_side_l = side_sensor_l.getState();
	mon_side_r = side_sensor_r.getState();

	//走行時のシーケンス制御
	running_state_controller.flip();

	//各種制御など
	line_following.pidFlip();
	velocity_control.flip();

	//モータを駆動
	drive_motor.controlFlip();
	//fan_motor.controlFlip();
	fan_motor.controlFlip();


	//その他
	battery.getVoltage();
	wheel_dial.flip();
	running_state_controller.timerCountUp();

	/*
	static uint16_t cnt;
	static int8_t led_state;
	cnt++;
	if(cnt >= 500){
		cnt = 0;

		led_state++;
		if(led_state >= 8) led_state = 0;

		led.set(led_state);
	}
	*/




}

void cppFlip100ns(void)
{
	following_sensor.storeValues();

}

void cppFlip10ms(void)
{
	running_state_controller.flip10ms();
}


void cppLoop(void)
{
	idle_state_controller.loop(); //アイドル状態ループ．走行するときはこのループから抜ける

}
