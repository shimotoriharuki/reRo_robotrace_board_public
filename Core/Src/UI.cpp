/*
 * UI.cpp
 *
 *  Created on: Jun 21, 2023
 *      Author: SHIMOTORI Haruki
 */

#include "UI.hpp"

int16_t mon_sum_cnt_l, mon_sum_cnt_r;

//-----Switch------//
Switch::Switch(GPIO_TypeDef *gpio_type, uint16_t gpio_num) : gpio_type_(gpio_type), gpio_num_(gpio_num){}

bool Switch::getStatus()
{
	bool ret = false;

	if(HAL_GPIO_ReadPin(gpio_type_, gpio_num_) == 0) ret = true;
	else ret = false;

	return ret;
}


//-----LED------//
LED::LED(){}

void LED::set(int8_t state)
{
	if((state & 0x01) == 0x01)	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	}

	if((state & 0x02) == 0x02){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	}

	if((state & 0x04) == 0x04){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	}

}

//-----WheelDial------//
WheelDial::WheelDial(Encoder *encoder) : is_left_cw_turned_(false), is_right_cw_turned_(false)
{
	encoder_ = encoder;
}

void WheelDial::flip()
{
	static uint16_t clear_timer;
	static int16_t sum_cnt_l, sum_cnt_r;

	//if(push_switch_.getStatus() == true){ //スイッチ押してるときだけ実行する処理の名残．このままだと変更した先のメニューにスイッチを押したらスタートの処理があるとすぐに実行されてしまう．ボタンを押し込んでるときだけこの処理は動いて，スタートとかのときに押すスイッチは押してから早い時間で話されたらONみたいにすると良さげ
		float cnt_l, cnt_r;
		encoder_->getCnt(cnt_l, cnt_r);

		//エンコーダのカウントを積算する
		sum_cnt_l += cnt_l;
		sum_cnt_r += cnt_r;
		mon_sum_cnt_l = sum_cnt_l;
		mon_sum_cnt_r = sum_cnt_r;


		//clear_timerが一定時間経過してsum_cnt*が0になるまでに一定値積算されたら回った判定
		if(sum_cnt_l >= TURNED_THRESHOLD){
			is_left_cw_turned_ = true;
			sum_cnt_l = 0;
		}
		else if(sum_cnt_l <= -TURNED_THRESHOLD){
			is_left_ccw_turned_ = true;
			sum_cnt_l = 0;
		}

		if(sum_cnt_r >= TURNED_THRESHOLD){
			is_right_cw_turned_ = true;
			sum_cnt_r = 0;
		}
		else if(sum_cnt_r <= -TURNED_THRESHOLD){
			is_right_ccw_turned_ = true;
			sum_cnt_r = 0;
		}


		//ホイールが回っていない時間を計測
		if(cnt_l == 0 && cnt_r == 0){
			clear_timer++;
		}
		//ホイールが一定時間回ってなかったら積算値をクリア
		if(clear_timer >= 500){
			sum_cnt_l = sum_cnt_r = 0;
			clear_timer = 0;
		}
	//}
}

bool WheelDial::isCW(bool left_or_right)
{
	bool ret = false;

	if(left_or_right == LEFT){
		if(is_left_cw_turned_ == true){
			ret = true;
			is_left_cw_turned_ = false;
			is_left_ccw_turned_ = false;
		}
	}
	else if(left_or_right == RIGHT){
		if(is_right_cw_turned_ == true){
			ret = true;
			is_right_cw_turned_ = false;
			is_right_ccw_turned_ = false;
		}
	}

	return ret;

}

bool WheelDial::isCCW(bool left_or_right)
{
	bool ret = false;

	if(left_or_right == LEFT){
		if(is_left_ccw_turned_ == true){
			ret = true;
			is_left_cw_turned_ = false;
			is_left_ccw_turned_ = false;
		}
	}
	else if(left_or_right == RIGHT){
		if(is_right_ccw_turned_ == true){
			ret = true;
			is_right_cw_turned_ = false;
			is_right_ccw_turned_ = false;
		}
	}

	return ret;

}
