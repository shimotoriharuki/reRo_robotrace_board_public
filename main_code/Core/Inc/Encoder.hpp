/*
 * Encoder.hpp
 *
 *  Created on: Jun 21, 2023
 *      Author: SHIMOTORI Haruki
 */

#ifndef INC_ENCODER_HPP_
#define INC_ENCODER_HPP_

//#define R_IMU 0.03//0.03 Lowpath filter constant. The smaller it is, the more effective/

#include "stm32f4xx_hal.h"
#include "main.h"

class Encoder{

private:
	float cnt_l_, cnt_r_;
	float distance_; //[mm]
	float total_cnt_l_, total_cnt_r_;
	float distance_10mm_;
	float total_distance_;
	float side_line_judge_distance_;
	float cross_line_judge_distance_;
	float goal_judge_distance_;
	float goal_area_distance_;

public:
	Encoder();
	void init();
	void update();
	void getCnt(float &, float &);
	float getDistance();
	float getDistance10mm();
	float getTotalDistance();
	void setTotalDistance(float);
	void clearDistance();
	float getTotalCnt();
	void clearDistance10mm();
	void clearTotalDistance();
	float getSideLineJudgeDistance();
	void clearSideLineJudgeDistance();
	float getCrossLineJudgeDistance();
	void clearCrossLineJudgeDistance();
	float getGoalJudgeDistance();
	void clearGoalJudgeDistance();
	float getGoalAreaDistance();
	void clearGoalAreaDistance();
};




#endif /* INC_ENCODER_HPP_ */
