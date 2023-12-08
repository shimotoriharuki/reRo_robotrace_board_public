/*
 * Encoder.cpp
 *
 *  Created on: Jun 21, 2023
 *      Author: SHIMOTORI Haruki
 */

#include "Encoder.hpp"
#include "globalDefine.h"

#define MAX_ENCODER_CNT 65535
#define CNT_OFFSET 32768
#define CORRECTION_COEFFICIENT float(1)

float mon_distance;
float mon_cnt_l, mon_cnt_r;
int16_t mon_total_cnt_l, mon_total_cnt_r;

Encoder::Encoder() : cnt_l_(0), cnt_r_(0), distance_(0), total_cnt_l_(0), total_cnt_r_(0), distance_10mm_(0), total_distance_(0),
		side_line_judge_distance_(), cross_line_judge_distance_(0), goal_judge_distance_(0), goal_area_distance_(0){}

void Encoder::init()
{
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
	TIM4 -> CNT = CNT_OFFSET;
	TIM8 -> CNT = CNT_OFFSET;
}

void Encoder::update()
{
	//初期化する
	//cnt_l_ = 0;
	//cnt_r_ = 0;
	//distance_ = 0;

	int16_t cnt_l = ((TIM4 -> CNT) - (CNT_OFFSET)) ;
	int16_t cnt_r = ((CNT_OFFSET) - (TIM8 -> CNT)) ;

	//次の割り込みまでのカウントを計測するために初期化
	TIM4 -> CNT = CNT_OFFSET;
	TIM8 -> CNT = CNT_OFFSET;

	mon_cnt_l = cnt_l;
	mon_cnt_r = cnt_r;
	mon_total_cnt_l += cnt_l;
	mon_total_cnt_r += cnt_r;

	cnt_l_ = float(cnt_l);
	cnt_r_ = float(cnt_r);

	distance_ = DISTANCE_PER_CNT * (cnt_l_ + cnt_r_) / 2;
	//distance_ = DISTANCE_PER_CNT * (cnt_l_) ;
	distance_10mm_ += distance_;
	total_distance_ += distance_;
	side_line_judge_distance_ += distance_;
	cross_line_judge_distance_ += distance_;
	goal_judge_distance_ += distance_;
	goal_area_distance_ += distance_;

	mon_distance = total_distance_;

}

void Encoder::getCnt(float &cnt_l, float &cnt_r)
{
	cnt_l = cnt_l_;
	cnt_r = cnt_r_;
}

float Encoder::getDistance()
{
	return distance_;
}

float Encoder::getDistance10mm()
{
	return distance_10mm_;
}

float Encoder::getTotalDistance()
{
	return total_distance_;
}

void Encoder::setTotalDistance(float true_distance){
	total_distance_ = true_distance;
}

/*
void Encoder::clearDistance()
{
	distance_ = 0;
}
*/

float Encoder::getTotalCnt()
{
	return (total_cnt_l_ + total_cnt_r_) / 2;
}

void Encoder::clearDistance10mm()
{
	//total_cnt_l_ = 0;
	//total_cnt_r_ = 0;
	distance_10mm_ = 0;
}

void Encoder::clearTotalDistance()
{
	total_distance_ = 0;
}

float Encoder::getSideLineJudgeDistance()
{
	return side_line_judge_distance_;
}

void Encoder::clearSideLineJudgeDistance()
{
	side_line_judge_distance_ = 0;
}
float Encoder::getCrossLineJudgeDistance()
{
	return cross_line_judge_distance_;
}

void Encoder::clearCrossLineJudgeDistance()
{
	cross_line_judge_distance_ = 0;
}

float Encoder::getGoalJudgeDistance()
{
	return goal_judge_distance_;
}

void Encoder::clearGoalJudgeDistance()
{
	goal_judge_distance_= 0;
}
float Encoder::getGoalAreaDistance()
{
	return goal_area_distance_;
}

void Encoder::clearGoalAreaDistance()
{
	goal_area_distance_= 0;
}




