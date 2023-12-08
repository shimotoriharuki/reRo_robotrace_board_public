/*
 * globalDefine.h
 *
 *  Created on: Jun 21, 2023
 *      Author: under
 */

#ifndef INC_GLOBALDEFINE_H_
#define INC_GLOBALDEFINE_H_


#define ADC_DATA_SIZE 13

#define DELTA_T 0.001
#define PI 3.1415926535

#define WHEEL_RADIUS 12 //[mm]
#define ENCODER_RESOLUTION 4096
#define REDUCTION_RATIO 0.328125 //Gear reduction ratio
#define DISTANCE_PER_CNT (2 * PI * WHEEL_RADIUS * REDUCTION_RATIO / ENCODER_RESOLUTION) //[mm/cnt]=[m/s]

#define MAX_BATTERY_VOLTAGE 12.6
#define MAX_BATTERY_VOLTAGE_AD_VALUE 3200//MAX_BATTERY_VOLTAGEのときのADCの値（実測）

#define LEFT false
#define RIGHT true

#define SUCTION_DUTY 1000 //700



#endif /* INC_GLOBALDEFINE_H_ */
