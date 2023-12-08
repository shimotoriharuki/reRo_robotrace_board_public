/*
 * MPU6500.h
 *
 *  Created on: 2023/05/14
 *      Author: iguchi
 */

#ifndef SRC_IMU_MPU6500_H_
#define SRC_IMU_MPU6500_H_

#include "main.h"

extern SPI_HandleTypeDef hspi2;
// IMUから取得したデータ
extern volatile int16_t 	xa, ya, za;
extern volatile int16_t 	xg, yg, zg;

#define CS_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define CS_SET   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

#define ACCEL_RANGE		2.0f		// 2[g]
#define GYRO_RANGE		2000.0f		// 2000[deg/s]
#define MAXDATA_RANGE	32764.0f	// 16bitデータの最大値
#define G_ACCELERATION	9.80665.0f	// G

#ifdef __cplusplus
extern "C" {
#endif

uint8_t read_byte(uint8_t);
void write_byte(uint8_t, uint8_t);
uint8_t IMU_init(void);
void read_gyro_data(void);
void read_accel_data(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_IMU_MPU6500_H_ */
