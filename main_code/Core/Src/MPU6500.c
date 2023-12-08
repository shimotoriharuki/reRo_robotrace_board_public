/*
 * MPU6500.c
 *
 *  Created on: 2023/05/14
 *      Author: iguchi
 */

#include "mpu6500.h"

volatile int16_t xa, ya, za;
volatile int16_t xg, yg, zg;

uint8_t read_byte( uint8_t reg ) {
	uint8_t ret,val;

		ret = reg | 0x80;
		CS_RESET;
		HAL_SPI_Transmit(&hspi2, &ret, 1, 100);
		HAL_SPI_Receive(&hspi2, &val, 1, 100);
		CS_SET;

	return val;
}

void write_byte( uint8_t reg, uint8_t val )  {
	uint8_t ret;

	ret = reg & 0x7F;
	CS_RESET;
	HAL_SPI_Transmit(&hspi2, &ret, 1, 100);
	HAL_SPI_Receive(&hspi2, &val, 1, 100);
	CS_SET;
}

uint8_t IMU_init() {
	uint8_t who_am_i, ret;
	ret = 0;

	who_am_i = read_byte( 0x75 );
	if ( who_am_i == 0x70 ) {
		ret = 1;
		write_byte(0x6B, 0x00);	//sleep mode解除
		HAL_Delay(100);
		write_byte(0x1A, 0x00);
		write_byte(0x1B, 0x18);
	}
	return ret;
}

void read_gyro_data() {
	//xg = ((int16_t)read_byte(0x43) << 8) | ((int16_t)read_byte(0x44));
	//yg = ((int16_t)read_byte(0x45) << 8) | ((int16_t)read_byte(0x46));
	zg = ((int16_t)read_byte(0x47) << 8) | ((int16_t)read_byte(0x48));
}

void read_accel_data() {
	xa = ((int16_t)read_byte(0x3B) << 8) | ((int16_t)read_byte(0x3C));
	//ya = ((int16_t)read_byte(0x3D) << 8) | ((int16_t)read_byte(0x3E));
	//za = ((int16_t)read_byte(0x3F) << 8) | ((int16_t)read_byte(0x40));
}
