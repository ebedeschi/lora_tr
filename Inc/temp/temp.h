/*
 * temp.h
 *
 *  Created on: 27 lug 2017
 *      Author: Emanuele
 */

#ifndef TEMP_TEMP_H_
#define TEMP_TEMP_H_

#include "stm32l4xx_hal.h"
#include "libtime/libtime.h"

#define DIM_TEMP 10 // temperature vector dimension: 10 elements
#define DIM_Q 10 // queue dimension

struct TempStruct{
	float temp[DIM_TEMP];
	uint8_t ct;
	uint8_t complete;
	struct TimeStampStruct t;
};

void insert(float );

uint8_t checkExtract();

struct TempStruct extract();

bool getAcquire();

void setAcquire(bool );

#endif /* TEMP_TEMP_H_ */
