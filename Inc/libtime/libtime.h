
#ifndef LIBTIME_LIBTIME_H_
#define LIBTIME_LIBTIME_H_

#include "sys/time.h"
#include "time.h"
#include "stm32l4xx_hal.h"
#include "vcom.h"

//#define MY_PRES_A		0x00		// max clock resolution
//#define MY_PRES_D		0x7FFF		// 32767 <- MAX_COUNT_PRESCALER_SINCRO - 1 = 32768 - 1 = 32767
//#define RES 			30.517578125	// (((long)1000000) / MY_PRES_D + 1) with MY_PRES_D = 0x7FFF
//#define FRACT_TO_MICROS	0.0002328306	// (1000000 / 2^32) -> micros = fraction * (1000000 / 2^32)
//#define MICROS_TO_FRACT	4294.967296		// (2^32 / 1000000) -> fraction = micros * (2^32 / 1000000)

struct TimeStampStruct{
	struct tm nowtm;	// data and time
	struct timeval tv;	// second & subsecond
};


struct TimeStampStruct global_ts;

struct TimeStampStruct convertInTimeStampStruct(RTC_TimeTypeDef RTC_TimeStruct, RTC_DateTypeDef RTC_DateStruct);

void extendTimeStampStruct(struct TimeStampStruct* ts);

void initTimeStampStruct(struct TimeStampStruct* ts);

struct TimeStampStruct getRTCTime();

void setRTCTime(struct TimeStampStruct ts);

struct TimeStampStruct myTime();

void printTime(struct TimeStampStruct ts);

void printRTC();

uint64_t getMicrosec(struct TimeStampStruct ts);

uint64_t getMillisec(struct TimeStampStruct ts);

struct TimeStampStruct getTimeStampStructfromMicrosec(uint64_t microsecond);

struct TimeStampStruct getTimeStampStructfromMillisec(uint64_t millisecond);

#endif
