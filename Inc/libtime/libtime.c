#include <libtime/libtime.h>
#include "rtc.h"

extern RTC_HandleTypeDef hrtc;

uint32_t shift_register = 0;
uint8_t shift_add = 0;

struct TimeStampStruct convertInTimeStampStruct(RTC_TimeTypeDef RTC_TimeStruct, RTC_DateTypeDef RTC_DateStruct)
{
	struct TimeStampStruct ts;
	// set Date
	ts.nowtm.tm_year = RTC_DateStruct.Year;
	ts.nowtm.tm_mon = RTC_DateStruct.Month;
	ts.nowtm.tm_mday = RTC_DateStruct.Date;
	ts.nowtm.tm_wday = RTC_DateStruct.WeekDay;

	//set Time
	ts.nowtm.tm_hour = RTC_TimeStruct.Hours;
	ts.nowtm.tm_min = RTC_TimeStruct.Minutes;
	ts.nowtm.tm_sec = RTC_TimeStruct.Seconds;
	ts.nowtm.tm_isdst = RTC_TimeStruct.TimeFormat;

	//set subsecond
	ts.tv.tv_sec = mktime(&ts.nowtm);
	// RES = round(((long)1000000) / MY_PRES_D + 1) with MY_PRES_D = 0x7FFF
	ts.tv.tv_usec = (suseconds_t) (((long)1000000) - (((double)RTC_TimeStruct.SubSeconds) * (double)RES));// + ((((long)1000000) - (((long)SubSecond) * RES)) * OFFSET);

	//ts.tv.tv_usec += (suseconds_t)(((long double)ts.tv.tv_usec) * ((long double)0.000103988));

	return ts;
}

void extendTimeStampStruct(struct TimeStampStruct* ts)
{
	 gmtime_r(&ts->tv, &ts->nowtm);
}

void initTimeStampStruct(struct TimeStampStruct* ts)
{
	// set Date
	ts->nowtm.tm_year = 0;
	ts->nowtm.tm_mon = 0;
	ts->nowtm.tm_mday = 0;
	ts->nowtm.tm_wday = 0;

	// set Time
	ts->nowtm.tm_hour = 0;
	ts->nowtm.tm_min = 0;
	ts->nowtm.tm_sec = 0;
	ts->nowtm.tm_isdst = 0;

	// set subsecond
	ts->tv.tv_sec = 0;
	ts->tv.tv_usec = 0;
}

struct TimeStampStruct getRTCTime()
{
	// first freez time
//	uint32_t SubSecond = RTC_GetSubSecond();

	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_DateTypeDef RTC_DateStruct;

	// init
//	RTC_TimeStructInit(&RTC_TimeStruct);
//	RTC_DateStructInit(&RTC_DateStruct);

	//get time
//	RTC_GetTime(RTC_FORMAT_BIN, &RTC_TimeStruct);
//	RTC_GetDate(RTC_FORMAT_BIN, &RTC_DateStruct);
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStruct, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);

	return convertInTimeStampStruct(RTC_TimeStruct, RTC_DateStruct);
}

void setRTCTime(struct TimeStampStruct ts)
{
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_DateTypeDef RTC_DateStruct;

	// init
//	RTC_TimeStructInit(&RTC_TimeStruct);
//	RTC_DateStructInit(&RTC_DateStruct);

	// set Date
	RTC_DateStruct.Year = ts.nowtm.tm_year;
	RTC_DateStruct.Month = ts.nowtm.tm_mon;
	RTC_DateStruct.Date = ts.nowtm.tm_mday;
	RTC_DateStruct.WeekDay = ts.nowtm.tm_wday;

	//set Time
	RTC_TimeStruct.Hours = ts.nowtm.tm_hour;
	RTC_TimeStruct.Minutes = ts.nowtm.tm_min;
	RTC_TimeStruct.Seconds = ts.nowtm.tm_sec;
	RTC_TimeStruct.TimeFormat = ts.nowtm.tm_isdst;

	// set
	HAL_RTC_SetTime(&hrtc, &RTC_TimeStruct, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);
//	RTC_SetTime(RTC_FORMAT_BIN, &RTC_TimeStruct);
//	RTC_SetDate(RTC_FORMAT_BIN, &RTC_DateStruct);

}

struct TimeStampStruct getTimeStampStructfromMicrosec(uint64_t microsecond)
{
	struct TimeStampStruct ts;

	ts.tv.tv_sec = microsecond / 1000000;
	ts.tv.tv_usec = microsecond % 1000000;

	//printf(">> %llu\n>> %llu\n>> %llu\n", microsecond, );

	extendTimeStampStruct(&ts);

	return ts;
}

struct TimeStampStruct getTimeStampStructfromMillisec(uint64_t millisecond)
{
	struct TimeStampStruct ts;

	ts.tv.tv_sec = millisecond / 1000;
	ts.tv.tv_usec = ( millisecond % 1000 ) * 1000;

	extendTimeStampStruct(&ts);

	return ts;
}

struct TimeStampStruct myTime()
{
	struct TimeStampStruct ts;

	// set Date
	ts.nowtm.tm_year = 2014-1900;
	ts.nowtm.tm_mon = 10;
	ts.nowtm.tm_mday = 27;
	ts.nowtm.tm_wday = 1;

	//set Time
	ts.nowtm.tm_hour = 5;
	ts.nowtm.tm_min = 21;
	ts.nowtm.tm_sec = 0;
	ts.nowtm.tm_isdst = RTC_HOURFORMAT12_PM;

	//set subsecond
	ts.tv.tv_sec = mktime(&ts.nowtm);
	ts.tv.tv_usec = (suseconds_t) 0;

	return ts;
}

void printTime(struct TimeStampStruct ts)
{
	char buf[80];
	strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts.nowtm);
	PRINTF("%s\n", buf);

//	time_t t_of_day;
//	t_of_day = mktime(&ts.nowtm);
//	PRINTF("seconds since the Epoch: %ld\n", (long) t_of_day);

	PRINTF("microsec: %ld%ld\n", (long)(ts.tv.tv_sec), (long)(ts.tv.tv_usec));

}

void printRTC()
{
	struct TimeStampStruct ts;
	ts = getRTCTime();
	printTime(ts);
}

uint64_t getMicrosec(struct TimeStampStruct ts)
{
	return (((uint64_t)(ts.tv.tv_sec)) * 1000000) + (uint64_t)(ts.tv.tv_usec);
}

uint64_t getMillisec(struct TimeStampStruct ts)
{
	return (((uint64_t)(ts.tv.tv_sec)) * 1000000) + ( (uint64_t)(ts.tv.tv_usec) / 1000 );
}

