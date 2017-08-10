/*
 * libsinc.c
 *
 *  Created on: 16 giu 2017
 *      Author: Emanuele
 */

#include "libsinc.h"

#define DEBUG_SINC 1

/* Time of send transmission of synchronization request by the node*/
struct TimeStampStruct _sendTs;

/* Time of receive transmission of synchronization request by the gateway */
struct TimeStampStruct _sincTs;

/* Time of set RTC, for OFFSET calculation */
struct TimeStampStruct _nowTs;

/* Time of synchronization time (with OFFSET adjustment) to set in RTC */
struct TimeStampStruct _newTs;

/* Time get of RTC after synchronization*/
struct TimeStampStruct _tempTs;

bool _packet_sinc = false;

struct SincStatus _sincStatus;

void startSinc()
{
	_sincStatus._packet_sinc=true;
	_sincStatus._send1=false;
	_sincStatus._receive1=false;
	_sincStatus._send2=false;
	_sincStatus._receive2=false;
}

void stopSinc()
{
	_sincStatus._packet_sinc=false;
	_sincStatus._send1=false;
	_sincStatus._receive1=false;
	_sincStatus._send2=false;
	_sincStatus._receive2=false;
}

struct SincStatus getStatus()
{
	return _sincStatus;
}

void setStatusSend1(bool s)
{
	_sincStatus._send1=s;
}

void setStatusReceive1(bool s)
{
	_sincStatus._receive1=s;
}

void setStatusSend2(bool s)
{
	_sincStatus._send2=s;
}

void setStatusReceive2(bool s)
{
	_sincStatus._receive2=s;
}

void sincRTC(struct TimeStampStruct sincTs)
{
	_sincTs = sincTs;

	/* sinc_ts in microseconds */
	uint64_t sinc_ts = getMicrosec(_sincTs);
	/* send_ts in microseconds */
	uint64_t send_ts = getMicrosec(_sendTs);
	/* now_ts in microseconds */
	uint64_t now_ts = 0;
	/* OFFSET in microseconds, between _sendTs e _nowTs */
	uint64_t offset = 0;
	/* new_ts new time of synchronization in microseconds, added of the OFFSET */
	uint64_t new_ts = 0;
	// Delay time in milliseconds from _newTs to next edge of second
	uint32_t delay_milli = 0;

	// get _nowTs for OFFSET calculation
	_nowTs = getRTCTime();
	now_ts = getMicrosec(_nowTs);

	// OFFSET calculation
	offset = now_ts - send_ts;
	// new time in microseconds, to set in RTC
	new_ts = sinc_ts + offset;
	// new time, to set in RTC
	_newTs = getTimeStampStructfromMicrosec(new_ts);

	// calculation of Delay, from millisecond in milliseconds
	delay_milli = ( 1000000 - _newTs.tv.tv_usec ) / 1000;
	// increment _newTs seconds of 1
	_newTs.tv.tv_sec = _newTs.tv.tv_sec + 1;
	_newTs.tv.tv_usec = 0;
	extendTimeStampStruct(&_newTs);

	// sperimental offset
	delay_milli -= 8;

	// run delay
	HAL_Delay(delay_milli);
	// set RTC with the new synchronization time
	setRTCTime(_newTs);

	//Log for verify sincronization
#if DEBUG_SINC
	PRINTF("Delay %d\r\n", delay_milli);
	_tempTs = getRTCTime();
	PRINTF("_tempTs\r\n");
	printTime(_tempTs);
	PRINTF("_sincTs\r\n");
	printTime(_sincTs);
	PRINTF("_nowTs\r\n");
	printTime(_nowTs);
	// difference between _sendTs and _sincTs
	int diff = 0;
	diff = sinc_ts - send_ts;
	PRINTF("diff = %d\r\n", (int)diff);
	PRINTF("OFFSET = %d\r\n", (int)offset);
	PRINTF("_newTs\r\n");
	printTime(_newTs);
#endif
}

struct TimeStampStruct getTimeStampFromBuffer(uint8_t* dataBuffer, uint8_t len)
{
	uint8_t s = 0;
	int i = 0;
	unsigned long long ts = 0;
	unsigned long long nl = 0;

	for(i=(len-1); i>=0; i--)
	{
	  nl = ( dataBuffer[i] );
	  ts |= nl << s;
	  s += 8;
	}
	return getTimeStampStructfromMillisec(ts);
}

void setSendTime()
{
	if(_sincStatus._packet_sinc==true && _sincStatus._receive1==false)
	{
		_sendTs = getRTCTime();
#if DEBUG_SINC
		PRINTF("_sendTs\r\n");
		printTime(_sendTs);
#endif
	}
//	_packet_sinc=!_packet_sinc;
}

bool getPacketSinc()
{
	return _packet_sinc;
}

void setPacketSinc(bool packet_sinc)
{
	_packet_sinc = packet_sinc;
}

struct TimeStampStruct getSendTs()
{
	return _sendTs;
}


