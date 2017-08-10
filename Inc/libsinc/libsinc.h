/*
 * libsinc.h
 *
 *  Created on: 16 giu 2017
 *      Author: Emanuele
 */

#ifndef LIBSINC_LIBSINC_H_
#define LIBSINC_LIBSINC_H_

#include <stdbool.h>
#include "libtime/libtime.h"

struct SincStatus{
	bool _packet_sinc;
	bool _send1;
	bool _receive1;
	bool _send2;
	bool _receive2;
	uint8_t _seed;
	uint8_t _timeout;
};

void sincRTC(struct TimeStampStruct );

struct TimeStampStruct getTimeStampFromBuffer(uint8_t* , uint8_t );

void setSendTime();

bool getPacketSinc();

void setPacketSinc(bool packet_sinc);

struct TimeStampStruct getSendTs();

struct SincStatus _sincStatus;

void startSinc();

void stopSinc();

struct SincStatus getStatus();

void setStatusSend1(bool );

void setStatusReceive1(bool );

void setStatusSend2(bool );

void setStatusReceive2(bool );

#endif /* LIBSINC_LIBSINC_H_ */
