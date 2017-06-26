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

void sincRTC(struct TimeStampStruct );

struct TimeStampStruct getTimeStampFromBuffer(uint8_t* , uint8_t );

void setSendTime();

bool getPacketSinc();

void setPacketSinc(bool packet_sinc);

struct TimeStampStruct getSendTs();

#endif /* LIBSINC_LIBSINC_H_ */
